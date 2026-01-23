/*
 * OpenTDCS Firmware
 * Copyright (C) 2024-2026 Peter Kuhar and OpenTDCS Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include <bluefruit.h>
#include <HardwarePWM.h>

const int PIN_PWM = 10;
const int PIN_ADC_CURRENT = A0;
const int PIN_ADC_OUTPUT = A1;
const int PIN_ADC_BATTERY = A2;

// LiPo battery measurement (XIAO internal)
const int PIN_LIPO_ENABLE = 14;  // P0.14 pull-down to enable divider
// P0.31 = AIN7, read via nRF52 SAADC directly

// Dickson multiplier pins
const int PIN_MULT_A1 = 9;   // D9 - drives with D7
const int PIN_MULT_B = 8;    // D8 - inverted
const int PIN_MULT_A2 = 7;   // D7 - drives with D9

// Dixon Multiplier PWM: 50% duty cycle
const uint16_t MULT_TOP = 16;    // PWM period 
const uint16_t MULT_DUTY = MULT_TOP/2;    // 50% duty

volatile bool multiplierEnabled = false;

const float ADC_REF = 3.3;
const float ADC_MAX = 1023.0;
const float RSENSE = 1000.0;
const float RBASE = 5000.0;    // base resistor
const float VBE = 0.65;         // base-emitter voltage
const float VDIV_OUT = 5.3;
const float VDIV_BAT = 5.3;
const float VDIV_LIPO = 2.96;  // (1M + 510k) / 510k

const uint16_t PWM_MAX = 255;  // 8-bit PWM
const uint16_t CURRENT_MAX_UA = 4000;
const uint16_t TEST_CURRENT_UA = 50;

// PID controller
const float KP = 0.02;   // conservative proportional
const float KI = 0.001;  // slow integral
const float INT_MAX = 50.0;  // max integral contribution (PWM counts)
float pidIntegral = 0;
float pidPwm = 0;

volatile uint16_t targetCurrent = 0;
volatile uint16_t totalTime = 0;
volatile uint16_t rampUp = 0;
volatile uint16_t rampDown = 0;
volatile uint16_t timeRemaining = 0;
volatile bool sessionActive = false;
volatile unsigned long sessionStart = 0;

uint16_t measuredCurrent = 0;
uint16_t batteryVoltage = 0;
uint16_t outputVoltage = 0;
uint16_t impedance = 0;
uint16_t setCurrent = 0;
uint16_t lipoVoltage = 0;

volatile bool isConnected = false;
uint8_t lastLedState = 0;  // 0=off, 1=green, 2=blue, 3=red

SoftwareTimer rampTimer;
SoftwareTimer measurementTimer;
SoftwareTimer blinkTimer;

BLEService tdcsService("a1b2c3d4-1234-5678-abcd-ef0123456789");
BLECharacteristic measChar("a1b2c3d5-1234-5678-abcd-ef0123456789", BLERead | BLENotify, 12);
BLECharacteristic timerChar("a1b2c3d6-1234-5678-abcd-ef0123456789", BLERead | BLEWrite | BLENotify, 8);

// Ownership token for HwPWM3 (any non-zero value)
#define MULT_PWM_TOKEN 0x4D554C54  // "MULT"

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);  // Wait up to 3s for serial
  Serial.println("Open-tDCS-Stack Starting...");

  // Reserve HwPWM3 for multiplier before analogWrite can claim it
  HwPWM3.takeOwnership(MULT_PWM_TOKEN);

  // Configure PWM: 8-bit resolution, ~62.5kHz frequency (16MHz / 256)
  analogWriteResolution(8);
  pinMode(PIN_PWM, OUTPUT);
  analogWrite(PIN_PWM, 0);  // Will use HwPWM0/1/2, not HwPWM3

  // Dickson multiplier pins use HwPWM3 (reserved above)

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  ledOff();

  analogReadResolution(10);

  // LiPo measurement enable pin - start disabled (high-Z)
  pinMode(PIN_LIPO_ENABLE, INPUT);

  Bluefruit.begin();
  Bluefruit.autoConnLed(false);  // Disable automatic LED control
  Bluefruit.setTxPower(-4);
  Bluefruit.setName("tDCS");
  Bluefruit.Periph.setConnectCallback(onConnect);
  Bluefruit.Periph.setDisconnectCallback(onDisconnect);
  
  tdcsService.begin();
  measChar.begin();
  timerChar.setWriteCallback(onTimerWrite);
  timerChar.begin();
  
  startAdv();

  rampTimer.begin(20, rampCallback);
  // rampTimer starts only during sessions

  measurementTimer.begin(500, measurementCallback);
  measurementTimer.start();

  // Blink timer for idle state (50ms LED on)
  blinkTimer.begin(50, blinkOffCallback, NULL, false);  // one-shot (repeating=false)

  // Multiplier starts off, enabled when connected or session active
  updateLED();

  Serial.println("Initialization complete. Advertising...");

  suspendLoop();
}

void loop() {}

void rampCallback(TimerHandle_t _handle) {
  updateSession();
  if (sessionActive) updatePID();
}

void blinkOffCallback(TimerHandle_t _handle) {
  ledOff();  // Turn off LED after 50ms blink
}

void multiplierOn() {
  if (multiplierEnabled) return;
  multiplierEnabled = true;
  
  // Configure HwPWM3 for multiplier
  HwPWM3.setMaxValue(MULT_TOP);
  HwPWM3.addPin(PIN_MULT_A1);
  HwPWM3.addPin(PIN_MULT_B);
  HwPWM3.addPin(PIN_MULT_A2);
  
  // Write PWM values: D9/D7 normal, D8 inverted
  HwPWM3.writePin(PIN_MULT_A1, MULT_DUTY, false);  // D9 - normal
  HwPWM3.writePin(PIN_MULT_B,  MULT_DUTY, true);   // D8 - inverted
  HwPWM3.writePin(PIN_MULT_A2, MULT_DUTY, false);  // D7 - normal
  
  // Enable high drive on all pins after PWM is running
  pinMode(PIN_MULT_A1, OUTPUT_H0H1);
  pinMode(PIN_MULT_B,  OUTPUT_H0H1);
  pinMode(PIN_MULT_A2, OUTPUT_H0H1);
  
  Serial.println("Multiplier ON (HwPWM3, 50% duty, D8 inverted, high drive)");
}

void multiplierOff() {
  if (!multiplierEnabled) return;
  multiplierEnabled = false;
  
  // Set duty to 0 (outputs go low)
  HwPWM3.writePin(PIN_MULT_A1, 0, false);
  HwPWM3.writePin(PIN_MULT_B,  0, false);
  HwPWM3.writePin(PIN_MULT_A2, 0, false);
  
  // Remove pins and stop
  HwPWM3.removePin(PIN_MULT_A1);
  HwPWM3.removePin(PIN_MULT_B);
  HwPWM3.removePin(PIN_MULT_A2);
  HwPWM3.stop();
  
  Serial.println("Multiplier OFF");
}

void measurementCallback(TimerHandle_t _handle) {
  updateMeasurements();
  updateBLE();

  // Apply test current when connected but not in session
  if (!sessionActive) {
    if (isConnected) {
      setCurrent = TEST_CURRENT_UA;
      applyPWM(TEST_CURRENT_UA);
    } else {
      setCurrent = 0;
      applyPWM(0);
    }
  }

  // Periodic status logging
  if (sessionActive || isConnected) {
    Serial.print("Set:");
    Serial.print(setCurrent);
    Serial.print("uA I:");
    Serial.print(measuredCurrent);
    Serial.print("uA Z:");
    Serial.print(impedance);
    Serial.print("Ω Vbat:");
    Serial.print(batteryVoltage);
    Serial.print("mV Vout:");
    Serial.print(outputVoltage);
    Serial.print("mV Vlipo:");
    Serial.print(lipoVoltage);
    Serial.print("mV");
    if (sessionActive) {
      Serial.print(" T-");
      Serial.print(timeRemaining);
      Serial.print("s");
    }
    Serial.println();
  }
}

void startAdv() {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addService(tdcsService);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.setInterval(160, 1600);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

void onConnect(uint16_t conn) {
  isConnected = true;
  Serial.println("BLE Connected");
  multiplierOn();
  updateLED();
}

void onDisconnect(uint16_t conn, uint8_t reason) {
  isConnected = false;
  Serial.print("BLE Disconnected. Reason: ");
  Serial.println(reason);
  Bluefruit.Advertising.start(0);
  if (!sessionActive) {
    multiplierOff();
  }
  updateLED();
}

uint16_t readAIN7() {
  // Read P0.31 (AIN7) using nRF52 SAADC directly
  // Configure channel 0 for AIN7
  NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_10bit;
  NRF_SAADC->CH[0].PSELP = SAADC_CH_PSELP_PSELP_AnalogInput7;
  NRF_SAADC->CH[0].PSELN = SAADC_CH_PSELN_PSELN_NC;
  NRF_SAADC->CH[0].CONFIG = (SAADC_CH_CONFIG_GAIN_Gain1_6 << SAADC_CH_CONFIG_GAIN_Pos) |
                            (SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos) |
                            (SAADC_CH_CONFIG_TACQ_10us << SAADC_CH_CONFIG_TACQ_Pos) |
                            (SAADC_CH_CONFIG_MODE_SE << SAADC_CH_CONFIG_MODE_Pos);
  
  volatile int16_t result;
  NRF_SAADC->RESULT.PTR = (uint32_t)&result;
  NRF_SAADC->RESULT.MAXCNT = 1;
  
  NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Enabled;
  NRF_SAADC->TASKS_START = 1;
  while (!NRF_SAADC->EVENTS_STARTED);
  NRF_SAADC->EVENTS_STARTED = 0;
  
  NRF_SAADC->TASKS_SAMPLE = 1;
  while (!NRF_SAADC->EVENTS_END);
  NRF_SAADC->EVENTS_END = 0;
  
  NRF_SAADC->TASKS_STOP = 1;
  while (!NRF_SAADC->EVENTS_STOPPED);
  NRF_SAADC->EVENTS_STOPPED = 0;
  
  NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Disabled;
  
  // Internal ref 0.6V, gain 1/6 = 3.6V range
  // Scale to match analogRead() 3.3V range: result * 3.6/3.3
  if (result < 0) result = 0;
  return (uint16_t)((result * 36UL) / 33UL);
}

void onTimerWrite(uint16_t conn, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  if (len >= 8) {
    targetCurrent = data[0] | (data[1] << 8);
    totalTime = data[2] | (data[3] << 8);
    rampUp = data[4] | (data[5] << 8);
    rampDown = data[6] | (data[7] << 8);

    if (totalTime > 0 && targetCurrent > 0) {
      timeRemaining = totalTime;
      sessionStart = millis();
      sessionActive = true;
      multiplierOn();  // Ensure multiplier is on for session
      rampTimer.start();
      updateLED();
      Serial.print("Session START: ");
      Serial.print(targetCurrent);
      Serial.print("uA, ");
      Serial.print(totalTime);
      Serial.print("s, ramp ");
      Serial.print(rampUp);
      Serial.print("/");
      Serial.println(rampDown);
    } else {
      sessionActive = false;
      timeRemaining = 0;
      setCurrent = 0;
      rampTimer.stop();
      applyPWM(0);
      if (!isConnected) {
        multiplierOff();
      }
      updateLED();
      Serial.println("Session STOP");
    }
  }
}

void updateMeasurements() {
  float vCurrent = analogRead(PIN_ADC_CURRENT) * ADC_REF / ADC_MAX;
  float vBattery = analogRead(PIN_ADC_BATTERY) * ADC_REF / ADC_MAX * VDIV_BAT;
  float vOutput = analogRead(PIN_ADC_OUTPUT) * ADC_REF / ADC_MAX * VDIV_OUT;
  
  // Read LiPo battery with GPIO-enabled divider
  pinMode(PIN_LIPO_ENABLE, OUTPUT);
  digitalWrite(PIN_LIPO_ENABLE, LOW);
  delayMicroseconds(100);
  float vLipo = readAIN7() * ADC_REF / ADC_MAX * VDIV_LIPO;
  pinMode(PIN_LIPO_ENABLE, INPUT);  // disable divider to save power
  
  // Total current through sense resistor
  float totalCurrentA = vCurrent / RSENSE;
  
  // Subtract base current: I_base = (V_pwm - Vbe - Vsense) / Rbase
  // V_pwm depends on setCurrent (same formula as applyPWM)
  float setCurrentA = setCurrent / 1000000.0;
  float vPwm = setCurrentA * RSENSE + VBE;
  float vBaseR = vPwm - VBE - vCurrent;
  float baseCurrentA = (vBaseR > 0) ? vBaseR / RBASE : 0;
  float currentA = totalCurrentA - baseCurrentA;
  if (currentA < 0) currentA = 0;
  
  measuredCurrent = (uint16_t)(currentA * 1000000);
  batteryVoltage = (uint16_t)(vBattery * 1000);
  outputVoltage = (uint16_t)(vOutput * 1000);
  lipoVoltage = (uint16_t)(vLipo * 1000);

  // Calculate impedance: R = V / I = (vBattery - vOutput) / current
  if (measuredCurrent > 10) {
    float electrodeV = vBattery - vOutput;
    impedance = (uint16_t)(electrodeV / currentA);
  } else {
    impedance = 0xFFFF;
  }
}

void updateSession() {
  if (sessionActive) {
    unsigned long elapsedMs = millis() - sessionStart;
    unsigned long elapsedSec = elapsedMs / 1000;
    unsigned long totalTimeMs = (unsigned long)totalTime * 1000;
    unsigned long rampUpMs = (unsigned long)rampUp * 1000;
    unsigned long rampDownMs = (unsigned long)rampDown * 1000;

    if (elapsedMs >= totalTimeMs) {
      sessionActive = false;
      timeRemaining = 0;
      setCurrent = 0;
      rampTimer.stop();
      applyPWM(0);
      if (!isConnected) {
        multiplierOff();
      }
      updateLED();
      Serial.println("Session COMPLETE");
      return;
    }

    timeRemaining = totalTime - elapsedSec;
    uint16_t current = targetCurrent;
    unsigned long remainingMs = totalTimeMs - elapsedMs;

    if (elapsedMs < rampUpMs && rampUp > 0) {
      current = ((unsigned long)targetCurrent * elapsedMs) / rampUpMs;
    } else if (remainingMs < rampDownMs && rampDown > 0) {
      current = ((unsigned long)targetCurrent * remainingMs) / rampDownMs;
    }

    setCurrent = current;
    // PID handles actual PWM output
  }
}

uint16_t readCurrentFast() {
  float vCurrent = analogRead(PIN_ADC_CURRENT) * ADC_REF / ADC_MAX;
  float totalCurrentA = vCurrent / RSENSE;
  
  // subtract base current: Ib = (Vpwm - Vbe - Vsense) / Rbase
  float vPwmActual = pidPwm * ADC_REF / PWM_MAX;
  float vBaseR = vPwmActual - VBE - vCurrent;
  float baseCurrentA = (vBaseR > 0) ? vBaseR / RBASE : 0;
  float currentA = totalCurrentA - baseCurrentA;
  if (currentA < 0) currentA = 0;
  
  return (uint16_t)(currentA * 1000000);
}

void updatePID() {
  if (setCurrent == 0) {
    pidIntegral = 0;
    pidPwm = 0;
    analogWrite(PIN_PWM, 0);
    return;
  }
  
  uint16_t current = readCurrentFast();
  float error = (float)setCurrent - (float)current;
  
  // feedforward: estimate PWM from desired current
  float currentA = setCurrent / 1000000.0;
  float vPwm = currentA * RSENSE + VBE;
  float feedforward = (vPwm / ADC_REF) * PWM_MAX;
  
  // PI correction with integral clamping
  pidIntegral += error;
  if (pidIntegral > INT_MAX / KI) pidIntegral = INT_MAX / KI;
  if (pidIntegral < -INT_MAX / KI) pidIntegral = -INT_MAX / KI;
  
  float correction = KP * error + KI * pidIntegral;
  pidPwm = feedforward + correction;
  
  if (pidPwm > PWM_MAX) pidPwm = PWM_MAX;
  if (pidPwm < 0) pidPwm = 0;
  
  analogWrite(PIN_PWM, (uint16_t)pidPwm);
  pinMode(PIN_PWM, OUTPUT_H0H1);
}

void applyPWM(uint16_t currentUA) {
  if (currentUA == 0) {
    pidIntegral = 0;
    pidPwm = 0;
    analogWrite(PIN_PWM, 0);
    return;
  }
  // open-loop for test current (non-session)
  float currentA = currentUA / 1000000.0;
  float vRsense = currentA * RSENSE;
  float vPwm = vRsense + VBE;
  uint16_t pwm = (uint16_t)(vPwm / ADC_REF * PWM_MAX);
  if (pwm > PWM_MAX) pwm = PWM_MAX;
  analogWrite(PIN_PWM, pwm);
  pinMode(PIN_PWM, OUTPUT_H0H1);
}

void updateBLE() {
  uint8_t meas[12];
  meas[0] = setCurrent & 0xFF;
  meas[1] = setCurrent >> 8;
  meas[2] = measuredCurrent & 0xFF;
  meas[3] = measuredCurrent >> 8;
  meas[4] = batteryVoltage & 0xFF;
  meas[5] = batteryVoltage >> 8;
  meas[6] = outputVoltage & 0xFF;
  meas[7] = outputVoltage >> 8;
  meas[8] = impedance & 0xFF;
  meas[9] = impedance >> 8;
  meas[10] = lipoVoltage & 0xFF;
  meas[11] = lipoVoltage >> 8;
  measChar.notify(meas, 12);
  
  uint8_t timer[8];
  timer[0] = targetCurrent & 0xFF;
  timer[1] = targetCurrent >> 8;
  timer[2] = timeRemaining & 0xFF;
  timer[3] = timeRemaining >> 8;
  timer[4] = rampUp & 0xFF;
  timer[5] = rampUp >> 8;
  timer[6] = rampDown & 0xFF;
  timer[7] = rampDown >> 8;
  timerChar.notify(timer, 8);
}

static unsigned long lastBlinkTime = 0;

void updateLED() {
  uint8_t newState;
  if (sessionActive) newState = 3;  // red (solid)
  else if (isConnected) newState = 2;  // blue (solid)
  else newState = 1;  // green (blinking)

  if (newState != lastLedState) {
    ledOff();
    blinkTimer.stop();
    if (newState == 3) ledRed();
    else if (newState == 2) ledBlue();
    // Green handled by blinking below
    lastLedState = newState;
    lastBlinkTime = 0;  // Reset blink timing
  }

  // Handle green blinking in idle state (50ms on every 4s)
  if (newState == 1) {
    unsigned long now = millis();
    if (now - lastBlinkTime >= 4000) {
      lastBlinkTime = now;
      ledGreen();
      blinkTimer.start();  // Turn off after 50ms
    }
  }
}

void ledOff() {
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
}

void ledRed() { digitalWrite(LED_RED, LOW); }
void ledGreen() { digitalWrite(LED_GREEN, LOW); }
void ledBlue() { digitalWrite(LED_BLUE, LOW); }