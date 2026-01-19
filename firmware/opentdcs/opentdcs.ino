#include <bluefruit.h>
#include <HardwarePWM.h>

const int PIN_PWM = 10;
const int PIN_ADC_CURRENT = A0;
const int PIN_ADC_OUTPUT = A1;
const int PIN_ADC_BATTERY = A2;

// Dickson multiplier pins
const int PIN_MULT_A1 = 9;   // D9 - drives with D7
const int PIN_MULT_B = 8;    // D8 - inverted
const int PIN_MULT_A2 = 7;   // D7 - drives with D9

// Dixon Multiplier PWM: 50% duty cycle
const uint16_t MULT_TOP = 32;    // PWM period 
const uint16_t MULT_DUTY = MULT_TOP/2;    // 50% duty

volatile bool multiplierEnabled = false;

const float ADC_REF = 3.3;
const float ADC_MAX = 1023.0;
const float RSENSE = 1000.0;
const float VDIV_OUT = 5.3;
const float VDIV_BAT = 5.3;

const uint16_t PWM_MAX = 255;  // 8-bit PWM
const uint16_t CURRENT_MAX_UA = 4000;
const uint16_t TEST_CURRENT_UA = 50;

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

volatile bool isConnected = false;
uint8_t lastLedState = 0;  // 0=off, 1=green, 2=blue, 3=red

SoftwareTimer rampTimer;
SoftwareTimer measurementTimer;
SoftwareTimer blinkTimer;

BLEService tdcsService("a1b2c3d4-1234-5678-abcd-ef0123456789");
BLECharacteristic measChar("a1b2c3d5-1234-5678-abcd-ef0123456789", BLERead | BLENotify, 10);
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
  
  float currentA = vCurrent / RSENSE;
  measuredCurrent = (uint16_t)(currentA * 1000000);
  batteryVoltage = (uint16_t)(vBattery * 1000);
  outputVoltage = (uint16_t)(vOutput * 1000);

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
    applyPWM(current);
  }
}

void applyPWM(uint16_t currentUA) {
  if (currentUA == 0) {
    analogWrite(PIN_PWM, 0);
    return;
  }
  float currentA = currentUA / 1000000.0;
  float vRsense = currentA * RSENSE;
  float vPwm = vRsense + 0.7;
  uint16_t pwm = (uint16_t)(vPwm / ADC_REF * PWM_MAX);
  if (pwm > PWM_MAX) pwm = PWM_MAX;
  analogWrite(PIN_PWM, pwm);
  pinMode(PIN_PWM, OUTPUT_H0H1);
}

void updateBLE() {
  uint8_t meas[10];
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
  measChar.notify(meas, 10);
  
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