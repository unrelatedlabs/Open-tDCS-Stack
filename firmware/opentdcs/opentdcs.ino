#include <bluefruit.h>

const int PIN_PWM = 3;
const int PIN_ADC_CURRENT = A0;
const int PIN_ADC_BATTERY = A1;
const int PIN_ADC_OUTPUT = A2;

const float ADC_REF = 3.3;
const float ADC_MAX = 1023.0;
const float RSENSE = 180.0;
const float VDIV = 6.0;
const uint16_t PWM_MAX = 255;
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

SoftwareTimer rampTimer;
SoftwareTimer measurementTimer;

BLEService tdcsService("a1b2c3d4-1234-5678-abcd-ef0123456789");
BLECharacteristic measChar("a1b2c3d5-1234-5678-abcd-ef0123456789", BLERead | BLENotify, 10);
BLECharacteristic timerChar("a1b2c3d6-1234-5678-abcd-ef0123456789", BLERead | BLEWrite | BLENotify, 8);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);  // Wait up to 3s for serial
  Serial.println("OpenTDCS Starting...");

  pinMode(PIN_PWM, OUTPUT);
  analogWrite(PIN_PWM, 0);

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  ledOff();

  analogReadResolution(10);
  
  Bluefruit.begin();
  Bluefruit.setTxPower(-4);
  Bluefruit.setName("tDCS");
  Bluefruit.Periph.setConnectCallback(onConnect);
  Bluefruit.Periph.setDisconnectCallback(onDisconnect);
  
  tdcsService.begin();
  measChar.begin();
  timerChar.setWriteCallback(onTimerWrite);
  timerChar.begin();
  
  startAdv();
  ledGreen();

  rampTimer.begin(20, rampCallback);
  // rampTimer starts only during sessions

  measurementTimer.begin(500, measurementCallback);
  measurementTimer.start();

  Serial.println("Initialization complete. Advertising...");

  suspendLoop();
}

void loop() {}

void rampCallback(TimerHandle_t _handle) {
  updateSession();
}

void measurementCallback(TimerHandle_t _handle) {
  updateMeasurements();
  updateBLE();
  updateLED();

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
    Serial.print("I:");
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
  updateLED();
}

void onDisconnect(uint16_t conn, uint8_t reason) {
  isConnected = false;
  Serial.print("BLE Disconnected. Reason: ");
  Serial.println(reason);
  Bluefruit.Advertising.start(0);
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
      rampTimer.start();
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
      Serial.println("Session STOP");
    }
  }
}

void updateMeasurements() {
  float vCurrent = analogRead(PIN_ADC_CURRENT) * ADC_REF / ADC_MAX;
  float vBattery = analogRead(PIN_ADC_BATTERY) * ADC_REF / ADC_MAX * VDIV;
  float vOutput = analogRead(PIN_ADC_OUTPUT) * ADC_REF / ADC_MAX * VDIV;
  
  float currentA = vCurrent / RSENSE;
  measuredCurrent = (uint16_t)(currentA * 1000000);
  batteryVoltage = (uint16_t)(vBattery * 1000);
  outputVoltage = (uint16_t)(vOutput * 1000);
  
  if (measuredCurrent > 10) {
    float electrodeV = vOutput - (vCurrent * VDIV);
    impedance = (uint16_t)((electrodeV / currentA) / 1000);
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
  uint16_t pwm = (uint32_t)currentUA * PWM_MAX / CURRENT_MAX_UA;
  if (pwm > PWM_MAX) pwm = PWM_MAX;
  analogWrite(PIN_PWM, pwm);
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

void updateLED() {
  ledOff();
  if (sessionActive) ledRed();
  else if (isConnected) ledBlue();
  else ledGreen();
}

void ledOff() {
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
}

void ledRed() { digitalWrite(LED_RED, LOW); }
void ledGreen() { digitalWrite(LED_GREEN, LOW); }
void ledBlue() { digitalWrite(LED_BLUE, LOW); }