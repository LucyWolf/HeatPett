#include <Arduino.h>
#include <bluefruit.h>
#include <nrf_gpio.h>
#include <nrf_power.h>

// Pins
#define MOTOR_LEFT   43   // D14 = P1.11
#define MOTOR_RIGHT  10   // D16 = P0.10
#define LED_PIN      15   // P0.15
#define BUTTON_PIN    6   // D1 = P0.06
#define BATTERY_PIN   4   // P0.04 = AIN2

// Motor Logik (wie im Original ESP8266 Code)
#define HAPTIC_ON  HIGH
#define HAPTIC_OFF LOW

// BLE
BLEUart bleuart;
bool connected = false;

// Button
unsigned long buttonPressStart = 0;
bool buttonHeld = false;
bool buttonWasReleased = true;

// LED Blink
unsigned long lastBlink = 0;

// Akku (angepasst für nRF52840 ADC)
#define ADCResolution        4095.0   // nRF52840 hat 12bit ADC
#define ADCVoltageMax        3.6
#define BATTERY_SHIELD_R1    100.0
#define BATTERY_SHIELD_R2    220.0
#define BATTERY_SHIELD_RESISTANCE 180.0
#define ADCMultiplier ((BATTERY_SHIELD_R1 + BATTERY_SHIELD_R2 + BATTERY_SHIELD_RESISTANCE) / BATTERY_SHIELD_R1)

float getBatteryLevel() {
  float voltage = ((float)analogRead(BATTERY_PIN)) * ADCVoltageMax / ADCResolution * ADCMultiplier;
  float level = 0.0f;
  if (voltage > 3.975f)
    level = (voltage - 2.920f) * 0.8f;
  else if (voltage > 3.678f)
    level = (voltage - 3.300f) * 1.25f;
  else if (voltage > 3.489f)
    level = (voltage - 3.400f) * 1.7f;
  else if (voltage > 3.360f)
    level = (voltage - 3.300f) * 0.8f;
  else
    level = (voltage - 3.200f) * 0.3f;
  level = (level - 0.05f) / 0.95f;
  return max(min(level, 1.0f), 0.0f);
}

void startupVibration() {
  digitalWrite(MOTOR_LEFT, HAPTIC_ON);
  digitalWrite(MOTOR_RIGHT, HAPTIC_ON);
  delay(400);
  digitalWrite(MOTOR_LEFT, HAPTIC_OFF);
  digitalWrite(MOTOR_RIGHT, HAPTIC_OFF);
  delay(100);
  digitalWrite(MOTOR_LEFT, HAPTIC_ON);
  digitalWrite(MOTOR_RIGHT, HAPTIC_ON);
  delay(400);
  digitalWrite(MOTOR_LEFT, HAPTIC_OFF);
  digitalWrite(MOTOR_RIGHT, HAPTIC_OFF);
  delay(100);
  digitalWrite(MOTOR_LEFT, HAPTIC_ON);
  digitalWrite(MOTOR_RIGHT, HAPTIC_ON);
  delay(600);
  digitalWrite(MOTOR_LEFT, HAPTIC_OFF);
  digitalWrite(MOTOR_RIGHT, HAPTIC_OFF);
}

void shutdownAnimation() {
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  for (int i = 255; i >= 0; i -= 3) {
    analogWrite(LED_PIN, i);
    delay(10);
  }
  analogWrite(LED_PIN, 0);
}

void goToSleep() {
  digitalWrite(MOTOR_LEFT, HAPTIC_OFF);
  digitalWrite(MOTOR_RIGHT, HAPTIC_OFF);
  analogWrite(MOTOR_LEFT, 0);
  analogWrite(MOTOR_RIGHT, 0);
  digitalWrite(LED_PIN, LOW);

  shutdownAnimation();

  nrf_gpio_cfg_sense_input(BUTTON_PIN, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
  sd_power_system_off();
}

void connect_callback(uint16_t conn_handle) {
  connected = true;
  digitalWrite(LED_PIN, LOW);
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  connected = false;
  digitalWrite(MOTOR_LEFT, HAPTIC_OFF);
  digitalWrite(MOTOR_RIGHT, HAPTIC_OFF);
  analogWrite(MOTOR_LEFT, 0);
  analogWrite(MOTOR_RIGHT, 0);
}

void setup() {
  pinMode(MOTOR_LEFT, OUTPUT);
  pinMode(MOTOR_RIGHT, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  digitalWrite(MOTOR_LEFT, HAPTIC_OFF);
  digitalWrite(MOTOR_RIGHT, HAPTIC_OFF);
  digitalWrite(LED_PIN, LOW);

  startupVibration();

  Bluefruit.begin();
  Bluefruit.setName("HeatPett");
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  bleuart.begin();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.start(0);
}

unsigned long lastBatteryUpdate = 0;

void loop() {
  // Motoren steuern (Original ESP8266 Logik übernommen)
  while (bleuart.available() > 0) {
    char data = bleuart.read();
    unsigned int haptic_right_level = (data & 0x0F) << 4;
    unsigned int haptic_left_level  = data & 0xF0;

    haptic_right_level |= haptic_right_level >> 4;
    haptic_left_level  |= haptic_left_level  >> 4;

    analogWrite(MOTOR_LEFT,  HAPTIC_OFF ? haptic_left_level  : 0xFF - haptic_left_level);
    analogWrite(MOTOR_RIGHT, HAPTIC_OFF ? haptic_right_level : 0xFF - haptic_right_level);
  }

  // Akkustand senden
  if (connected && millis() - lastBatteryUpdate >= 1000) {
    lastBatteryUpdate = millis();
    uint8_t batLevel = (uint8_t)round(max(min(getBatteryLevel() * 100.0f, 100.0f), 0.0f));
    bleuart.write(batLevel);
  }

  // Button
  bool buttonPressed = (digitalRead(BUTTON_PIN) == LOW);

  if (buttonPressed) {
    digitalWrite(LED_PIN, HIGH);

    if (buttonWasReleased) {
      buttonPressStart = millis();
      buttonWasReleased = false;
      buttonHeld = false;
    }

    if (!buttonHeld && millis() - buttonPressStart >= 5000) {
      buttonHeld = true;
      goToSleep();
    }

  } else {
    buttonWasReleased = true;
    buttonHeld = false;

    if (connected) {
      digitalWrite(LED_PIN, LOW);
    }
  }

  // LED alle 3 Sekunden kurz blinken wenn nicht verbunden
  if (!connected && !buttonPressed) {
    unsigned long now = millis();
    if (now - lastBlink >= 3000) {
      lastBlink = now;
      digitalWrite(LED_PIN, HIGH);
      delay(80);
      digitalWrite(LED_PIN, LOW);
    }
  }
}