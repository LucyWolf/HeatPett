#include <Arduino.h>
#include <bluefruit.h>
#include <nrf_gpio.h>
#include <nrf_power.h>

// ═══════════════════════════════════════════
//  PINS
// ═══════════════════════════════════════════
#define MOTOR_LEFT   43   // D14 = P1.11
#define MOTOR_RIGHT  10   // D16 = P0.10
#define BUTTON_PIN    6   // D1  = P0.06
#define BATTERY_PIN   4   // P0.04 = AIN2

// ═══════════════════════════════════════════
//  EINSTELLUNGEN
// ═══════════════════════════════════════════
#define HAPTIC_ON          HIGH
#define HAPTIC_OFF         LOW
#define SLEEP_HOLD_MS      3000   // ms bis Sleep
#define BATTERY_SEND_MS    1000   // ms zwischen Akkustand senden

// ═══════════════════════════════════════════
//  AKKU
// ═══════════════════════════════════════════
#define ADCResolution             4095.0
#define ADCVoltageMax             3.6
#define BATTERY_SHIELD_R1         100.0
#define BATTERY_SHIELD_R2         220.0
#define BATTERY_SHIELD_RESISTANCE 180.0
#define ADCMultiplier ((BATTERY_SHIELD_R1 + BATTERY_SHIELD_R2 + BATTERY_SHIELD_RESISTANCE) / BATTERY_SHIELD_R1)

// ═══════════════════════════════════════════
//  GLOBALE VARIABLEN
// ═══════════════════════════════════════════
BLEUart bleuart;
bool connected         = false;
bool buttonWasReleased = true;
bool buttonHeld        = false;
unsigned long buttonPressStart  = 0;
unsigned long lastBatteryUpdate = 0;

// ═══════════════════════════════════════════
//  AKKU FUNKTION
// ═══════════════════════════════════════════
float getBatteryLevel() {
  float voltage = ((float)analogRead(BATTERY_PIN)) * ADCVoltageMax / ADCResolution * ADCMultiplier;
  float level = 0.0f;
  if      (voltage > 3.975f) level = (voltage - 2.920f) * 0.8f;
  else if (voltage > 3.678f) level = (voltage - 3.300f) * 1.25f;
  else if (voltage > 3.489f) level = (voltage - 3.400f) * 1.7f;
  else if (voltage > 3.360f) level = (voltage - 3.300f) * 0.8f;
  else                       level = (voltage - 3.200f) * 0.3f;
  level = (level - 0.05f) / 0.95f;
  return max(min(level, 1.0f), 0.0f);
}

// ═══════════════════════════════════════════
//  VIBRATION
// ═══════════════════════════════════════════
void startupVibration() {
  for (int i = 0; i < 2; i++) {
    digitalWrite(MOTOR_LEFT,  HAPTIC_ON);
    digitalWrite(MOTOR_RIGHT, HAPTIC_ON);
    delay(400);
    digitalWrite(MOTOR_LEFT,  HAPTIC_OFF);
    digitalWrite(MOTOR_RIGHT, HAPTIC_OFF);
    delay(100);
  }
  digitalWrite(MOTOR_LEFT,  HAPTIC_ON);
  digitalWrite(MOTOR_RIGHT, HAPTIC_ON);
  delay(600);
  digitalWrite(MOTOR_LEFT,  HAPTIC_OFF);
  digitalWrite(MOTOR_RIGHT, HAPTIC_OFF);
}

void stopMotors() {
  digitalWrite(MOTOR_LEFT,  HAPTIC_OFF);
  digitalWrite(MOTOR_RIGHT, HAPTIC_OFF);
  analogWrite(MOTOR_LEFT,  0);
  analogWrite(MOTOR_RIGHT, 0);
}

// ═══════════════════════════════════════════
//  SLEEP / SHUTDOWN
// ═══════════════════════════════════════════
void goToSleep() {
  stopMotors();
  nrf_gpio_cfg_sense_input(BUTTON_PIN, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
  sd_power_system_off();
}

// ═══════════════════════════════════════════
//  BLE CALLBACKS
// ═══════════════════════════════════════════
void connect_callback(uint16_t conn_handle) {
  connected = true;
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  connected = false;
  stopMotors();
}

// ═══════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════
void setup() {
  pinMode(MOTOR_LEFT,  OUTPUT);
  pinMode(MOTOR_RIGHT, OUTPUT);
  pinMode(BUTTON_PIN,  INPUT_PULLUP);

  stopMotors();
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

// ═══════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════
void loop() {

  // ── Motoren steuern ──────────────────────
  while (bleuart.available() > 0) {
    char data = bleuart.read();
    unsigned int haptic_right = (data & 0x0F) << 4;
    unsigned int haptic_left  =  data & 0xF0;
    haptic_right |= haptic_right >> 4;
    haptic_left  |= haptic_left  >> 4;
    analogWrite(MOTOR_LEFT,  HAPTIC_OFF ? haptic_left  : 0xFF - haptic_left);
    analogWrite(MOTOR_RIGHT, HAPTIC_OFF ? haptic_right : 0xFF - haptic_right);
  }

  // ── Akkustand senden ─────────────────────
  if (connected && millis() - lastBatteryUpdate >= BATTERY_SEND_MS) {
    lastBatteryUpdate = millis();
    uint8_t bat = (uint8_t)round(max(min(getBatteryLevel() * 100.0f, 100.0f), 0.0f));
    bleuart.write(bat);
  }

  // ── Button ───────────────────────────────
  bool buttonPressed = (digitalRead(BUTTON_PIN) == LOW);

  if (buttonPressed) {
    if (buttonWasReleased) {
      buttonPressStart  = millis();
      buttonWasReleased = false;
      buttonHeld        = false;
    }

    // Nach SLEEP_HOLD_MS → Sleep
    if (!buttonHeld && millis() - buttonPressStart >= SLEEP_HOLD_MS) {
      buttonHeld = true;
      goToSleep();
    }

  } else {
    buttonWasReleased = true;
    buttonHeld        = false;
  }
}