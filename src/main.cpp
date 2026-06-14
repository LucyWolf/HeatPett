#include <Arduino.h>
#include <bluefruit.h>
#include <nrf_gpio.h>
#include <nrf_power.h>

// ═══════════════════════════════════════════
//  VERSION
// ═══════════════════════════════════════════
#define VERSION "1.0.0"

// ═══════════════════════════════════════════
//  PINS
// ═══════════════════════════════════════════
#define MOTOR_LEFT   43   // D14 = P1.11
#define MOTOR_RIGHT  10   // D16 = P0.10
#define LED_PIN      15   // P0.15
#define BUTTON_PIN    6   // D1  = P0.06
#define BATTERY_PIN   4   // P0.04 = AIN2

// ═══════════════════════════════════════════
//  EINSTELLUNGEN
// ═══════════════════════════════════════════
#define HAPTIC_ON            HIGH
#define HAPTIC_OFF           LOW
#define PAIRING_HOLD_MS      3000   // ms bis Pairing Mode
#define SLEEP_HOLD_MS        5000   // ms bis Sleep
#define PAIRING_DURATION_MS  20000  // ms Pairing Mode Dauer
#define PAIRING_BLINK_MS     150    // ms Blink Intervall im Pairing Mode
#define BATTERY_SEND_MS      1000   // ms zwischen Akkustand senden

int motorStrength = 255;  // 0-255, per Serial anpassbar

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
bool pairingMode       = false;
bool buttonWasReleased = true;
bool buttonHeld        = false;
unsigned long buttonPressStart  = 0;
unsigned long lastBatteryUpdate = 0;
unsigned long pairingStart      = 0;
unsigned long lastPairingBlink  = 0;
bool pairingLedState            = false;

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
  Serial.println("[VIBRATION] Startup START");
  analogWrite(MOTOR_LEFT,  motorStrength);
  analogWrite(MOTOR_RIGHT, motorStrength);
  delay(2000);
  analogWrite(MOTOR_LEFT,  0);
  analogWrite(MOTOR_RIGHT, 0);
  Serial.println("[VIBRATION] Startup DONE");
}

void stopMotors() {
  digitalWrite(MOTOR_LEFT,  HAPTIC_OFF);
  digitalWrite(MOTOR_RIGHT, HAPTIC_OFF);
  analogWrite(MOTOR_LEFT,  0);
  analogWrite(MOTOR_RIGHT, 0);
}

// ═══════════════════════════════════════════
//  PAIRING MODE
// ═══════════════════════════════════════════
void startPairingMode() {
  Serial.println("[PAIRING] Mode START");
  pairingMode  = true;
  pairingStart = millis();
  Bluefruit.Advertising.start(0);
}

void stopPairingMode() {
  Serial.println("[PAIRING] Mode STOP");
  pairingMode = false;
  digitalWrite(LED_PIN, LOW);
  if (!connected) {
    Bluefruit.Advertising.stop();
  }
}

// ═══════════════════════════════════════════
//  SLEEP / SHUTDOWN
// ═══════════════════════════════════════════
void goToSleep() {
  Serial.println("[POWER] Going to sleep...");
  delay(100);
  stopMotors();
  stopPairingMode();
  digitalWrite(LED_PIN, LOW);
  nrf_gpio_cfg_sense_input(BUTTON_PIN, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
  sd_power_system_off();
}

// ═══════════════════════════════════════════
//  BLE CALLBACKS
// ═══════════════════════════════════════════
void connect_callback(uint16_t conn_handle) {
  Serial.println("[BLE] Connected!");
  connected = true;
  if (pairingMode) stopPairingMode();
  digitalWrite(LED_PIN, LOW);
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  Serial.println("[BLE] Disconnected!");
  connected = false;
  stopMotors();
}

// ═══════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("══════════════════════════════");
  Serial.print("  HeatPett v");
  Serial.println(VERSION);
  Serial.println("══════════════════════════════");
  Serial.println("[SYS] Starting...");

  pinMode(MOTOR_LEFT,  OUTPUT);
  pinMode(MOTOR_RIGHT, OUTPUT);
  pinMode(LED_PIN,     OUTPUT);
  pinMode(BUTTON_PIN,  INPUT_PULLUP);

  stopMotors();
  digitalWrite(LED_PIN, LOW);

  Serial.print("[SYS] Motor strength: ");
  Serial.println(motorStrength);
  Serial.println("[SYS] Commands: 's=255' to set strength (0-255)");

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
  Bluefruit.Advertising.restartOnDisconnect(false);
  Bluefruit.Advertising.start(0);

  Serial.println("[BLE] Advertising started");
  Serial.println("[SYS] Ready!");
  Serial.println("══════════════════════════════");
}

// ═══════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════
void loop() {

  // ── Serial Commands ───────────────────────
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("s=")) {
      int val = cmd.substring(2).toInt();
      val = constrain(val, 0, 255);
      motorStrength = val;
      Serial.print("[SYS] Motor strength set to: ");
      Serial.println(motorStrength);
    }
  }

  // ── Motoren steuern ──────────────────────
  while (bleuart.available() > 0) {
    char data = bleuart.read();
    unsigned int haptic_right = (data & 0x0F) << 4;
    unsigned int haptic_left  =  data & 0xF0;
    haptic_right |= haptic_right >> 4;
    haptic_left  |= haptic_left  >> 4;

    haptic_left  = (haptic_left  * motorStrength) / 255;
    haptic_right = (haptic_right * motorStrength) / 255;

    Serial.print("[MOTOR] L: ");
    Serial.print(haptic_left);
    Serial.print(" R: ");
    Serial.println(haptic_right);

    analogWrite(MOTOR_LEFT,  haptic_left);
    analogWrite(MOTOR_RIGHT, haptic_right);
  }

  // ── Akkustand senden ─────────────────────
  if (connected && millis() - lastBatteryUpdate >= BATTERY_SEND_MS) {
    lastBatteryUpdate = millis();
    uint8_t bat = (uint8_t)round(max(min(getBatteryLevel() * 100.0f, 100.0f), 0.0f));
    Serial.print("[BAT] ");
    Serial.print(bat);
    Serial.println("%");
    bleuart.write(bat);
  }

  // ── Pairing Mode LED Blinken ─────────────
  if (pairingMode) {
    unsigned long now = millis();
    if (now - pairingStart >= PAIRING_DURATION_MS) {
      stopPairingMode();
    } else {
      if (now - lastPairingBlink >= PAIRING_BLINK_MS) {
        lastPairingBlink = now;
        pairingLedState  = !pairingLedState;
        digitalWrite(LED_PIN, pairingLedState);
      }
    }
  }

  // ── Button ───────────────────────────────
  bool buttonPressed = (digitalRead(BUTTON_PIN) == LOW);

  if (buttonPressed) {
    if (buttonWasReleased) {
      buttonPressStart  = millis();
      buttonWasReleased = false;
      buttonHeld        = false;
      Serial.println("[BUTTON] Pressed");
    }

    unsigned long heldTime = millis() - buttonPressStart;

    // 5s → Sleep
    if (!buttonHeld && heldTime >= SLEEP_HOLD_MS) {
      Serial.println("[BUTTON] 5s held → Sleep");
      buttonHeld = true;
      goToSleep();
    }
    // 3s → Pairing Mode
    else if (!buttonHeld && heldTime >= PAIRING_HOLD_MS) {
      Serial.println("[BUTTON] 3s held → Pairing Mode");
      buttonHeld = true;
      if (pairingMode) {
        stopPairingMode();
      } else {
        startPairingMode();
      }
    }

  } else {
    if (!buttonWasReleased) {
      Serial.println("[BUTTON] Released");
    }
    buttonWasReleased = true;
    buttonHeld        = false;
  }
}