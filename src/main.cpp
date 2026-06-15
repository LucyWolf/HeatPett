#include <Arduino.h>
#include <bluefruit.h>
#include <nrf_gpio.h>
#include <nrf_power.h>

#define VERSION "1.0.2"

// ═══════════════════════════════════════════
//  PINS
// ═══════════════════════════════════════════
#define MOTOR_LEFT   43   // D14 = P1.11
#define MOTOR_RIGHT  10   // D16 = P0.10
#define LED_PIN      15   // P0.15
#define BUTTON_PIN    6   // D1  = P0.06
#define BATTERY_PIN   4   // P0.04 = AIN2 (battery voltage)

// ═══════════════════════════════════════════
//  SETTINGS
// ═══════════════════════════════════════════
#define PAIRING_HOLD_MS      3000   // hold duration to enter pairing mode
#define SLEEP_HOLD_MS        5000   // hold duration to enter sleep
#define PAIRING_DURATION_MS  20000  // pairing mode auto-timeout
#define PAIRING_BLINK_MS     150    // LED blink interval in pairing mode
#define KEEPALIVE_MS         1000   // BLE keepalive interval
#define IDLE_BLINK_ON_MS     80   // LED on duration when not connected
#define IDLE_BLINK_MS        2000   // LED off duration when not connected

int motorStrength = 255;  // motor PWM strength (0-255), adjustable via serial

// ═══════════════════════════════════════════
//  GLOBAL VARIABLES
// ═══════════════════════════════════════════
BLEUart bleuart;

volatile bool connected   = false;
volatile bool pairingMode = false;
bool buttonWasReleased = true;
bool sleepTriggered    = false;

unsigned long buttonPressStart = 0;
unsigned long lastKeepAlive    = 0;
unsigned long pairingStart     = 0;
unsigned long lastPairingBlink = 0;

bool pairingLedState = false;

// ═══════════════════════════════════════════
//  MOTOR
// ═══════════════════════════════════════════
void stopMotors() {
  analogWrite(MOTOR_LEFT,  0);
  analogWrite(MOTOR_RIGHT, 0);
}

void startupVibration() {
  // Single strong pulse on startup to indicate power on
  Serial.println("[VIBRATION] Startup START");
  analogWrite(MOTOR_LEFT,  motorStrength);
  analogWrite(MOTOR_RIGHT, motorStrength);
  delay(1000);
  stopMotors();
  Serial.println("[VIBRATION] Startup DONE");
}

// ═══════════════════════════════════════════
//  LED
// ═══════════════════════════════════════════
void shutdownAnimation() {
  // LED off → pause → on → slow fade out
  digitalWrite(LED_PIN, LOW);
  delay(400);
  digitalWrite(LED_PIN, HIGH);
  delay(150);
  for (int i = 255; i >= 0; i -= 3) {
    analogWrite(LED_PIN, i);
    delay(10);
  }
  analogWrite(LED_PIN, 0);
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
//  SLEEP
// ═══════════════════════════════════════════
void goToSleep() {
  Serial.println("[POWER] Going to sleep...");
  delay(100);

  stopMotors();
  pairingMode = false;
  Bluefruit.Advertising.stop();

  // Play shutdown animation before sleeping
  shutdownAnimation();

  // Wait for button release to avoid immediate wake-up
  while (digitalRead(BUTTON_PIN) == LOW) {
    delay(10);
  }
  delay(50); // debounce

  // Configure button pin as wake-up source
  nrf_gpio_cfg_sense_input(
    BUTTON_PIN,
    NRF_GPIO_PIN_PULLUP,
    NRF_GPIO_PIN_SENSE_LOW
  );

  Serial.println("[POWER] SYSTEM OFF - press button to wake");
  delay(100);

  sd_power_system_off();
  NRF_POWER->SYSTEMOFF = 1; // fallback
  while (1) {}
}

// ═══════════════════════════════════════════
//  BLE CALLBACKS
// ═══════════════════════════════════════════
void connect_callback(uint16_t conn_handle) {
  Serial.println("[BLE] Connected!");
  connected     = true;
  pairingMode   = false;
  lastKeepAlive = millis();
  digitalWrite(LED_PIN, LOW);
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  Serial.print("[BLE] Disconnected! Reason=");
  Serial.println(reason);
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

  // Auto-restart advertising after disconnect
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.start(0);

  Serial.println("[BLE] Advertising started");
  Serial.println("[SYS] Ready!");
  Serial.println("[INFO] Hold 3s → Pairing mode | Hold 5s → Sleep");
}

// ═══════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════
void loop() {

  // ── Serial Commands ─────────────────────
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("s=")) {
      int val = constrain(cmd.substring(2).toInt(), 0, 255);
      motorStrength = val;
      Serial.print("[SYS] Motor strength set to: ");
      Serial.println(motorStrength);
    }
  }

  // ── Motor control (only when BLE connected) ──
  if (connected) {
    while (bleuart.available() > 0) {
      uint8_t data = bleuart.read();

      // Decode PatStrap nibble encoding
      unsigned int haptic_right = (data & 0x0F) << 4;
      unsigned int haptic_left  =  data & 0xF0;
      haptic_right |= haptic_right >> 4;
      haptic_left  |= haptic_left  >> 4;

      // Scale by motor strength
      haptic_left  = (haptic_left  * motorStrength) / 255;
      haptic_right = (haptic_right * motorStrength) / 255;

      analogWrite(MOTOR_LEFT,  haptic_left);
      analogWrite(MOTOR_RIGHT, haptic_right);
    }
  } else {
    stopMotors();
  }

  // ── BLE keepalive ───────────────────────
  if (connected && millis() - lastKeepAlive >= KEEPALIVE_MS) {
    lastKeepAlive = millis();
    bleuart.write((uint8_t)100);
    Serial.println("[BLE] Keepalive sent");
  }

  // ── Button logic ────────────────────────
  bool buttonPressed = (digitalRead(BUTTON_PIN) == LOW);

  if (buttonPressed) {
    if (buttonWasReleased) {
      buttonPressStart  = millis();
      buttonWasReleased = false;
      sleepTriggered    = false;
      Serial.println("[BUTTON] Pressed");
    }

    unsigned long heldTime = millis() - buttonPressStart;

    // LED on while button held (not in pairing mode)
    if (!pairingMode) {
      digitalWrite(LED_PIN, HIGH);
    }

    // 5s → sleep (highest priority)
    if (!sleepTriggered && heldTime >= SLEEP_HOLD_MS) {
      sleepTriggered = true;
      Serial.println("[BUTTON] 5s held → Sleep");
      goToSleep();
    }

  } else {
    if (!buttonWasReleased) {
      unsigned long heldTime = millis() - buttonPressStart;

      // 3s release → toggle pairing mode
      if (!sleepTriggered && heldTime >= PAIRING_HOLD_MS) {
        Serial.println("[BUTTON] Released after 3s → Toggle Pairing");
        if (pairingMode) stopPairingMode();
        else startPairingMode();
      }

      Serial.println("[BUTTON] Released");
    }

    // LED off on release (not in pairing mode)
    if (!pairingMode) {
      digitalWrite(LED_PIN, LOW);
    }

    buttonWasReleased = true;
    sleepTriggered    = false;
  }

  // ── Pairing mode LED blink ───────────────
  if (pairingMode && !buttonPressed) {
    unsigned long now = millis();
    if (now - pairingStart >= PAIRING_DURATION_MS) {
      stopPairingMode(); // auto-timeout after 20s
    } else if (now - lastPairingBlink >= PAIRING_BLINK_MS) {
      lastPairingBlink = now;
      pairingLedState  = !pairingLedState;
      digitalWrite(LED_PIN, pairingLedState);
    }
  }

  // ── Idle LED blink (not connected, not pairing, button not pressed) ──
  if (!connected && !pairingMode && !buttonPressed) {
    unsigned long now    = millis();
    unsigned long period = IDLE_BLINK_ON_MS + IDLE_BLINK_MS; // total cycle
    unsigned long phase  = now % period;

    if (phase < IDLE_BLINK_ON_MS) {
      digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(LED_PIN, LOW);
    }
  }
}