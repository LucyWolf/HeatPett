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
#define BATTERY_PIN    4  // P0.04 = AIN2 (SuperMini built-in battery divider)

// ═══════════════════════════════════════════
//  SETTINGS
// ═══════════════════════════════════════════
#define PAIRING_HOLD_MS      3000   // hold duration to enter pairing mode
#define SLEEP_HOLD_MS        5000   // hold duration to enter sleep
#define PAIRING_DURATION_MS  20000  // pairing mode auto-timeout
#define PAIRING_BLINK_MS     150    // LED blink interval in pairing mode
#define IDLE_BLINK_ON_MS     80     // LED on duration when not connected
#define IDLE_BLINK_MS        2000   // LED off duration when not connected
#define BATTERY_INTERVAL_MS  15000  // how often to send battery % to dongle

// UART command bytes (motor data uses 0x00–0xEF)
#define CMD_PAIR    0xFE
#define CMD_UNPAIR  0xFD

int motorStrength = 255;  // motor PWM strength (0-255), adjustable via serial

// ═══════════════════════════════════════════
//  HELPERS
// ═══════════════════════════════════════════
String formatUptime() {
  unsigned long s = millis() / 1000;
  unsigned long m = s / 60;
  unsigned long h = m / 60;
  char buf[16];
  sprintf(buf, "%02lu:%02lu:%02lu", h, m % 60, s % 60);
  return String(buf);
}

// ═══════════════════════════════════════════
//  GLOBAL VARIABLES
// ═══════════════════════════════════════════
BLEUart    bleuart;
BLEService ecosystemSvc("B5A47D3E-8C21-4F68-92A0-1E3D5C7B9F04");

volatile bool connected   = false;
volatile bool pairingMode = false;
bool buttonWasReleased = true;
bool sleepTriggered    = false;

unsigned long buttonPressStart = 0;
unsigned long lastBatSend      = 0;
unsigned long pairingStart     = 0;
unsigned long lastPairingBlink = 0;

bool pairingLedState = false;

// ═══════════════════════════════════════════
//  BATTERY
// ═══════════════════════════════════════════
uint8_t batteryPercent() {
  analogReference(AR_INTERNAL_3_0);
  analogReadResolution(12);
  int raw = analogRead(BATTERY_PIN);
  // SuperMini built-in divider: 100kΩ + 100kΩ → Vbat = Vpin * 2
  float vpin = raw * (3.0f / 4096.0f);
  float vbat = vpin * 2.0f;
  int pct = (int)((vbat - 3.0f) / 1.2f * 100.0f);  // 3.0V=0%, 4.2V=100%
  return (uint8_t)constrain(pct, 0, 100);
}

// ═══════════════════════════════════════════
//  MOTOR
// ═══════════════════════════════════════════
void stopMotors() {
  analogWrite(MOTOR_LEFT,  0);
  analogWrite(MOTOR_RIGHT, 0);
}

void startupVibration() {
  Serial.println("[VIBRATION] Startup START");
  analogWrite(MOTOR_LEFT,  motorStrength);
  analogWrite(MOTOR_RIGHT, motorStrength);
  delay(300);
  stopMotors();
  delay(150);
  analogWrite(MOTOR_LEFT,  motorStrength);
  analogWrite(MOTOR_RIGHT, motorStrength);
  delay(300);
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
//  ADVERTISING HELPERS
// ═══════════════════════════════════════════
void setNormalAdvertising() {
  Bluefruit.Advertising.stop();
  Bluefruit.Advertising.clearData();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addName();
}

void setPairingAdvertising() {
  Bluefruit.Advertising.stop();
  Bluefruit.Advertising.clearData();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addService(ecosystemSvc);
  Bluefruit.Advertising.addName();
}

// ═══════════════════════════════════════════
//  PAIRING MODE
// ═══════════════════════════════════════════
void startPairingMode() {
  Serial.println("[PAIRING] Mode START (20s)");
  pairingMode  = true;
  pairingStart = millis();
  setPairingAdvertising();
  Bluefruit.Advertising.start(0);
}

void stopPairingMode() {
  Serial.println("[PAIRING] Mode STOP");
  pairingMode = false;
  digitalWrite(LED_PIN, LOW);
  setNormalAdvertising();
  if (!connected) Bluefruit.Advertising.start(0);
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
  connected   = true;
  pairingMode = false;
  lastBatSend = millis() - BATTERY_INTERVAL_MS; // send battery on next loop
  digitalWrite(LED_PIN, LOW);
  setNormalAdvertising();
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

  ecosystemSvc.begin();
  bleuart.begin();

  setNormalAdvertising();  // normal mode: no ecosystem UUID, dongle connects by address
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

    } else if (cmd == "info") {
      Serial.println("info");
      Serial.println("HeatPett v" VERSION);
      Serial.println("Board: nRF52840");
      Serial.print  ("Connected: ");    Serial.println(connected   ? "YES" : "NO");
      Serial.print  ("Pairing mode: "); Serial.println(pairingMode ? "YES" : "NO");
      Serial.print  ("Motor strength: "); Serial.println(motorStrength);
      Serial.print  ("Battery: "); Serial.print(batteryPercent()); Serial.println("%");
      Serial.print  ("Uptime: "); Serial.println(formatUptime());

    } else if (cmd == "pairing" || cmd == "pair") {
      Serial.println("pairing");
      if (pairingMode) stopPairingMode();
      else startPairingMode();

    } else if (cmd == "reboot") {
      Serial.println("reboot");
      Serial.println("Rebooting...");
      delay(200);
      NVIC_SystemReset();

    } else if (cmd == "dfu") {
      Serial.println("dfu");
      Serial.println("Entering UF2 bootloader...");
      delay(200);
      NRF_POWER->GPREGRET = 0x57;
      NVIC_SystemReset();

    } else if (cmd == "uptime") {
      Serial.println("uptime");
      Serial.print("Uptime: "); Serial.println(formatUptime());

    } else if (cmd == "battery") {
      analogReference(AR_INTERNAL_3_0);
      analogReadResolution(12);
      int raw = analogRead(BATTERY_PIN);
      float vpin = raw * (3.0f / 4096.0f);
      float vbat = vpin * 2.0f;
      Serial.println("battery");
      Serial.print("Raw ADC: "); Serial.println(raw);
      Serial.print("Vpin:    "); Serial.print(vpin, 3); Serial.println(" V");
      Serial.print("Vbat:    "); Serial.print(vbat, 3); Serial.println(" V");
      Serial.print("Battery: "); Serial.print(batteryPercent()); Serial.println("%");

    } else if (cmd == "scan") {
      // Scan all nRF52840 AIN pins to find battery
      const int ainPins[] = {2, 3, 4, 5, 28, 29, 30, 31};
      const char* ainNames[] = {"AIN0/P0.02","AIN1/P0.03","AIN2/P0.04","AIN3/P0.05",
                                "AIN4/P0.28","AIN5/P0.29","AIN6/P0.30","AIN7/P0.31"};
      Serial.println("scan");
      analogReference(AR_INTERNAL_3_0);
      analogReadResolution(12);
      for (int i = 0; i < 8; i++) {
        int raw = analogRead(ainPins[i]);
        float v = raw * (3.0f / 4096.0f);
        Serial.print(ainNames[i]); Serial.print(" raw="); Serial.print(raw);
        Serial.print(" v="); Serial.print(v, 3); Serial.println("V");
      }

    } else if (cmd == "meow") {
      Serial.println("meow");
      Serial.println("(^=◕ᴥ◕=^)");
      Serial.println("HeatPett says: purrrr...");
    }
  }

  // ── Motor control (only when BLE connected) ──
  if (connected) {
    while (bleuart.available() > 0) {
      uint8_t data = bleuart.read();

      // Command bytes
      if (data == CMD_PAIR) {
        Serial.println("[BLE] Remote: start pairing");
        startPairingMode();
        continue;
      }
      if (data == CMD_UNPAIR) {
        Serial.println("[BLE] Remote: stop pairing");
        stopPairingMode();
        continue;
      }

      // Decode PatStrap nibble encoding (0x00–0xEF)
      unsigned int haptic_right = (data & 0x0F) << 4;
      unsigned int haptic_left  =  data & 0xF0;
      haptic_right |= haptic_right >> 4;
      haptic_left  |= haptic_left  >> 4;

      haptic_left  = (haptic_left  * motorStrength) / 255;
      haptic_right = (haptic_right * motorStrength) / 255;

      analogWrite(MOTOR_LEFT,  haptic_left);
      analogWrite(MOTOR_RIGHT, haptic_right);
    }
  } else {
    stopMotors();
  }

  // ── Battery send ────────────────────────
  if (connected && millis() - lastBatSend >= BATTERY_INTERVAL_MS) {
    lastBatSend = millis();
    uint8_t pct = batteryPercent();
    bleuart.write(pct);
    Serial.print("[BAT] Sent: "); Serial.print(pct); Serial.println("%");
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