#include <Arduino.h>
#include <bluefruit.h>
#include <nrf_gpio.h>
#include <nrf_power.h>

// ═══════════════════════════════════════════
//  VERSION
// ═══════════════════════════════════════════
#define VERSION "1.0.1"

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
#define PAIRING_HOLD_MS      3000
#define SLEEP_HOLD_MS        5000
#define PAIRING_DURATION_MS  20000
#define PAIRING_BLINK_MS     150
#define BATTERY_SEND_MS      5000

int motorStrength = 255;

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
unsigned long buttonPressStart  = 0;
unsigned long lastBatteryUpdate = 0;
unsigned long pairingStart      = 0;
unsigned long lastPairingBlink  = 0;
bool pairingLedState            = false;

// ═══════════════════════════════════════════
//  VIBRATION
// ═══════════════════════════════════════════
void stopMotors() {
  analogWrite(MOTOR_LEFT,  0);
  analogWrite(MOTOR_RIGHT, 0);
  digitalWrite(MOTOR_LEFT,  HAPTIC_OFF);
  digitalWrite(MOTOR_RIGHT, HAPTIC_OFF);
}

void startupVibration() {
  Serial.println("[VIBRATION] Startup");
  analogWrite(MOTOR_LEFT,  motorStrength);
  analogWrite(MOTOR_RIGHT, motorStrength);
  delay(2000);
  stopMotors();
  Serial.println("[VIBRATION] Done");
}

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
//  SHUTDOWN ANIMATION
// ═══════════════════════════════════════════
void shutdownAnimation() {
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
  if (pairingMode) return;
  Serial.println("[PAIRING] Start");
  pairingMode  = true;
  pairingStart = millis();
  Bluefruit.Advertising.stop();
  delay(50);
  Bluefruit.Advertising.start(0);
}

void stopPairingMode() {
  if (!pairingMode) return;
  Serial.println("[PAIRING] Stop");
  pairingMode = false;
  digitalWrite(LED_PIN, LOW);
  Bluefruit.Advertising.stop();
  if (!connected) {
    delay(50);
    Bluefruit.Advertising.start(0);
  }
}

// ═══════════════════════════════════════════
//  SLEEP
// ═══════════════════════════════════════════
void goToSleep() {
  Serial.println("[POWER] Going to sleep...");
  delay(100);
  stopMotors();
  
  // Pairing-Mode beenden falls aktiv
  if (pairingMode) {
    stopPairingMode();
  }
  
  Bluefruit.Advertising.stop();
  shutdownAnimation();
  nrf_gpio_cfg_sense_input(BUTTON_PIN, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
  Serial.println("[POWER] SYSTEM OFF - Press button to wake");
  delay(100);
  sd_power_system_off();
  NRF_POWER->SYSTEMOFF = 1;
  while(1);
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
  Serial.print("[BLE] Disconnected! Reason: ");
  Serial.println(reason);
  connected = false;
  stopMotors();
  Bluefruit.Advertising.start(0);
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

  Serial.println("[SYS] Commands: 's=255' strength, 'v' version");

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
  Serial.println("[INFO] 3s hold → Pairing | 5s hold → Sleep");
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
      int val = constrain(cmd.substring(2).toInt(), 0, 255);
      motorStrength = val;
      Serial.print("[SYS] Motor strength: ");
      Serial.println(motorStrength);
    } else if (cmd == "v") {
      Serial.print("HeatPett v");
      Serial.println(VERSION);
    }
  }

  // ── Motoren steuern (nur wenn verbunden) ──
  if (connected) {
    while (bleuart.available() > 0) {
      char data = bleuart.read();
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

  // ── Akkustand senden ─────────────────────
  if (connected && millis() - lastBatteryUpdate >= BATTERY_SEND_MS) {
    lastBatteryUpdate = millis();
    uint8_t bat = (uint8_t)round(max(min(getBatteryLevel() * 100.0f, 100.0f), 0.0f));
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

  // ── BUTTON LOGIC (KOMPLETT NEU) ───────────
  bool buttonPressed = (digitalRead(BUTTON_PIN) == LOW);
  static unsigned long lastPressTime = 0;
  static bool pairingTriggered = false;
  static bool sleepTriggered = false;

  if (buttonPressed) {
    // LED AN bei Tastendruck
    if (!pairingMode) digitalWrite(LED_PIN, HIGH);
    
    if (buttonWasReleased) {
      buttonPressStart = millis();
      buttonWasReleased = false;
      pairingTriggered = false;
      sleepTriggered = false;
      Serial.println("[BUTTON] Pressed");
    }
    
    unsigned long heldTime = millis() - buttonPressStart;
    
    // PRIORITÄT 1: Sleep bei 5+ Sekunden (überschreibt alles)
    if (!sleepTriggered && heldTime >= SLEEP_HOLD_MS) {
      sleepTriggered = true;
      Serial.println("[BUTTON] 5s reached → Sleep (overrides pairing)");
      goToSleep();
    }
    
  } else {
    // Button losgelassen
    if (!buttonWasReleased) {
      unsigned long heldTime = millis() - buttonPressStart;
      
      // PRIORITÄT 2: Nur Pairing wenn NICHT im Sleep-Bereich (5s+)
      if (!sleepTriggered && heldTime >= PAIRING_HOLD_MS && heldTime < SLEEP_HOLD_MS) {
        Serial.print("[BUTTON] 3s reached → Toggle Pairing (held ");
        Serial.print(heldTime);
        Serial.println("ms)");
        
        if (pairingMode) {
          stopPairingMode();
        } else {
          startPairingMode();
        }
        pairingTriggered = true;
      }
      
      Serial.print("[BUTTON] Released after ");
      Serial.print(heldTime);
      Serial.println("ms");
    }
    
    // LED AUS (wenn nicht im Pairing)
    if (!pairingMode) digitalWrite(LED_PIN, LOW);
    
    buttonWasReleased = true;
  }
}