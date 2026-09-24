/*
 * ============================================================
 *  COMP50069 – Hardware, Microcontrollers and Sensors
 *  Scenario 1: The Secure Industrial Loading Gate
 *  ESP32 Firmware – Wokwi Simulation
 * ============================================================
 *  @file     main.ino
 *  @brief    IoT-based embedded system for automated industrial
 *            loading gate with autonomous, manual, and safety modes.
 *            Implements Option D Hybrid Recovery routine.
 *  @author   [Your Name]
 *  @date     [Date]
 * ============================================================
 */

// ---------------- LIBRARIES ----------------
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

// ---------------- PIN DEFINITIONS ----------------
#define PIN_PIR              4
#define PIN_ULTRASONIC_TRIG  5
#define PIN_ULTRASONIC_ECHO  18
#define PIN_POTENTIOMETER    34

#define PIN_SERVO            13
#define PIN_BUZZER           26
#define PIN_LED_GREEN        32
#define PIN_LED_RED          33

#define PIN_BTN_RESET        27
#define PIN_BTN_MANUAL       14

#define OLED_WIDTH           128
#define OLED_HEIGHT          64
#define OLED_ADDR            0x3C
#define PIN_OLED_SDA         21
#define PIN_OLED_SCL         22

// ---------------- SYSTEM CONSTANTS ----------------
#define TIMED_IDLE_MS             3000
#define MAX_SAFETY_EVENTS         3
#define LED_BLINK_INTERVAL_MS     500
#define OLED_UPDATE_INTERVAL_MS   200
#define ADC_SAMPLES               10
#define HOLD_DELAY_MIN_MS         1000
#define HOLD_DELAY_MAX_MS         10000
#define ULTRASONIC_TIMEOUT_US     30000
#define DISTANCE_MAX_CM           400
#define DISTANCE_MIN_VALID_CM     2

#define SERVO_CLOSED_ANGLE        0
#define SERVO_OPEN_ANGLE          90

// ---------------- STATE MACHINE ----------------
enum SystemState {
  STATE_AUTONOMOUS,
  STATE_MANUAL,
  STATE_SAFETY_HALT,
  STATE_TIMED_IDLE,
  STATE_PERMANENT_FREEZE
};

SystemState currentState = STATE_AUTONOMOUS;

// ---------------- GLOBAL OBJECTS ----------------
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
Servo gateServo;

// ---------------- GLOBAL VARIABLES ----------------
unsigned long lastStateChange   = 0;
unsigned long lastLedBlink      = 0;
unsigned long lastSerialPrint   = 0;
unsigned long lastOledUpdate    = 0;
unsigned long lastBuzzerToggle  = 0;
unsigned long safetyEventCount  = 0;
unsigned long lastMotionTime    = 0;
unsigned long safetyClearTime   = 0;
bool          gateOpen          = false;
bool          ledGreenState     = false;
bool          buzzerState       = false;
int           potRawValue       = 0;
int           potSmoothed       = 0;
int           holdDelayMs       = 3000;
bool          motionDetected    = false;
float         distanceCm        = 999.0;
bool          obstructionActive = false;
int           obstructionThresholdCm = 20;
String        serialBuffer      = "";

// ---------------- FUNCTION PROTOTYPES ----------------
void changeState(SystemState newState);
const char* stateName(SystemState s);
void updateOLED();
int  readPotentiometer();
void updateHoldDelay(int smoothedValue);
bool readPIR();
float readUltrasonic();
void checkSafety(unsigned long now);
void openGate();
void closeGate();
void handleAutonomousMode(unsigned long now);
void handleSafetyHalt(unsigned long now);
void handleTimedIdle(unsigned long now);
void handlePermanentFreeze(unsigned long now);
bool readButton(int pin);
void processSerialCommand(String cmd);
void printHelp();
void printStatus();

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("========================================");
  Serial.println(" Secure Industrial Loading Gate");
  Serial.println(" COMP50069 – Scenario 1");
  Serial.println(" Option D – Hybrid Recovery Routine");
  Serial.println("========================================");
  Serial.println("System booting...");

  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("[ERROR] OLED init failed!");
    while (true) { delay(10); }
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Booting...");
  display.display();
  Serial.println("[OK] OLED initialized.");

  gateServo.attach(PIN_SERVO);
  gateServo.write(SERVO_CLOSED_ANGLE);
  Serial.println("[OK] Servo initialized (gate CLOSED).");

  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_POTENTIOMETER, INPUT);
  pinMode(PIN_PIR, INPUT);
  pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);
  pinMode(PIN_ULTRASONIC_ECHO, INPUT);
  pinMode(PIN_BTN_RESET, INPUT_PULLUP);
  pinMode(PIN_BTN_MANUAL, INPUT_PULLUP);

  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_RED, LOW);
  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);

  changeState(STATE_AUTONOMOUS);
  Serial.println("Boot complete.");
  Serial.println("========================================");
  Serial.println("Type 'help' for commands.");
  Serial.println("========================================");
}

// ---------------- MAIN LOOP ----------------
void loop() {
  unsigned long now = millis();

  // --- Serial command input ---
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        processSerialCommand(serialBuffer);
        serialBuffer = "";
      }
    } else {
      serialBuffer += c;
      if (serialBuffer.length() > 64) serialBuffer = ""; // overflow guard
    }
  }

  // --- Read sensors ---
  potRawValue = readPotentiometer();
  potSmoothed = potRawValue;
  updateHoldDelay(potSmoothed);

  if (readPIR()) {
    motionDetected = true;
    lastMotionTime = now;
  } else if (now - lastMotionTime > 2000) {
    motionDetected = false;
  }

  distanceCm = readUltrasonic();

  if (readButton(PIN_BTN_MANUAL)) {
    if (currentState == STATE_AUTONOMOUS) {
      changeState(STATE_MANUAL);
      openGate();
      Serial.println("[MANUAL] Manual Override ENABLED – gate locked open.");
    } else if (currentState == STATE_MANUAL) {
      changeState(STATE_AUTONOMOUS);
      Serial.println("[MANUAL] Manual Override DISABLED – returning to autonomous.");
    }
  }

  if (currentState == STATE_AUTONOMOUS ||
      currentState == STATE_SAFETY_HALT ||
      currentState == STATE_TIMED_IDLE) {
    checkSafety(now);
  }

  switch (currentState) {
    case STATE_AUTONOMOUS:      handleAutonomousMode(now); break;
    case STATE_SAFETY_HALT:     handleSafetyHalt(now);     break;
    case STATE_TIMED_IDLE:      handleTimedIdle(now);      break;
    case STATE_PERMANENT_FREEZE: handlePermanentFreeze(now); break;
    case STATE_MANUAL: break;
  }

  if (now - lastLedBlink >= LED_BLINK_INTERVAL_MS) {
    lastLedBlink = now;
    ledGreenState = !ledGreenState;
    if (currentState == STATE_AUTONOMOUS) {
      digitalWrite(PIN_LED_GREEN, ledGreenState ? HIGH : LOW);
      digitalWrite(PIN_LED_RED, LOW);
    } else if (currentState == STATE_MANUAL) {
      digitalWrite(PIN_LED_GREEN, ledGreenState ? HIGH : LOW);
      digitalWrite(PIN_LED_RED, ledGreenState ? LOW : HIGH);
    } else if (currentState == STATE_SAFETY_HALT ||
               currentState == STATE_PERMANENT_FREEZE) {
      digitalWrite(PIN_LED_RED, ledGreenState ? HIGH : LOW);
      digitalWrite(PIN_LED_GREEN, LOW);
    } else if (currentState == STATE_TIMED_IDLE) {
      digitalWrite(PIN_LED_GREEN, ledGreenState ? HIGH : LOW);
      digitalWrite(PIN_LED_RED, !ledGreenState ? HIGH : LOW);
    }
  }

  if (currentState == STATE_SAFETY_HALT) {
    if (now - lastBuzzerToggle >= 200) {
      lastBuzzerToggle = now;
      buzzerState = !buzzerState;
      digitalWrite(PIN_BUZZER, buzzerState ? HIGH : LOW);
    }
  } else if (currentState == STATE_PERMANENT_FREEZE) {
    if (now - lastBuzzerToggle >= 800) {
      lastBuzzerToggle = now;
      buzzerState = !buzzerState;
      digitalWrite(PIN_BUZZER, buzzerState ? HIGH : LOW);
    }
  } else {
    digitalWrite(PIN_BUZZER, LOW);
  }

  if (now - lastSerialPrint >= 2000) {
    lastSerialPrint = now;
    Serial.print("[STATUS] State: ");
    Serial.print(stateName(currentState));
    Serial.print(" | PIR: ");
    Serial.print(motionDetected ? "MOTION" : "CLEAR ");
    Serial.print(" | Dist: ");
    Serial.print(distanceCm, 1);
    Serial.print("cm | Events: ");
    Serial.print(safetyEventCount);
    Serial.print(" | Gate: ");
    Serial.println(gateOpen ? "OPEN" : "CLOSED");
  }

  if (now - lastOledUpdate >= OLED_UPDATE_INTERVAL_MS) {
    lastOledUpdate = now;
    updateOLED();
  }
}

// ---------------- SERIAL COMMAND HANDLER ----------------

/**
 * @brief  Process a serial command string.
 * @param  cmd The full command string.
 */
void processSerialCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();

  if (cmd == "help") {
    printHelp();
  } else if (cmd == "status") {
    printStatus();
  } else if (cmd == "events") {
    Serial.print("[INFO] Safety events: ");
    Serial.println(safetyEventCount);
  } else if (cmd == "reset") {
    safetyEventCount = 0;
    Serial.println("[INFO] Safety counter reset.");
  } else if (cmd == "mode") {
    Serial.print("[INFO] Current mode: ");
    Serial.println(stateName(currentState));
  } else if (cmd.startsWith("threshold ")) {
    int val = cmd.substring(10).toInt();
    if (val >= 5 && val <= 200) {
      obstructionThresholdCm = val;
      Serial.print("[INFO] Threshold set to ");
      Serial.print(val);
      Serial.println(" cm");
    } else {
      Serial.println("[ERROR] Threshold must be 5–200 cm");
    }
  } else {
    Serial.print("[ERROR] Unknown command: ");
    Serial.println(cmd);
    Serial.println("Type 'help' for available commands.");
  }
}

void printHelp() {
  Serial.println("========================================");
  Serial.println(" Available Commands:");
  Serial.println("  help              – Show this list");
  Serial.println("  status            – Show system status");
  Serial.println("  mode              – Show current mode");
  Serial.println("  events            – Show safety event count");
  Serial.println("  reset             – Reset safety counter");
  Serial.println("  threshold <cm>    – Set obstruction threshold");
  Serial.println("========================================");
}

void printStatus() {
  Serial.println("========================================");
  Serial.print(" State:     "); Serial.println(stateName(currentState));
  Serial.print(" Gate:      "); Serial.println(gateOpen ? "OPEN" : "CLOSED");
  Serial.print(" Distance:  "); Serial.print(distanceCm, 1); Serial.println(" cm");
  Serial.print(" Threshold: "); Serial.print(obstructionThresholdCm); Serial.println(" cm");
  Serial.print(" PIR:       "); Serial.println(motionDetected ? "MOTION" : "CLEAR");
  Serial.print(" Events:    "); Serial.println(safetyEventCount);
  Serial.print(" HoldDelay: "); Serial.print(holdDelayMs); Serial.println(" ms");
  Serial.print(" Pot value: "); Serial.println(potSmoothed);
  Serial.println("========================================");
}

// ---------------- HELPER FUNCTIONS ----------------

void changeState(SystemState newState) {
  currentState = newState;
  lastStateChange = millis();
  Serial.print("[STATE CHANGE] -> ");
  Serial.println(stateName(newState));
}

const char* stateName(SystemState s) {
  switch (s) {
    case STATE_AUTONOMOUS:       return "AUTONOMOUS";
    case STATE_MANUAL:           return "MANUAL";
    case STATE_SAFETY_HALT:      return "SAFETY_HALT";
    case STATE_TIMED_IDLE:       return "TIMED_IDLE";
    case STATE_PERMANENT_FREEZE: return "PERM_FREEZE";
    default:                     return "UNKNOWN";
  }
}

int readPotentiometer() {
  long sum = 0;
  for (int i = 0; i < ADC_SAMPLES; i++) {
    sum += analogRead(PIN_POTENTIOMETER);
    delayMicroseconds(100);
  }
  return (int)(sum / ADC_SAMPLES);
}

void updateHoldDelay(int smoothedValue) {
  holdDelayMs = map(smoothedValue, 0, 4095, HOLD_DELAY_MIN_MS, HOLD_DELAY_MAX_MS);
}

bool readPIR() { return digitalRead(PIN_PIR) == HIGH; }

float readUltrasonic() {
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);

  long duration = pulseIn(PIN_ULTRASONIC_ECHO, HIGH, ULTRASONIC_TIMEOUT_US);
  if (duration == 0) return 999.0;

  float cm = duration * 0.0343 / 2.0;
  if (cm < DISTANCE_MIN_VALID_CM || cm > DISTANCE_MAX_CM) return 999.0;
  return cm;
}

void checkSafety(unsigned long now) {
  bool obstruction = (distanceCm < obstructionThresholdCm);
  obstructionActive = obstruction;

  if (obstruction && currentState == STATE_AUTONOMOUS) {
    safetyEventCount++;
    Serial.print("[SAFETY] Obstruction! Event #");
    Serial.println(safetyEventCount);

    if (safetyEventCount >= MAX_SAFETY_EVENTS) {
      changeState(STATE_PERMANENT_FREEZE);
    } else {
      changeState(STATE_SAFETY_HALT);
    }
  }
}

void openGate()  { gateServo.write(SERVO_OPEN_ANGLE);  gateOpen = true;  Serial.println("[GATE] Opening..."); }
void closeGate() { gateServo.write(SERVO_CLOSED_ANGLE); gateOpen = false; Serial.println("[GATE] Closing..."); }

void handleAutonomousMode(unsigned long now) {
  if (motionDetected && !gateOpen) { openGate(); lastMotionTime = now; }
  if (gateOpen && !motionDetected && (now - lastMotionTime > holdDelayMs)) closeGate();
}

void handleSafetyHalt(unsigned long now) {
  if (!obstructionActive) {
    safetyClearTime = now;
    changeState(STATE_TIMED_IDLE);
    Serial.println("[RECOVERY] Obstruction cleared. Entering timed idle...");
  }
}

void handleTimedIdle(unsigned long now) {
  if (now - safetyClearTime >= TIMED_IDLE_MS) {
    if (!obstructionActive) {
      openGate();
      Serial.println("[RECOVERY] Path clear. Gate retreated to OPEN.");
      safetyEventCount = 0;
      changeState(STATE_AUTONOMOUS);
    } else {
      changeState(STATE_SAFETY_HALT);
    }
  }
}

void handlePermanentFreeze(unsigned long now) {
  if (readButton(PIN_BTN_RESET)) {
    Serial.println("[RESET] Error Reset pressed. System recovering...");
    safetyEventCount = 0;
    obstructionActive = false;
    closeGate();
    changeState(STATE_AUTONOMOUS);
  }
}

bool readButton(int pin) {
  static unsigned long lastPressReset  = 0;
  static unsigned long lastPressManual = 0;
  static bool lastStateReset  = HIGH;
  static bool lastStateManual = HIGH;

  bool current = digitalRead(pin);
  unsigned long now = millis();

  if (pin == PIN_BTN_RESET) {
    if (lastStateReset == HIGH && current == LOW && (now - lastPressReset > 300)) {
      lastPressReset = now; lastStateReset = current; return true;
    }
    lastStateReset = current;
  } else if (pin == PIN_BTN_MANUAL) {
    if (lastStateManual == HIGH && current == LOW && (now - lastPressManual > 300)) {
      lastPressManual = now; lastStateManual = current; return true;
    }
    lastStateManual = current;
  }
  return false;
}

void updateOLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("LOADING GATE");
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  display.setCursor(0, 14);
  display.print("State: ");
  display.println(stateName(currentState));

  display.setCursor(0, 26);
  if (currentState == STATE_PERMANENT_FREEZE) {
    display.println("!! SYSTEM LOCKED !!");
  } else if (currentState == STATE_SAFETY_HALT) {
    display.println("!! SAFETY BLOCK !!");
  } else if (currentState == STATE_TIMED_IDLE) {
    display.println("IDLE - WAITING...");
  } else if (currentState == STATE_MANUAL) {
    display.println("MANUAL OVERRIDE");
  } else {
    display.print("Dist: ");
    display.print(distanceCm, 1);
    display.println("cm");
  }

  display.setCursor(0, 38);
  display.print("Gate: ");
  display.print(gateOpen ? "OPEN " : "CLOSED");
  display.print(" E:");
  display.println(safetyEventCount);

  display.setCursor(0, 50);
  if (currentState == STATE_PERMANENT_FREEZE) {
    display.println("PRESS RESET BTN");
  } else if (currentState == STATE_MANUAL) {
    display.println("BTN: EXIT MANUAL");
  } else {
    display.print("Del: ");
    display.print(holdDelayMs);
    display.println("ms");
  }

  display.display();
}
