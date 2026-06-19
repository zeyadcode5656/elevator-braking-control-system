/*
  ==============================================================
  ELEVATOR BRAKING CONTROL SYSTEM
  Mechatronics Engineering - Electric Drives Project
  Helwan National University - 3rd Year

  Hardware:
    - ESP32 Microcontroller
    - L298N Motor Driver (H-Bridge)
    - 3x Hall Effect Sensors (Floor 0, 1, 2)
    - 4x4 Matrix Keypad
    - 16x2 I2C LCD (address 0x27)
    - 4x Push Buttons (3 floor select + 1 emergency stop)

  NOTE: Reconstructed from the project's technical documentation
  report (pin mapping, algorithm formulas, debounce timings, and
  function signatures). Verify against your actual wiring before
  flashing, and adjust pin numbers if your build differs.
  ==============================================================
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>

// ---------------- LCD ----------------
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ---------------- Motor Driver (L298N) ----------------
const int ENA = 23;   // PWM speed control
const int IN1 = 18;   // Direction control line 1
const int IN2 = 19;   // Direction control line 2

const int PWM_CHANNEL = 0;
const int PWM_FREQ    = 1000;  // 1 kHz
const int PWM_RES     = 8;     // 8-bit (0-255)

// ---------------- Hall Effect Sensors ----------------
// Active LOW, internal pull-up, 80 ms debounce
const int SENSOR_PINS[3] = {32, 33, 25};   // Floor 0, 1, 2 - adjust to your wiring
const unsigned long SENSOR_DEBOUNCE_MS = 80;

// ---------------- Push Buttons ----------------
// 3 floor buttons + 1 emergency stop, 50 ms debounce
const int BUTTON_PINS[3] = {26, 27, 14};   // B0, B1, B2 - adjust to your wiring
const int ESTOP_BUTTON_PIN = 12;
const unsigned long BUTTON_DEBOUNCE_MS = 50;

bool lastButtonState[4] = {HIGH, HIGH, HIGH, HIGH};
unsigned long lastButtonChangeTime[4] = {0, 0, 0, 0};

// ---------------- Keypad (4x4) ----------------
const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[ROWS] = {13, 15, 4, 16};   // adjust to your wiring
byte colPins[COLS] = {17, 5, 22, 21};   // adjust to your wiring
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// ---------------- Motion Profile Parameters ----------------
const int   MAX_SPEED       = 200;   // full-speed PWM
const int   RAMP_MIN_SPEED  = 120;   // initial PWM at start of ramp
const int   BRAKE_END_SPEED = 70;    // PWM at end of braking curve
const unsigned long RAMP_TIME_MS  = 600;   // acceleration phase duration
const unsigned long BRAKE_TIME_MS = 700;   // deceleration phase duration
const unsigned long HOLD_TIME_MS  = 150;   // hold phase duration
const unsigned long COAST_TIME_MS = 150;   // coast-to-zero phase duration

// ---------------- State Machine ----------------
enum ElevatorState {
  IDLE,
  FLOOR_SELECT,
  CONFIRM,
  RAMPING,
  FULL_SPEED,
  BRAKING,
  STOPPED,
  EMERGENCY_STOP
};
ElevatorState currentState = IDLE;

int currentFloor   = 0;       // last known / current floor (0,1,2)
int targetFloor    = -1;      // selected destination
int travelDir      = 0;       // 1 = up, -1 = down
bool positionKnown = true;

unsigned long sensorActiveSince[3] = {0, 0, 0};
bool sensorDebouncing[3] = {false, false, false};

String lcdLine1 = "";
String lcdLine2 = "";

// ==============================================================
// MOTOR CONTROL FUNCTIONS
// ==============================================================

void motorUp(int spd) {
  // Rotates motor clockwise (upward direction) at specified PWM speed
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  ledcWrite(PWM_CHANNEL, constrain(spd, 0, 255));
}

void motorDown(int spd) {
  // Rotates motor counterclockwise (downward direction) at specified PWM speed
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  ledcWrite(PWM_CHANNEL, constrain(spd, 0, 255));
}

void motorStop() {
  // Applies active braking by setting both IN1 and IN2 HIGH
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  ledcWrite(PWM_CHANNEL, 0);
}

void motorRun(int dir, int spd) {
  // Unified motor control: dir = 1 for up, dir = -1 for down
  if (dir == 1) motorUp(spd);
  else if (dir == -1) motorDown(spd);
  else motorStop();
}

// ==============================================================
// LCD HELPERS
// ==============================================================

void lcdSet(String line1, String line2) {
  // Updates LCD only when content has changed, reducing I2C traffic/flicker
  if (line1 == lcdLine1 && line2 == lcdLine2) return;
  lcdLine1 = line1;
  lcdLine2 = line2;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

void showTempMsg(String line1, String line2, ElevatorState nextState) {
  // Displays a temporary message for 2 seconds, then transitions to nextState
  lcdSet(line1, line2);
  delay(2000);
  currentState = nextState;
}

// ==============================================================
// BUTTON DEBOUNCE
// ==============================================================

bool readButton(int idx, int pin) {
  // Debounced button reading; returns true on a valid press
  // after the debounce period (50 ms), tracking press/release edges.
  bool reading = digitalRead(pin);
  bool pressed = false;

  if (reading != lastButtonState[idx]) {
    lastButtonChangeTime[idx] = millis();
  }

  if ((millis() - lastButtonChangeTime[idx]) > BUTTON_DEBOUNCE_MS) {
    if (reading == LOW && lastButtonState[idx] == HIGH) {
      pressed = true;
    }
  }

  lastButtonState[idx] = reading;
  return pressed;
}

// ==============================================================
// SENSOR DEBOUNCE / FLOOR DETECTION
// ==============================================================

void checkSensors() {
  // Monitors floor sensors during motion. Implements an 80 ms debounce
  // and triggers the braking sequence once the target floor sensor
  // has been continuously active (LOW) for the full debounce window.
  for (int i = 0; i < 3; i++) {
    bool active = (digitalRead(SENSOR_PINS[i]) == LOW);

    if (active) {
      if (!sensorDebouncing[i]) {
        sensorDebouncing[i] = true;
        sensorActiveSince[i] = millis();
      } else if (millis() - sensorActiveSince[i] >= SENSOR_DEBOUNCE_MS) {
        if (currentState == RAMPING || currentState == FULL_SPEED) {
          if (i == targetFloor) {
            currentState = BRAKING;
          }
        }
      }
    } else {
      sensorDebouncing[i] = false;
    }
  }
}

int detectInitialFloor() {
  // On startup, checks all three floor sensors.
  // Defaults to ground floor (0) for safety if none are active.
  for (int i = 0; i < 3; i++) {
    if (digitalRead(SENSOR_PINS[i]) == LOW) return i;
  }
  return 0;
}

// ==============================================================
// MOTION PROFILE (ACCELERATION / DECELERATION CURVES)
// ==============================================================

int rampSpeed(unsigned long elapsed) {
  // Square-root acceleration curve:
  // speed = rampMinSpeed + (maxSpeed - rampMinSpeed) * sqrt(progress)
  float progress = constrain((float)elapsed / RAMP_TIME_MS, 0.0, 1.0);
  float curve = sqrt(progress);
  return RAMP_MIN_SPEED + (int)((MAX_SPEED - RAMP_MIN_SPEED) * curve);
}

int brakeSpeed(unsigned long elapsed) {
  // Inverse square-root deceleration curve:
  // speed = maxSpeed - (maxSpeed - brakeEndSpeed) * (1 - sqrt(1 - progress))
  float progress = constrain((float)elapsed / BRAKE_TIME_MS, 0.0, 1.0);
  float curve = 1.0 - sqrt(1.0 - progress);
  return MAX_SPEED - (int)((MAX_SPEED - BRAKE_END_SPEED) * curve);
}

// ==============================================================
// ELEVATOR MOVEMENT LOGIC
// ==============================================================

void goToFloor(int floor) {
  // Initiates movement to the specified floor.
  // Handles the same-floor case and emergency-recovery direction logic.
  if (floor == currentFloor && positionKnown) {
    showTempMsg("Already at", "Floor " + String(floor), IDLE);
    return;
  }

  targetFloor = floor;

  if (!positionKnown) {
    // Emergency-recovery: direction inferred from last known floor
    travelDir = (targetFloor > currentFloor) ? 1 : -1;
  } else {
    travelDir = (targetFloor > currentFloor) ? 1 : -1;
  }

  currentState = RAMPING;
}

void runMotionProfile() {
  static unsigned long phaseStart = 0;
  static ElevatorState lastState = IDLE;

  if (currentState != lastState) {
    phaseStart = millis();
    lastState = currentState;
  }

  unsigned long elapsed = millis() - phaseStart;

  switch (currentState) {

    case RAMPING: {
      checkSensors();
      int spd = rampSpeed(elapsed);
      motorRun(travelDir, spd);
      lcdSet("Moving to F" + String(targetFloor), "Accelerating...");
      if (elapsed >= RAMP_TIME_MS) currentState = FULL_SPEED;
      break;
    }

    case FULL_SPEED: {
      checkSensors();
      motorRun(travelDir, MAX_SPEED);
      lcdSet("Moving to F" + String(targetFloor), "Full Speed");
      break;
    }

    case BRAKING: {
      int spd = brakeSpeed(elapsed);
      motorRun(travelDir, spd);
      lcdSet("Arriving F" + String(targetFloor), "Braking...");
      if (elapsed >= BRAKE_TIME_MS) {
        phaseStart = millis();
        // proceed to Hold phase manually
        unsigned long holdStart = millis();
        while (millis() - holdStart < HOLD_TIME_MS) {
          motorRun(travelDir, BRAKE_END_SPEED);
        }
        // Coast phase: linear ramp-down to 0
        unsigned long coastStart = millis();
        while (millis() - coastStart < COAST_TIME_MS) {
          float p = (float)(millis() - coastStart) / COAST_TIME_MS;
          int spdCoast = BRAKE_END_SPEED * (1.0 - p);
          motorRun(travelDir, spdCoast);
        }
        motorStop();
        currentFloor = targetFloor;
        positionKnown = true;
        currentState = STOPPED;
      }
      break;
    }

    case STOPPED: {
      lcdSet("Arrived Floor", String(currentFloor));
      delay(800);
      currentState = IDLE;
      break;
    }

    default:
      break;
  }
}

// ==============================================================
// EMERGENCY STOP
// ==============================================================

void triggerEmergencyStop() {
  motorStop();
  positionKnown = false;
  currentState = EMERGENCY_STOP;
  lcdSet("EMERGENCY STOP", "Select floor");
}

// ==============================================================
// SETUP
// ==============================================================

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA, PWM_CHANNEL);

  for (int i = 0; i < 3; i++) pinMode(SENSOR_PINS[i], INPUT_PULLUP);
  for (int i = 0; i < 3; i++) pinMode(BUTTON_PINS[i], INPUT_PULLUP);
  pinMode(ESTOP_BUTTON_PIN, INPUT_PULLUP);

  lcd.init();
  lcd.backlight();

  currentFloor = detectInitialFloor();
  positionKnown = true;

  lcdSet("Elevator Ready", "Floor " + String(currentFloor));
  motorStop();
}

// ==============================================================
// MAIN LOOP
// ==============================================================

void loop() {
  char key = keypad.getKey();

  // ---- Emergency stop (keypad D or dedicated button) ----
  if (key == 'D' || readButton(3, ESTOP_BUTTON_PIN)) {
    triggerEmergencyStop();
  }

  switch (currentState) {

    case IDLE: {
      lcdSet("Select Floor", "0/1/2 or Keypad");

      // Direct button control (no confirmation)
      for (int i = 0; i < 3; i++) {
        if (readButton(i, BUTTON_PINS[i])) {
          goToFloor(i);
        }
      }

      // Keypad control (requires confirmation)
      if (key >= '0' && key <= '2') {
        targetFloor = key - '0';
        currentState = CONFIRM;
      }
      break;
    }

    case CONFIRM: {
      lcdSet("Go to Floor " + String(targetFloor) + "?", "# OK   * Cancel");
      if (key == '#') {
        goToFloor(targetFloor);
      } else if (key == '*') {
        currentState = IDLE;
      }
      break;
    }

    case EMERGENCY_STOP: {
      lcdSet("EMERGENCY STOP", "Select new floor");
      if (key >= '0' && key <= '2') {
        goToFloor(key - '0');
      }
      for (int i = 0; i < 3; i++) {
        if (readButton(i, BUTTON_PINS[i])) {
          goToFloor(i);
        }
      }
      break;
    }

    case RAMPING:
    case FULL_SPEED:
    case BRAKING:
    case STOPPED:
      runMotionProfile();
      break;

    default:
      break;
  }
}
