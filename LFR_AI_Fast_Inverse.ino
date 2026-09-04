#include <Arduino.h>
#include <EEPROM.h>

/*
 ==============================================================================
    AI HIGH-SPEED COMPETITION LFR  --  v2  (INVERSE / DUAL-POLARITY TRACK)
 ==============================================================================
 What changed vs v1:

 1. INVERSE MODE ADDED
    - Single inversion point inside readSensorsFast(). Every threshold below
      it (380 / 550 / 600) keeps working unchanged in both polarities.

 2. AUTO POLARITY DETECTION
    - When >=5 sensors read "line" AND the opposite polarity shows a coherent
      line near the centre, the track has flipped -> switch mode.
    - When >=5 sensors read "line" AND the opposite polarity shows nothing,
      the surface is uniform -> that is the finish box.

 3. FINISH DETECTOR NO LONGER HIJACKS THE INVERSE ZONE
    - Requires a sustained uniform surface + opposite-polarity blank + a
      forward verification nudge.

 4. SEPARATE EEPROM CALIBRATION SLOT PER POLARITY
    - Black tape on white board and white tape on black board have different
      reflectance ranges. Calibrating in inverse mode fills the inverse slot.
    - If the inverse slot was never calibrated, it falls back to the normal one.

 5. JUNCTION FLAG BUG FIXED
    - v1 could only ever set hasFarLeft/hasFarRight to true, so a false
      trigger could never be cancelled. Now the branch must still be visible
      after the alignment nudge (confirmation instead of OR).

 ------------------------------------------------------------------------------
 BUTTON MAP
    Short press (< 1 s) ............ Start / Stop
    Hold 1-3 s (LED solid) ......... Auto-calibrate current polarity
    Hold > 3 s (LED blinking) ...... Toggle NORMAL <-> INVERSE and save
 ==============================================================================
*/

// ================= PIN DEFINITIONS =================
const byte PIN_S1 = A7; // Far Left (Branch detector)
const byte PIN_S2 = A6; // Mid Left
const byte PIN_S3 = A3; // Center Left
const byte PIN_S4 = A2; // Center Right
const byte PIN_S5 = A1; // Mid Right
const byte PIN_S6 = A0; // Far Right (Branch detector)

const byte PWMA = 3;  // Left Motor Speed
const byte AIN2 = 4;  // Left Motor Dir 2
const byte AIN1 = 5;  // Left Motor Dir 1
const byte STBY = 6;  // TB6612 Standby Pin
const byte BIN1 = 7;  // Right Motor Dir 1
const byte BIN2 = 8;  // Right Motor Dir 2
const byte PWMB = 9;  // Right Motor Speed

const byte BUTTON = 11;
const byte LED    = 13;

// ================= SPEED & TUNING =================
int BASE_SPEED     = 135; // Cruising speed
int MAX_SPEED      = 200; // Cap speed
int TURN_SPEED     = 125; // Controlled pivot turn speed
int GAP_SPEED      = 130; // Dashed line crossing speed

// PID Constants
float Kp = 0.058;
float Ki = 0.0000;
float Kd = 0.280;

// ================= DETECTION THRESHOLDS =================
const int TH_LINE     = 380; // getPosition() active threshold / glare rejection
const int TH_BRANCH   = 600; // Far sensor -> 90 degree branch
const int TH_CONFIRM  = 550; // Re-read confirmation threshold
const int TH_CENTER   = 550; // Pivot turn exit threshold

// ================= INVERSE MODE CONFIG =================
const bool AUTO_INVERSE       = true; // set false to use manual toggle only
const bool BOOT_IN_NORMAL     = true; // always power up in normal polarity

const unsigned long INVERSE_CONFIRM_MS = 130; // sustained hint before flipping
const unsigned long FINISH_CONFIRM_MS  = 260; // sustained uniform before finish
const int  INVERSE_MAX_OFFSET = 1200; // opposite-polarity line must be centre-ish

bool inverseMode = false;

// Sensor weights (-2500 Far Left to +2500 Far Right)
const int sensorWeights[6] = {-2500, -1500, -500, 500, 1500, 2500};
const byte sensorPins[6]   = {PIN_S1, PIN_S2, PIN_S3, PIN_S4, PIN_S5, PIN_S6};

// Calibration Buffers  [0] = NORMAL polarity, [1] = INVERSE polarity
int  sensorMin[2][6];
int  sensorMax[2][6];
bool calValid[2] = {false, false};

int rawValues[6];
int normValues[6];

// State Tracking
int lastError = 0;
unsigned long gapStartTime = 0;
const unsigned long GAP_TIMEOUT = 450; // ms to shoot across dashed line
bool isRunning = false;
unsigned long runStartTime = 0;
unsigned long floodStart   = 0; // when >=5 sensors first read as "line"
unsigned long lastFlipTime = 0;
const unsigned long FLIP_LOCKOUT = 600; // ms before another flip is allowed

// ================= FUNCTION DECLARATIONS =================
void setMotors(int left, int right);
void stopMotors();
void brakeMotors();
void readSensorsFast();
int  getPosition(int &activeCount);
int  getOppositePosition(int &oppCount);
void handleJunction(bool hasLeft, bool hasRight);
void pivotTurnLeft();
void pivotTurnRight();
void setInverseMode(bool v);
void autoCalibrate();
void saveCalibration();
bool loadCalibration();
void handleButton();

// ================= SETUP =================
void setup() {
  // Fast ADC Prescaler (16us per sample -> 6x faster than default)
  ADCSRA = (ADCSRA & 0xF8) | 0x05;

  Serial.begin(115200);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(LED, OUTPUT);

  for (int i = 0; i < 6; i++) pinMode(sensorPins[i], INPUT);

  digitalWrite(STBY, HIGH);
  stopMotors();

  if (loadCalibration()) {
    Serial.println("AI Calibration Loaded!");
    Serial.print("  Normal slot : "); Serial.println(calValid[0] ? "OK" : "EMPTY");
    Serial.print("  Inverse slot: "); Serial.println(calValid[1] ? "OK" : "EMPTY (falls back to normal)");
  } else {
    for (int m = 0; m < 2; m++) {
      for (int i = 0; i < 6; i++) {
        sensorMin[m][i] = 120;
        sensorMax[m][i] = 850;
      }
    }
    calValid[0] = false;
    calValid[1] = false;
    Serial.println("No calibration found - using defaults. Please calibrate.");
  }

  if (BOOT_IN_NORMAL) inverseMode = false;

  Serial.print("Start polarity: ");
  Serial.println(inverseMode ? "INVERSE (white line)" : "NORMAL (black line)");
  Serial.println("LFR Ready: Left -> Right -> Straight Priority Active.");
}

// ================= MAIN LOOP =================
void loop() {
  handleButton();

  if (!isRunning) {
    stopMotors();
    return;
  }

  // 1. FAST SENSOR READING
  readSensorsFast();

  int activeCount = 0;
  int position = getPosition(activeCount);

  // -------------------------------------------------------------
  // 1. UNIFORM SURFACE  ->  POLARITY FLIP  or  FINISH BOX
  //    Both look identical to v1 (activeCount == 6). They are separated
  //    by testing what the OPPOSITE polarity would see:
  //       coherent centred line  -> the track inverted
  //       nothing at all         -> solid finish box
  // -------------------------------------------------------------
  if (activeCount >= 5) {
    if (floodStart == 0) floodStart = millis();
    unsigned long held = millis() - floodStart;

    int oppCount = 0;
    int oppPos   = getOppositePosition(oppCount);

    bool inverseHint = (oppCount >= 1 && oppCount <= 3 &&
                        abs(oppPos) < INVERSE_MAX_OFFSET);

    // ---- A. Track polarity changed ----
    if (AUTO_INVERSE && inverseHint &&
        held > INVERSE_CONFIRM_MS &&
        (millis() - lastFlipTime) > FLIP_LOCKOUT) {

      setInverseMode(!inverseMode);
      lastFlipTime = millis();
      floodStart   = 0;
      lastError    = 0;
      gapStartTime = 0;

      setMotors(GAP_SPEED, GAP_SPEED); // carry momentum into the new zone
      delay(30);
      return;
    }

    // While an inverse hint is building, do NOT pivot. Far-left and
    // far-right are both "black" here and would fire a false 90 turn.
    if (inverseHint) {
      setMotors(GAP_SPEED, GAP_SPEED);
      return;
    }

    // ---- B. Solid finish box (uniform, no line in either polarity) ----
    if (oppCount == 0 &&
        held > FINISH_CONFIRM_MS &&
        (millis() - runStartTime > 2500)) {

      setMotors(100, 100);
      delay(90);
      readSensorsFast();

      int ac = 0; getPosition(ac);
      int oc = 0; getOppositePosition(oc);

      if (ac >= 5 && oc == 0) {
        brakeMotors();
        isRunning = false;
        digitalWrite(LED, HIGH);
        Serial.println(">>> FINISH POINT REACHED! STOPPED. <<<");
        return;
      }
    }
  } else {
    floodStart = 0;
  }

  // -------------------------------------------------------------
  // 2. 90-DEGREE JUNCTION / T-BRANCH DETECTION (Left Priority Rule)
  // -------------------------------------------------------------
  bool hasFarLeft  = (normValues[0] > TH_BRANCH);
  bool hasFarRight = (normValues[5] > TH_BRANCH);

  if (hasFarLeft || hasFarRight) {
    handleJunction(hasFarLeft, hasFarRight);
    return;
  }

  // -------------------------------------------------------------
  // 3. DASHED LINE GAP / NO LINE (activeCount == 0)
  //    Works in both polarities because normValues are already inverted.
  // -------------------------------------------------------------
  if (activeCount == 0) {
    if (abs(lastError) < 900) {
      if (gapStartTime == 0) gapStartTime = millis();

      if (millis() - gapStartTime < GAP_TIMEOUT) {
        setMotors(GAP_SPEED, GAP_SPEED);
        return;
      }
    }

    if (lastError < 0) {
      setMotors(-TURN_SPEED, TURN_SPEED); // Spin Left
    } else {
      setMotors(TURN_SPEED, -TURN_SPEED); // Spin Right
    }
    return;
  }

  // -------------------------------------------------------------
  // 4. SMOOTH HIGH-SPEED PID (Curves, Circles & Straights)
  // -------------------------------------------------------------
  gapStartTime = 0;
  int error = position;

  float P = (float)error;
  float D = (float)(error - lastError);
  lastError = error;

  int correction = (int)(Kp * P + Kd * D);

  int leftMotor  = BASE_SPEED + correction;
  int rightMotor = BASE_SPEED - correction;

  setMotors(leftMotor, rightMotor);
}

// ================= POLARITY SWITCH =================
void setInverseMode(bool v) {
  inverseMode = v;
  Serial.print(">>> POLARITY SWITCHED -> ");
  Serial.println(inverseMode ? "INVERSE (white line on black)" : "NORMAL (black line on white)");

  // Quick LED confirmation without stalling the run
  digitalWrite(LED, LOW);  delay(25);
  digitalWrite(LED, HIGH);
}

// ================= 90-DEGREE JUNCTION HANDLER (LEFT -> RIGHT -> STRAIGHT) =================
void handleJunction(bool hasFarLeft, bool hasFarRight) {
  // Move forward slightly so wheels align with the intersection axis
  setMotors(BASE_SPEED - 20, BASE_SPEED - 20);
  delay(40);
  readSensorsFast();

  // CONFIRMATION (v1 bug fix): a real branch is still visible after the
  // nudge. A curve that only brushed the far sensor is now cancelled.
  hasFarLeft  = hasFarLeft  && (normValues[0] > TH_CONFIRM);
  hasFarRight = hasFarRight && (normValues[5] > TH_CONFIRM);

  // PRIORITY 1: Turn LEFT if left branch exists
  if (hasFarLeft) {
    pivotTurnLeft();
    return;
  }

  // PRIORITY 2: Turn RIGHT if right branch exists (and no left)
  if (hasFarRight) {
    pivotTurnRight();
    return;
  }

  // PRIORITY 3: Continue STRAIGHT
  setMotors(BASE_SPEED, BASE_SPEED);
  delay(30);
}

// ================= RESPONSIVE PIVOT TURNS (ZERO BLIND DELAY) =================
void pivotTurnLeft() {
  setMotors(-TURN_SPEED, TURN_SPEED);
  delay(50); // rotate off the original perpendicular line

  unsigned long start = millis();
  while (millis() - start < 700) {
    readSensorsFast();
    if (normValues[2] > TH_CENTER || normValues[3] > TH_CENTER) break;
  }
  lastError = 0;
}

void pivotTurnRight() {
  setMotors(TURN_SPEED, -TURN_SPEED);
  delay(50);

  unsigned long start = millis();
  while (millis() - start < 700) {
    readSensorsFast();
    if (normValues[2] > TH_CENTER || normValues[3] > TH_CENTER) break;
  }
  lastError = 0;
}

// ================= FAST SENSOR READING & POSITION =================
void readSensorsFast() {
  // Pick the calibration slot for the active polarity, fall back to normal
  byte m = inverseMode ? 1 : 0;
  if (!calValid[m]) m = 0;

  for (int i = 0; i < 6; i++) {
    rawValues[i] = analogRead(sensorPins[i]);

    int lo = sensorMin[m][i];
    int hi = sensorMax[m][i];
    if (hi - lo < 30) { lo = 120; hi = 850; } // guard against divide-by-zero

    int v = map(rawValues[i], lo, hi, 0, 1000);
    v = constrain(v, 0, 1000);

    // ---- THE SINGLE INVERSION POINT ----
    if (inverseMode) v = 1000 - v;

    normValues[i] = v;
  }
}

int getPosition(int &activeCount) {
  long weightedSum = 0;
  long totalVal = 0;
  activeCount = 0;

  for (int i = 0; i < 6; i++) {
    int val = normValues[i];
    if (val > TH_LINE) {
      activeCount++;
      weightedSum += (long)val * sensorWeights[i];
      totalVal += val;
    }
  }

  if (totalVal == 0) return lastError;
  return (int)(weightedSum / totalVal);
}

// What the array would report if the polarity were flipped.
// Used only to tell an inverse zone apart from the finish box.
int getOppositePosition(int &oppCount) {
  long weightedSum = 0;
  long totalVal = 0;
  oppCount = 0;

  for (int i = 0; i < 6; i++) {
    int val = 1000 - normValues[i];
    if (val > TH_LINE) {
      oppCount++;
      weightedSum += (long)val * sensorWeights[i];
      totalVal += val;
    }
  }

  if (totalVal == 0) return 0;
  return (int)(weightedSum / totalVal);
}

// ================= MOTOR DRIVE =================
void setMotors(int left, int right) {
  left  = constrain(left, -MAX_SPEED, MAX_SPEED);
  right = constrain(right, -MAX_SPEED, MAX_SPEED);

  if (left >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, left);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, -left);
  }

  if (right >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, right);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, -right);
  }
}

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

void brakeMotors() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, 255);
  analogWrite(PWMB, 255);
}

// ================= BUTTON =================
void handleButton() {
  if (digitalRead(BUTTON) != LOW) return;
  delay(40);
  if (digitalRead(BUTTON) != LOW) return;

  unsigned long pStart = millis();
  unsigned long blinkT = millis();
  bool blinkState = false;

  while (digitalRead(BUTTON) == LOW) {
    unsigned long held = millis() - pStart;

    if (held > 3000) {                 // fast blink = inverse toggle armed
      if (millis() - blinkT > 100) {
        blinkState = !blinkState;
        digitalWrite(LED, blinkState);
        blinkT = millis();
      }
    } else if (held > 1000) {          // solid = calibrate armed
      digitalWrite(LED, HIGH);
    }
  }

  unsigned long held = millis() - pStart;

  if (held > 3000) {
    // ---- Manual polarity toggle + save ----
    stopMotors();
    isRunning = false;
    setInverseMode(!inverseMode);
    saveCalibration();

    for (int i = 0; i < (inverseMode ? 2 : 1); i++) {
      digitalWrite(LED, HIGH); delay(220);
      digitalWrite(LED, LOW);  delay(220);
    }
  }
  else if (held > 1000) {
    // ---- Calibrate the CURRENT polarity slot ----
    stopMotors();
    isRunning = false;
    autoCalibrate();
  }
  else {
    // ---- Start / Stop ----
    isRunning = !isRunning;
    if (isRunning) {
      lastError    = 0;
      gapStartTime = 0;
      floodStart   = 0;
      lastFlipTime = 0;
      runStartTime = millis();
      digitalWrite(LED, HIGH);
      delay(150);
    } else {
      stopMotors();
      digitalWrite(LED, LOW);
    }
  }
}

// ================= AUTO CALIBRATION =================
void autoCalibrate() {
  byte m = inverseMode ? 1 : 0;

  Serial.print("Auto-Calibrating slot: ");
  Serial.println(m == 0 ? "NORMAL" : "INVERSE");
  digitalWrite(LED, HIGH);

  for (int i = 0; i < 6; i++) {
    sensorMin[m][i] = 1023;
    sensorMax[m][i] = 0;
  }

  unsigned long start = millis();
  while (millis() - start < 3500) {
    if ((millis() - start) % 500 < 250) {
      setMotors(110, -110);
    } else {
      setMotors(-110, 110);
    }

    for (int i = 0; i < 6; i++) {
      int val = analogRead(sensorPins[i]);
      if (val < sensorMin[m][i]) sensorMin[m][i] = val;
      if (val > sensorMax[m][i]) sensorMax[m][i] = val;
    }
    delay(3);
  }

  stopMotors();
  calValid[m] = true;
  saveCalibration();
  digitalWrite(LED, LOW);
  Serial.println("Calibration Done & Saved to EEPROM!");
}

// ================= EEPROM STORAGE =================
// Layout: magic | 6x(min,max) NORMAL | 6x(min,max) INVERSE | validFlags | mode
const int EE_MAGIC = 23063;

void saveCalibration() {
  int addr = 0;
  EEPROM.put(addr, (int)EE_MAGIC); addr += sizeof(int);

  for (byte m = 0; m < 2; m++) {
    for (byte i = 0; i < 6; i++) {
      EEPROM.put(addr, sensorMin[m][i]); addr += sizeof(int);
      EEPROM.put(addr, sensorMax[m][i]); addr += sizeof(int);
    }
  }

  int flags = (calValid[0] ? 1 : 0) | (calValid[1] ? 2 : 0);
  EEPROM.put(addr, flags); addr += sizeof(int);
  EEPROM.put(addr, (int)(inverseMode ? 1 : 0));
}

bool loadCalibration() {
  int addr = 0;
  int magic;
  EEPROM.get(addr, magic); addr += sizeof(int);
  if (magic != EE_MAGIC) return false;

  for (byte m = 0; m < 2; m++) {
    for (byte i = 0; i < 6; i++) {
      EEPROM.get(addr, sensorMin[m][i]); addr += sizeof(int);
      EEPROM.get(addr, sensorMax[m][i]); addr += sizeof(int);

      if (sensorMin[m][i] < 0 || sensorMin[m][i] > 1023 ||
          sensorMax[m][i] < 0 || sensorMax[m][i] > 1023) {
        sensorMin[m][i] = 120;
        sensorMax[m][i] = 850;
      }
    }
  }

  int flags;
  EEPROM.get(addr, flags); addr += sizeof(int);
  calValid[0] = (flags & 1) != 0;
  calValid[1] = (flags & 2) != 0;

  int md;
  EEPROM.get(addr, md);
  inverseMode = (md == 1);

  return calValid[0];
}
