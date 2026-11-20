#define MODE 0

const uint8_t ENA = 11, IN1 = 2, IN2 = 3;
const uint8_t ENB = 10, IN3 = 4, IN4 = 5;
const uint8_t TRIG_F = 6,  ECHO_F = 7;
const uint8_t TRIG_L = 8,  ECHO_L = 9;
const uint8_t TRIG_R = 13, ECHO_R = 12;
const uint8_t IR_PIN = A5;
const uint8_t IR_BLACK = HIGH;

struct StartConfig {
  const char* name;
  bool followLeft;
};
const StartConfig STARTS[] = {
  { "Red A",  true  },
  { "Red B",  true  },
  { "Red C",  false },
  { "Blue A", false },
  { "Blue B", false },
  { "Blue C", true  },
};
const uint8_t START = 0;

#define USE_SIDE_SWITCH
const uint8_t SIDE_SWITCH_PIN = A0;

const float SIDE_ANGLE_DEG = 90;

int SPEED_PCT = 78;

int BASE_PWM  = 78;
int SLOW_PWM  = 60;
int TURN_PWM  = 78;
int MIN_PWM   = 50;

#define LOW_FREQ_PWM

const int KICK_BELOW = 140;
const int KICK_MS    = 15;
int LEFT_TRIM = 0;
int RIGHT_TRIM = 0;

float SIDE_CENTRE_CM = 8.0;
float WALL_CM        = 13.0;
float OPEN_CM        = 14.0;
float FRONT_CENTRE_CM = 7.0;
float FRONT_OPEN_CM  = 16.0;
float FRONT_SLOW_CM  = 18.0;
float MAX_CM         = 80.0;

#define AUTO_CALIBRATE

float KP = 8.0;
float KD = 10.0;
int   MAX_CORR = 40;
float MAX_ERR_CM = 4.0;

float DANGER_SIDE_CM = 1.5;
int   GUARD_FAST = 60;
int   GUARD_SLOW = 0;

int GAP_FWD_MS  = 180;
int EXIT_FWD_MS = 300;

float EXIT_DIST_CM = 10.0;
int   EXIT_MAX_MS  = 1500;
int TURN90_MS   = 200;

#define ARC_TURNS
int ARC_OUTER_PWM = 80;
int ARC_INNER_PWM = 0;
int ARC90_MS      = 200;
int ARC_GAP_MS    = 60;

int ARC_MIN_PCT  = 95;
int ARC_MAX_PCT  = 130;

int PIVOT_MIN_MS   = 140;
int PIVOT_EXTRA_MS = 30;
int PIVOT_MAX_MS   = 700;
int TURN180_MS  = 600;
int PARK_FWD_MS = 1000;
int START_DELAY_MS = 2000;

const unsigned long ECHO_TIMEOUT_US = 6000;
const uint8_t PING_GAP_MS = 2;
const float   TOO_CLOSE_CM = 2.5;
const uint8_t NO_ECHO_CONFIRM = 4;
const float   NEAR_HOLD_CM = 12.0;

const float   STILL_CM       = 1.5;
const uint8_t STUCK_COUNT    = 9;
const int     UNJAM_PWM      = 160;
const int     UNJAM_MS       = 200;
const float   UNJAM_DIFF_CM  = 1.0;
const int     BACKUP_MS      = 120;
const int     STUCK_AGAIN_MS = 2000;
const uint8_t OPEN_CONFIRM = 1;

float dF = MAX_CM, dL = MAX_CM, dR = MAX_CM;
float rawL = MAX_CM, rawR = MAX_CM;
float lastErr = 0;
float sideFactor;
const StartConfig* cfg;
bool followLeft;
bool finished = false;
bool stopped = false;
bool sawWhite = false;
uint8_t blackCount = 0;
uint8_t missF = 0, missL = 0, missR = 0;
uint8_t openL = 0, openR = 0;
unsigned long stuckSince = 0, lastShake = 0;
uint8_t stillCount = 0;
bool stuckFlag = false;
float refF = -100, refL = -100, refR = -100;
float stuckRef = -100;

void checkSerialStop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'x' || c == 'X') stopped = true;
  }
}

void writeMotors(int left, int right);
int lastLeft = 0, lastRight = 0;

void waitMs(unsigned long ms);

void setMotors(int left, int right) {
  if (stopped) { left = 0; right = 0; }
  left  = (int)((long)left  * SPEED_PCT / 100);
  right = (int)((long)right * SPEED_PCT / 100);

  bool wasStill = (lastLeft == 0 && lastRight == 0);
  bool kickL = left  != 0 && abs(left)  < KICK_BELOW && (wasStill || (lastLeft  != 0 && (lastLeft  > 0) != (left  > 0)));
  bool kickR = right != 0 && abs(right) < KICK_BELOW && (wasStill || (lastRight != 0 && (lastRight > 0) != (right > 0)));
  lastLeft = left; lastRight = right;
  if (kickL || kickR) {
    int kl = kickL ? (left  > 0 ? 255 : -255) : left;
    int kr = kickR ? (right > 0 ? 255 : -255) : right;
    writeMotors(kl, kr);
    waitMs(KICK_MS);
  }
  writeMotors(left, right);
}

void writeMotors(int left, int right) {
  if (left  > 0) left  += LEFT_TRIM;  else if (left  < 0) left  -= LEFT_TRIM;
  if (right > 0) right += RIGHT_TRIM; else if (right < 0) right -= RIGHT_TRIM;
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  digitalWrite(IN1, right >= 0 ? LOW : HIGH);
  digitalWrite(IN2, right >= 0 ? HIGH : LOW);
  analogWrite(ENA, abs(right));
  digitalWrite(IN3, left >= 0 ? LOW : HIGH);
  digitalWrite(IN4, left >= 0 ? HIGH : LOW);
  analogWrite(ENB, abs(left));
}

void stopMotors() { setMotors(0, 0); }

void brake(int l, int r) {
  setMotors(-l / 2, -r / 2);
  waitMs(30);
  stopMotors();
  waitMs(40);
}

bool stripeSeen = false;
void pollIR() {
  if (digitalRead(IR_PIN) == IR_BLACK) {
    if (blackCount < 255) blackCount++;
    if (sawWhite && blackCount >= 2) stripeSeen = true;
  } else {
    blackCount = 0;
    sawWhite = true;
  }
}
bool stripeHit() { return stripeSeen; }

void waitMs(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) { pollIR(); delayMicroseconds(500); }
}

float readCm(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long t = pulseIn(echo, HIGH, ECHO_TIMEOUT_US);
  if (t == 0) return -1;
  float cm = t / 58.0;
  if (cm < TOO_CLOSE_CM) cm = TOO_CLOSE_CM;
  return cm;
}

float acceptReading(float oldV, float newV, uint8_t &misses) {
  if (newV < 0) {
    if (misses < 255) misses++;

    if (oldV < NEAR_HOLD_CM) return oldV;
    return (misses >= NO_ECHO_CONFIRM) ? MAX_CM : oldV;
  }
  misses = 0;
  return newV;
}

float smooth(float oldV, float newV) {
  return (fabs(newV - oldV) > 8) ? newV : 0.6 * newV + 0.4 * oldV;
}

uint8_t countOpen(uint8_t count, float raw) {
  if (raw < 0) return count;
  if (raw * sideFactor > OPEN_CM) return count < 255 ? count + 1 : 255;
  return 0;
}
bool sideOpenL() { return openL >= OPEN_CONFIRM; }
bool sideOpenR() { return openR >= OPEN_CONFIRM; }

void trackStuck() {
  if (lastLeft == 0 && lastRight == 0) return;
  float moved = fabs(dF - refF) + fabs(dL - refL) + fabs(dR - refR);
  if (moved > STILL_CM) { refF = dF; refL = dL; refR = dR; stillCount = 0; }
  else {
    if (stillCount < 255) stillCount++;
    if (stillCount >= STUCK_COUNT) stuckFlag = true;
  }
}
bool stuckNow() { return stuckFlag; }

void readAll() {
  pollIR(); float f = readCm(TRIG_F, ECHO_F); delay(PING_GAP_MS);
  pollIR(); float l = readCm(TRIG_L, ECHO_L); delay(PING_GAP_MS);
  pollIR(); float r = readCm(TRIG_R, ECHO_R); delay(PING_GAP_MS);
  pollIR();
  rawL = l; rawR = r;
  f = acceptReading(dF, f, missF);
  l = acceptReading(dL / sideFactor, l, missL);
  r = acceptReading(dR / sideFactor, r, missR);
  dF = smooth(dF, f);
  dL = smooth(dL, l * sideFactor);
  dR = smooth(dR, r * sideFactor);

  openL = countOpen(openL, rawL);
  openR = countOpen(openR, rawR);
  trackStuck();
  checkSerialStop();
}

bool wallGuard() {
  if (dL < DANGER_SIDE_CM && dL <= dR) {
    setMotors(GUARD_FAST, GUARD_SLOW); lastErr = 0; return true;
  }
  if (dR < DANGER_SIDE_CM) {
    setMotors(GUARD_SLOW, GUARD_FAST); lastErr = 0; return true;
  }
  return false;
}

void steer(int base) {
  if (wallGuard()) return;

  float fw = followLeft ? dL : dR;
  float ow = followLeft ? dR : dL;
  float err;
  if (fw < WALL_CM)      err = fw - SIDE_CENTRE_CM;
  else if (ow < WALL_CM) err = SIDE_CENTRE_CM - ow;
  else                   err = 0;
  if (!followLeft) err = -err;
  err = constrain(err, -MAX_ERR_CM, MAX_ERR_CM);
  float corr = KP * err + KD * (err - lastErr);
  lastErr = err;
  corr = constrain(corr, -MAX_CORR, MAX_CORR);
  setMotors(constrain((int)(base - corr), MIN_PWM, 255),
            constrain((int)(base + corr), MIN_PWM, 255));
}

void park();

void driveForward(int ms, bool brakeAtEnd) {
  unsigned long t0 = millis();
  while (millis() - t0 < (unsigned long)ms && !stopped) {
    readAll();
    if (stripeHit()) { park(); return; }
    if (stuckNow()) break;
    if (dF < FRONT_CENTRE_CM) break;
    steer(dF < FRONT_SLOW_CM ? SLOW_PWM : BASE_PWM);
  }
  if (brakeAtEnd) brake(BASE_PWM, BASE_PWM);
}

void pivot(int dir, int ms) {
  openL = openR = 0;
  setMotors(-dir * TURN_PWM, dir * TURN_PWM);
  waitMs(ms);
  brake(-dir * TURN_PWM, dir * TURN_PWM);
  lastErr = 0;
}

void arcTurn(int dir) {
  openL = openR = 0;
  int outer = ARC_OUTER_PWM, inner = ARC_INNER_PWM;
  if (dir > 0) setMotors(inner, outer);
  else         setMotors(outer, inner);
  waitMs(ARC90_MS);
  lastErr = 0;
}

void arcUntilWall(int dir) {
  openL = openR = 0;
  if (dir > 0) setMotors(ARC_INNER_PWM, ARC_OUTER_PWM);
  else         setMotors(ARC_OUTER_PWM, ARC_INNER_PWM);
  unsigned long t0 = millis();
  unsigned long arcMin = (unsigned long)ARC90_MS * ARC_MIN_PCT / 100;
  unsigned long arcMax = (unsigned long)ARC90_MS * ARC_MAX_PCT / 100;
  uint8_t seen = 0;
  while (!stopped) {
    unsigned long el = millis() - t0;
    if (el > arcMax) break;
    readAll();
    if (stripeHit()) { park(); return; }
    if (stuckNow()) break;
    float fw = (dir > 0) ? dL : dR;
    if (el >= arcMin) {

      if (dF < FRONT_CENTRE_CM) break;
      if (dL < DANGER_SIDE_CM || dR < DANGER_SIDE_CM) break;

      if (fw < OPEN_CM) { if (++seen >= 2) break; }
      else seen = 0;
    }
  }
  lastErr = 0;
}

void pivotUntilClear(int dir) {
  openL = openR = 0;
  setMotors(-dir * TURN_PWM, dir * TURN_PWM);
  unsigned long t0 = millis();
  uint8_t clear = 0;
  while (!stopped) {
    unsigned long el = millis() - t0;
    if (el > (unsigned long)PIVOT_MAX_MS) break;
    readAll();
    if (stuckNow()) break;
    if (el >= (unsigned long)PIVOT_MIN_MS) {
      if (dF > FRONT_OPEN_CM) { if (++clear >= 3) { waitMs(PIVOT_EXTRA_MS); break; } }
      else clear = 0;
    }
  }
  brake(-dir * TURN_PWM, dir * TURN_PWM);
  lastErr = 0;
}

void park() {
  Serial.println(F("STRIPE -> parking"));
  setMotors(SLOW_PWM, SLOW_PWM);
  unsigned long t0 = millis();
  while (digitalRead(IR_PIN) == IR_BLACK && millis() - t0 < 1500) {}
  setMotors(SLOW_PWM, SLOW_PWM);
  delay(PARK_FWD_MS);
  brake(SLOW_PWM, SLOW_PWM);
  stopMotors();
  finished = true;
}

int followDir() { return followLeft ? +1 : -1; }

float sensorAverage() {
  float sum = 0; uint8_t n = 0;
  if (dF < MAX_CM - 1) { sum += dF; n++; }
  if (dL < MAX_CM - 1) { sum += dL; n++; }
  if (dR < MAX_CM - 1) { sum += dR; n++; }
  return n ? sum / n : -1;
}

void shakeLoose() {
  bool again = lastShake != 0 && (millis() - lastShake) < (unsigned long)STUCK_AGAIN_MS;
  int ms = again ? UNJAM_MS * 2 : UNJAM_MS;
  for (uint8_t i = 0; i < 3; i++) readAll();
  if (dL + UNJAM_DIFF_CM < dR) {
    Serial.println(F("STUCK: left closer -> reverse RIGHT wheel"));
    setMotors(0, -UNJAM_PWM);
    waitMs(ms);
    brake(0, -UNJAM_PWM);
  } else if (dR + UNJAM_DIFF_CM < dL) {
    Serial.println(F("STUCK: right closer -> reverse LEFT wheel"));
    setMotors(-UNJAM_PWM, 0);
    waitMs(ms);
    brake(-UNJAM_PWM, 0);
  } else {
    Serial.println(F("STUCK: sides equal -> tiny back-up"));
    setMotors(-BASE_PWM, -BASE_PWM);
    waitMs(again ? BACKUP_MS * 2 : BACKUP_MS);
    brake(-BASE_PWM, -BASE_PWM);
  }
  openL = openR = 0;
  lastErr = 0;
  lastShake = millis();
  stuckSince = millis(); stuckRef = -100;
}

bool checkStuck() {
  if (!stuckFlag) return false;
  shakeLoose();
  stuckFlag = false;
  stillCount = 0;
  refF = refL = refR = -100;
  return true;
}

void leaveJunction() {
  for (uint8_t i = 0; i < 3; i++) readAll();
  float f0 = dF;
  bool useDist = f0 < MAX_CM - 1;
  unsigned long limit = useDist ? (unsigned long)EXIT_MAX_MS : (unsigned long)EXIT_FWD_MS;
  unsigned long t0 = millis();
  uint8_t reached = 0;
  while (millis() - t0 < limit && !stopped) {
    readAll();
    if (stripeHit()) { park(); return; }
    if (stuckNow()) break;
    if (dF < FRONT_CENTRE_CM) break;
    if (useDist) {
      if (dF <= f0 - EXIT_DIST_CM) { if (++reached >= 2) break; }
      else reached = 0;
    }
    steer(dF < FRONT_SLOW_CM ? SLOW_PWM : BASE_PWM);
  }

  openL = openR = 0;
}

void turnInto(int dir) {
#ifdef ARC_TURNS
  if (dF < FRONT_CENTRE_CM) {

    stopMotors();
    pivotUntilClear(dir);
  } else {
    arcUntilWall(dir);
  }
#else
  stopMotors();
  pivotUntilClear(dir);
#endif
  if (finished || stopped) return;
  leaveJunction();
}

void runMaze() {
  if (finished || stopped) { stopMotors(); return; }
  readAll();
  if (stripeHit()) { park(); return; }
  if (checkStuck()) return;

  bool fwOpen = followLeft ? sideOpenL() : sideOpenR();
  bool owOpen = followLeft ? sideOpenR() : sideOpenL();

  if (fwOpen || owOpen) {

    driveForward(ARC_GAP_MS, false);
    if (finished || stopped) return;
    readAll();
    fwOpen = followLeft ? sideOpenL() : sideOpenR();
    owOpen = followLeft ? sideOpenR() : sideOpenL();
    if (fwOpen) {
      Serial.println(followLeft ? F("TURN LEFT (1st)") : F("TURN RIGHT (1st)"));
      turnInto(followDir());
    } else if (owOpen) {
      Serial.println(followLeft ? F("TURN RIGHT (2nd)") : F("TURN LEFT (2nd)"));
      turnInto(-followDir());
    }
  }
  else if (dF < FRONT_CENTRE_CM) {

    stopMotors();
    Serial.println(F("WALL AHEAD -> turn away"));
    pivotUntilClear(-followDir());
  }
  else {

    steer(dF < FRONT_SLOW_CM ? SLOW_PWM : BASE_PWM);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
#ifdef LOW_FREQ_PWM

  TCCR1B = (TCCR1B & 0b11111000) | 0x04;
  TCCR2B = (TCCR2B & 0b11111000) | 0x06;
#endif
  stopMotors();
  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
  pinMode(IR_PIN, INPUT);

  sideFactor = sin(SIDE_ANGLE_DEG * PI / 180.0);
  cfg = &STARTS[START];
  followLeft = cfg->followLeft;
#ifdef USE_SIDE_SWITCH
  pinMode(SIDE_SWITCH_PIN, INPUT_PULLUP);
  delay(5);
  followLeft = (digitalRead(SIDE_SWITCH_PIN) == HIGH);
  Serial.print(F("Switch -> "));
#else
  Serial.print(F("Start: ")); Serial.print(cfg->name); Serial.print(F(" -> "));
#endif
  Serial.print(F("following ")); Serial.println(followLeft ? F("LEFT wall") : F("RIGHT wall"));
  delay(START_DELAY_MS);
  for (uint8_t i = 0; i < 8; i++) readAll();
#ifdef AUTO_CALIBRATE
  {
    float fw = followLeft ? dL : dR;
    if (fw > 1.0 && fw < 15.0) {
      SIDE_CENTRE_CM = fw;
      WALL_CM = fw + 6.0;
      OPEN_CM = fw + 10.0;
      DANGER_SIDE_CM = fw * 0.6;
      Serial.print(F("Auto-cal: centre=")); Serial.print(SIDE_CENTRE_CM, 1);
      Serial.print(F(" wall<")); Serial.print(WALL_CM, 1);
      Serial.print(F(" open>")); Serial.print(OPEN_CM, 1);
      Serial.print(F(" danger<")); Serial.println(DANGER_SIDE_CM, 1);
    } else {
      Serial.println(F("Auto-cal skipped: no wall on the followed side, using defaults"));
    }
  }
#endif
}

void loop() {
#if MODE == 0
  runMaze();

#elif MODE == 1
  readAll();
  Serial.print(F("F ")); Serial.print(dF, 1);
  Serial.print(F("\tL raw ")); Serial.print(rawL, 1);
  Serial.print(F(" side ")); Serial.print(dL, 1);
  Serial.print(F("\tR raw ")); Serial.print(rawR, 1);
  Serial.print(F(" side ")); Serial.print(dR, 1);
  Serial.print(F("\tIR ")); Serial.print(digitalRead(IR_PIN));
#ifdef USE_SIDE_SWITCH
  Serial.print(F("\tSW ")); Serial.print(digitalRead(SIDE_SWITCH_PIN) == HIGH ? F("LEFT") : F("RIGHT"));
#endif
  Serial.print(F("\t[")); Serial.print(sideOpenL() ? 'L' : '-');
  Serial.print(dF > FRONT_OPEN_CM ? 'S' : '-');
  Serial.print(sideOpenR() ? 'R' : '-'); Serial.println(']');
  delay(120);

#elif MODE == 2
  Serial.println(F("LEFT wheel forward"));  setMotors(BASE_PWM, 0); delay(1000); stopMotors(); delay(700);
  Serial.println(F("RIGHT wheel forward")); setMotors(0, BASE_PWM); delay(1000); stopMotors(); delay(700);
  Serial.println(F("BOTH forward"));        setMotors(BASE_PWM, BASE_PWM); delay(1000); stopMotors(); delay(2500);

#elif MODE == 3
  Serial.println(F("LEFT 90"));  pivot(+1, TURN90_MS);  delay(1500);
  Serial.println(F("RIGHT 90")); pivot(-1, TURN90_MS);  delay(1500);
  Serial.println(F("180"));      pivot(+1, TURN180_MS); delay(1500);
#ifdef ARC_TURNS
  Serial.println(F("ARC LEFT"));  arcTurn(+1); stopMotors(); delay(1500);
  Serial.println(F("ARC RIGHT")); arcTurn(-1); stopMotors(); delay(3000);
#endif

#elif MODE == 4

  if (!finished && !stopped) {
    readAll();
    if (dF < FRONT_CENTRE_CM) { brake(BASE_PWM, BASE_PWM); finished = true; }
    else steer(dF < FRONT_SLOW_CM ? SLOW_PWM : BASE_PWM);
  } else stopMotors();
#endif
}
