/*
  MAZE RUNNER v24 - for your robot (Uno + Sensor Shield + L298N + 3x HC-SR04 + IR)
  ---------------------------------------------------------------------------
  Maze: 20 x 20 cm cells, white floor, black stripe in front of the parking zone.

  HOW IT DRIVES - wall follower (switch on A0: OFF = left wall, ON = right wall)
   - Corridor: hugs the followed wall.
   - Followed wall opens -> turns into it (arc until the wall is seen again).
   - Wall ahead -> pivots away from the followed wall until the front is clear.
   - Openings only count with REAL far echoes; a missing echo is "unknown".
   - Wall guard: swerves away if any wall gets dangerously close.
   - IR on A5 sees the black stripe -> crosses it -> drives in -> stops.

  YOUR WIRING (from your last chat)
   L298N  ENA=D11 IN1=D2 IN2=D3  -> RIGHT wheel
          ENB=D10 IN3=D4 IN4=D5  -> LEFT wheel   (ENA/ENB jumper caps removed)
   Front  TRIG=D6  ECHO=D7
   Left   TRIG=D8  ECHO=D9
   Right  TRIG=D13 ECHO=D12
   IR     OUT=A5   (0 = white floor, 1 = black stripe)

  TEST ORDER (Serial Monitor 115200, wheels lifted for 2 and 3)
   MODE 1 sensors -> MODE 2 motors -> MODE 3 turns -> MODE 4 corridor -> MODE 0 run
   Send 'x' at any time to stop the motors.
*/

// =================== MODE ===================
// 0 = run maze, 1 = print sensors, 2 = motor test, 3 = turn test, 4 = corridor test
#define MODE 0

// =================== PINS ===================
const uint8_t ENA = 11, IN1 = 2, IN2 = 3;   // right wheel
const uint8_t ENB = 10, IN3 = 4, IN4 = 5;   // left wheel
const uint8_t TRIG_F = 6,  ECHO_F = 7;
const uint8_t TRIG_L = 8,  ECHO_L = 9;
const uint8_t TRIG_R = 13, ECHO_R = 12;
const uint8_t IR_PIN = A5;
const uint8_t IR_BLACK = HIGH;

// =================== START / WALL SIDE ===================
// Best wall per start (simulated on your maze picture, squares driven):
//   Red A  : either (24)   switch either   Blue A : either (24)  switch either
//   Red B  : LEFT   (20)   switch OFF      Blue B : RIGHT  (20)  switch ON
//   Red C  : RIGHT  (16)   switch ON       Blue C : LEFT   (16)  switch OFF
// Place the robot facing the only open side of its start square
// (Red A: right, Blue A: left, Red C / Blue C: up, Red B / Blue B: down).
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
const uint8_t START = 0;    // <-- 0..5, only used when USE_SIDE_SWITCH is off

// Switch / jumper: one leg -> S at A0, other leg -> G.
//   OFF (open)   -> follow LEFT wall
//   ON  (closed) -> follow RIGHT wall
// Read once at power-up/reset, so bumping it during a run changes nothing.
#define USE_SIDE_SWITCH
const uint8_t SIDE_SWITCH_PIN = A0;

// =================== SENSOR GEOMETRY ===================
// Angle of the side sensors from straight ahead: 45 = diagonal (now), 90 = sideways
const float SIDE_ANGLE_DEG = 90;

// =================== TUNING ===================
// MASTER SPEED: scales EVERY motor command (driving, turns, guard, unjam).
// 100 = as tuned below. Lower = whole robot slower. Go down in steps of 10;
// if a wheel stops turning on the floor, go back up one step.
// Changing it changes how far timed turns go, so re-check ARC90_MS afterwards.
int SPEED_PCT = 78;

int BASE_PWM  = 78;    // corridor speed - raise once it's reliable
int SLOW_PWM  = 60;    // when a front wall is close (below ~60-70 TT motors may stall)
int TURN_PWM  = 78;    // pivot speed (50 was too weak: robot only wiggled at walls)
int MIN_PWM   = 50;     // a wheel never goes slower than this while steering

// LOW-SPEED POWER
// 1) Low PWM frequency: the Uno's default ~490 Hz PWM is too fast for small TT
//    motors, so at low duty they get weak and buzz. ~122 Hz gives each pulse
//    time to build real current = much more push at low speed.
#define LOW_FREQ_PWM
// 2) Kick-start: when a wheel starts from standstill (or reverses) at a low
//    speed, give it a short full-power pulse to break friction, then drop to
//    the requested speed.
const int KICK_BELOW = 140;  // only kick when the requested speed is below this
const int KICK_MS    = 15;   // length of the full-power pulse
int LEFT_TRIM = 0;     // + if the robot curves left when it should go straight
int RIGHT_TRIM = 0;    // + if it curves right

// Distances in cm. Side values are "sideways" distances (already corrected for 45 deg).
// Calibrate with MODE 1: put the robot in the centre of a corridor and read SIDE.
float SIDE_CENTRE_CM = 8.0;  // sideways distance to a wall when centred
float WALL_CM        = 13.0;  // side below this = wall there
float OPEN_CM        = 14.0; // side above this = opening
float FRONT_CENTRE_CM = 7.0; // front reading when robot is at cell centre facing a wall
float FRONT_OPEN_CM  = 16.0; // front above this = can go straight
float FRONT_SLOW_CM  = 18.0;
float MAX_CM         = 80.0;

// AUTO CALIBRATE: at power-up the robot measures how far the followed wall is
// and uses that as SIDE_CENTRE_CM, then sets WALL_CM / OPEN_CM from it.
// Place the robot CENTRED in the start square before switching on.
#define AUTO_CALIBRATE

// Steering (PD) - SIDE_CENTRE_CM is the hugging distance
float KP = 8.0;   // was 14: far too strong at BASE 100 -> swerved into the wall
float KD = 10.0;
int   MAX_CORR = 40;   // was 80: let one wheel nearly stop -> sharp dive into wall
float MAX_ERR_CM = 4.0; // one bad (45 deg) reading can only push this hard

// WALL GUARD - overrides everything else. If a side sensor says a wall is
// dangerously close, the robot immediately swerves away from it, whatever the
// steering / arc / wall-following logic wanted. Auto-calibration sets it to
// 60% of the normal hugging distance.
float DANGER_SIDE_CM = 1.5;   // sideways distance that counts as "about to hit"
int   GUARD_FAST = 60;       // wheel on the wall side
int   GUARD_SLOW = 0;         // wheel on the far side (0 = swing away sharply)

// Timed moves (ms) - calibrate with MODE 3/4
int GAP_FWD_MS  = 180;  // after a side opening appears, drive this to reach cell centre
int EXIT_FWD_MS = 300;  // after a turn: blind drive time, used only when the front sensor sees no wall

// LEAVING THE JUNCTION AFTER A TURN (measured by the front sensor, not by time)
// After a turn the robot ignores side openings until the wall ahead has come
// EXIT_DIST_CM closer, i.e. until it has really driven out of the junction
// square. Without this, a slow robot is still inside the junction when the
// timer ends, sees the old straight-ahead path (or the corridor it came from)
// as an "opening", and turns again.
float EXIT_DIST_CM = 10.0;  // raise if it still turns twice at one junction, lower if it overshoots the next opening
int   EXIT_MAX_MS  = 1500;  // safety limit for that drive
int TURN90_MS   = 200;  // pivot 90 deg at TURN_PWM (used when a wall is ahead)

// Smooth arc turn into an opening on the followed side (no stopping).
// Inner wheel almost still, outer wheel fast -> robot swings around the inner wheel.
#define ARC_TURNS               // comment out to use stop-and-pivot turns instead
int ARC_OUTER_PWM = 80; // was 220 (much faster than BASE) -> violent spin into the wall
int ARC_INNER_PWM = 0;   // 0 = turn around the inner wheel; small + = wider arc
int ARC90_MS      = 200; // time for a 90 deg arc - calibrate in MODE 3. EVERY arc turn is sized from this.
int ARC_GAP_MS    = 60;  // drive this long after the opening is seen, then start the arc

// SENSOR-GUIDED TURNS (no fixed angle)
// Arc into an opening: about 90 deg, fine-tuned by the side sensor.
// The limits are a percentage of ARC90_MS, so one turn can never become a U-turn.
// (Before: fixed 500..650 ms. Your robot spins 180 deg in ~400 ms, so every arc
//  was at least a half turn unless a wall stopped it early.)
int ARC_MIN_PCT  = 95;   // ignore the side sensor until 70% of a 90 deg turn (skips the corner post)
int ARC_MAX_PCT  = 130;  // hard stop at 130% of a 90 deg turn (~115 deg)
// Pivot at a front wall: keep pivoting until the front is clear.
int PIVOT_MIN_MS   = 140; // don't check the front before this (~70% of a 90 deg pivot)
int PIVOT_EXTRA_MS = 30;  // keep turning a little after "clear" to finish squaring up
int PIVOT_MAX_MS   = 700; // safety limit (a bit more than a 180 deg pivot)
int TURN180_MS  = 600;  // pivot 180 deg
int PARK_FWD_MS = 1000;  // after crossing the stripe, drive this far into the zone
int START_DELAY_MS = 2000;

const unsigned long ECHO_TIMEOUT_US = 6000; // ~100 cm, keeps the loop fast
const uint8_t PING_GAP_MS = 2;
const float   TOO_CLOSE_CM = 2.5;     // readings below this are clamped
const uint8_t NO_ECHO_CONFIRM = 4;    // misses in a row before "no wall"
const float   NEAR_HOLD_CM = 12.0;    // below this, a lost echo keeps the close value
// STUCK DETECTOR: if the AVERAGE of the sensor readings stays still while the
// motors are driving, the robot isn't really moving -> back up a little.
// The three sensor readings together must change by more than STILL_CM, or
// after STUCK_COUNT readings in a row the robot counts as stuck.
// Readings are now counted EVERYWHERE (straights, turns, pivots), about 40-100
// per second, so 30 is roughly half a second. Too low = it "unjams" while
// driving normally. (5 only worked before because one loop counted once.)
const float   STILL_CM       = 1.5;
const uint8_t STUCK_COUNT    = 9;
const int     UNJAM_PWM      = 160;   // when stuck: speed of the ONE wheel that reverses
const int     UNJAM_MS       = 200;   // when stuck: how long that wheel reverses
const float   UNJAM_DIFF_CM  = 1.0;   // sides must differ by this much to pick a side
const int     BACKUP_MS      = 120;   // only if both sides look the same: tiny straight back-up
const int     STUCK_AGAIN_MS = 2000;  // stuck again this soon -> unjam twice as long
const uint8_t OPEN_CONFIRM = 1;       // "open" readings in a row before turning

// =================== STATE ===================
float dF = MAX_CM, dL = MAX_CM, dR = MAX_CM;   // front raw, sides sideways-corrected
float rawL = MAX_CM, rawR = MAX_CM;
float lastErr = 0;
float sideFactor;                 // sin(angle)
const StartConfig* cfg;           // pointer to the selected start
bool followLeft;
bool finished = false;
bool stopped = false;
bool sawWhite = false;
uint8_t blackCount = 0;
uint8_t missF = 0, missL = 0, missR = 0;   // consecutive no-echo counts
uint8_t openL = 0, openR = 0;              // consecutive "opening" counts
unsigned long stuckSince = 0, lastShake = 0;
uint8_t stillCount = 0;                    // readings in a row with no change
bool stuckFlag = false;                    // set by trackStuck(), cleared after the unjam
float refF = -100, refL = -100, refR = -100;
float stuckRef = -100;

// ---------------------------------------------------------------- safety
void checkSerialStop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'x' || c == 'X') stopped = true;
  }
}

// ---------------------------------------------------------------- motors
// left/right: -255..255, + = forward
void writeMotors(int left, int right);
int lastLeft = 0, lastRight = 0;

void waitMs(unsigned long ms);

void setMotors(int left, int right) {
  if (stopped) { left = 0; right = 0; }
  left  = (int)((long)left  * SPEED_PCT / 100);
  right = (int)((long)right * SPEED_PCT / 100);
  // Kick-start only when the whole robot was standing still, or when a wheel
  // reverses. (Before: the inner wheel of a turn got a full-power kick every
  // time it restarted, which jerked the robot forward and widened the turn.)
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
  // From your test: forward = IN1 LOW / IN2 HIGH (right), IN3 LOW / IN4 HIGH (left)
  digitalWrite(IN1, right >= 0 ? LOW : HIGH);
  digitalWrite(IN2, right >= 0 ? HIGH : LOW);
  analogWrite(ENA, abs(right));
  digitalWrite(IN3, left >= 0 ? LOW : HIGH);
  digitalWrite(IN4, left >= 0 ? HIGH : LOW);
  analogWrite(ENB, abs(left));
}

void stopMotors() { setMotors(0, 0); }

void brake(int l, int r) {          // short reverse pulse kills coasting
  setMotors(-l / 2, -r / 2);
  waitMs(30);
  stopMotors();
  waitMs(40);
}

// ---------------------------------------------------------------- IR stripe
// The stripe is REMEMBERED once seen (stripeSeen), so it can't be lost if the
// robot crosses it during a turn, a brake or a kick-start.
bool stripeSeen = false;
void pollIR() {
  if (digitalRead(IR_PIN) == IR_BLACK) {
    if (blackCount < 255) blackCount++;
    if (sawWhite && blackCount >= 2) stripeSeen = true;
  } else {
    blackCount = 0;
    sawWhite = true;               // ignore black until we have seen white floor
  }
}
bool stripeHit() { return stripeSeen; }

// delay() that keeps watching the IR sensor
void waitMs(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) { pollIR(); delayMicroseconds(500); }
}

// ---------------------------------------------------------------- ultrasonic
float readCm(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long t = pulseIn(echo, HIGH, ECHO_TIMEOUT_US);
  if (t == 0) return -1;                 // no echo: UNKNOWN, not "far away"
  float cm = t / 58.0;
  if (cm < TOO_CLOSE_CM) cm = TOO_CLOSE_CM;   // HC-SR04 is garbage this close
  return cm;
}

// A missing echo next to a wall usually means "too close / bad angle", not
// "opening". Keep the last good value; only believe "far" after several misses.
float acceptReading(float oldV, float newV, uint8_t &misses) {
  if (newV < 0) {
    if (misses < 255) misses++;
    // If the last good reading was CLOSE, a missing echo means we are now
    // too close for the sensor (nose on the wall) - stay "close", never "far".
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
  if (raw < 0) return count;                         // no echo: unknown
  if (raw * sideFactor > OPEN_CM) return count < 255 ? count + 1 : 255;
  return 0;                                          // real wall seen
}
bool sideOpenL() { return openL >= OPEN_CONFIRM; }
bool sideOpenR() { return openR >= OPEN_CONFIRM; }

// Runs after EVERY sensor reading, wherever the robot is in its program.
// While the motors are driving, if the readings don't change for STUCK_COUNT
// readings in a row, it raises stuckFlag. The flag stays up until handled.
void trackStuck() {
  if (lastLeft == 0 && lastRight == 0) return;          // not driving: don't count
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
  // An opening must be a REAL echo from far away, seen several times.
  // A missing echo (-1) is "unknown": it never counts as open, and it doesn't
  // cancel an opening either. A real close echo cancels it immediately.
  openL = countOpen(openL, rawL);
  openR = countOpen(openR, rawR);
  trackStuck();
  checkSerialStop();
}

// ---------------------------------------------------------------- driving
// PD: hug the followed wall at SIDE_CENTRE_CM. If that wall is missing for a
// moment, hold the same distance from the other wall instead.
// Returns true if it had to swerve away from a wall (and already set motors).
bool wallGuard() {
  if (dL < DANGER_SIDE_CM && dL <= dR) {      // left wall too close -> swerve right
    setMotors(GUARD_FAST, GUARD_SLOW); lastErr = 0; return true;
  }
  if (dR < DANGER_SIDE_CM) {                  // right wall too close -> swerve left
    setMotors(GUARD_SLOW, GUARD_FAST); lastErr = 0; return true;
  }
  return false;
}

void steer(int base) {
  if (wallGuard()) return;                    // safety first
  // Hug the followed wall at SIDE_CENTRE_CM. If it's missing for a moment,
  // hold the same distance from the other wall instead.
  float fw = followLeft ? dL : dR;            // followed wall
  float ow = followLeft ? dR : dL;            // other wall
  float err;                                   // + = steer toward followed wall
  if (fw < WALL_CM)      err = fw - SIDE_CENTRE_CM;
  else if (ow < WALL_CM) err = SIDE_CENTRE_CM - ow;
  else                   err = 0;
  if (!followLeft) err = -err;                 // + err = steer LEFT
  err = constrain(err, -MAX_ERR_CM, MAX_ERR_CM);
  float corr = KP * err + KD * (err - lastErr);
  lastErr = err;
  corr = constrain(corr, -MAX_CORR, MAX_CORR);
  setMotors(constrain((int)(base - corr), MIN_PWM, 255),
            constrain((int)(base + corr), MIN_PWM, 255));
}

void park();

// drive forward with steering for ms; stops at cell centre if a wall is ahead
void driveForward(int ms, bool brakeAtEnd) {
  unsigned long t0 = millis();
  while (millis() - t0 < (unsigned long)ms && !stopped) {
    readAll();
    if (stripeHit()) { park(); return; }
    if (stuckNow()) break;                       // stuck: let runMaze unjam
    if (dF < FRONT_CENTRE_CM) break;
    steer(dF < FRONT_SLOW_CM ? SLOW_PWM : BASE_PWM);
  }
  if (brakeAtEnd) brake(BASE_PWM, BASE_PWM);
}

void pivot(int dir, int ms) {           // dir +1 = left, -1 = right
  openL = openR = 0;
  setMotors(-dir * TURN_PWM, dir * TURN_PWM);
  waitMs(ms);
  brake(-dir * TURN_PWM, dir * TURN_PWM);
  lastErr = 0;
}

// Arc turn while moving. dir +1 = left, -1 = right
void arcTurn(int dir) {
  openL = openR = 0;
  int outer = ARC_OUTER_PWM, inner = ARC_INNER_PWM;
  if (dir > 0) setMotors(inner, outer);   // left: right wheel is outer
  else         setMotors(outer, inner);
  waitMs(ARC90_MS);
  lastErr = 0;
}

// Arc toward the followed side until its wall shows up again.
// Works for 90 deg corners AND for going all the way round the end of a wall.
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
    if (stuckNow()) break;                       // stuck mid-turn: let runMaze unjam
    float fw = (dir > 0) ? dL : dR;
    if (el >= arcMin) {
      // Only after most of the turn is done may a close wall end it early.
      // (Before: these checks could end the arc after a few milliseconds, so the
      // turn was chopped into many tiny pieces with forward drives in between.)
      if (dF < FRONT_CENTRE_CM) break;
      if (dL < DANGER_SIDE_CM || dR < DANGER_SIDE_CM) break;
      // "wall is back" = anything within corridor range. (Was WALL_CM: with a
      // 45 deg sensor the new wall often never read that close, so the arc
      // kept spinning straight into the wall.)
      if (fw < OPEN_CM) { if (++seen >= 2) break; }
      else seen = 0;
    }
  }
  lastErr = 0;
}

// Pivot until the front sensor sees a clear path.
// One function for both cases: a corner stops at ~90 deg, a dead end at ~180 deg.
void pivotUntilClear(int dir) {
  openL = openR = 0;
  setMotors(-dir * TURN_PWM, dir * TURN_PWM);
  unsigned long t0 = millis();
  uint8_t clear = 0;
  while (!stopped) {
    unsigned long el = millis() - t0;
    if (el > (unsigned long)PIVOT_MAX_MS) break;
    readAll();
    if (stuckNow()) break;                       // stuck mid-pivot: let runMaze unjam
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
  while (digitalRead(IR_PIN) == IR_BLACK && millis() - t0 < 1500) {}  // cross it
  setMotors(SLOW_PWM, SLOW_PWM);       // stay slow all the way into the zone
  delay(PARK_FWD_MS);
  brake(SLOW_PWM, SLOW_PWM);
  stopMotors();
  finished = true;
}

// ---------------------------------------------------------------- wall following
// dir: +1 = left, -1 = right
int followDir() { return followLeft ? +1 : -1; }

// Average of the sensors that currently see something (ignores "far" ones).
// Returns -1 when nothing is in range (then we can't tell if we're moving).
float sensorAverage() {
  float sum = 0; uint8_t n = 0;
  if (dF < MAX_CM - 1) { sum += dF; n++; }
  if (dL < MAX_CM - 1) { sum += dL; n++; }
  if (dR < MAX_CM - 1) { sum += dR; n++; }
  return n ? sum / n : -1;
}

// Stuck: find which side is jammed against a wall (the LOWER side reading)
// and reverse ONLY the wheel on the OTHER side. The robot swings its nose away
// from the wall almost in place, instead of backing down the corridor, so it
// can't back into a junction it already passed and take the wrong way.
//   left closer  -> right wheel reverses -> nose swings right, away from left wall
//   right closer -> left wheel reverses  -> nose swings left, away from right wall
void shakeLoose() {
  bool again = lastShake != 0 && (millis() - lastShake) < (unsigned long)STUCK_AGAIN_MS;
  int ms = again ? UNJAM_MS * 2 : UNJAM_MS;
  for (uint8_t i = 0; i < 3; i++) readAll();      // fresh side readings
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
  openL = openR = 0;          // forget any "opening" seen while jammed
  lastErr = 0;
  lastShake = millis();
  stuckSince = millis(); stuckRef = -100;
}

// Called at the top of every runMaze() pass. If trackStuck() raised the flag
// (anywhere: straight, arc, pivot), unjam now. Afterwards everything is reset,
// so it triggers again if the robot is still stuck.
bool checkStuck() {
  if (!stuckFlag) return false;
  shakeLoose();
  stuckFlag = false;
  stillCount = 0;
  refF = refL = refR = -100;
  return true;
}

// Blind drive out of the junction square after a turn. No turn decisions are
// made in here. It ends when the wall ahead is EXIT_DIST_CM closer than it was
// right after the turn. If the front sensor has nothing to measure, it falls
// back to the old fixed time (EXIT_FWD_MS).
void leaveJunction() {
  for (uint8_t i = 0; i < 3; i++) readAll();
  float f0 = dF;
  bool useDist = f0 < MAX_CM - 1;                 // a real wall is in range ahead
  unsigned long limit = useDist ? (unsigned long)EXIT_MAX_MS : (unsigned long)EXIT_FWD_MS;
  unsigned long t0 = millis();
  uint8_t reached = 0;
  while (millis() - t0 < limit && !stopped) {
    readAll();
    if (stripeHit()) { park(); return; }
    if (stuckNow()) break;                        // stuck: let runMaze unjam
    if (dF < FRONT_CENTRE_CM) break;              // wall ahead: done
    if (useDist) {
      if (dF <= f0 - EXIT_DIST_CM) { if (++reached >= 2) break; }
      else reached = 0;
    }
    steer(dF < FRONT_SLOW_CM ? SLOW_PWM : BASE_PWM);
  }
  // Forget openings seen while still inside the junction: from here on a side
  // only counts as open after a fresh far echo.
  openL = openR = 0;
}

// Turn into an opening on side dir (+1 = left, -1 = right), then drive out of
// the junction square.
void turnInto(int dir) {
#ifdef ARC_TURNS
  if (dF < FRONT_CENTRE_CM) {
    // wall right ahead: no room for an arc, turn on the spot into the opening
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

// TURN PRIORITY (switch on A0 picks the first side: OFF = left, ON = right)
//   1st: opening on the followed side      -> turn into it
//   2nd: opening on the OTHER side         -> turn into it (even if straight is open)
//   3rd: straight ahead
//   nothing open                           -> turn around (dead end)
// Simulated on your maze picture: 16 squares (the shortest path) from all 6
// starts, with no dead ends, for both switch positions.
void runMaze() {
  if (finished || stopped) { stopMotors(); return; }
  readAll();
  if (stripeHit()) { park(); return; }
  if (checkStuck()) return;

  bool fwOpen = followLeft ? sideOpenL() : sideOpenR();   // followed side
  bool owOpen = followLeft ? sideOpenR() : sideOpenL();   // other side

  if (fwOpen || owOpen) {
    // 1) + 2) A side opened -> roll to the turn point and look again
    driveForward(ARC_GAP_MS, false);
    if (finished || stopped) return;
    readAll();
    fwOpen = followLeft ? sideOpenL() : sideOpenR();
    owOpen = followLeft ? sideOpenR() : sideOpenL();
    if (fwOpen) {                                  // 1st priority
      Serial.println(followLeft ? F("TURN LEFT (1st)") : F("TURN RIGHT (1st)"));
      turnInto(followDir());
    } else if (owOpen) {                           // 2nd priority
      Serial.println(followLeft ? F("TURN RIGHT (2nd)") : F("TURN LEFT (2nd)"));
      turnInto(-followDir());
    }                                              // neither: false alarm, keep driving
  }
  else if (dF < FRONT_CENTRE_CM) {
    // Wall ahead and no side opening seen -> turn away from the followed wall
    // until the front is clear (~90 deg at a corner, ~180 deg at a dead end).
    stopMotors();
    Serial.println(F("WALL AHEAD -> turn away"));
    pivotUntilClear(-followDir());
  }
  else {
    // 3) Straight: hug the followed wall, slow down near a front wall
    steer(dF < FRONT_SLOW_CM ? SLOW_PWM : BASE_PWM);
  }
}

// ---------------------------------------------------------------- setup / loop
void setup() {
  Serial.begin(115200);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
#ifdef LOW_FREQ_PWM
  // Timer1 -> D10 (ENB), Timer2 -> D11 (ENA): prescaler 256 = ~122 Hz.
  // Timer0 (millis/delay) is untouched.
  TCCR1B = (TCCR1B & 0b11111000) | 0x04;
  TCCR2B = (TCCR2B & 0b11111000) | 0x06;
#endif
  stopMotors();                                   // motors off first thing
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
  followLeft = (digitalRead(SIDE_SWITCH_PIN) == HIGH);   // OFF = left, ON = right
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
    if (fw > 1.0 && fw < 15.0) {          // a real wall on the followed side
      SIDE_CENTRE_CM = fw;
      WALL_CM = fw + 6.0;                 // wall still there
      OPEN_CM = fw + 10.0;                // next wall is a whole square away
      DANGER_SIDE_CM = fw * 0.6;          // closer than this = about to hit
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
  // put it in a straight corridor: drives and centres until a wall is ahead
  if (!finished && !stopped) {
    readAll();
    if (dF < FRONT_CENTRE_CM) { brake(BASE_PWM, BASE_PWM); finished = true; }
    else steer(dF < FRONT_SLOW_CM ? SLOW_PWM : BASE_PWM);
  } else stopMotors();
#endif
}
