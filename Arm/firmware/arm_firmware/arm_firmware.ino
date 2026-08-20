/* ===========================================================================
   arm_firmware.ino  -  3-DOF robotic arm, 3 x 28BYJ-48 via ULN2003
   ---------------------------------------------------------------------------
   Target : Arduino Uno
   Wiring : BASE   ULN2003 IN1..IN4 -> D2  D3  D4  D5
            MIDDLE ULN2003 IN1..IN4 -> D6  D7  D8  D9
            UPPER  ULN2003 IN1..IN4 -> D10 D11 D12 D13
            External 5V -> ULN2003 motor supply. GND common with Arduino.

   Why not Stepper.h:
       Stepper::step() busy-waits until the whole move finishes. During that
       time the serial port is never read, so an emergency stop cannot arrive.
       This firmware advances ONE step at a time from loop(), so STOP is
       always honoured within a single step interval.

   Protocol: newline-terminated ASCII. Every reply starts "OK " or "ERR ".
             Unsolicited events start "EV ".
   =========================================================================== */

#define FW_NAME    "ARM3DOF"
#define FW_VERSION "1.0"
#define NJOINTS    3

/* ---------- half-step sequence, bit0=IN1 bit1=IN2 bit2=IN3 bit3=IN4 ------- */
static const uint8_t HALFSTEP[8] = {
  0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001
};

/* ---------- per-joint state ---------------------------------------------- */
struct Joint {
  uint8_t  pin[4];
  long     pos;              // current position in steps (0 == home)
  long     target;           // commanded position in steps
  float    stepsPerRev;      // calibration: steps for one JOINT revolution
  float    gearRatio;        // extra reduction between motor and joint
  int8_t   dir;              // +1 or -1, flips physical sense
  float    minDeg, maxDeg;   // soft limits
  unsigned long lastStepUs;
  bool     energised;
};

Joint J[NJOINTS];

/* ---------- global state -------------------------------------------------- */
unsigned long stepIntervalUs = 8000UL;   // 125 steps/s ~ 1.8 RPM. Deliberately slow.
unsigned long minIntervalUs  = 1200UL;   // below this a 28BYJ-48 stalls
float         maxDeltaDeg    = 10.0f;    // biggest change accepted per command
bool          estopped       = false;
bool          armed          = false;
bool          holdTorque     = false;
unsigned long watchdogMs     = 4000UL;   // 0 = disabled
unsigned long lastCmdMs      = 0;

char    buf[72];
uint8_t buflen = 0;

/* ---------- helpers ------------------------------------------------------- */
static inline float stepsPerDeg(const Joint &j) {
  return (j.stepsPerRev * j.gearRatio) / 360.0f;
}
static inline float jointAngle(const Joint &j) {
  return ((float)j.pos * (float)j.dir) / stepsPerDeg(j);
}
static inline long angleToSteps(const Joint &j, float deg) {
  return (long)lroundf(deg * stepsPerDeg(j)) * (long)j.dir;
}

void writeCoils(Joint &j, uint8_t mask) {
  for (uint8_t i = 0; i < 4; i++) digitalWrite(j.pin[i], (mask >> i) & 1);
}
void releaseCoils(Joint &j) { writeCoils(j, 0); j.energised = false; }
void releaseAll() { for (uint8_t i = 0; i < NJOINTS; i++) releaseCoils(J[i]); }

bool anyMoving() {
  for (uint8_t i = 0; i < NJOINTS; i++) if (J[i].pos != J[i].target) return true;
  return false;
}

void doEstop(const __FlashStringHelper *why) {
  for (uint8_t i = 0; i < NJOINTS; i++) J[i].target = J[i].pos;   // freeze in place
  releaseAll();
  estopped = true;
  armed    = false;
  Serial.print(F("EV ESTOP "));
  Serial.println(why);
}

/* ---------- setup --------------------------------------------------------- */
void setupJoint(uint8_t idx, uint8_t p0, uint8_t p1, uint8_t p2, uint8_t p3) {
  Joint &j = J[idx];
  j.pin[0] = p0; j.pin[1] = p1; j.pin[2] = p2; j.pin[3] = p3;
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(j.pin[i], OUTPUT);
    digitalWrite(j.pin[i], LOW);
  }
  j.pos = 0; j.target = 0;
  j.stepsPerRev = 4076.0f;   // exact 28BYJ-48 half-step: 63.68395 gearbox * 64
  j.gearRatio   = 1.0f;
  j.dir         = 1;
  j.minDeg      = -10.0f;    // safe default; host overwrites via LIM
  j.maxDeg      =  10.0f;
  j.lastStepUs  = 0;
  j.energised   = false;
}

void setup() {
  Serial.begin(115200);
  setupJoint(0,  2,  3,  4,  5);   // BASE
  setupJoint(1,  6,  7,  8,  9);   // MIDDLE
  setupJoint(2, 10, 11, 12, 13);   // UPPER
  releaseAll();
  lastCmdMs = millis();
  Serial.print(F("EV READY "));
  Serial.print(F(FW_NAME));
  Serial.print(' ');
  Serial.println(F(FW_VERSION));
}

/* ---------- motion -------------------------------------------------------- */
void serviceMotors() {
  if (estopped) return;
  unsigned long now = micros();
  for (uint8_t i = 0; i < NJOINTS; i++) {
    Joint &j = J[i];
    if (j.pos == j.target) continue;
    if ((unsigned long)(now - j.lastStepUs) < stepIntervalUs) continue;
    j.lastStepUs = now;
    j.pos += (j.target > j.pos) ? 1 : -1;
    uint8_t phase = (uint8_t)(((j.pos % 8) + 8) % 8);
    writeCoils(j, HALFSTEP[phase]);
    j.energised = true;
  }
  if (!holdTorque && !anyMoving()) {
    for (uint8_t i = 0; i < NJOINTS; i++) if (J[i].energised) releaseCoils(J[i]);
  }
}

void serviceWatchdog() {
  if (watchdogMs == 0 || estopped) return;
  if (!anyMoving()) return;
  if ((unsigned long)(millis() - lastCmdMs) > watchdogMs) doEstop(F("WATCHDOG"));
}

/* ---------- command parsing ----------------------------------------------- */
char *nextTok(char **p) {
  while (**p == ' ') (*p)++;
  if (**p == 0) return NULL;
  char *s = *p;
  while (**p && **p != ' ') (*p)++;
  if (**p) { **p = 0; (*p)++; }
  return s;
}

void printPos() {
  Serial.print(F("OK POS"));
  for (uint8_t i = 0; i < NJOINTS; i++) {
    Serial.print(' ');
    Serial.print(jointAngle(J[i]), 3);
  }
  Serial.print(F(" busy "));  Serial.print(anyMoving() ? 1 : 0);
  Serial.print(F(" estop ")); Serial.print(estopped ? 1 : 0);
  Serial.print(F(" armed ")); Serial.println(armed ? 1 : 0);
}

void handle(char *line) {
  lastCmdMs = millis();
  char *p = line;
  char *cmd = nextTok(&p);
  if (!cmd) return;
  for (char *q = cmd; *q; q++) *q = toupper(*q);

  /* --- always available, even while e-stopped --- */
  if (!strcmp(cmd, "PING")) { Serial.println(F("OK PONG")); return; }

  if (!strcmp(cmd, "ID")) {
    Serial.print(F("OK "));          Serial.print(F(FW_NAME));
    Serial.print(' ');               Serial.print(F(FW_VERSION));
    Serial.print(F(" JOINTS "));     Serial.println(NJOINTS);
    return;
  }

  if (!strcmp(cmd, "STOP")) {
    doEstop(F("COMMANDED"));
    Serial.println(F("OK STOP"));
    return;
  }

  if (!strcmp(cmd, "GET")) { printPos(); return; }

  if (!strcmp(cmd, "RESUME")) {
    for (uint8_t i = 0; i < NJOINTS; i++) J[i].target = J[i].pos;
    estopped = false;
    armed    = false;            // deliberate: must re-arm after an e-stop
    Serial.println(F("OK RESUME"));
    return;
  }

  if (!strcmp(cmd, "ARM")) {
    char *a = nextTok(&p);
    if (!a) { Serial.println(F("ERR ARGS")); return; }
    if (estopped) { Serial.println(F("ERR ESTOPPED")); return; }
    armed = (atoi(a) != 0);
    Serial.print(F("OK ARM ")); Serial.println(armed ? 1 : 0);
    return;
  }

  /* --- configuration --- */
  if (!strcmp(cmd, "SPEED")) {                 // SPEED <microseconds per step>
    char *a = nextTok(&p);
    if (!a) { Serial.println(F("ERR ARGS")); return; }
    unsigned long v = strtoul(a, NULL, 10);
    if (v < minIntervalUs) v = minIntervalUs;  // clamp, never fail open
    stepIntervalUs = v;
    Serial.print(F("OK SPEED ")); Serial.println(stepIntervalUs);
    return;
  }

  if (!strcmp(cmd, "CAL")) {                   // CAL <i> <steps_per_rev> <gear> <dir>
    char *a = nextTok(&p); char *b = nextTok(&p);
    char *c = nextTok(&p); char *d = nextTok(&p);
    if (!a || !b || !c || !d) { Serial.println(F("ERR ARGS")); return; }
    int i = atoi(a);
    if (i < 0 || i >= NJOINTS) { Serial.println(F("ERR INDEX")); return; }
    float spr = atof(b), gr = atof(c);
    if (spr < 1.0f || gr <= 0.0f) { Serial.println(F("ERR RANGE")); return; }
    J[i].stepsPerRev = spr;
    J[i].gearRatio   = gr;
    J[i].dir         = (atoi(d) < 0) ? -1 : 1;
    J[i].pos = 0; J[i].target = 0;             // recalibration invalidates position
    Serial.print(F("OK CAL ")); Serial.println(i);
    return;
  }

  if (!strcmp(cmd, "LIM")) {                   // LIM <i> <min_deg> <max_deg>
    char *a = nextTok(&p); char *b = nextTok(&p); char *c = nextTok(&p);
    if (!a || !b || !c) { Serial.println(F("ERR ARGS")); return; }
    int i = atoi(a);
    if (i < 0 || i >= NJOINTS) { Serial.println(F("ERR INDEX")); return; }
    float lo = atof(b), hi = atof(c);
    if (hi < lo) { Serial.println(F("ERR RANGE")); return; }
    J[i].minDeg = lo; J[i].maxDeg = hi;
    Serial.print(F("OK LIM ")); Serial.println(i);
    return;
  }

  if (!strcmp(cmd, "DELTA")) {                 // DELTA <deg>
    char *a = nextTok(&p);
    if (!a) { Serial.println(F("ERR ARGS")); return; }
    float v = atof(a);
    if (v <= 0.0f) { Serial.println(F("ERR RANGE")); return; }
    maxDeltaDeg = v;
    Serial.print(F("OK DELTA ")); Serial.println(maxDeltaDeg, 3);
    return;
  }

  if (!strcmp(cmd, "HOLD")) {                  // HOLD <0|1>
    char *a = nextTok(&p);
    if (!a) { Serial.println(F("ERR ARGS")); return; }
    holdTorque = (atoi(a) != 0);
    if (!holdTorque && !anyMoving()) releaseAll();
    Serial.print(F("OK HOLD ")); Serial.println(holdTorque ? 1 : 0);
    return;
  }

  if (!strcmp(cmd, "WD")) {                    // WD <milliseconds>, 0 disables
    char *a = nextTok(&p);
    if (!a) { Serial.println(F("ERR ARGS")); return; }
    watchdogMs = strtoul(a, NULL, 10);
    Serial.print(F("OK WD ")); Serial.println(watchdogMs);
    return;
  }

  if (!strcmp(cmd, "ZERO")) {                  // adopt the current pose as home
    if (anyMoving()) { Serial.println(F("ERR BUSY")); return; }
    for (uint8_t i = 0; i < NJOINTS; i++) { J[i].pos = 0; J[i].target = 0; }
    Serial.println(F("OK ZERO"));
    return;
  }

  /* --- motion: blocked while e-stopped or disarmed --- */
  if (!strcmp(cmd, "MOVEJ")) {                 // MOVEJ <i> <deg>
    if (estopped) { Serial.println(F("ERR ESTOPPED")); return; }
    if (!armed)   { Serial.println(F("ERR DISARMED")); return; }
    char *a = nextTok(&p); char *b = nextTok(&p);
    if (!a || !b) { Serial.println(F("ERR ARGS")); return; }
    int i = atoi(a);
    if (i < 0 || i >= NJOINTS) { Serial.println(F("ERR INDEX")); return; }
    float deg = atof(b);
    Joint &j = J[i];
    if (deg < j.minDeg || deg > j.maxDeg) { Serial.println(F("ERR LIMIT")); return; }
    if (fabsf(deg - jointAngle(j)) > maxDeltaDeg + 1e-4f) {
      Serial.println(F("ERR DELTA")); return;
    }
    j.target = angleToSteps(j, deg);
    printPos();
    return;
  }

  if (!strcmp(cmd, "MOVE")) {                  // MOVE <a0> <a1> <a2>, absolute degrees
    if (estopped) { Serial.println(F("ERR ESTOPPED")); return; }
    if (!armed)   { Serial.println(F("ERR DISARMED")); return; }
    float want[NJOINTS];
    for (uint8_t i = 0; i < NJOINTS; i++) {
      char *a = nextTok(&p);
      if (!a) { Serial.println(F("ERR ARGS")); return; }
      want[i] = atof(a);
    }
    /* Validate every joint BEFORE moving any, so a rejected joint moves nothing. */
    for (uint8_t i = 0; i < NJOINTS; i++) {
      Joint &j = J[i];
      if (want[i] < j.minDeg || want[i] > j.maxDeg) {
        Serial.print(F("ERR LIMIT J")); Serial.println(i); return;
      }
      if (fabsf(want[i] - jointAngle(j)) > maxDeltaDeg + 1e-4f) {
        Serial.print(F("ERR DELTA J")); Serial.println(i); return;
      }
    }
    for (uint8_t i = 0; i < NJOINTS; i++) J[i].target = angleToSteps(J[i], want[i]);
    printPos();
    return;
  }

  Serial.print(F("ERR UNKNOWN ")); Serial.println(cmd);
}

/* ---------- loop ---------------------------------------------------------- */
void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf[buflen] = 0;
      if (buflen > 0) handle(buf);
      buflen = 0;
    } else if (buflen < sizeof(buf) - 1) {
      buf[buflen++] = c;
    } else {
      buflen = 0;                              // overlong line: drop it
      Serial.println(F("ERR OVERFLOW"));
    }
  }
  serviceMotors();
  serviceWatchdog();
}
