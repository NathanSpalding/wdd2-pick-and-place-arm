/*
  WDD2 arm firmware (ESP32-S3, Arduino-ESP32 core 3.x)
  ====================================================
  Runs the arm from the GUI (ui_app.py). One text line per message, newline-terminated.

  PC -> ESP
    J,b,s,e,w   joint targets in degrees (GUI sends 50 Hz while mirroring)
    G,1 / G,0   close / open the gripper
    S           software stop: hold position, ignore J until R
    R           reset / re-arm (after S, a fault or a released physical E-stop)
    M,j,deg,v   test move without the GUI: joint j (0-3) to deg at v deg/s (S-curve)
    D,j,gain    active damping gain for joint j (s, 0 = off), e.g. D,0,0.03 ; D alone lists them
    Q,j,amp     re-settle for joint j: if it hunts while holding, push amp deg past the
                target and come back slowly (sign = side), e.g. Q,0,1 ; 0 = off ; Q lists
    O,j[,s]     measure joint j's swing/noise (peak-to-peak, std) over s seconds (default 5)
    X,j,0|1     trim loop off/on for joint j (step tests); X alone lists
    V,0 / V,1   A and I lines off / on. OFF after every reset, so the Serial Monitor is quiet;
                the GUI sends V,1 when it connects. E changes and all replies always print.
    T           print I2C/loop timing and J-stream statistics, then reset the counters
    P           print status of every joint (target, measured, trim, pulse)
    H           help

  ESP -> PC
    A,b,s,e,w   measured joint angles in degrees (AS5600, 50 Hz)
    I,amps      gripper servo current (INA219, 50 Hz)
    E,1 / E,0   physical E-stop pressed / released (on change and every 1 s)
    K,1         grasp confirmed        K,0  grasp failed (gripper re-opened)
    K,2         grip lost while holding
    F,joint,why fault (ENC = encoder not answering, TRACK = joint far from target);
                arm holds, J ignored until R
    # ...       information for a human; the GUI shows it as the last message

  Control (200 Hz loop)
    1. Read the 4 AS5600 encoders through the TCA9548A and the INA219 (I2C at 400 kHz).
    2. S-curve trajectory for each joint (not the gripper):
       a. rebuild a smooth target from the J stream: the latest J plus its velocity,
          extrapolated for at most 40 ms (covers late or bunched J messages),
          rate-limited to MAX_RATE (SYNC_RATE right after arming or a watchdog hold);
       b. two moving averages of 80 ms in series. A moving average turns a velocity
          step into a ramp (bounded acceleration); two in series also ramp the
          acceleration (bounded jerk), so every start and stop follows an S-curve.
          It never overshoots, and adds 80 ms of delay while moving (none at rest).
    3. Outer trim loop: once a joint's target has been still for SETTLE_MS, integrate
       the encoder error beyond a deadband into a bounded trim (degrees).
    4. pulse = P0 + (command + trim) / K   (calibration fit from the bring-up tests).
    Each servo still closes its own fast position loop on its potentiometer; the
    trim loop is a slow outer loop that removes steady-state error (hysteresis,
    load sag) using the joint encoders.

  Safety
    - No pulses at power-up: the servos stay limp until the first J or R.
    - Arming starts from the measured angles, so the arm does not jump.
    - Watchdog: no J for 200 ms -> hold the last target; resume slowly.
    - Physical E-stop (GPIO 41 HIGH): pulses stop at once (the relay also cuts the
      servo rails). Releasing it leaves the servos limp; nothing moves until a new
      J or R, and the GUI blocks J until its E-stop is reset (which sends R).
    - Encoder failure or a large tracking error -> hold and report F.
    - Gripper pulse is always limited to 950-1575 us here, whatever is sent.
*/

#include <Wire.h>
#include <esp_arduino_version.h>

// ---------------- pins and bus ----------------
const int SDA_PIN = 39, SCL_PIN = 40, ESTOP_PIN = 41;
const uint8_t MUX_ADDR = 0x70, AS5600_ADDR = 0x36, INA219_ADDR = 0x40;
const float SHUNT_OHM = 0.1f;
const uint32_t I2C_HZ = 400000;

const int NJ = 4;                                   // base, shoulder, elbow, wrist
const int GRIP = 4;
const int SERVO_PIN[5] = {11, 12, 13, 14, 16};
const char *JNAME[5] = {"base", "shoulder", "elbow", "wrist", "gripper"};

// ---------------- calibration (bring-up 9 and 11 Oct 2026) ----------------
const int   ZERO_RAW[NJ] = {1055, 117, 645, 444};   // raw AS5600 count at 0 deg
const int   DIR[NJ]      = {  +1,  -1,  -1,  -1};
const float P0_US[NJ]    = {1623.0f, 1558.0f, 1562.0f, 1548.0f};    // pulse at 0 deg
const float K_DEG_US[NJ] = {0.1276f, -0.1292f, -0.1358f, -0.1361f}; // deg per us
const float LIM_LO[NJ]   = {-68.0f, -30.0f, -90.0f, -85.0f};        // URDF limits
const float LIM_HI[NJ]   = { 68.0f,  75.0f,  90.0f,  97.0f};
const float TRIM_MARGIN  = 3.0f;    // command + trim may pass a limit by this much (stops are 4-6 deg further)

// ---------------- outer trim loop ----------------
bool TRIM_ON[NJ]  = {false, true, true, true};         // X,j,0/1 switches it (step tests).
// Base trim off by default: in the 8.4 step test (12 Oct) the base trim kept a +-0.44 deg hunt
// going through the horn play (3 of 6 holds); with it off every hold was still and within 0.3 deg.
// X,0,1 switches it back on.
const float TRIM_KI[NJ]  = {1.0f, 2.0f, 2.0f, 2.0f};  // deg/s of trim per deg of error
const float TRIM_DB[NJ]  = {0.5f, 0.3f, 0.3f, 0.3f};  // deadband (base 0.5: hunting dies out since the aluminium horn, 12 Oct)
const float TRIM_MAX[NJ] = {3.0f, 5.0f, 5.0f, 5.0f};  // trim clamp (deg)

// ---------------- active damping (for hunting caused by backlash) ----------------
// While a joint holds still, the command is nudged against the measured joint speed:
//   command += -DAMP_KD * speed   (speed from the encoder, 20 ms means 20 ms apart)
// Off (0) by default; tune with D,joint,gain and measure with O,joint.
float DAMP_KD[NJ] = {0.0f, 0.0f, 0.0f, 0.0f};          // s (deg of command per deg/s)
const float DAMP_MAX = 2.0f;                           // deg, clamp on the nudge
float dampOut[NJ] = {0, 0, 0, 0};
// ---------------- re-settle (for a joint that hunts in its backlash) ----------------
// While a joint holds still, its encoder swing over the last 300 ms is watched. If it
// exceeds RS_DETECT, the command is pushed RS_AMP past the target and brought back
// slowly, so the gears come to rest against one flank. Up to RS_MAX_TRIES per hold.
// Off (0) by default; set with Q,joint,amp (deg, sign = side to approach from).
float RS_AMP[NJ] = {0.0f, 0.0f, 0.0f, 0.0f};
const float RS_DETECT = 0.5f;            // deg peak-to-peak over the window
const int   RS_WIN = 60;                 // 60 x 5 ms = 300 ms
const int   RS_MAX_TRIES = 3;
float rsBuf[NJ][RS_WIN]; int rsIdx = 0, rsFill[NJ] = {0, 0, 0, 0};
uint32_t rsStart[NJ] = {0, 0, 0, 0}, rsCool[NJ] = {0, 0, 0, 0};
int rsTries[NJ] = {0, 0, 0, 0};
uint32_t rsHoldSince[NJ] = {0, 0, 0, 0};  // when the joint last became settled
float rsOut[NJ] = {0, 0, 0, 0};
uint32_t rsAttempts = 0;                 // total, shown by T

// swing measurement (O command)
int oscJ = -1; uint32_t oscT0 = 0; float oscMin, oscMax; double oscMean, oscM2; int oscN = 0;  // Welford
const uint32_t SETTLE_MS = 150;                       // target still this long before trimming

// ---------------- motion ----------------
const uint32_t LOOP_US = 5000;        // 200 Hz
const uint32_t EXTRAP_MS = 40;        // extrapolate the J stream at most this far
const int MA_N = 16;                  // 16 x 5 ms = 80 ms per moving average (S-curve)
const uint32_t WATCHDOG_MS = 200;
const float MAX_RATE  = 180.0f;       // deg/s safety cap on the command
const float SYNC_RATE = 40.0f;        // deg/s after arming / watchdog (slow moves stick-slip, see 11 Oct tests)
const int TELEMETRY_DIV = 4;          // A and I every 4 loops = 50 Hz

// ---------------- gripper ----------------
const int GRIP_OPEN_US = 1500, GRIP_CLOSE_US = 950;
const int GRIP_MIN_US = 950, GRIP_MAX_US = 1575;
const uint32_t GRASP_DELAY_MS = 450;  // closing transients last up to ~350 ms (capture 11 Oct)
const int   GRASP_N = 40;             // 200 ms window at 200 Hz
const float GRASP_THRESH_A = 0.04f;   // median above this = holding (cube ~0.11 A, empty 0.009 A)
const uint32_t LOST_MS = 500;

// ---------------- faults ----------------
const int ENC_FAIL_MAX = 20;                // 20 missed reads in a row (100 ms) = encoder fault
uint32_t encErrCount[4] = {0, 0, 0, 0};     // single missed reads (I2C glitches), shown by T
const float TRACK_ERR_DEG = 15.0f;
const uint32_t TRACK_ERR_MS = 1000;

// ---------------- state ----------------
enum Mode { IDLE, RUN, STOPPED, ESTOP, FAULT };
const char *MODE_NAME[] = {"IDLE", "RUN", "STOPPED", "ESTOP", "FAULT"};
Mode mode = IDLE;

const int MB = 10;                          // 10 samples = 50 ms
float mbuf[NJ][MB];
int mIdx = 0, mCount = 0;
float meas[NJ], measF[NJ], measA[NJ];       // latest, 50 ms mean, 20 ms mean (deg)
bool encOK[NJ] = {false, false, false, false};
int encFail[NJ] = {0, 0, 0, 0};

float cmd[NJ], goal[NJ], trim[NJ];
float p1[NJ], vt[NJ];                       // stage-1 target and J-stream velocity
float ma1[NJ][MA_N], ma2[NJ][MA_N];        // S-curve moving-average buffers
int maIdx = 0;
float lastGoal[NJ];
uint32_t stillSince[NJ], trackSince[NJ];
uint32_t goalT0 = 0, lastJ = 0;
bool syncing = false, wdHold = false;
bool localMove = false; float localRate = 20.0f;   // M test move
// J-stream statistics (sent as a # line every 5 s while running)
uint32_t jN = 0, jSum = 0, jMax = 0, jGap40 = 0, wdCount = 0, lastStats = 0;
int pulseUs[5] = {0, 0, 0, 0, 0};
bool attached[5] = {false, false, false, false, false};

enum GState { G_OFF, G_OPEN, G_CLOSING, G_HOLD, G_LOST };
GState gst = G_OFF;
uint32_t gT0 = 0, lowSince = 0;
float ibuf[GRASP_N];
int iIdx = 0;
float iLatest = 0;

int estopCount = 0;
bool estopPressed = false;
uint32_t lastEsend = 0, lastFsend = 0, lastArmTry = 0;
char faultJoint = 0; const char *faultWhy = "";

uint32_t tickCount = 0;
uint32_t ioSum = 0, ioMax = 0, execMax = 0, ioN = 0, overruns = 0;
uint32_t perMin = 0xFFFFFFFF, perMax = 0, lastTick = 0; uint64_t perSum = 0; uint32_t perN = 0;  // loop period
uint32_t oscDurMs = 5000;
bool telemetry = false;                // A and I lines: off until V,1 (the GUI sends V,1 when it connects)
bool hostWas = false;                  // USB serial host connected (debounced; prints the banner on connect)
uint32_t hostLastSeen = 0;
String rx;

// ---------------- servo PWM (50 Hz, 14 bit) ----------------
uint32_t usToDuty(int us) { return (uint32_t)(((long)us * 16383L + 10000L) / 20000L); }  // rounded

void servoWrite(int n, int us) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  if (!attached[n]) { ledcAttach(SERVO_PIN[n], 50, 14); attached[n] = true; }
  ledcWrite(SERVO_PIN[n], usToDuty(us));
#else
  if (!attached[n]) { ledcSetup(n, 50, 14); ledcAttachPin(SERVO_PIN[n], n); attached[n] = true; }
  ledcWrite(n, usToDuty(us));
#endif
  pulseUs[n] = us;
}

void servoOff(int n) {
  if (!attached[n]) return;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcDetach(SERVO_PIN[n]);
#else
  ledcDetachPin(SERVO_PIN[n]);
#endif
  pinMode(SERVO_PIN[n], OUTPUT);
  digitalWrite(SERVO_PIN[n], LOW);
  attached[n] = false;
  pulseUs[n] = 0;
}

// ---------------- I2C ----------------
bool muxSelect(int ch) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(ch < 0 ? 0 : (1 << ch));
  return Wire.endTransmission() == 0;
}

bool readReg16(uint8_t addr, uint8_t reg, uint16_t &val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, (uint8_t)2) != 2) return false;
  val = ((uint16_t)Wire.read() << 8) | Wire.read();
  return true;
}

bool readReg8(uint8_t addr, uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, (uint8_t)1) != 1) return false;
  val = Wire.read();
  return true;
}

float rawToDeg(int j, uint16_t raw) {
  int d = (int)(raw & 0x0FFF) - ZERO_RAW[j];
  if (d > 2048) d -= 4096;
  if (d < -2048) d += 4096;
  return DIR[j] * d * 360.0f / 4096.0f;
}

bool magnetOK(int j) {                      // used when arming
  uint8_t st = 0;
  if (!muxSelect(j) || !readReg8(AS5600_ADDR, 0x0B, st)) { muxSelect(-1); return false; }
  muxSelect(-1);
  return (st & 0x20) && !(st & 0x10) && !(st & 0x08);
}

void readSensors() {
  for (int j = 0; j < NJ; j++) {
    uint16_t raw;
    if (muxSelect(j) && readReg16(AS5600_ADDR, 0x0C, raw)) {
      meas[j] = rawToDeg(j, raw);
      encFail[j] = 0;
      encOK[j] = true;
    } else {
      encErrCount[j]++;
      if (++encFail[j] >= ENC_FAIL_MAX) encOK[j] = false;
    }
  }
  muxSelect(-1);
  for (int j = 0; j < NJ; j++) mbuf[j][mIdx] = meas[j];
  mIdx = (mIdx + 1) % MB;
  if (mCount < MB) mCount++;
  for (int j = 0; j < NJ; j++) {
    float sF = 0, sA = 0;
    for (int k = 0; k < mCount; k++) {
      float v = mbuf[j][(mIdx - 1 - k + MB) % MB];
      sF += v;
      if (k < 4) sA += v;
    }
    measF[j] = sF / mCount;
    measA[j] = sA / min(mCount, 4);
  }
  uint16_t sh;
  if (readReg16(INA219_ADDR, 0x01, sh)) iLatest = (int16_t)sh * 0.01f / 1000.0f / SHUNT_OHM;
  ibuf[iIdx] = iLatest;
  iIdx = (iIdx + 1) % GRASP_N;
}

float medianCurrent() {
  float a[GRASP_N];
  for (int i = 0; i < GRASP_N; i++) a[i] = ibuf[i];
  for (int i = 1; i < GRASP_N; i++) {       // insertion sort, 40 values
    float v = a[i]; int k = i - 1;
    while (k >= 0 && a[k] > v) { a[k + 1] = a[k]; k--; }
    a[k + 1] = v;
  }
  return 0.5f * (a[GRASP_N / 2 - 1] + a[GRASP_N / 2]);
}

// ---------------- joints ----------------
float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

int jointPulse(int j, float deg) {
  deg = clampf(deg, LIM_LO[j] - TRIM_MARGIN, LIM_HI[j] + TRIM_MARGIN);
  int us = (int)lroundf(P0_US[j] + deg / K_DEG_US[j]);
  return constrain(us, 500, 2500);
}

bool allEncodersOK() {
  for (int j = 0; j < NJ; j++) if (!encOK[j]) return false;
  return true;
}

void resetFilter(int j, float v) {
  p1[j] = cmd[j] = v; vt[j] = 0;
  for (int k = 0; k < MA_N; k++) ma1[j][k] = ma2[j][k] = v;
}

// Start (or restart) control from the measured angles, so nothing jumps.
bool arm() {
  if (estopPressed) { Serial.println("# cannot arm: physical E-stop pressed"); return false; }
  for (int j = 0; j < NJ; j++) {
    if (!encOK[j] || !magnetOK(j)) {
      Serial.printf("# cannot arm: %s encoder not OK\n", JNAME[j]);
      return false;
    }
  }
  uint32_t now = millis();
  for (int j = 0; j < NJ; j++) {
    resetFilter(j, measF[j]);                 // filter starts at the measured angle
    goal[j] = lastGoal[j] = clampf(measF[j], LIM_LO[j], LIM_HI[j]);
    trim[j] = 0;
    stillSince[j] = now;
    trackSince[j] = 0;
    servoWrite(j, jointPulse(j, cmd[j]));
  }
  goalT0 = now; lastJ = now;
  syncing = true; wdHold = false;
  mode = RUN;
  Serial.println("# armed");
  return true;
}

void enterFault(int j, const char *why) {
  mode = FAULT;
  faultJoint = j; faultWhy = why;
  for (int k = 0; k < NJ; k++) { goal[k] = p1[k]; vt[k] = 0; }   // decelerate smoothly and hold
  Serial.printf("F,%s,%s\n", JNAME[j], why);
  lastFsend = millis();
}

void enterEstop() {
  for (int n = 0; n < 5; n++) servoOff(n);
  mode = ESTOP;
  gst = G_OFF;
  for (int j = 0; j < NJ; j++) trim[j] = 0;
}

void updateJoints(uint32_t now) {
  const float dt = LOOP_US * 1e-6f;
  bool moving = (mode == RUN);
  if (mode == RUN && !localMove && now - lastJ > WATCHDOG_MS && !wdHold) {   // watchdog: hold the last target
    wdHold = true; wdCount++;
    for (int j = 0; j < NJ; j++) vt[j] = 0;
  }
  if (mode == RUN || mode == STOPPED || mode == FAULT) {        // S-curve trajectory
    float ext = (mode == RUN && !wdHold) ? min(now - goalT0, EXTRAP_MS) * 1e-3f : 0.0f;
    float rate = syncing ? SYNC_RATE : (localMove ? localRate : MAX_RATE);
    bool reached = true;
    for (int j = 0; j < NJ; j++) {
      float des = clampf(goal[j] + vt[j] * ext, LIM_LO[j], LIM_HI[j]);
      p1[j] += clampf(des - p1[j], -rate * dt, rate * dt);
      ma1[j][maIdx] = p1[j];
      float s1 = 0;
      for (int k = 0; k < MA_N; k++) s1 += ma1[j][k];
      ma2[j][maIdx] = s1 / MA_N;
      float s2 = 0;
      for (int k = 0; k < MA_N; k++) s2 += ma2[j][k];
      cmd[j] = s2 / MA_N;
      if (fabsf(goal[j] - p1[j]) > 0.5f) reached = false;
    }
    maIdx = (maIdx + 1) % MA_N;
    if (syncing && reached) syncing = false;
  }
  if (mode == RUN || mode == STOPPED || mode == FAULT) {
    for (int j = 0; j < NJ; j++) {
      if (fabsf(goal[j] - lastGoal[j]) > 0.05f) { lastGoal[j] = goal[j]; stillSince[j] = now; }
      bool settled = (now - stillSince[j] > SETTLE_MS) && fabsf(cmd[j] - goal[j]) < 0.05f;
      float err = cmd[j] - measF[j];
      // outer trim loop: slow integral on the encoder error, only while the joint is still
      if (TRIM_ON[j] && settled && encOK[j] && mode != FAULT && rsStart[j] == 0 && fabsf(err) > TRIM_DB[j]) {
        float e = err - copysignf(TRIM_DB[j], err);
        trim[j] = clampf(trim[j] + TRIM_KI[j] * e * dt, -TRIM_MAX[j], TRIM_MAX[j]);
      }
      // tracking check: a still joint far from its target (blocked, slipped, collision)
      if (moving && settled && encOK[j] && fabsf(err) > TRACK_ERR_DEG) {
        if (trackSince[j] == 0) trackSince[j] = now;
        else if (now - trackSince[j] > TRACK_ERR_MS) { enterFault(j, "TRACK"); }
      } else {
        trackSince[j] = 0;
      }
      // active damping while holding still
      dampOut[j] = 0;
      if (DAMP_KD[j] != 0.0f && settled && encOK[j] && mCount >= 8) {
        float a = 0, b = 0;
        for (int k = 0; k < 4; k++) {
          a += mbuf[j][(mIdx - 1 - k + MB) % MB];
          b += mbuf[j][(mIdx - 5 - k + MB) % MB];
        }
        float speed = (a - b) / 4.0f / 0.020f;            // deg/s
        dampOut[j] = clampf(-DAMP_KD[j] * speed, -DAMP_MAX, DAMP_MAX);
      }
      // re-settle: watch the swing while holding, re-approach from one side if it hunts
      rsBuf[j][rsIdx] = meas[j];
      if (rsFill[j] < RS_WIN) rsFill[j]++;
      if (now - stillSince[j] < SETTLE_MS) { rsTries[j] = 0; }        // new target: fresh tries
      if (!settled) rsHoldSince[j] = now;
      rsOut[j] = 0;
      if (rsStart[j] != 0) {                                          // re-approach in progress
        uint32_t t = now - rsStart[j];
        float A = RS_AMP[j];
        if (t < 100) rsOut[j] = A * t / 100.0f;                       // push past the target
        else if (t < 200) rsOut[j] = A;                               // hold
        else if (t < 600) rsOut[j] = A * (1.0f - (t - 200) / 400.0f); // come back slowly
        else { rsStart[j] = 0; rsCool[j] = now + 700; rsFill[j] = 0; }
      } else if (RS_AMP[j] != 0.0f && settled && encOK[j] && mode == RUN &&
                 now - rsHoldSince[j] > 600 &&                // the move itself has died away
                 rsFill[j] >= RS_WIN && (int32_t)(now - rsCool[j]) >= 0) {
        float lo = 1e9, hi = -1e9;
        for (int k = 0; k < RS_WIN; k++) { lo = min(lo, rsBuf[j][k]); hi = max(hi, rsBuf[j][k]); }
        if (hi - lo > RS_DETECT) {
          if (rsTries[j] < RS_MAX_TRIES) {
            rsTries[j]++; rsAttempts++;
            rsStart[j] = now;
            Serial.printf("# %s re-settle %d (swing %.2f deg)\n", JNAME[j], rsTries[j], hi - lo);
          } else if (rsTries[j] == RS_MAX_TRIES) {
            rsTries[j]++;
            Serial.printf("# %s re-settle gave up (swing %.2f deg)\n", JNAME[j], hi - lo);
          }
        }
      }
      if (mode != ESTOP && mode != IDLE)
        servoWrite(j, jointPulse(j, cmd[j] + trim[j] + dampOut[j] + rsOut[j]));
    }
  }
  rsIdx = (rsIdx + 1) % RS_WIN;
  if ((mode == RUN || mode == STOPPED) && !allEncodersOK()) {
    for (int j = 0; j < NJ; j++) if (!encOK[j]) { enterFault(j, "ENC"); break; }
  }
}

// ---------------- gripper ----------------
void gripperWrite(int us) { servoWrite(GRIP, constrain(us, GRIP_MIN_US, GRIP_MAX_US)); }

void updateGripper(uint32_t now) {
  if (gst == G_CLOSING && now - gT0 >= GRASP_DELAY_MS + GRASP_N * LOOP_US / 1000) {
    float med = medianCurrent();              // window = the 200 ms after the settling delay
    if (med > GRASP_THRESH_A) {
      gst = G_HOLD; lowSince = 0;
      Serial.printf("K,1\n# grasp confirmed, median %.3f A\n", med);
    } else {
      gripperWrite(GRIP_OPEN_US);
      gst = G_OPEN;
      Serial.printf("K,0\n# grasp failed, median %.3f A, gripper re-opened\n", med);
    }
  } else if (gst == G_HOLD && tickCount % 10 == 0) {
    float med = medianCurrent();
    if (med < GRASP_THRESH_A) {
      if (lowSince == 0) lowSince = now;
      else if (now - lowSince > LOST_MS) {
        gst = G_LOST;
        Serial.printf("K,2\n# grip lost, median %.3f A\n", med);
      }
    } else {
      lowSince = 0;
    }
  }
}

// ---------------- commands ----------------
void help() {
  Serial.println("# commands: J,b,s,e,w | G,1 | G,0 | S | R | M,j,deg,v test move | D,j,gain damping | Q,j,amp re-settle | O,j[,s] swing | X,j,0/1 trim | V,0/1 telemetry | T timing | P status | H help");
}

void printStatus() {
  Serial.printf("# mode %s%s%s | E-stop %s\n", MODE_NAME[mode], syncing ? " (syncing)" : "",
                wdHold ? " (watchdog hold)" : "", estopPressed ? "PRESSED" : "released");
  for (int j = 0; j < NJ; j++)
    Serial.printf("#  %-8s target %7.2f  cmd %7.2f  meas %7.2f  trim %+5.2f  pulse %4d  enc %s\n",
                  JNAME[j], goal[j], cmd[j], measF[j], trim[j], pulseUs[j], encOK[j] ? "OK" : "FAIL");
  const char *gname[] = {"off", "open", "closing", "holding", "lost"};
  Serial.printf("#  gripper  %s  pulse %4d  current %.3f A  median %.3f A\n",
                gname[gst], pulseUs[GRIP], iLatest, medianCurrent());
}

void printJStats() {
  if (jN == 0) { Serial.println("# J stream: none received"); return; }
  Serial.printf("# J stream: %lu msgs, mean gap %.1f ms, max %lu ms, gaps>40ms %lu, watchdog holds %lu\n",
                (unsigned long)jN, jSum / (float)jN, (unsigned long)jMax,
                (unsigned long)jGap40, (unsigned long)wdCount);
  jN = jSum = jMax = jGap40 = wdCount = 0;
}

void printTiming() {
  if (ioN == 0) { Serial.println("# no samples yet"); return; }
  Serial.printf("# I2C read (4 encoders + INA219): mean %lu us, max %lu us over %lu loops | "
                "loop exec max %lu us of %lu us | overruns %lu | I2C %lu Hz\n",
                (unsigned long)(ioSum / ioN), (unsigned long)ioMax, (unsigned long)ioN,
                (unsigned long)execMax, (unsigned long)LOOP_US, (unsigned long)overruns,
                (unsigned long)I2C_HZ);
  ioSum = ioMax = execMax = ioN = overruns = 0;
  if (perN) Serial.printf("# loop period over %lu loops: mean %.1f us, min %lu us, max %lu us (target %lu us)\n",
                          (unsigned long)perN, perSum / (double)perN, (unsigned long)perMin,
                          (unsigned long)perMax, (unsigned long)LOOP_US);
  perSum = 0; perN = 0; perMin = 0xFFFFFFFF; perMax = 0;
  Serial.printf("# encoder read errors since last T: base %lu, shoulder %lu, elbow %lu, wrist %lu\n",
                (unsigned long)encErrCount[0], (unsigned long)encErrCount[1],
                (unsigned long)encErrCount[2], (unsigned long)encErrCount[3]);
  for (int j = 0; j < NJ; j++) encErrCount[j] = 0;
  Serial.printf("# re-settle attempts since last T: %lu\n", (unsigned long)rsAttempts);
  rsAttempts = 0;
  printJStats();
}

void handleLine(String s) {
  s.trim();
  if (s.length() == 0) return;
  char c = toupper(s.charAt(0));
  uint32_t now = millis();
  if (c == 'J') {
    float v[NJ];
    int n = sscanf(s.c_str() + 2, "%f,%f,%f,%f", &v[0], &v[1], &v[2], &v[3]);
    if (n != NJ || s.charAt(1) != ',') return;
    if (mode == IDLE) {
      if (now - lastArmTry < 1000) return;    // one attempt per second if arming fails
      lastArmTry = now;
      if (!arm()) return;
    }
    if (mode != RUN) return;                  // STOPPED, ESTOP, FAULT ignore J
    localMove = false;
    if (lastJ != 0 && (int32_t)(now - lastJ) > 0) {   // skip the J that armed the arm
      uint32_t gap = now - lastJ;
      jN++; jSum += gap; if (gap > jMax) jMax = gap; if (gap > 40) jGap40++;
    }
    bool resume = wdHold;
    if (resume) { wdHold = false; syncing = true; }   // resume slowly after a gap
    float dtj = max((int32_t)(now - lastJ), (int32_t)8) * 1e-3f;
    for (int j = 0; j < NJ; j++) {
      float g = clampf(v[j], LIM_LO[j], LIM_HI[j]);
      // J-stream velocity for the extrapolation (none while syncing or resuming)
      vt[j] = (syncing || resume) ? 0.0f : clampf((g - goal[j]) / dtj, -MAX_RATE, MAX_RATE);
      goal[j] = g;
    }
    goalT0 = now;
    lastJ = now;
  } else if (c == 'G') {
    if (mode == ESTOP) { Serial.println("# gripper ignored: E-stop"); return; }
    if (s.length() >= 3 && s.charAt(2) == '1') {
      gripperWrite(GRIP_CLOSE_US);
      gst = G_CLOSING; gT0 = now;
    } else {
      gripperWrite(GRIP_OPEN_US);
      gst = G_OPEN;
    }
  } else if (c == 'S') {
    if (mode == RUN) {
      for (int j = 0; j < NJ; j++) { goal[j] = p1[j]; vt[j] = 0; }   // smooth stop, then hold
      mode = STOPPED;
      Serial.println("# stopped (software): holding, send R to resume");
    }
  } else if (c == 'R') {
    if (mode == ESTOP && estopPressed) { Serial.println("# release the physical E-stop first"); return; }
    if (mode == RUN) { Serial.println("# already running"); return; }
    if (mode == STOPPED) {                    // resume from where it is holding
      for (int j = 0; j < NJ; j++) { goal[j] = p1[j]; vt[j] = 0; }
      lastJ = goalT0 = now; syncing = true; wdHold = false;
      mode = RUN;
      Serial.println("# resumed");
      return;
    }
    arm();
  } else if (c == 'M') {
    int j; float deg, v;
    if (sscanf(s.c_str() + 2, "%d,%f,%f", &j, &deg, &v) != 3 || j < 0 || j >= NJ) {
      Serial.println("# use M,joint,deg,speed  e.g. M,1,40,20  (joint 0-3, speed deg/s)");
      return;
    }
    if (mode == IDLE || mode == ESTOP) { if (!arm()) return; }
    if (mode != RUN) { Serial.println("# not running: send R first"); return; }
    for (int k = 0; k < NJ; k++) { goal[k] = p1[k]; vt[k] = 0; }
    goal[j] = clampf(deg, LIM_LO[j], LIM_HI[j]);
    localRate = clampf(v, 1.0f, MAX_RATE);
    localMove = true; syncing = false; wdHold = false;
    goalT0 = lastJ = now;
    Serial.printf("# test move: %s to %.1f deg at %.0f deg/s\n", JNAME[j], goal[j], localRate);
  } else if (c == 'D') {
    int j; float g;
    if (s.length() > 1 && sscanf(s.c_str() + 2, "%d,%f", &j, &g) == 2 && j >= 0 && j < NJ) {
      DAMP_KD[j] = clampf(g, -0.2f, 0.2f);
    }
    Serial.printf("# damping gains (s): base %.3f, shoulder %.3f, elbow %.3f, wrist %.3f\n",
                  DAMP_KD[0], DAMP_KD[1], DAMP_KD[2], DAMP_KD[3]);
  } else if (c == 'Q') {
    int j; float a;
    if (s.length() > 1 && sscanf(s.c_str() + 2, "%d,%f", &j, &a) == 2 && j >= 0 && j < NJ) {
      RS_AMP[j] = clampf(a, -3.0f, 3.0f);
      rsTries[j] = 0; rsStart[j] = 0; rsFill[j] = 0;
    }
    Serial.printf("# re-settle amplitude (deg, 0 = off): base %.2f, shoulder %.2f, elbow %.2f, wrist %.2f\n",
                  RS_AMP[0], RS_AMP[1], RS_AMP[2], RS_AMP[3]);
  } else if (c == 'O') {
    int j, secs = 5;
    int n = sscanf(s.c_str() + 2, "%d,%d", &j, &secs);
    if (n < 1 || j < 0 || j >= NJ) { Serial.println("# use O,joint[,seconds]  e.g. O,0 or O,1,60"); return; }
    secs = constrain(secs, 1, 120);
    oscDurMs = secs * 1000UL;
    oscJ = j; oscT0 = now; oscN = 0; oscMin = 1e9; oscMax = -1e9; oscMean = oscM2 = 0;
    Serial.printf("# measuring %s swing for %d s...\n", JNAME[j], secs);
  } else if (c == 'V') {
    if (s.length() >= 3) telemetry = (s.charAt(2) != '0');
    Serial.printf("# telemetry (A and I lines, 1 s E heartbeat): %s\n", telemetry ? "on" : "off");
  } else if (c == 'X') {
    int j, on;
    if (sscanf(s.c_str() + 2, "%d,%d", &j, &on) == 2 && j >= 0 && j < NJ) {
      TRIM_ON[j] = (on != 0);
      if (!TRIM_ON[j]) trim[j] = 0;
    }
    Serial.printf("# trim loop: base %s, shoulder %s, elbow %s, wrist %s\n",
                  TRIM_ON[0] ? "on" : "off", TRIM_ON[1] ? "on" : "off",
                  TRIM_ON[2] ? "on" : "off", TRIM_ON[3] ? "on" : "off");
  } else if (c == 'T') {
    printTiming();
  } else if (c == 'P') {
    printStatus();
  } else if (c == 'H') {
    help();
  }
}

void pollSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { if (rx.length()) handleLine(rx); rx = ""; }
    else if (rx.length() < 80) rx += ch;
  }
}

// ---------------- E-stop ----------------
void updateEstop(uint32_t now) {
  bool raw = digitalRead(ESTOP_PIN);          // HIGH = pressed (NC contact to GND opened)
  if (raw != estopPressed) {
    if (++estopCount >= 3) {                  // 3 loops = 15 ms debounce
      estopPressed = raw; estopCount = 0;
      Serial.printf("E,%d\n", estopPressed ? 1 : 0);
      lastEsend = now;
      if (estopPressed) enterEstop();
      else if (mode == ESTOP) {               // released: stay limp, nothing moves until J or R
        mode = IDLE;
        Serial.println("# E-stop released: servos limp until the next J or R");
      }
    }
  } else {
    estopCount = 0;
  }
  if (telemetry && now - lastEsend >= 1000) { Serial.printf("E,%d\n", estopPressed ? 1 : 0); lastEsend = now; }
}

// ---------------- main ----------------
void tick() {
  uint32_t t0 = micros();
  if (lastTick != 0) {                                  // loop period, start to start
    uint32_t per = t0 - lastTick;
    perSum += per; perN++; if (per < perMin) perMin = per; if (per > perMax) perMax = per;
  }
  lastTick = t0;
  uint32_t now = millis();
  readSensors();
  uint32_t io = micros() - t0;
  ioSum += io; ioN++; if (io > ioMax) ioMax = io;

  updateEstop(now);
  updateJoints(now);
  updateGripper(now);

  if (mode == RUN && jN > 0 && now - lastStats >= 5000) { printJStats(); lastStats = now; }
  if (oscJ >= 0) {                                   // O command: swing of one joint over 5 s
    float v = meas[oscJ];
    oscMin = min(oscMin, v); oscMax = max(oscMax, v);
    oscN++; double d = v - oscMean; oscMean += d / oscN; oscM2 += d * (v - oscMean);   // Welford: no cancellation
    if (now - oscT0 >= oscDurMs) {
      float mean = (float)oscMean, sd = (oscN > 1) ? (float)sqrt(oscM2 / (oscN - 1)) : 0.0f;
      Serial.printf("# %s over %lu s: peak-to-peak %.2f deg, std %.2f deg, mean %.2f deg (target %.2f), damping %.3f\n",
                    JNAME[oscJ], (unsigned long)(oscDurMs / 1000), oscMax - oscMin, sd, mean, goal[oscJ], DAMP_KD[oscJ]);
      oscJ = -1;
    }
  }
  if (mode == FAULT && now - lastFsend >= 1000) {
    Serial.printf("F,%s,%s\n", JNAME[(int)faultJoint], faultWhy);
    lastFsend = now;
  }
  if (telemetry && tickCount % TELEMETRY_DIV == 0) {
    Serial.printf("A,%.2f,%.2f,%.2f,%.2f\n", measA[0], measA[1], measA[2], measA[3]);
    float s = 0;
    for (int k = 1; k <= 4; k++) s += ibuf[(iIdx - k + GRASP_N) % GRASP_N];
    Serial.printf("I,%.3f\n", s / 4);
  }
  tickCount++;
  uint32_t ex = micros() - t0;
  if (ex > execMax) execMax = ex;
}

void setup() {
  for (int n = 0; n < 5; n++) { pinMode(SERVO_PIN[n], OUTPUT); digitalWrite(SERVO_PIN[n], LOW); }
  pinMode(ESTOP_PIN, INPUT);                  // external 10 k pull-up
  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT && ARDUINO_USB_MODE
  Serial.setTxTimeoutMs(0);                   // never block the loop if the PC is not reading
#endif
  Wire.begin(SDA_PIN, SCL_PIN, I2C_HZ);
  Wire.setTimeOut(3);                         // ms: a stuck device cannot hang the loop
  delay(500);
  for (int k = 0; k < MB; k++) readSensors(); // fill the averaging buffers
  estopPressed = digitalRead(ESTOP_PIN);
  mode = estopPressed ? ESTOP : IDLE;
  Serial.println("# WDD2 firmware ready: servos limp until the first J or R");
  help();
  Serial.printf("E,%d\n", estopPressed ? 1 : 0);
  lastEsend = millis();
}

void loop() {
  bool host = (bool)Serial;                     // a serial monitor or the GUI has the port open
  uint32_t ms = millis();
  if (host) {
    if (!hostWas) {                             // only after the port was really closed for > 1 s
      Serial.printf("# WDD2 firmware ready, mode %s, telemetry %s (V,1 = on)\n", MODE_NAME[mode], telemetry ? "on" : "off");
      help();
    }
    hostWas = true; hostLastSeen = ms;
  } else if (hostWas && ms - hostLastSeen > 1000) {
    hostWas = false;
  }
  pollSerial();
  static uint32_t next = micros();
  if ((int32_t)(micros() - next) >= 0) {
    next += LOOP_US;
    if ((int32_t)(micros() - next) > 0) { next = micros() + LOOP_US; overruns++; }
    tick();
  }
}
