/* RAMPS 1.4 + TMC2209 (single-wire UART on pin 63/A9) + Movement Queue + Accel/Decel + Sensorless Homing on DIAG -> MIN endstops (X: D3, Y: D14, Z: D18) + Robust homing: phase-specific SG thresholds, sg-min-speed clamp, DIAG de-bounce, de-latch + Coil output on RAMPS D8 (PWM-capable): on/off and PWM control Board: Arduino MEGA 2560 Shield: RAMPS 1.4 Drivers: TMC2209 STEP/DIR with UART enabled (shared single-wire on A9/pin 63) Console (115200): <dX> <dY> <dZ> -> enqueue one relative move (steps) multiple of 3 numbers -> enqueue many moves in one paste (spaces or new lines) e.g. 5000 5000 5000 2000 2000 2000 -300 -300 -300 start -> execute queued moves (accel/decel, synchronized) list -> list queued moves clear -> clear queue speed <v> -> set default speed (steps/s) accel <a> -> set default acceleration (steps/s^2) mstep <n> -> set microstepping live (1..256) status -> show current defaults/microsteps p -> print current XYZ step positions help -> print help Sensorless homing & tuning: home -> sensorless home all axes (DIAG→MIN) homems <n> -> set homing microsteps (1..256) homev <fast> <slow> -> set homing fast/slow speeds (steps/s) homea <a> -> set homing acceleration (steps/s^2) homeback <steps> -> set backoff steps between passes sgfast <x> <y> <z> -> set SGTHRS for FAST pass (0..255) sgslow <x> <y> <z> -> set SGTHRS for SLOW pass (0..255) sgmin <sps> -> set minimum slow speed for valid SG (steps/s, default 0) tpwm <n> -> set TPWMTHRS (optional) tcool <n> -> set TCOOLTHRS (optional) endi <0|1> -> invert endstop logic if DIAG polarity differs estop -> disable drivers immediately stepsmm <x> <y> <z> -> set steps/mm per axis (utility) Coil (RAMPS D8): coil on -> coil 100% (fully on) coil off -> coil 0% (off) coil <0..255> -> set PWM duty raw coil <0..100>% -> set PWM duty percent IK: ik x y z [x y z ...] -> inverse kinematics to relative steps & enqueue DANCE: dance -> home; mstep 64 & move -7500 -7500 -7500; then mstep 4 & run sequence */
#define TMCSTEPPER_USE_SW_SERIAL
#include <SoftwareSerial.h>
#include <TMCStepper.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <math.h>
float g_thetaPrev1 = 34.0f, g_thetaPrev2 = 34.0f, g_thetaPrev3 = 34.0f;

// ---------------- RAMPS 1.4 pins ----------------
const uint8_t X_STEP_PIN = 54; // A0
const uint8_t X_DIR_PIN  = 55; // A1
const uint8_t X_ENABLE_PIN = 38;

const uint8_t Y_STEP_PIN = 60; // A6
const uint8_t Y_DIR_PIN  = 61; // A7
const uint8_t Y_ENABLE_PIN = 56; // A2

const uint8_t Z_STEP_PIN = 46;
const uint8_t Z_DIR_PIN  = 48;
const uint8_t Z_ENABLE_PIN = 62; // A8

// Direction sense (as you had them)
const bool X_DIR_INVERT = true;
const bool Y_DIR_INVERT = true;
const bool Z_DIR_INVERT = false;

// ---------------- Endstops (DIAG -> MIN) ----------------
const uint8_t X_MIN_PIN = 3;
const uint8_t Y_MIN_PIN = 14;
const uint8_t Z_MIN_PIN = 18;

// If DIAG behavior is inverted on your setup, flip via "endi 1"
bool g_endstopInvert = false;

// ---------------- Coil output (RAMPS D8) ----------------
const uint8_t COIL_PIN = 9; // (kept exactly as you sent)
uint8_t g_coilPWM = 0;      // 0..255, default OFF

// ---------------- TMC2209 UART (single wire) ----------------
#define UART_PIN 63 // A9 on MEGA — Hydra signal line
#define R_SENSE 0.11f
#define TMC_BAUD 115200

// UART addresses via MS1/MS2 (CFG) jumpers
const uint8_t ADDR_X = 0;
const uint8_t ADDR_Y = 1;
const uint8_t ADDR_Z = 2;

// Pin-based constructor (single-wire: RX=TX=UART_PIN)
TMC2209Stepper driverX = TMC2209Stepper(UART_PIN, UART_PIN, R_SENSE, ADDR_X);
TMC2209Stepper driverY = TMC2209Stepper(UART_PIN, UART_PIN, R_SENSE, ADDR_Y);
TMC2209Stepper driverZ = TMC2209Stepper(UART_PIN, UART_PIN, R_SENSE, ADDR_Z);

// ---------------- Motion layer ----------------
AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper stepperZ(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);
MultiStepper multi; // present but not used for accel execution

// Initial motion caps (PRESERVED)
const float   DEFAULT_MAX_SPEED_SPS = 4000.0;  // steps/s
const float   DEFAULT_ACCEL_SPS2    = 8000.0;  // steps/s^2
const uint16_t CURRENT_mA           = 1700;
const uint16_t MICROSTEPS_BOOT      = 64;

// Runtime adjustables (global defaults)
float g_defaultSpeed = DEFAULT_MAX_SPEED_SPS;
float g_defaultAccel = DEFAULT_ACCEL_SPS2;
uint16_t g_microsteps = MICROSTEPS_BOOT;

// Optional steps/mm (utility for delta later)
float STEPS_PER_MM_X = 80.0f;
float STEPS_PER_MM_Y = 80.0f;
float STEPS_PER_MM_Z = 80.0f;

// ---------------- Sensorless homing tuning (PRESERVED) ----------------
uint16_t g_homeMicrosteps = 16;   // homing microsteps
float    g_homeFastSPS    = 400.0f;  // fast seek speed (steps/s)
float    g_homeSlowSPS    = 400.0f;  // slow approach (steps/s)
float    g_homeAccel      = 1500.0f;  // homing acceleration (steps/s^2)
long     g_homeBackoff    = 0;        // steps to back off between hits
float    g_sgMinSPS       = 0.0f;     // minimum step rate where SG is valid

// StallGuard thresholds (0..255)
uint8_t  g_sg_fast_x = 2, g_sg_fast_y = 2, g_sg_fast_z = 2;
uint8_t  g_sg_slow_x = 6, g_sg_slow_y = 6, g_sg_slow_z = 6;

// Threshold registers (advanced tuning; defaults)
uint32_t g_TPWMTHRS = 0;
uint32_t g_TCOOLTHRS = 0;

// ---------------- Movement queue ----------------
struct Move { long dx, dy, dz; float vmax; /* 0 => use g_defaultSpeed */ };
const uint8_t QUEUE_MAX = 64;
Move queueBuf[QUEUE_MAX];
uint8_t qHead = 0, qTail = 0;

bool queueIsEmpty() { return qHead == qTail; }
bool enqueueMove(long dx, long dy, long dz, float vmax = 0.0f) {
  uint8_t nextTail = (qTail + 1) % QUEUE_MAX;
  if (nextTail == qHead) return false;
  queueBuf[qTail] = { dx, dy, dz, vmax };
  qTail = nextTail;
  return true;
}
bool dequeueMove(Move &m) {
  if (queueIsEmpty()) return false;
  m = queueBuf[qHead];
  qHead = (qHead + 1) % QUEUE_MAX;
  return true;
}
void clearQueue() { qHead = qTail = 0; }

void printQueue() {
  Serial.print(F("Queue size150: "));
  uint8_t n = (qTail + QUEUE_MAX - qHead) % QUEUE_MAX;
  Serial.println(n);
  uint8_t idx = qHead;
  for (uint8_t i = 0; i < n; ++i) {
    Serial.print('#'); Serial.print(i + 1); Serial.print(F(": "));
    Serial.print(queueBuf[idx].dx); Serial.print(' ');
    Serial.print(queueBuf[idx].dy); Serial.print(' ');
    Serial.print(queueBuf[idx].dz); Serial.print(F(" vmax="));
    Serial.println(queueBuf[idx].vmax == 0 ? g_defaultSpeed : queueBuf[idx].vmax);
    idx = (idx + 1) % QUEUE_MAX;
  }
}

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  <dX> <dY> <dZ>          Enqueue one relative move (steps)"));
  Serial.println(F("  paste N*3 numbers       Enqueue many moves (spaces or new lines)"));
  Serial.println(F("  start                   Execute queued moves (accel/decel, synchronized)"));
  Serial.println(F("  speed <v>               Set default speed (steps/s)"));
  Serial.println(F("  accel <a>               Set default acceleration (steps/s^2)"));
  Serial.println(F("  mstep <n>               Set microstepping live (1..256)"));
  Serial.println(F("  status                  Show current defaults and microsteps"));
  Serial.println(F("  list                    Show queued moves"));
  Serial.println(F("  clear                   Empty the queue"));
  Serial.println(F("  p                       Print current step positions"));
  Serial.println(F("  home                    Sensorless home all axes (DIAG→MIN)"));
  Serial.println(F("  homems <n>              Set homing microsteps"));
  Serial.println(F("  homev <fast> <slow>     Set homing fast/slow speeds (steps/s)"));
  Serial.println(F("  homea <a>               Set homing acceleration (steps/s^2)"));
  Serial.println(F("  homeback <steps>        Set homing backoff steps"));
  Serial.println(F("  sgfast <x> <y> <z>      Set SGTHRS FAST pass (0..255)"));
  Serial.println(F("  sgslow <x> <y> <z>      Set SGTHRS SLOW pass (0..255)"));
  Serial.println(F("  sgmin <sps>             Set min SG-valid speed for slow pass"));
  Serial.println(F("  tpwm <n>                Set TPWMTHRS"));
  Serial.println(F("  tcool <n>               Set TCOOLTHRS"));
  Serial.println(F("  endi <0|1>              Endstop invert (polarity of DIAG)"));
  Serial.println(F("  estop                   Emergency stop (disable drivers)"));
  Serial.println(F("  stepsmm <x> <y> <z>     Set steps/mm per axis (utility)"));
  Serial.println(F("  coil on/off or coil <0..255> or coil <0..100>%  (RAMPS D8)"));
  Serial.println(F("  ik x y z [x y z ...]    IK to relative steps & enqueue"));
  Serial.println(F("  dance                   Home, mstep 64 & move -7500 -7500 -7500; then mstep 4 & run sequence"));
  Serial.println(F("  help                    Show this help"));
}

// ---------------- Utilities ----------------
void enableDrivers(bool on) {
  digitalWrite(X_ENABLE_PIN, on ? LOW : HIGH);
  digitalWrite(Y_ENABLE_PIN, on ? LOW : HIGH);
  digitalWrite(Z_ENABLE_PIN, on ? LOW : HIGH);
}

inline bool rawEndstopRead(uint8_t pin) {
  bool lowIsTriggered = (digitalRead(pin) == LOW);
  return g_endstopInvert ? !lowIsTriggered : lowIsTriggered;
}

void applyMicrostepsAll(uint16_t ms) {
  if      (ms >= 256) ms = 256;
  else if (ms >= 128) ms = 128;
  else if (ms >=  64) ms =  64;
  else if (ms >=  32) ms =  32;
  else if (ms >=  16) ms =  16;
  else if (ms >=   8) ms =   8;
  else if (ms >=   4) ms =   4;
  else if (ms >=   2) ms =   2;
  else                ms =   1;

  driverX.microsteps(ms);
  driverY.microsteps(ms);
  driverZ.microsteps(ms);
  g_microsteps = ms;

  Serial.print(F("[TMC] Microsteps set to ")); Serial.println(ms);
}

void initDriver(TMC2209Stepper &drv, const char *name) {
  drv.beginSerial(TMC_BAUD);
  drv.begin();
  drv.pdn_disable(true);
  drv.I_scale_analog(false);
  drv.rms_current(CURRENT_mA);
  drv.microsteps(MICROSTEPS_BOOT);
  drv.en_spreadCycle(false); // stealthChop for normal operation
  drv.toff(5);
  drv.blank_time(24);
  drv.TCOOLTHRS(g_TCOOLTHRS);
  drv.TPWMTHRS(g_TPWMTHRS);
  drv.intpol(true);

  Serial.print(F("[TMC] ")); Serial.print(name);
  Serial.print(F(" IFCNT=")); Serial.print(drv.IFCNT());
  Serial.print(F("  msteps=")); Serial.print(drv.microsteps());
  Serial.print(F("  Irms=")); Serial.print(CURRENT_mA); Serial.println(F("mA"));
}

bool readLine(String &out) {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') { out.trim(); return true; }
    out += c;
    if (out.length() > 240) out.remove(0, out.length() - 240);
  }
  return false;
}

// ---------------- Coil helpers ----------------
void coilApplyPWM(uint8_t pwm) {
  g_coilPWM = pwm;
  if (g_coilPWM == 0) {
    digitalWrite(COIL_PIN, LOW);
  } else if (g_coilPWM >= 255) {
    digitalWrite(COIL_PIN, HIGH);
  } else {
    analogWrite(COIL_PIN, g_coilPWM); // default PWM
  }
  Serial.print(F("[COIL] PWM=")); Serial.print(g_coilPWM);
  Serial.print(F(" (")); Serial.print((int)((g_coilPWM * 100UL) / 255UL)); Serial.println(F("%)"));
}

void parseCoil(const String &line) {
  if (!line.startsWith("coil")) return;

  String lower = line; lower.toLowerCase();

  if (lower.equals("coil on") || lower.equals("coilon")) {
    coilApplyPWM(255); return;
  }
  if (lower.equals("coil off") || lower.equals("coiloff")) {
    coilApplyPWM(0); return;
  }

  char *cstr = strdup(line.c_str());
  char *tok = strtok(cstr, " \t");
  tok = strtok(NULL, " \t");
  if (!tok) { Serial.println(F("Usage: coil on|off | coil <0..255> | coil <0..100>%")); free(cstr); return; }

  size_t len = strlen(tok);
  if (len > 0 && tok[len - 1] == '%') {
    tok[len - 1] = '\0';
    int pct = atoi(tok);
    if (pct < 0) pct = 0; if (pct > 100) pct = 100;
    uint8_t pwm = (uint8_t)((pct * 255L) / 100L);
    coilApplyPWM(pwm); free(cstr); return;
  }
  long val = atol(tok);
  if (val < 0) val = 0; if (val > 255) val = 255;
  coilApplyPWM((uint8_t)val);
  free(cstr);
}

// ---- Synchronized trapezoid with accel/decel ----
void runTrapezoidSynchronized(long tx, long ty, long tz, float vmax, float a_default) {
  long cx = stepperX.currentPosition();
  long cy = stepperY.currentPosition();
  long cz = stepperZ.currentPosition();

  long dx = tx - cx, dy = ty - cy, dz = tz - cz;
  long adx = labs(dx), ady = labs(dy), adz = labs(dz);
  long dmax = max(adx, max(ady, adz));
  if (dmax == 0) return;

  float vx = (adx ? vmax * (float)adx / (float)dmax : 0.0f);
  float vy = (ady ? vmax * (float)ady / (float)dmax : 0.0f);
  float vz = (adz ? vmax * (float)adz / (float)dmax : 0.0f);

  if (adx && vx < 5.0f) vx = 5.0f;
  if (ady && vy < 5.0f) vy = 5.0f;
  if (adz && vz < 5.0f) vz = 5.0f;

  float ax = (adx ? a_default * (float)adx / (float)dmax : 0.0f);
  float ay = (ady ? a_default * (float)ady / (float)dmax : 0.0f);
  float az = (adz ? a_default * (float)adz / (float)dmax : 0.0f);

  if (adx && ax < 100.0f) ax = 100.0f;
  if (ady && ay < 100.0f) ay = 100.0f;
  if (adz && az < 100.0f) az = 100.0f;

  stepperX.setMaxSpeed(vx); stepperX.setAcceleration(ax); stepperX.moveTo(tx);
  stepperY.setMaxSpeed(vy); stepperY.setAcceleration(ay); stepperY.moveTo(ty);
  stepperZ.setMaxSpeed(vz); stepperZ.setAcceleration(az); stepperZ.moveTo(tz);

  while ( stepperX.distanceToGo() || stepperY.distanceToGo() || stepperZ.distanceToGo() ) {
    stepperX.run(); stepperY.run(); stepperZ.run();
  }
}

// ---- Execute queued moves with accel/decel ----
void execAllQueuedMoves() {
  if (queueIsEmpty()) {
    Serial.println(F("Queue empty. Nothing to do."));
    return;
  }
  Serial.println(F("Starting queued moves..."));
  enableDrivers(true);
  delay(5);

  Move m;
  uint16_t moveIdx = 0;
  while (dequeueMove(m)) {
    ++moveIdx;

    long targets[3];
    targets[0] = stepperX.currentPosition() + m.dx;
    targets[1] = stepperY.currentPosition() + m.dy;
    targets[2] = stepperZ.currentPosition() + m.dz;

    float vmax = (m.vmax > 0.0f) ? m.vmax : g_defaultSpeed;

    Serial.print(F("Move #")); Serial.print(moveIdx);
    Serial.print(F(" to [")); Serial.print(targets[0]); Serial.print(' ');
    Serial.print(targets[1]); Serial.print(' ');
    Serial.print(targets[2]);
    Serial.print(F("]  vmax=")); Serial.print(vmax);
    Serial.print(F("  accel=")); Serial.println(g_defaultAccel);

    runTrapezoidSynchronized(targets[0], targets[1], targets[2], vmax, g_defaultAccel);
  }

  Serial.println(F("All queued moves complete."));
}

// ---------------- Sensorless Homing ----------------
struct SavedTMC { uint16_t msteps; };
SavedTMC saveDrvX, saveDrvY, saveDrvZ;

void saveDriverState() {
  saveDrvX.msteps = g_microsteps;
  saveDrvY.msteps = g_microsteps;
  saveDrvZ.msteps = g_microsteps;
}

void applyHomingProfileFast() {
  driverX.microsteps(g_homeMicrosteps);
  driverY.microsteps(g_homeMicrosteps);
  driverZ.microsteps(g_homeMicrosteps);

  driverX.SGTHRS(g_sg_fast_x);
  driverY.SGTHRS(g_sg_fast_y);
  driverZ.SGTHRS(g_sg_fast_z);

  driverX.TPWMTHRS(g_TPWMTHRS);
  driverY.TPWMTHRS(g_TPWMTHRS);
  driverZ.TPWMTHRS(g_TPWMTHRS);
  driverX.TCOOLTHRS(g_TCOOLTHRS);
  driverY.TCOOLTHRS(g_TCOOLTHRS);
  driverZ.TCOOLTHRS(g_TCOOLTHRS);

  Serial.print(F("[HOME] FAST: µsteps=")); Serial.print(g_homeMicrosteps);
  Serial.print(F(" SGTHRS=")); Serial.print(g_sg_fast_x); Serial.print('/');
  Serial.print(g_sg_fast_y); Serial.print('/'); Serial.println(g_sg_fast_z);
}

void applyHomingProfileSlow() {
  driverX.microsteps(16);
  driverY.microsteps(16);
  driverZ.microsteps(16);

  driverX.SGTHRS(g_sg_slow_x);
  driverY.SGTHRS(g_sg_slow_y);
  driverZ.SGTHRS(g_sg_slow_z);

  Serial.print(F("[HOME] SLOW: SGTHRS=")); Serial.print(g_sg_slow_x); Serial.print('/');
  Serial.print(g_sg_slow_y); Serial.print('/'); Serial.println(g_sg_slow_z);
}

// --------- IK reference at home ---------
const float HOME_X = 0.0f, HOME_Y = 0.0f, HOME_Z = -266.0f;

void restoreRunProfile() {
  driverX.en_spreadCycle(false);
  driverY.en_spreadCycle(false);
  driverZ.en_spreadCycle(false);

  driverX.microsteps(g_microsteps);
  driverY.microsteps(g_microsteps);
  driverZ.microsteps(g_microsteps);

  Serial.println(F("[HOME] Restored run profile (stealthChop + user microsteps)."));
}

struct Debounce {
  uint8_t cnt = 0;
  bool update(bool asserted) {
    if (asserted) { if (cnt < 10) cnt++; }
    else cnt = 0;
    return cnt >= 4;
  }
};

bool waitForEndstopRelease(uint8_t pin, uint32_t timeout_ms = 1500) {
  uint32_t t0 = millis();
  while (millis() - t0 < timeout_ms) {
    if (!rawEndstopRead(pin)) return true;
    delay(2);
  }
  return false;
}

void homeAll() {
  enableDrivers(true);
  delay(2);

  saveDriverState();
  applyHomingProfileFast();

  stepperX.setAcceleration(g_homeAccel);
  stepperY.setAcceleration(g_homeAccel);
  stepperZ.setAcceleration(g_homeAccel);

  stepperX.setMaxSpeed(g_homeFastSPS);
  stepperY.setMaxSpeed(g_homeFastSPS);
  stepperZ.setMaxSpeed(g_homeFastSPS);

  stepperX.move(200000L);
  stepperY.move(200000L);
  stepperZ.move(200000L);

  Serial.println(F("[HOME] Fast seek..."));
  Debounce dbx, dby, dbz;
  while ( stepperX.distanceToGo() || stepperY.distanceToGo() || stepperZ.distanceToGo() ) {
    if (dbx.update(rawEndstopRead(X_MIN_PIN))) stepperX.stop();
    if (dby.update(rawEndstopRead(Y_MIN_PIN))) stepperY.stop();
    if (dbz.update(rawEndstopRead(Z_MIN_PIN))) stepperZ.stop();
    stepperX.run(); stepperY.run(); stepperZ.run();
  }

  applyHomingProfileSlow();

  float slow = g_homeSlowSPS;
  if (slow < g_sgMinSPS) {
    Serial.print(F("[HOME] Slow speed raised from ")); Serial.print(slow);
    Serial.print(F(" to sgmin ")); Serial.println(g_sgMinSPS);
    slow = g_sgMinSPS;
  }

  stepperX.setMaxSpeed(slow);
  stepperY.setMaxSpeed(slow);
  stepperZ.setMaxSpeed(slow);

  stepperX.move(0);
  stepperY.move(0);
  stepperZ.move(0);

  Debounce dbx2, dby2, dbz2;
  Serial.println(F("[HOME] Slow re-approach..."));
  while ( stepperX.distanceToGo() || stepperY.distanceToGo() || stepperZ.distanceToGo() ) {
    if (dbx2.update(rawEndstopRead(X_MIN_PIN))) stepperX.stop();
    if (dby2.update(rawEndstopRead(Y_MIN_PIN))) stepperY.stop();
    if (dbz2.update(rawEndstopRead(Z_MIN_PIN))) stepperZ.stop();
    stepperX.run(); stepperY.run(); stepperZ.run();
  }

  // Zero joint step counters at the physical home
  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);
  stepperZ.setCurrentPosition(0);

  // >>> Set logical Cartesian reference to (0,0,-266) via IK angles <<<
  float t1,t2,t3;
  if (delta_calcInverse(HOME_X, HOME_Y, HOME_Z, t1, t2, t3) == 0) {
    g_thetaPrev1 = t1; g_thetaPrev2 = t2; g_thetaPrev3 = t3;
    Serial.print(F("[HOME] Logical pose set to X/Y/Z = 0/0/"));
    Serial.print(HOME_Z);
    Serial.print(F("  (θ1/θ2/θ3 = "));
    Serial.print(t1); Serial.print('/');
    Serial.print(t2); Serial.print('/');
    Serial.print(t3); Serial.println(F(" deg)"));
  } else {
    Serial.println(F("[HOME] WARNING: IK(0,0,-266) unreachable; keeping previous angle reference."));
  }

  Serial.println(F("[HOME] Homing done."));

  restoreRunProfile();
}

// ---------------- Delta kinematics (unchanged from your math) ----------------
const float e   = 80.0f;    // end effector
const float f   = 252.9f;   // base
const float re  = 239.8f;
const float rf  = 190.0f;

const float sqrt3 = 1.7320508075688772f;
const float pi    = 3.141592653f;
const float sin120 = sqrt3/2.0f;
const float cos120 = -0.5f;
const float tan60  = sqrt3;
const float sin30  = 0.5f;
const float tan30  = 1.0f/sqrt3;

int delta_calcAngleYZ(float x0, float y0, float z0, float &theta) {
  float y1 = -0.5f * 0.57735f * f; // f/2 * tg 30
  y0 -= 0.5f * 0.57735f * e;       // shift center to edge
  float a = (x0*x0 + y0*y0 + z0*z0 + rf*rf - re*re - y1*y1)/(2.0f*z0);
  float b = (y1 - y0)/z0;
  float d = -(a + b*y1)*(a + b*y1) + rf*(b*b*rf + rf);
  if (d < 0) return -1;
  float yj = (y1 - a*b - sqrtf(d)) / (b*b + 1.0f);
  float zj = a + b*yj;
  theta = 180.0f * atanf(-zj/(y1 - yj)) / pi + ((yj > y1) ? 180.0f : 0.0f);
  return 0;
}

int delta_calcInverse(float x0, float y0, float z0, float &theta1, float &theta2, float &theta3) {
  theta1 = theta2 = theta3 = 0.0f;
  int status = delta_calcAngleYZ(x0, y0, z0, theta1);
  if (status == 0) status = delta_calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120 - x0*sin120, z0, theta2);
  if (status == 0) status = delta_calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120 + x0*sin120, z0, theta3);
  return status;
}

// Stored joint angles (deg) for generating *relative* step moves

// StepRev calculator: 200 * microsteps * 4
long stepRevFromMicrosteps(uint16_t microsteps) {
  return (long)200L * (long)microsteps * 4L;
}

// Parse:  ik x y z [x y z ...]   -> enqueue relative moves from IK
void parseIK(const String &line) {
  if (!line.startsWith("ik")) return;

  char *cstr = strdup(line.c_str());
  char *tok = strtok(cstr, " \t,;");
  tok = strtok(NULL, " \t,;");

  const int MAXTOK = 300; // up to 100 IK points per command
  float vals[MAXTOK];
  int n = 0;
  while (tok && n < MAXTOK) { vals[n++] = atof(tok); tok = strtok(NULL, " \t,;"); }
  free(cstr);

  if (n == 0 || (n % 3) != 0) {
    Serial.println(F("Usage: ik x y z [x y z ...] (multiples of 3 floats)"));
    return;
  }

  long stepRev = stepRevFromMicrosteps(g_microsteps);
  int enq = 0;
  for (int i = 0; i < n; i += 3) {
    float x = vals[i], y = vals[i+1], z = vals[i+2];

    float t1, t2, t3;
    int st = delta_calcInverse(x, y, z, t1, t2, t3);
    if (st != 0) {
      Serial.print(F("[IK] Unreachable target at index ")); Serial.print(i/3);
      Serial.print(F(" -> ")); Serial.print(x); Serial.print(' ');
      Serial.print(y); Serial.print(' '); Serial.println(z);
      break;
    }

    long dx = (long)floorf( ( (g_thetaPrev1 - t1) * stepRev ) / 360.0f );
    long dy = (long)floorf( ( ( -(-g_thetaPrev2 + t2) ) * stepRev ) / 360.0f );
    long dz = (long)floorf( ( ( -(-g_thetaPrev3 + t3) ) * stepRev ) / 360.0f );

    if (!enqueueMove(dx, dy, dz, 0.0f)) { Serial.println(F("Queue full during IK sequence.")); break; }
    enq++;

    g_thetaPrev1 = t1; g_thetaPrev2 = t2; g_thetaPrev3 = t3;
  }
  Serial.print(F("[IK] Enqueued from IK: ")); Serial.println(enq);
}

// ---- multi-move parser (triples only, no per-move speed) ----
void parseMovesTriplesOnly(const String &line) {
  char *cstr = strdup(line.c_str());
  char *tok = strtok(cstr, " \t,;");
  const int MAXTOK = 384; // up to 128 tokens (i.e., 42 moves) per line; queue limits to 64 moves total
  long vals[MAXTOK];
  int n = 0;
  while (tok && n < MAXTOK) { vals[n++] = atol(tok); tok = strtok(NULL, " \t,;"); }
  free(cstr);

  if (n == 0) { Serial.println(F("Unrecognized input. Type 'help' for commands.")); return; }
  if (n % 3 != 0) {
    Serial.println(F("Input count must be multiples of 3: dx dy dz [dx dy dz] ..."));
    return;
  }

  int movesEnq = 0;
  for (int i=0; i<n; i+=3) {
    long dx = vals[i], dy = vals[i+1], dz = vals[i+2];
    if (!enqueueMove(dx,dy,dz,0.0f)) { Serial.println(F("Queue full mid-sequence.")); break; }
    movesEnq++;
  }
  Serial.print(F("Enqueued moves: ")); Serial.println(movesEnq);
}

// -------------------- DANCE --------------------
void doDance() {
  Serial.println(F("[DANCE] Start"));
  // Save current microsteps to restore later
  uint16_t prevMS = g_microsteps;

  // 1) Home
  homeAll();

  // 2) mstep 64, move -7500 -7500 -7500
  Serial.println(F("[DANCE] mstep 64 then move -7500 -7500 -7500"));
  applyMicrostepsAll(64);
  clearQueue();
  enqueueMove(-7500, -7500, -7500, 0.0f);
  execAllQueuedMoves();

  // 3) mstep 4, run the given sequence
  Serial.println(F("[DANCE] mstep 4 then run sequence"));
  applyMicrostepsAll(4);
  clearQueue();
  const long seq[][3] = {
    {  400,  400,  400},
    { -200, -200,    0},
    {  200,  200,    0},
    { -200,    0, -200},
    {  200,    0,  200},
    {    0, -200, -200},
    {    0,  200,  200},
    { -400, -400, -400},
    {  400,  400,  400},
    { -400, -400,    0},
    {  400,  400,    0},
    {    0, -400, -400},
    {    0,  400,  400},
    { -400,    0, -400},
    {  400,    0,  400},
  };
  for (uint8_t i = 0; i < sizeof(seq)/sizeof(seq[0]); ++i) {
    if (!enqueueMove(seq[i][0], seq[i][1], seq[i][2], 0.0f)) {
      Serial.println(F("[DANCE] Queue full unexpectedly."));
      break;
    }
  }
  execAllQueuedMoves();

  // Restore previous microsteps
  Serial.print(F("[DANCE] Restore mstep ")); Serial.println(prevMS);
  applyMicrostepsAll(prevMS);
  Serial.println(F("[DANCE] Done"));
}

// ---------------- Setup / Loop ----------------
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println(F("\n== RAMPS 1.4 + TMC2209 UART (Hydra on pin 63) | Queue + Accel + Sensorless Home + Coil =="));
  printHelp();
  Serial.println();

  pinMode(X_ENABLE_PIN, OUTPUT);
  pinMode(Y_ENABLE_PIN, OUTPUT);
  pinMode(Z_ENABLE_PIN, OUTPUT);
  enableDrivers(false);

  pinMode(X_MIN_PIN, INPUT_PULLUP);
  pinMode(Y_MIN_PIN, INPUT_PULLUP);
  pinMode(Z_MIN_PIN, INPUT_PULLUP);

  // Coil output setup: default OFF
  pinMode(COIL_PIN, OUTPUT);
  digitalWrite(COIL_PIN, LOW);

  stepperX.setPinsInverted(X_DIR_INVERT, false, false);
  stepperY.setPinsInverted(Y_DIR_INVERT, false, false);
  stepperZ.setPinsInverted(Z_DIR_INVERT, false, false);

  stepperX.setMaxSpeed(DEFAULT_MAX_SPEED_SPS);
  stepperY.setMaxSpeed(DEFAULT_MAX_SPEED_SPS);
  stepperZ.setMaxSpeed(DEFAULT_MAX_SPEED_SPS);

  stepperX.setMinPulseWidth(3);
  stepperY.setMinPulseWidth(3);
  stepperZ.setMinPulseWidth(3);

  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);
  stepperZ.setCurrentPosition(0);

  multi.addStepper(stepperX);
  multi.addStepper(stepperY);
  multi.addStepper(stepperZ);

  // Init drivers on shared UART
  initDriver(driverX, "X");
  initDriver(driverY, "Y");
  initDriver(driverZ, "Z");

  // Initial thresholds on boot (informational)
  driverX.SGTHRS(g_sg_fast_x); driverY.SGTHRS(g_sg_fast_y); driverZ.SGTHRS(g_sg_fast_z);
  driverX.TPWMTHRS(g_TPWMTHRS); driverY.TPWMTHRS(g_TPWMTHRS); driverZ.TPWMTHRS(g_TPWMTHRS);
  driverX.TCOOLTHRS(g_TCOOLTHRS); driverY.TCOOLTHRS(g_TCOOLTHRS); driverZ.TCOOLTHRS(g_TCOOLTHRS);

  Serial.print(F("[TMC] GCONF X/Y/Z = 0x"));
  Serial.print(driverX.GCONF(), HEX); Serial.print(F("/0x"));
  Serial.print(driverY.GCONF(), HEX); Serial.print(F("/0x"));
  Serial.println(driverZ.GCONF(), HEX);

  // Ensure coil reflects default PWM (0)
  coilApplyPWM(0);
}

void parseSpeedCommand(const String &line) {
  String lower = line; lower.toLowerCase();
  if (!lower.startsWith("speed")) return;
  char *cstr = strdup(line.c_str());
  char *tok = strtok(cstr, " \t");
  float v = 0; bool ok = false;
  while ((tok = strtok(NULL, " \t")) != NULL) { v = atof(tok); if (v > 0) { ok = true; break; } }
  if (ok) { g_defaultSpeed = v; Serial.print(F("Default speed set to ")); Serial.println(g_defaultSpeed); }
  else    { Serial.println(F("Usage: speed <steps_per_second> e.g. speed 9000")); }
  free(cstr);
}

void parseAccelCommand(const String &line) {
  String lower = line; lower.toLowerCase();
  if (!lower.startsWith("accel")) return;
  char *cstr = strdup(line.c_str());
  char *tok = strtok(cstr, " \t");
  float a = 0; bool ok = false;
  while ((tok = strtok(NULL, " \t")) != NULL) { a = atof(tok); if (a > 0) { ok = true; break; } }
  if (ok) { g_defaultAccel = a; Serial.print(F("Default acceleration set to ")); Serial.println(g_defaultAccel); }
  else    { Serial.println(F("Usage: accel <steps_per_second_squared> e.g. accel 8000")); }
  free(cstr);
}

void parseMstepCommand(const String &line) {
  String lower = line; lower.toLowerCase();
  if (!lower.startsWith("mstep")) return;
  char *cstr = strdup(line.c_str());
  char *tok = strtok(cstr, " \t");
  long ms = 0; bool ok = false;
  while ((tok = strtok(NULL, " \t")) != NULL) { ms = atol(tok); if (ms > 0) { ok = true; break; } }
  if (ok) applyMicrostepsAll((uint16_t)ms);
  else    Serial.println(F("Usage: mstep <1|2|4|8|16|32|64|128|256>"));
  free(cstr);
}

void parseStepsPerMM(const String &line) {
  if (!line.startsWith("stepsmm")) return;
  float a, b, c;
  if (sscanf(line.c_str(), "stepsmm %f %f %f", &a, &b, &c) == 3) {
    STEPS_PER_MM_X = a; STEPS_PER_MM_Y = b; STEPS_PER_MM_Z = c;
    Serial.print(F("Steps/mm set to ")); Serial.print(a); Serial.print(' ');
    Serial.print(b); Serial.print(' '); Serial.println(c);
  } else {
    Serial.println(F("Usage: stepsmm <x> <y> <z>"));
  }
}

void parseHomeMs(const String &line) {
  if (!line.startsWith("homems")) return;
  long ms = 0;
  if (sscanf(line.c_str(), "homems %ld", &ms) == 1 && ms > 0) {
    if      (ms >= 256) g_homeMicrosteps = 256;
    else if (ms >= 128) g_homeMicrosteps = 128;
    else if (ms >=  64) g_homeMicrosteps =  64;
    else if (ms >=  32) g_homeMicrosteps =  32;
    else if (ms >=  16) g_homeMicrosteps =  16;
    else if (ms >=   8) g_homeMicrosteps =   8;
    else if (ms >=   4) g_homeMicrosteps =   4;
    else if (ms >=   2) g_homeMicrosteps =   2;
    else                g_homeMicrosteps =   1;
    Serial.print(F("Homing microsteps set to ")); Serial.println(g_homeMicrosteps);
  } else {
    Serial.println(F("Usage: homems <1|2|4|8|16|32|64|128|256>"));
  }
}

void parseHomeV(const String &line) {
  if (!line.startsWith("homev")) return;
  float f, s;
  if (sscanf(line.c_str(), "homev %f %f", &f, &s) == 2 && f > 0 && s > 0) {
    g_homeFastSPS = f; g_homeSlowSPS = s;
    Serial.print(F("Homing speeds set fast/slow = ")); Serial.print(f);
    Serial.print('/'); Serial.println(s);
  } else {
    Serial.println(F("Usage: homev <fast_sps> <slow_sps> e.g. homev 4000 2000"));
  }
}

void parseHomeA(const String &line) {
  if (!line.startsWith("homea")) return;
  float a;
  if (sscanf(line.c_str(), "homea %f", &a) == 1 && a > 0) {
    g_homeAccel = a;
    Serial.print(F("Homing acceleration set to ")); Serial.println(g_homeAccel);
  } else {
    Serial.println(F("Usage: homea <accel_sps2> e.g. homea 4000"));
  }
}

void parseHomeBack(const String &line) {
  if (!line.startsWith("homeback")) return;
  long v;
  if (sscanf(line.c_str(), "homeback %ld", &v) == 1 && v > 0) {
    g_homeBackoff = v;
    Serial.print(F("Homing backoff set to ")); Serial.println(g_homeBackoff);
  } else {
    Serial.println(F("Usage: homeback <steps> e.g. homeback 600"));
  }
}

void parseSGFast(const String &line) {
  if (!line.startsWith("sgfast")) return;
  int x, y, z;
  if (sscanf(line.c_str(), "sgfast %d %d %d", &x, &y, &z) == 3) {
    g_sg_fast_x = constrain(x, 0, 255);
    g_sg_fast_y = constrain(y, 0, 255);
    g_sg_fast_z = constrain(z, 0, 255);
    Serial.print(F("FAST SGTHRS X/Y/Z = ")); Serial.print(g_sg_fast_x); Serial.print('/');
    Serial.print(g_sg_fast_y); Serial.print('/'); Serial.println(g_sg_fast_z);
  } else {
    Serial.println(F("Usage: sgfast <X> <Y> <Z>"));
  }
}

void parseSGSlow(const String &line) {
  if (!line.startsWith("sgslow")) return;
  int x, y, z;
  if (sscanf(line.c_str(), "sgslow %d %d %d", &x, &y, &z) == 3) {
    g_sg_slow_x = constrain(x, 0, 255);
    g_sg_slow_y = constrain(y, 0, 255);
    g_sg_slow_z = constrain(z, 0, 255);
    Serial.print(F("SLOW SGTHRS X/Y/Z = ")); Serial.print(g_sg_slow_x); Serial.print('/');
    Serial.print(g_sg_slow_y); Serial.print('/'); Serial.println(g_sg_slow_z);
  } else {
    Serial.println(F("Usage: sgslow <X> <Y> <Z>"));
  }
}

void parseSGMin(const String &line) {
  if (!line.startsWith("sgmin")) return;
  float v;
  if (sscanf(line.c_str(), "sgmin %f", &v) == 1 && v > 0) {
    g_sgMinSPS = v;
    Serial.print(F("SG minimum speed set to ")); Serial.println(g_sgMinSPS);
  } else {
    Serial.println(F("Usage: sgmin <steps_per_second> e.g. sgmin 1500"));
  }
}

void parseTPWM(const String &line) {
  if (!line.startsWith("tpwm")) return;
  unsigned long v;
  if (sscanf(line.c_str(), "tpwm %lu", &v) == 1) {
    g_TPWMTHRS = v;
    driverX.TPWMTHRS(v); driverY.TPWMTHRS(v); driverZ.TPWMTHRS(v);
    Serial.print(F("TPWMTHRS set to ")); Serial.println(v);
  } else {
    Serial.println(F("Usage: tpwm <value>"));
  }
}

void parseTCOOL(const String &line) {
  if (!line.startsWith("tcool")) return;
  unsigned long v;
  if (sscanf(line.c_str(), "tcool %lu", &v) == 1) {
    g_TCOOLTHRS = v;
    driverX.TCOOLTHRS(v); driverY.TCOOLTHRS(v); driverZ.TCOOLTHRS(v);
    Serial.print(F("TCOOLTHRS set to ")); Serial.println(v);
  } else {
    Serial.println(F("Usage: tcool <value>"));
  }
}

void parseEndInv(const String &line) {
  if (!line.startsWith("endi")) return;
  int v;
  if (sscanf(line.c_str(), "endi %d", &v) == 1) {
    g_endstopInvert = (v != 0);
    Serial.print(F("Endstop invert = ")); Serial.println(g_endstopInvert ? "ON" : "OFF");
  } else {
    Serial.println(F("Usage: endi <0|1>"));
  }
}

void loop() {
  static String line;
  if (readLine(line)) {
    if (line.length() == 0) { line = ""; return; }

    if      (line.equalsIgnoreCase("start"))     { execAllQueuedMoves(); }
    else if (line.equalsIgnoreCase("clear"))     { clearQueue(); Serial.println(F("Queue cleared.")); }
    else if (line.equalsIgnoreCase("list"))      { printQueue(); }
    else if (line.equalsIgnoreCase("help"))      { printHelp(); }
    else if (line.equalsIgnoreCase("status")) {
      Serial.print(F("Default speed = ")); Serial.println(g_defaultSpeed);
      Serial.print(F("Default accel = ")); Serial.println(g_defaultAccel);
      Serial.print(F("Microsteps     = ")); Serial.println(g_microsteps);
      Serial.print(F("Home µsteps    = ")); Serial.println(g_homeMicrosteps);
      Serial.print(F("Home V fast/sl = ")); Serial.print(g_homeFastSPS); Serial.print('/'); Serial.println(g_homeSlowSPS);
      Serial.print(F("Home accel     = ")); Serial.println(g_homeAccel);
      Serial.print(F("Backoff steps  = ")); Serial.println(g_homeBackoff);
      Serial.print(F("FAST SG X/Y/Z  = ")); Serial.print(g_sg_fast_x); Serial.print('/'); Serial.print(g_sg_fast_y); Serial.print('/'); Serial.println(g_sg_fast_z);
      Serial.print(F("SLOW SG X/Y/Z  = ")); Serial.print(g_sg_slow_x); Serial.print('/'); Serial.print(g_sg_slow_y); Serial.print('/'); Serial.println(g_sg_slow_z);
      Serial.print(F("SG min speed   = ")); Serial.println(g_sgMinSPS);
      Serial.print(F("TPWMTHRS       = ")); Serial.println(g_TPWMTHRS);
      Serial.print(F("TCOOLTHRS      = ")); Serial.println(g_TCOOLTHRS);
      Serial.print(F("Endstop invert = ")); Serial.println(g_endstopInvert ? "ON" : "OFF");
      Serial.print(F("Coil PWM       = ")); Serial.print(g_coilPWM);
      Serial.print(F(" (")); Serial.print((int)((g_coilPWM * 100UL) / 255UL)); Serial.println(F("%)"));
    }
    else if (line.equalsIgnoreCase("S") || line.equalsIgnoreCase("Stop") || line.equalsIgnoreCase("estop")) {
      enableDrivers(false);
      Serial.println(F("EMERGENCY STOP - drivers disabled."));
    }
    else if (line.equalsIgnoreCase("p")) {
      Serial.print(F("Pos [X Y Z] = "));
      Serial.print(stepperX.currentPosition()); Serial.print(' ');
      Serial.print(stepperY.currentPosition()); Serial.print(' ');
      Serial.println(stepperZ.currentPosition());
    }
    else if (line.equalsIgnoreCase("home"))      { homeAll(); }
    else if (line.equalsIgnoreCase("dance"))     { doDance(); }
    else if (line.startsWith("coil"))            { parseCoil(line); }
    else if (line.startsWith("ik"))              { parseIK(line); }          // IK
    else if (line.startsWith("speed"))           { parseSpeedCommand(line); }
    else if (line.startsWith("accel"))           { parseAccelCommand(line); }
    else if (line.startsWith("mstep"))           { parseMstepCommand(line); }
    else if (line.startsWith("stepsmm"))         { parseStepsPerMM(line); }
    else if (line.startsWith("homems"))          { parseHomeMs(line); }
    else if (line.startsWith("homev"))           { parseHomeV(line); }
    else if (line.startsWith("homea"))           { parseHomeA(line); }
    else if (line.startsWith("homeback"))        { parseHomeBack(line); }
    else if (line.startsWith("sgfast"))          { parseSGFast(line); }
    else if (line.startsWith("sgslow"))          { parseSGSlow(line); }
    else if (line.startsWith("sgmin"))           { parseSGMin(line); }
    else if (line.startsWith("tpwm"))            { parseTPWM(line); }
    else if (line.startsWith("tcool"))           { parseTCOOL(line); }
    else if (line.startsWith("endi"))            { parseEndInv(line); }
    else {
      // numbers → enqueue in triples (no per-move speed)
      parseMovesTriplesOnly(line);
    }
    line = "";
  }
}
