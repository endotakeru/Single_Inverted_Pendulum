#include <Arduino.h>

// ***PINs***
const uint8_t PIN_ANGLE = A0; // P3022 output, A0
const uint8_t PIN_STEP = 2; // STEP, D2
const uint8_t PIN_DIR = 3; // Direction, D3
const uint8_t PIN_EN = 4; // Enable, D4

// ***Motor driver, LOW = motor ON***
const bool ENABLE_ACTIVE_LOW = true;

// ***Constants***
const unsigned long CONTROL_INTERVAL_US = 5000;

// STEP-pulse spacing limits
const unsigned long MIN_STEP_INTERVAL_US = 500;
const unsigned long MAX_STEP_INTERVAL_US = 6100;
const unsigned int STEP_PULSE_US = 3; // STEP signal width

// PD Output = FORCE (Newtons) -> accel -> integrate to velocity -> step rate 
const float CART_MASS_KG     = 0.226; // cart-accel effective mass = Delta/C
const float MAX_CART_FORCE_N = 60.0;  // motor force limit
const int   DIR_SIGN       = 1;   // cart direction

// Safety
const float SAFETY_ANGLE_DEG = 90; 
const float MAX_X_M = 0.22; // usable half-travel
const int SAFETY_TRIP_COUNT = 7; // short noise debounce

// Homing
const bool HOME_ON_STARTUP = true;
const int HOMING_DIR = +1;
const unsigned long HOMING_STEP_INTERVAL_US = 8000; // slow

// Arming
const float ARM_ANGLE_DEG = 5.0;         // initial disturbance window
const unsigned long ARM_STABLE_MS = 500; // stay there for this long

// Serial debug
const bool DEBUG_ENABLED = true;
const unsigned long DEBUG_INTERVAL_MS = 200; // print interval

// Sensor Calibration Values
const float DEGREES_PER_COUNT = 360.0/1024.0; // Arduino = 10 bit

int ANGLE_OFFSET_COUNTS = 538; // Upright ADC Value
const int SENSOR_SIGN = +1;
const float TARGET_ANGLE_DEG = 0.0;
const float TARGET_X_M = 0.0;

// Cart drive geometry
const int   FULL_STEPS_PER_REV = 200;   // 1.8 deg NEMA 17
const int   MICROSTEPS         = 1;    // DRV8825 microstep setting
const float PULLEY_CIRCUM_M    = 0.040; // GT2 20T pulley: 2 mm pitch * 20T
const float METERS_PER_STEP    = PULLEY_CIRCUM_M / (FULL_STEPS_PER_REV * MICROSTEPS);
const float V_MAX = METERS_PER_STEP / (MIN_STEP_INTERVAL_US * 1e-6f); 
const float V_MIN = METERS_PER_STEP / (MAX_STEP_INTERVAL_US * 1e-6f); 
const float DEG2RAD = PI / 180.0f;

// PD Gains
float kp_theta = 11.0, kd_theta = 1.7;
float kp_x = 2.0, kd_x = 1.2;

// float kp_theta = 12.0, kd_theta = 2.4;   // saved set
// float kp_x = 1.7, kd_x = 1.3;

const float TAU_D = 0.02; // derivative low-pass filter constant

// ***Variable Control Variables***
int rawAngle = 0; // prev raw ADC
float currentAngleDeg = 0.0;
float cartX_m = 0.0;

// PD variables
float thetaErr=0, thetaDeriv=0, thetaPrev=0;
float xErr=0, xDeriv=0, xPrev=0;
float force = 0.0;
float vCmd  = 0.0;
long stepCount = 0; // net steps from center
unsigned long lastControlUs = 0, lastStepUs = 0, lastDebugMs=0; // when things last happened
bool safetyTripped = false;

// ***Functions***

// angle & pos reading
float readAngle() {
  const int N_SAMPLES       = 6;   // oversample
  const int MAX_JUMP_COUNTS = 40;  // ~14 deg/tick (bigger jump = noise)
  static int rejectCount = 0;      // consecutive rejected readings

  long sum = 0;
  for (int i = 0; i < N_SAMPLES; i++) sum += analogRead(PIN_ANGLE); // oversample
  int avg = (int)(sum / N_SAMPLES); // average

  if (avg > 5 && avg < 1018 && abs(avg - rawAngle) < MAX_JUMP_COUNTS) {
    rawAngle = avg; rejectCount = 0;   // plausible -> accept
  } else if (++rejectCount >= 4) {
    rawAngle = avg; rejectCount = 0;   // persisted 4 ticks -> accept
  }                                    // else: hold last good rawAngle

  return SENSOR_SIGN * (rawAngle - ANGLE_OFFSET_COUNTS) * DEGREES_PER_COUNT;
}

float readCartPosition() {
  return stepCount * METERS_PER_STEP;
}

void computePD(float dt) {
  if (dt <= 0.0f) return; // if dt too small, don't compute PD

  // errors (rad, rad/s, m, m/s)
  float thErr = (currentAngleDeg - TARGET_ANGLE_DEG) * DEG2RAD;
  float rawDth = (thErr - thetaPrev) / dt;
  thetaDeriv += (rawDth - thetaDeriv) * (dt / (TAU_D + dt)); // filtered
  thetaPrev = thErr;

  float xE = cartX_m - TARGET_X_M;
  float rawDx = (xE - xPrev) / dt;
  xDeriv += (rawDx - xDeriv) * (dt / (TAU_D + dt)); // filtered
  xPrev = xE; 

  // PD Output (Newtons)
  force = kp_theta * thErr + kd_theta * thetaDeriv + kp_x * xE + kd_x * xDeriv;

  // clamp force to limit
  force = constrain(force, -MAX_CART_FORCE_N, MAX_CART_FORCE_N);

  // force -> cart acceleration -> integrate to velocity
  float aCmd = force / CART_MASS_KG;
  vCmd += aCmd * dt; 
  if (vCmd >  V_MAX) vCmd =  V_MAX; 
  if (vCmd < -V_MAX) vCmd = -V_MAX; 

  thetaErr = thErr; // radians
  xErr = xE; 
}
// Motor driver
void driveStepper() {
  float speed = fabs(vCmd);
  if (speed < V_MIN) return; // slower than min step rate -> hold 

  int dir = (DIR_SIGN * vCmd >= 0.0f) ? +1 : -1; 
  digitalWrite(PIN_DIR, (dir > 0) ? HIGH : LOW);

  unsigned long interval = (unsigned long)(METERS_PER_STEP / speed * 1e6f); // us per step
  if (interval < MIN_STEP_INTERVAL_US) interval = MIN_STEP_INTERVAL_US; 
  if (interval > MAX_STEP_INTERVAL_US) interval = MAX_STEP_INTERVAL_US; 

  unsigned long nowUs = micros();
  if (nowUs - lastStepUs >= interval) {
    lastStepUs = nowUs;
    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(PIN_STEP, LOW);
    stepCount += dir; 
  }
}

// Safety check
void safetyCheck() {
  static int overAngleCount = 0;

  // Angle comes from the ADC, which can spike from motor noise -> debounce it.
  if (fabs(currentAngleDeg) > SAFETY_ANGLE_DEG) {
    if (++overAngleCount >= SAFETY_TRIP_COUNT) safetyTripped = true;
  } else {
    overAngleCount = 0; // any good reading clears the count (only consecutive spikes matter)
  }

  // Position comes from stepCount (a clean integer, never spikes) -> trip immediately.
  if (fabs(cartX_m) > MAX_X_M) safetyTripped = true;

  if (safetyTripped) {
    digitalWrite(PIN_STEP, LOW);
    enableDriver(false);
  }
}

void enableDriver(bool on) {
  bool pinHigh = ENABLE_ACTIVE_LOW ? (!on) : on;
  digitalWrite(PIN_EN, pinHigh ? HIGH : LOW);
}

// Homing slow step pulse
void homePulse() {
  digitalWrite(PIN_STEP, HIGH);
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(PIN_STEP, LOW);
  delayMicroseconds(HOMING_STEP_INTERVAL_US);
}

// Homing
void homeCart() {
  long fullTravel = (long)(2.0f * MAX_X_M / METERS_PER_STEP); // full travel in steps
  long seat = (long)(fullTravel * 1.2f); // slight overshoot length
  long half = fullTravel/2; // half travel in steps

  // drive towards end
  digitalWrite(PIN_DIR, (HOMING_DIR > 0) ? HIGH : LOW);
  for (long i = 0; i < seat; i++) homePulse();

  // back off to the center
  digitalWrite(PIN_DIR, (HOMING_DIR > 0) ? LOW : HIGH);
  for (long i = 0; i < half; i++) homePulse();

  stepCount = 0; // x = 0
}

// Arming gate after homing
void waitForUpright() {
  unsigned long inWindowSince = 0; // millis() when we entered the window (0 = not in)
  unsigned long lastMsgMs = 0;
  Serial.println(F("# Homing done. Raise the pendulum upright to arm..."));

  for (;;) {
    currentAngleDeg = readAngle();
    unsigned long nowMs = millis();

    if (fabs(currentAngleDeg - TARGET_ANGLE_DEG) <= ARM_ANGLE_DEG) {
      if (inWindowSince == 0) inWindowSince = nowMs;   
      if (nowMs - inWindowSince >= ARM_STABLE_MS) {    
        Serial.println(F("# Armed. Balancing..."));
        return;
      }
    } else {
      inWindowSince = 0;                               
    }

    if (nowMs - lastMsgMs >= 500) {                    
      lastMsgMs = nowMs;
      Serial.print(F("# waiting... angle=")); Serial.print(currentAngleDeg, 1);
      Serial.println(F(" deg"));
    }
  }
}

// Debug
void debugPrint() {
  Serial.print("raw=");      Serial.print(rawAngle);
  Serial.print("  th=");     Serial.print(currentAngleDeg, 2);
  Serial.print("deg  x=");   Serial.print(cartX_m * 100.0, 2);
  Serial.print("cm  e_th="); Serial.print(thetaErr * 180.0 / PI, 2); 
  Serial.print("deg  e_x="); Serial.print(xErr * 100.0, 2); 
  Serial.print("cm  F=");    Serial.print(force, 2); 
  Serial.print("N  v=");     Serial.print(vCmd, 3); 
  Serial.print("  steps=");  Serial.print(stepCount);
  if (safetyTripped) Serial.print("   [SAFETY TRIP - press RESET]");
  Serial.println();
}

// ***Main Functions***

void setup() {
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_STEP, LOW);
  digitalWrite(PIN_DIR, LOW);
  enableDriver(true);

  Serial.begin(115200); // baud rate

  if (HOME_ON_STARTUP) homeCart();
  else stepCount = 0; // if homing = false, current position = center

  waitForUpright();

  thetaPrev = (currentAngleDeg - TARGET_ANGLE_DEG) * DEG2RAD;
  thetaDeriv = 0;
  xPrev = 0; xDeriv = 0;
  force = 0; vCmd = 0;

  unsigned long now = micros(); // current time [Us]
  lastControlUs = now; lastStepUs = now; lastDebugMs = millis();
}

void loop() {
  unsigned long nowUs = micros();

  if (nowUs - lastControlUs >= CONTROL_INTERVAL_US){
    float dt = (nowUs-lastControlUs)*1e-6f;
    lastControlUs = nowUs;

    currentAngleDeg = readAngle();
    cartX_m = readCartPosition();
    safetyCheck();

    // print one snapshot the instant the safety trips
    static bool tripPrinted = false;
    if (safetyTripped && !tripPrinted) { debugPrint(); tripPrinted = true; }

    if (!safetyTripped) computePD(dt); // PD
  }

  if (!safetyTripped) driveStepper();

  // periodic debug (one last line prints when trips)
  if (DEBUG_ENABLED && !safetyTripped && (millis() - lastDebugMs >= DEBUG_INTERVAL_MS)) {
    lastDebugMs = millis();
    debugPrint();
  }
}
