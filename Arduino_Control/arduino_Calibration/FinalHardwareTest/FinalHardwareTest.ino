// Confirm the sensor AND motor work together before enabling closed-loop control.  
// Drives the motor while continuously displaying the live sensor reading and the current motor state.

// sensor (from arduino_IP.ino)
const uint8_t PIN_ANGLE = A0;
const float   VREF              = 5.0;
const float   ADC_FULLSCALE     = 1023.0;
const float   DEGREES_PER_COUNT = 360.0 / 1024.0;
const int     SENSOR_SIGN       = +1;
int           ANGLE_OFFSET_COUNTS = 538;

// motor (from arduino_IP.ino)
const uint8_t PIN_STEP = 2;
const uint8_t PIN_DIR  = 3;
const uint8_t PIN_EN   = 4;
const bool    ENABLE_ACTIVE_LOW = true;
const unsigned int  STEP_PULSE_US        = 3;
const unsigned long MIN_STEP_INTERVAL_US = 200;
const unsigned long MAX_STEP_INTERVAL_US = 6100;

// run state
unsigned long stepInterval = 2500;
unsigned long lastStepUs   = 0;
unsigned long lastFlipMs   = 0;
unsigned long lastPrintMs  = 0;
const unsigned long REVERSE_PERIOD_MS = 1000;
const unsigned long PRINT_PERIOD_MS   = 200;
bool running = false;
int  dir     = +1;

void enableDriver(bool on) {
  bool pinHigh = ENABLE_ACTIVE_LOW ? (!on) : on;
  digitalWrite(PIN_EN, pinHigh ? HIGH : LOW);
}

void printStatus() {
  int   raw   = analogRead(PIN_ANGLE);
  float volt  = raw * VREF / ADC_FULLSCALE;
  float angle = SENSOR_SIGN * (raw - ANGLE_OFFSET_COUNTS) * DEGREES_PER_COUNT;
  Serial.print(F("raw:"));    Serial.print(raw);
  Serial.print(F("\tvolt:"));  Serial.print(volt, 3);
  Serial.print(F("\tangle:")); Serial.print(angle, 2);
  Serial.print(F("\tdir:"));   Serial.print(running ? (dir > 0 ? F("FWD") : F("REV")) : F("STOP"));
  Serial.print(F("\tinterval:"));Serial.println(running ? stepInterval : 0UL);
}

void setup() {
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR,  OUTPUT);
  pinMode(PIN_EN,   OUTPUT);
  digitalWrite(PIN_STEP, LOW);
  enableDriver(false);

  Serial.begin(115200);
  Serial.println(F("# Final Hardware Test (sensor + motor together)"));
  Serial.println(F("# g=go  s=stop  +=faster  -=slower"));
}

void loop() {
  // commands
  if (Serial.available()) {
    char c = Serial.read();
    if      (c == 'g') { running = true;  enableDriver(true);  lastFlipMs = millis(); }
    else if (c == 's') { running = false; enableDriver(false); }
    else if (c == '+') { if (stepInterval > MIN_STEP_INTERVAL_US) stepInterval -= 200; }
    else if (c == '-') { if (stepInterval < MAX_STEP_INTERVAL_US) stepInterval += 200; }
  }

  // auto-reverse so the cart doesn't run off the rail
  if (running && millis() - lastFlipMs >= REVERSE_PERIOD_MS) {
    lastFlipMs = millis();
    dir = -dir;
  }

  // non-blocking stepping
  if (running) {
    digitalWrite(PIN_DIR, (dir > 0) ? HIGH : LOW);
    unsigned long nowUs = micros();
    if (nowUs - lastStepUs >= stepInterval) {
      lastStepUs = nowUs;
      digitalWrite(PIN_STEP, HIGH);
      delayMicroseconds(STEP_PULSE_US);
      digitalWrite(PIN_STEP, LOW);
    }
  }

  // live readout (sensor + motor state)
  if (millis() - lastPrintMs >= PRINT_PERIOD_MS) {
    lastPrintMs = millis();
    printStatus();
  }
}
