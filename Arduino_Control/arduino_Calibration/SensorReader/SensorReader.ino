// Verify the P3022 rotary sensor is wired correctly and read sensibly.
// Continuously prints:  raw ADC count, voltage, estimated angle.

// Pin (from arduino_IP.ino)
const uint8_t PIN_ANGLE = A0;

// ADC / sensor scale (from arduino_IP.ino)
const float   VREF             = 5.0;
const float   ADC_FULLSCALE    = 1023.0;
const float   DEGREES_PER_COUNT = 360.0 / 1024.0;
int           ANGLE_OFFSET_COUNTS = 538;
const int     SENSOR_SIGN      = +1;         

void setup() {
  Serial.begin(115200);
  // Instructions printed ONCE so they don't pollute the Serial Plotter stream.
  Serial.println(F("# P3022 Sensor Reader"));
  Serial.println(F("# Open Serial Monitor to read numbers, or Serial Plotter to graph."));
  Serial.println(F("# Columns -> raw:<counts>  volt:<V>  angle:<deg>"));
  Serial.println(F("#"));
  Serial.println(F("# HOW TO FIND OFFSET & SCALE:"));
  Serial.println(F("#  OFFSET: hold the pendulum exactly upright, note 'raw'. That count"));
  Serial.println(F("#          is your ANGLE_OFFSET_COUNTS (use SensorCalibration to set it)."));
  Serial.println(F("#  SCALE : rotate the shaft a known angle (e.g. 90 deg against a stop),"));
  Serial.println(F("#          note the change in 'raw'. degrees_per_count = 90 / (delta raw)."));
  Serial.println(F("#          For a 360-deg P3022 this is ~360/1024; verify it matches."));
  Serial.println(F("#  SIGN  : if angle moves the wrong way vs reality, set SENSOR_SIGN=-1."));
}

void loop() {
  int   raw   = analogRead(PIN_ANGLE);
  float volt  = raw * VREF / ADC_FULLSCALE;
  float angle = SENSOR_SIGN * (raw - ANGLE_OFFSET_COUNTS) * DEGREES_PER_COUNT;

  // "label:value" format is readable in Serial Monitor AND graphs in Serial Plotter.
  Serial.print(F("raw:"));   Serial.print(raw);
  Serial.print(F("\tvolt:")); Serial.print(volt, 3);
  Serial.print(F("\tangle:"));Serial.println(angle, 2);

  delay(50);   // ~20 Hz: smooth on the plotter, readable on the monitor
}
