// Determine the correct UPRIGHT offset for the P3022 sensor WITHOUT recompiling.  
// Set it live over Serial, confirm it, then copy the final number into arduino_IP.ino.

const uint8_t PIN_ANGLE = A0;               
const float   VREF              = 5.0;
const float   ADC_FULLSCALE     = 1023.0;
const float   DEGREES_PER_COUNT = 360.0 / 1024.0;
const int     SENSOR_SIGN       = +1;

int ANGLE_OFFSET_COUNTS = 538; // edit

// Average several reads so noise doesn't bias the captured zero.
int readAveragedRaw(int n = 200) {
  long sum = 0;
  for (int i = 0; i < n; i++) { sum += analogRead(PIN_ANGLE); delay(2); }
  return (int)(sum / n);
}

float toAngle(int raw) {
  return SENSOR_SIGN * (raw - ANGLE_OFFSET_COUNTS) * DEGREES_PER_COUNT;
}

void printStatus() {
  int   raw  = analogRead(PIN_ANGLE);
  float volt = raw * VREF / ADC_FULLSCALE;
  Serial.print(F("OFFSET=")); Serial.print(ANGLE_OFFSET_COUNTS);
  Serial.print(F("  raw="));  Serial.print(raw);
  Serial.print(F("  volt=")); Serial.print(volt, 3);
  Serial.print(F("  angle="));Serial.print(toAngle(raw), 2);
  Serial.print(F(" deg  (deg/count="));
  Serial.print(DEGREES_PER_COUNT, 4); Serial.println(F(")"));
}

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  zero        capture current (averaged) position as upright zero"));
  Serial.println(F("  offset <n>  set offset to raw count <n> manually"));
  Serial.println(F("  status      show offset + live raw/volt/angle"));
  Serial.println(F("  help        show this list"));
}

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd == "zero") {
    ANGLE_OFFSET_COUNTS = readAveragedRaw();
    Serial.print(F("Captured upright. New OFFSET=")); Serial.println(ANGLE_OFFSET_COUNTS);
    Serial.println(F("-> copy this number into ANGLE_OFFSET_COUNTS in arduino_IP.ino"));
  }
  else if (cmd.startsWith("offset")) {
    ANGLE_OFFSET_COUNTS = cmd.substring(6).toInt();
    Serial.print(F("OFFSET set to ")); Serial.println(ANGLE_OFFSET_COUNTS);
  }
  else if (cmd == "status") { printStatus(); }
  else if (cmd == "help")   { printHelp();  }
  else if (cmd.length())    { Serial.print(F("Unknown command: ")); Serial.println(cmd);
                              Serial.println(F("Type 'help'.")); }
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("# P3022 Sensor Calibration"));
  Serial.println(F("# Set line ending to 'Newline'. Type 'help' for commands."));
  printHelp();
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleCommand(line);
  }
}
