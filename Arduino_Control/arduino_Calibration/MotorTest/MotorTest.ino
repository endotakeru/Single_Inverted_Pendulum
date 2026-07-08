// Verify the DRV8825 driver and stepper motor turn correctly, both directions, at a controllable speed.

const uint8_t PIN_STEP = 2;         
const uint8_t PIN_DIR  = 3;          
const uint8_t PIN_EN   = 4;            
const bool    ENABLE_ACTIVE_LOW = true;  

const unsigned int  STEP_PULSE_US        = 3;    
const unsigned long MIN_STEP_INTERVAL_US = 500;   
const unsigned long MAX_STEP_INTERVAL_US = 6000;  

unsigned long stepInterval = 2500;     // current speed
unsigned long lastStepUs   = 0;
bool running   = false;
int  dir       = +1;

void enableDriver(bool on) {
  bool pinHigh = ENABLE_ACTIVE_LOW ? (!on) : on;
  digitalWrite(PIN_EN, pinHigh ? HIGH : LOW);
}

void printHelp() {
  Serial.println(F("Commands: f=forward  r=reverse  s=stop  +=faster  -=slower  h=help"));
}

void setup() {
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR,  OUTPUT);
  pinMode(PIN_EN,   OUTPUT);
  digitalWrite(PIN_STEP, LOW);
  enableDriver(false);                 // start disabled/stopped

  Serial.begin(115200);
  Serial.println(F("# DRV8825 Motor Test"));
  printHelp();
}

void loop() {
  // --- handle serial commands ---
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'f': dir = +1; running = true;  enableDriver(true);
                Serial.println(F("forward")); break;
      case 'r': dir = -1; running = true;  enableDriver(true);
                Serial.println(F("reverse")); break;
      case 's': running = false; enableDriver(false);
                Serial.println(F("stopped")); break;
      case '+': if (stepInterval > MIN_STEP_INTERVAL_US) stepInterval -= 200;
                Serial.print(F("interval=")); Serial.println(stepInterval); break;
      case '-': if (stepInterval < MAX_STEP_INTERVAL_US) stepInterval += 200;
                Serial.print(F("interval=")); Serial.println(stepInterval); break;
      case 'h': printHelp(); break;
      default: break; 
    }
  }

  // non-blocking step generation (keeps serial responsive)
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
}
