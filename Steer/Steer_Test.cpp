// --- Ultra-Minimal Steering Test (no CAN, no switches) ---
// Wiring: STEP -> pin 4, DIR -> pin 5

const int STEP_PIN = 12;
const int DIR_PIN  = 13;

// Timing for each step pulse (microseconds)
unsigned int stepHighUs = 100;   // HIGH time
unsigned int stepLowUs  = 100;   // LOW time

// How many steps per jog
long jogSteps = 5000;

// Pulse one step
inline void pulseStep() {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(stepHighUs);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(stepLowUs);
}

//RIght
void right(){
  digitalWrite(DIR_PIN, LOW);
  for (long i = 0; i < jogSteps; ++i) pulseStep();
}

//LEFT
void left(){
  digitalWrite(DIR_PIN, HIGH);
  for (long i = 0; i < jogSteps; ++i) pulseStep();
}


void setup() {
  Serial.begin(115200);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN,  OUTPUT);
  digitalWrite(STEP_PIN, LOW);

  Serial.println(F("\n=== Steering Test (STEP=4, DIR=5) ==="));
  Serial.println(F("Commands:"));
  Serial.println(F("  R       -> jog RIGHT"));
  Serial.println(F("  L       -> jog LEFT"));
  Serial.println(F("  N####   -> set jog steps (e.g. N2500)"));
  Serial.println(F("  U####   -> set pulse delay us (e.g. U300)"));
  Serial.print (F("Start: jogSteps=")); Serial.print(jogSteps);
  Serial.print (F(", stepUs=")); Serial.println(stepHighUs);
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n'); cmd.trim();
    if (cmd == "R") {
      Serial.print(F("RIGHT ")); Serial.print(jogSteps); Serial.println(F(" steps"));
      right();
    } else if (cmd == "L") {
      Serial.print(F("LEFT ")); Serial.print(jogSteps); Serial.println(F(" steps"));
      left();
    } else if (cmd.startsWith("N")) {
      long n = cmd.substring(1).toInt();
      if (n > 0) { jogSteps = n; }
      Serial.print(F("jogSteps=")); Serial.println(jogSteps);
    } else if (cmd.startsWith("U")) {
      int u = cmd.substring(1).toInt();
      if (u >= 1) { stepHighUs = stepLowUs = (unsigned int)u; }
      Serial.print(F("pulseUs=")); Serial.println(stepHighUs);
    } else if (cmd.length() > 0) {
      Serial.println(F("Use: R | L | N#### | U####"));
    }
  }
}

/* Optional: simple automatic sweep (uncomment to use)
void loop() {
  move(true,  2000); delay(300);
  move(false, 2000); delay(300);
}
*/
