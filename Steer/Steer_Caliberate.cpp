// --- Ultra-Minimal Steering Test (no CAN, no switches) ---
// Wiring: STEP -> pin 4, DIR -> pin 5

const int CR = 19;


const int Rswitch = 20;
const int Lswitch = 21;


const int enc0   = A0;

const int STEP_PIN = 5;
const int DIR_PIN  = 4;

// Timing for each step pulse (microseconds)
unsigned int stepHighUs = 50;   // HIGH time
unsigned int stepLowUs  = 50;   // LOW time

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
void Dright(){
  digitalWrite(DIR_PIN, LOW);
  while (digitalRead(Rswitch) != LOW) {
    pulseStep();
  }

}

//LEFTR
void Dleft(){
  digitalWrite(DIR_PIN, HIGH);
  while (digitalRead(Lswitch) != LOW) {
    pulseStep();
  }
}

void Cright(){
  digitalWrite(DIR_PIN, LOW);
  while (digitalRead(CR) != LOW and digitalRead(Rswitch) != LOW) {
    pulseStep();
  }
}

void Cleft(){
  digitalWrite(DIR_PIN, HIGH);
  while (digitalRead(CR) != LOW and digitalRead(Lswitch) != LOW) {
    pulseStep();
  }
}

void center(){
  int enc = analogRead(enc0);
  Serial.println(enc);
  while(enc < 530 or enc > 540){
    enc = analogRead(enc0);
    if (enc < 535){
      digitalWrite(DIR_PIN, LOW);
      pulseStep();
    } else {
      digitalWrite(DIR_PIN, HIGH);
      pulseStep();

    }
  }

}

void setup() {
  Serial.begin(115200);
  pinMode(Rswitch, INPUT_PULLDOWN);
  pinMode(Lswitch, INPUT_PULLDOWN);
  pinMode(CR, INPUT_PULLDOWN);

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
      Dright();
    } else if (cmd == "L") {
      Serial.print(F("LEFT ")); Serial.print(jogSteps); Serial.println(F(" steps"));
      Dleft();
    } else if (cmd.startsWith("N")) {
      long n = cmd.substring(1).toInt();
      if (n > 0) { jogSteps = n; }
      Serial.print(F("jogSteps=")); Serial.println(jogSteps);
    } else if (cmd.startsWith("U")) {
      int u = cmd.substring(1).toInt();
      if (u >= 1) { stepHighUs = stepLowUs = (unsigned int)u; }
      Serial.print(F("pulseUs=")); Serial.println(stepHighUs);
    } else if (cmd == "C"){
      Serial.print("Centering...");
      center();
      delay(200);
    }
  }
  Cright();
  int enc = analogRead(enc0);
  Serial.println(enc);
  delay(200);

}

/* Optional: simple automatic sweep (uncomment to use)
void loop() {
  move(true,  2000); delay(300);
  move(false, 2000); delay(300);
}
*/
