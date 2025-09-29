// --- Ultra-Minimal Steering Test (no CAN, no switches) ---
// Wiring: STEP -> pin 4, DIR -> pin 5
#include <Arduino.h>
#include <FlexCAN_T4.h>

// ----------------- CAN Config -----------------
#define CAN_BITRATE   250000
#define CAN_ID_CMD    0x200     // command frames we accept
#define CAN_ID_STATUS 0x201

// Commands (keep your old semantics)
const uint8_t CMD_RIGHT = 7;
const uint8_t CMD_LEFT  = 8;
const uint8_t CMD_CENTER = 3;

const uint8_t CMD_STATUS_REQ = 0;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;


const int leftAnalogReading = 350; // raw at full extend
const int rightAnalogReading = 820; // raw at full retract
const int BAND = 5;

// Control
volatile uint8_t currentCommand = CMD_CENTER; // default
uint8_t lastCommand = 0xFF;

const int Rswitch = 20;
const int Lswitch = 21;

const int encoder   = A0;

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
  while (digitalRead(Lswitch) == LOW) {
    pulseStep();
  }
}

void Cleft(){
  digitalWrite(DIR_PIN, HIGH);
  while (digitalRead(Rswitch) == LOW) {
    pulseStep();
  }
}

void center(){
  int enc = analogRead(encoder);
  Serial.println(enc);
  while(enc < 530 or enc > 540){
    enc = analogRead(encoder);
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

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN,  OUTPUT);
  digitalWrite(STEP_PIN, LOW);

  Can1.begin();
  Can1.setBaudRate(CAN_BITRATE);
  Serial.println("STEERCAN up @ 250k");
}

void loop() {
  CAN_message_t msg;
  while (Can1.read(msg)) {
    if (msg.len >= 1) {
      if (msg.id == CAN_ID_CMD) {                   // remove this check to accept any ID
        uint8_t cmd = msg.buf[0];
        if ((cmd == CMD_RIGHT || cmd == CMD_LEFT || cmd == CMD_CENTER) && cmd != lastCommand) {
          currentCommand = cmd;
          lastCommand = cmd;
          Serial.print("CMD: "); Serial.println(cmd == CMD_RIGHT ? "RIGHT" : "LEFT");
        } else if (cmd == CMD_STATUS_REQ) {
          sendStatusNow(); // immediate feedback on request
        }
      }
    }
  }

  if (currentCommand == CMD_RIGHT) {
    Dright();
  } else if(currentCommand == CMD_LEFT){ // CMD_EXTEND
    Dleft();
  } else if(currentCommand == CMD_CENTER){
    center();
  }
}





void sendStatusNow() {
  // Read sensor once for status
  long acc = 0;
  for (int i=0;i<8;i++) { acc += analogRead(encoder); delayMicroseconds(300); }
  int raw = (int)(acc / 8);

  // Flags: inside bands near min/max
  uint8_t flags = 0;
  // Since lower raw value = extended, higher raw value = retracted
  if (raw >= (rightAnalogReading - BAND)) flags |= 0x01; // bit1: near EXTENDED (low raw value ~8)
  if (raw <= (leftAnalogReading + BAND)) flags |= 0x02; // bit0: near RETRACTED (high raw value ~50)

  CAN_message_t m;
  m.id  = CAN_ID_STATUS;
  m.len = 8;
  // pack little-endian
  m.buf[0] = raw & 0xFF;
  m.buf[1] = (raw >> 8) & 0xFF;
  m.buf[2] = 0;
  m.buf[3] = 0;
  m.buf[4] = currentCommand;  // 7 or 8
  m.buf[5] = flags;
  m.buf[6] = (uint8_t)stepHighUs;
  m.buf[7] = 0;
  Can1.write(m);
}

/* Optional: simple automatic sweep (uncomment to use)
void loop() {
  move(true,  2000); delay(300);
  move(false, 2000); delay(300);
}
*/
