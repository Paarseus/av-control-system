// Teensy 4.1 — Steering over CAN with linear setpoint only [-1,1], NO 1-byte commands
// ID 0x200 (CMD): Byte0 = center flag (0/1), Byte1 = steer setpoint int8 [-127..127] -> [-1..1]
// ID 0x201 (STATUS): TX basic status; we also accept a request here: Byte0 == 0

#include <Arduino.h>
#include <FlexCAN_T4.h>

// ----------------- CAN Config -----------------
#define CAN_BITRATE     250000
#define CAN_ID_CMD      0x200
#define CAN_ID_STATUS   0x201

const uint8_t CMD_STATUS_REQ = 0;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

// ----------------- Analog limits (calibrate these) -----------------
const int leftAnalogReading  = 350;  // raw at full LEFT
const int rightAnalogReading = 820;  // raw at full RIGHT
const int BAND               = 5;    // allowed error band (raw units)

// ----------------- Switches & stepper -----------------
const int Rswitch   = 20;   // RIGHT limit switch pin
const int Lswitch   = 21;   // LEFT  limit switch pin
const int encoder   = A0;   // analog position feedback

const int STEP_PIN  = 5;
const int DIR_PIN   = 4;

// Step pulse timing (us)
unsigned int stepHighUs = 50;
unsigned int stepLowUs  = 50;

// ----------------- State -----------------
volatile float steer_setpoint = 0.0f;   // desired [-1..1]



// ----------------- Setup & Loop -----------------
void setup() {
  Serial.begin(115200);

  pinMode(Rswitch, INPUT_PULLDOWN);  // adjust if your switches are wired differently
  pinMode(Lswitch, INPUT_PULLDOWN);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN,  OUTPUT);
  digitalWrite(STEP_PIN, LOW);

  analogReadResolution(10);

  Can1.begin();
  Can1.setBaudRate(CAN_BITRATE);
  Serial.println("STEERCAN up @ 250k (linear setpoint only)");
}

void loop() {
  // --- CAN RX ---
  CAN_message_t msg;
  while (Can1.read(msg)) {
    // Linear interface only: ID 0x200, Byte0=center flag, Byte1=int8 setpoint
    if (msg.id == CAN_ID_CMD && msg.len >= 2) {
      bool   centerFlag = (msg.buf[0] != 0);
      int8_t raw_i8     = (int8_t)msg.buf[1];       // -127..127
      float  v          = (float)raw_i8 / 127.0f;   // -> [-1..1]
      steer_setpoint    = clamp11(v);
      if (centerFlag) steer_setpoint = 0.0f;
    }
    // Status request on 0x201 with Byte0==0
    else if (msg.id == CAN_ID_STATUS && msg.len >= 1 && msg.buf[0] == CMD_STATUS_REQ) {
      sendStatusNow();
    }
  }

  // --- Motion control (non-blocking) ---
  steer_update();

  // (Optional) periodic status
  static uint32_t last_status = 0;
  uint32_t now = millis();
  if (now - last_status >= 100) {
    last_status = now;
    sendStatusNow();
  }
}




// ----------------- Helpers -----------------
static inline float clamp11(float v){
  if (v < -1) return -1;
  if (v >  1) return  1;
  return v;
}
static inline int target_from_linear(float v_m11){
  // map [-1..1] -> [leftAnalogReading .. rightAnalogReading]
  float t01 = 0.5f * (v_m11 + 1.0f); // -> [0..1]
  return (int)lroundf(leftAnalogReading + t01 * (rightAnalogReading - leftAnalogReading));
}
static inline bool rightLimit() { return digitalRead(Rswitch) == LOW; } // adjust polarity if needed
static inline bool leftLimit()  { return digitalRead(Lswitch)  == LOW; }

// Single step in current DIR
inline void pulseStep() {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(stepHighUs);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(stepLowUs);
}

// Non-blocking position update toward setpoint
void steer_update(){
  int raw = analogRead(encoder);
  int tgt = target_from_linear(steer_setpoint);

  if (raw < tgt - BAND) {
    // need to move RIGHT (increase raw)
    if (!rightLimit()) {
      digitalWrite(DIR_PIN, LOW);  // wiring: LOW = right
      pulseStep();
    }
  } else if (raw > tgt + BAND) {
    // need to move LEFT (decrease raw)
    if (!leftLimit()) {
      digitalWrite(DIR_PIN, HIGH); // wiring: HIGH = left
      pulseStep();
    }
  } else {
    // in-band: do nothing
  }
}

// ----------------- Status TX -----------------
void sendStatusNow() {
  // Average a few samples for cleaner status
  long acc = 0;
  for (int i=0;i<8;i++) { acc += analogRead(encoder); delayMicroseconds(300); }
  int raw = (int)(acc / 8);

  // Flags near ends
  uint8_t flags = 0;
  if (raw >= (rightAnalogReading - BAND)) flags |= 0x01; // near RIGHT
  if (raw <= (leftAnalogReading  + BAND)) flags |= 0x02; // near LEFT

  int8_t sp_i8 = (int8_t)lroundf(clamp11(steer_setpoint) * 127.0f);

  CAN_message_t m;
  m.id  = CAN_ID_STATUS;
  m.len = 8;
  m.buf[0] = (uint8_t)(raw & 0xFF);
  m.buf[1] = (uint8_t)((raw >> 8) & 0xFF);
  m.buf[2] = 0;                      // reserved
  m.buf[3] = 0;                      // reserved
  m.buf[4] = 0;                      // reserved
  m.buf[5] = flags;                  // limit flags
  m.buf[6] = (uint8_t)stepHighUs;    // step timing (debug)
  m.buf[7] = (uint8_t)sp_i8;         // current setpoint as int8 [-127..127]
  Can1.write(m);
}