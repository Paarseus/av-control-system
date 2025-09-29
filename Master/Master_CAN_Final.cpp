#include <Arduino.h>
#include <FlexCAN_T4.h>

// ---- Match your node's IDs & bitrate ----
#define CAN_BITRATE     250000

#define THROTTLE_ID      0x100
#define THROTTLE_STATUS   0x101

#define BRAKE_ID      0x300
#define BRAKE_STATUS   0x301

#define STEER_ID    0x200
#define STEER_STATUS   0x201

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;


void setup() {
  Serial.begin(115200);
  delay(200);
  Can1.begin();
  Can1.setBaudRate(CAN_BITRATE);
  Serial.println("Master CAN up @ 250k");
}



void loop() {
  // ---- TODO: replace with your real inputs/sensors/UI ----
  bool  estop          = false;
  float throttleValue  = 0.4f;   // [0..1]
  char  driveMode      = 'D';    // 'N','D','S','R'
  float brakeValue     = 0.0f;   // [0..1]
  bool  centerSteer    = false;  // center command
  float steerValue     = 0.0f;   // [-1..1], left negative, right positive

  if (estop) {
    sendEstop();
  } else {
    sendThrottle(estop, throttleValue, driveMode);
    sendBrake(estop, brakeValue);
    sendSteer(centerSteer, steerValue);
  }

  delay(10); // ~100 Hz send rate (adjust as needed)
}

// ---- senders ----
void sendThrottle(bool estop, float value01, char driveMode) {
  CAN_message_t m;
  m.id  = THROTTLE_ID;
  m.len = 3;
  m.buf[0] = estop ? 1 : 0;            // E-STOP flag
  m.buf[1] = pack_f01_u8(value01);     // throttle 0..255
  m.buf[2] = (uint8_t)driveMode;       // e.g., 'N','D','S','R'
  Can1.write(m);
}

void sendBrake(bool estop, float value01) {
  CAN_message_t m;
  m.id  = BRAKE_ID;
  m.len = 2;
  m.buf[0] = estop ? 1 : 0;            // E-STOP flag
  m.buf[1] = pack_f01_u8(value01);     // brake 0..255
  Can1.write(m);
}

void sendSteer(bool center, float valueMinus1to1) {
  CAN_message_t m;
  m.id  = STEER_ID;
  m.len = 2;
  m.buf[0] = center ? 1 : 0;           // "recenter" flag (or 0)
  m.buf[1] = (uint8_t)pack_f11_i8(valueMinus1to1); // signed -127..127
  Can1.write(m);
}

void sendEstop() {
  sendBrake(true, 1.0f);
  sendThrottle(true, 0.0f, 'N');
  sendSteer(true, 0.0f);
}









// ---- helpers: pack normalized floats to bytes ----
static inline uint8_t pack_f01_u8(float v) {      // 0..1 -> 0..255
  if (v < 0) v = 0; if (v > 1) v = 1;
  return (uint8_t)(v * 255.0f + 0.5f);
}
static inline int8_t pack_f11_i8(float v) {       // -1..1 -> -127..127
  if (v < -1) v = -1; if (v >  1) v =  1;
  float s = v * 127.0f;
  return (int8_t)(s + (s >= 0 ? 0.5f : -0.5f));
}