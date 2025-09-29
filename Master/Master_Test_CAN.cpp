#include <Arduino.h>
#include <FlexCAN_T4.h>

// ---- Match your node's IDs & bitrate ----
#define CAN_BITRATE     250000
#define CAN_ID_CMD      0x200
#define CAN_ID_STATUS   0x201

// Commands (must match the brake node)
const uint8_t CMD_STATUS_REQ = 0;
const uint8_t CMD_EXTEND     = 7;
const uint8_t CMD_RETRACT    = 8;

// Teensy 4.x using CAN1
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

// App timing
const uint32_t STATUS_PERIOD_MS = 100;   // how often to request status
uint32_t tNextStatus = 0;

// Track what we last commanded
uint8_t currentCmd = CMD_EXTEND;

// Helpers
void sendCmd(uint8_t cmd) {
  CAN_message_t m;
  m.id  = CAN_ID_CMD;
  m.len = 1;
  m.buf[0] = cmd;
  Can1.write(m);
}

void requestStatus() {
  sendCmd(CMD_STATUS_REQ);
}

void maybeFlipCommandFromFlags(uint8_t flags) {
  // flags: bit0=near retract, bit1=near extend (as per your node)
  bool nearRetract = flags & 0x01;
  bool nearExtend  = flags & 0x02;

  if (currentCmd == CMD_EXTEND && nearExtend) {
    currentCmd = CMD_RETRACT;
    sendCmd(currentCmd);
    Serial.println("-> Auto switch: RETRACT (at near-extend)");
  } else if (currentCmd == CMD_RETRACT && nearRetract) {
    currentCmd = CMD_EXTEND;
    sendCmd(currentCmd);
    Serial.println("-> Auto switch: EXTEND (at near-retract)");
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // Bring up CAN
  Can1.begin();
  Can1.setBaudRate(CAN_BITRATE);

  Serial.println("Master CAN up @ 250k. Sending initial EXTEND...");
  sendCmd(currentCmd);  // start by extending
  tNextStatus = millis();  // kick off status polling
}

void loop() {
  // Periodic status request
  uint32_t now = millis();
  if ((int32_t)(now - tNextStatus) >= 0) {
    tNextStatus += STATUS_PERIOD_MS;
    requestStatus();
  }

  // Read & decode any status frames
  CAN_message_t rx;
  while (Can1.read(rx)) {
    if (rx.id == CAN_ID_STATUS && rx.len >= 8) {
      // Decode your brake node's payload:
      uint16_t raw      = (uint16_t)rx.buf[0] | ((uint16_t)rx.buf[1] << 8);
      int16_t  lenCenti = (int16_t) (rx.buf[2] | (rx.buf[3] << 8));
      uint8_t  nodeCmd  = rx.buf[4];  // 7 or 8
      uint8_t  flags    = rx.buf[5];  // bit0 near retract, bit1 near extend
      uint8_t  speed    = rx.buf[6];

      // Print status
      Serial.print("STATUS | raw=");
      Serial.print(raw);
      Serial.print("  len=");
      Serial.print(lenCenti / 100.0f);
      Serial.print("\"  nodeCmd=");
      Serial.print(nodeCmd == CMD_EXTEND ? "EXTEND" : "RETRACT");
      Serial.print("  flags=");
      Serial.print(flags, BIN);
      Serial.print("  speed=");
      Serial.println(speed);

      // Auto-flip command at ends
      maybeFlipCommandFromFlags(flags);
    }
  }
}
