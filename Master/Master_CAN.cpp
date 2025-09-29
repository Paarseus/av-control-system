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


void control();
void encode();
void 