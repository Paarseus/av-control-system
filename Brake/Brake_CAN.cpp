
#include <Arduino.h>
#include <FlexCAN_T4.h>


// ----------------- CAN Config -----------------
#define CAN_BITRATE   250000
#define CAN_ID_CMD    0x300      // command frames we accept
#define CAN_ID_STATUS 0x301

// Commands (keep your old semantics)
const uint8_t CMD_EXTEND  = 7;
const uint8_t CMD_RETRACT = 8;
const uint8_t CMD_STATUS_REQ = 0;


FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

// ----------------- Actuator I/O -----------------
const int RPWM = 12;                 // ensure these are PWM-capable pins on your wiring
const int LPWM = 11;
const int LIN_ACT_AIN = A0;

const int Speed = 255;            // 0..255
const float strokeLength = 2.0f;  // inches
float targetLength = 0.70f;       // desired extension (in)
float retractedLength = 0.00f;    // fully retracted (in)

// Calibrate these from your actuator:
const int maxAnalogReading = 800; // raw at full extend
const int minAnalogReading = 745; // raw at full retract
const int BAND = 5;

// Control
volatile uint8_t currentCommand = CMD_RETRACT; // default
uint8_t lastCommand = 0xFF;


static inline float mapf(float x, float inMin, float inMax, float outMin, float outMax) {
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}


void setup() {
  Serial.begin(115200);
  delay(200);

  // PWM outputs + sensor
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(LIN_ACT_AIN, INPUT);
  analogReadResolution(10); // Teensy default is 10-bit (0..1023)

  // Bring up CAN1
  Can1.begin();
  Can1.setBaudRate(CAN_BITRATE);
  Serial.println("BRAKECAN up @ 250k");
}



void loop() {
  // --- CAN RX: read first byte as command (7 or 8) ---
  CAN_message_t msg;
  while (Can1.read(msg)) {
    if (msg.len >= 1) {
      if (msg.id == CAN_ID_CMD) {                   // remove this check to accept any ID
        uint8_t cmd = msg.buf[0];
        if ((cmd == CMD_EXTEND || cmd == CMD_RETRACT) && cmd != lastCommand) {
          currentCommand = cmd;
          lastCommand = cmd;
          Serial.print("CMD: "); Serial.println(cmd == CMD_EXTEND ? "EXTEND" : "RETRACT");
        } else if (cmd == CMD_STATUS_REQ) {
          sendStatusNow(); // immediate feedback on request
        }
      }
    }
  }

  // --- Run the last command ---
  if (currentCommand == CMD_RETRACT) {
    retract();
  } else { // CMD_EXTEND
    extend();
  }
}





// ---- simple extend/retract routines with a small band ----
void extend() {
  int sensorVal = analogRead(LIN_ACT_AIN);
  //int targetAnalog = (int)mapf(targetLength, 0.0f, strokeLength,
  //                            (float)minAnalogReading, (float)maxAnalogReading);
  int targetAnalog = maxAnalogReading; 
  
  if (sensorVal < targetAnalog - BAND) {
    analogWrite(RPWM, Speed);  // extend
    analogWrite(LPWM, 0);
  } else {
    analogWrite(RPWM, 0);      // stop in band or beyond
    analogWrite(LPWM, 0);
  }

  // debug (optional)
  // Serial.printf("EXT raw=%d tgt=%d\n", sensorVal, targetAnalog);
}


void retract() {
  int sensorVal = analogRead(LIN_ACT_AIN);
  // int retractAnalog = (int)mapf(retractedLength, 0.0f, strokeLength,
  //                                 (float)minAnalogReading, (float)maxAnalogReading);
  int retractAnalog = minAnalogReading; 

  if (sensorVal > retractAnalog + BAND) {
    analogWrite(RPWM, 0);      // retract
    analogWrite(LPWM, Speed);
  } else {
    analogWrite(RPWM, 0);      // stop in band or below
    analogWrite(LPWM, 0);
  }

  // debug (optional)
  // Serial.printf("RET raw=%d tgt=%d\n", sensorVal, retractAnalog);
}



void sendStatusNow() {
  // Read sensor once for status
  long acc = 0;
  for (int i=0;i<8;i++) { acc += analogRead(LIN_ACT_AIN); delayMicroseconds(300); }
  int raw = (int)(acc / 8);
  // Map to inches (0..strokeLength), then to hundredths of an inch
  float inches = mapf((float)raw, (float)minAnalogReading, (float)maxAnalogReading, 0.0f, strokeLength);
  int16_t lenCenti = (int16_t)roundf(inches * 100.0f);

  // Flags: inside bands near min/max
  uint8_t flags = 0;
  // Since lower raw value = extended, higher raw value = retracted
  if (raw >= (maxAnalogReading - BAND)) flags |= 0x02; // bit1: near EXTENDED (low raw value ~8)
  if (raw <= (minAnalogReading + BAND)) flags |= 0x01; // bit0: near RETRACTED (high raw value ~50)

  CAN_message_t m;
  m.id  = CAN_ID_STATUS;
  m.len = 8;
  // pack little-endian
  m.buf[0] = raw & 0xFF;
  m.buf[1] = (raw >> 8) & 0xFF;
  m.buf[2] = lenCenti & 0xFF;
  m.buf[3] = (lenCenti >> 8) & 0xFF;
  m.buf[4] = currentCommand;  // 7 or 8
  m.buf[5] = flags;
  m.buf[6] = (uint8_t)Speed;
  m.buf[7] = 0;
  Can1.write(m);
}