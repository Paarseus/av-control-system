// Teensy 4.1 — BRAKE CAN Receiver + Linear Actuator Control
// ID 0x300: CMD frame (Byte0=E-STOP 0/1, Byte1=Brake 0..255)
// ID 0x301: STATUS frame (we TX status; we also accept a request: Byte0==0)

#include <Arduino.h>
#include <FlexCAN_T4.h>

// ----------------- CAN Config -----------------
#define CAN_BITRATE     250000
#define CAN_ID_CMD      0x300      // incoming brake command
#define CAN_ID_STATUS   0x301      // outgoing status (and request ID)

const uint8_t CMD_STATUS_REQ = 0x00;  // if a frame arrives on 0x301 with first byte==0, send status

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

// ----------------- Actuator I/O -----------------
// NOTE: RPWM/LPWM must be PWM-capable pins in your wiring
const int RPWM = 12;
const int LPWM = 11;
const int LIN_ACT_AIN = A0;

// Tuning
const uint8_t  Speed         = 255;    // PWM duty for motion (0..255)
const float    strokeLength  = 2.0f;   // actuator stroke in inches (for status only)
const int      BAND          = 5;      // raw ADC deadband around target

// Calibrate these from your actuator sensor:
const int maxAnalogReading = 805; // raw at FULL EXTEND
const int minAnalogReading = 735; // raw at FULL RETRACT

// ----------------- Runtime State -----------------
volatile bool  estop = true;    // start safe
volatile float brake = 0.0f;    // normalized [0..1]

uint32_t last_cmd_ms = 0;
const uint32_t WATCHDOG_MS = 100;  // fail-safe if no CMD frame within this time

enum : uint8_t { CMD_EXTEND = 7, CMD_RETRACT = 8, CMD_STOP = 9 };
volatile uint8_t currentCommand = CMD_STOP;

// ----------------- Helpers -----------------
static inline float clamp01(float v){
  if (v < 0) return 0;
  if (v > 1) return 1;
  return v;
}

static inline float mapf(float x, float inMin, float inMax, float outMin, float outMax) {
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

// Forward decls
void linearextend(float linear);
void sendStatusNow();

// ----------------- Setup -----------------
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(LIN_ACT_AIN, INPUT);
  analogReadResolution(10);   // 0..1023 on Teensy
  // analogWriteFrequency(RPWM, 20000);  // optional: quieter PWM
  // analogWriteFrequency(LPWM, 20000);

  // Bring up CAN
  Can1.begin();
  Can1.setBaudRate(CAN_BITRATE);
  Serial.println("BRAKE CAN up @ 250k");
}

// ----------------- Main Loop -----------------
void loop() {
  // Drain all pending CAN frames
  CAN_message_t msg;
  if (Can1.read(msg)) {
    if (msg.id == CAN_ID_CMD && msg.len >= 2) {
      // Byte0 = E-STOP (0/1), Byte1 = Brake 0..255
      estop = (msg.buf[0] != 0);
      brake = clamp01((float)msg.buf[1] / 255.0f);
      last_cmd_ms = millis();
    }
    else if (msg.id == CAN_ID_STATUS && msg.len >= 1 && msg.buf[0] == CMD_STATUS_REQ) {
      // Status request
      sendStatusNow();
    }
  }

  // Watchdog: if commands stop arriving, force full brake (E-STOP)
  if (millis() - last_cmd_ms > WATCHDOG_MS) {
    estop = true;
    brake = 0.0f;
  }

  // Actuation
  if (estop) {
    linearextend(1.0f);   // full apply on E-STOP
  } else {
    linearextend(brake);  // 0..1 target position
  }

  // (Optional) periodic status (e.g., 10 Hz)
  static uint32_t last_status = 0;
  uint32_t now = millis();
  if (now - last_status >= 100) {
    last_status = now;
    sendStatusNow();
  }
}

// ----------------- Control -----------------
void linearextend(float linear){
  linear = clamp01(linear);

  // Map desired 0..1 → raw ADC range (min=retracted, max=extended)
  int targetAnalog = (int)((maxAnalogReading - minAnalogReading) * linear + minAnalogReading);

  int sensorVal = analogRead(LIN_ACT_AIN);

  if (sensorVal < targetAnalog - BAND) {
    // Need to EXTEND
    analogWrite(RPWM, Speed);
    analogWrite(LPWM, 0);
    currentCommand = CMD_EXTEND;
  }
  else if (sensorVal > targetAnalog + BAND) {
    // Need to RETRACT
    analogWrite(RPWM, 0);
    analogWrite(LPWM, Speed);
    currentCommand = CMD_RETRACT;
  }
  else {
    // In band → STOP
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
    currentCommand = CMD_STOP;
  }
}

// ----------------- Status TX -----------------
void sendStatusNow() {
  // Average a few samples for cleaner status
  long acc = 0;
  for (int i = 0; i < 8; ++i) { acc += analogRead(LIN_ACT_AIN); delayMicroseconds(300); }
  int raw = (int)(acc / 8);

  // Map to inches (0..strokeLength), then to hundredths of an inch
  float inches = mapf((float)raw, (float)minAnalogReading, (float)maxAnalogReading, 0.0f, strokeLength);
  if (inches < 0) inches = 0;
  if (inches > strokeLength) inches = strokeLength;
  int16_t lenCenti = (int16_t)roundf(inches * 100.0f);

  // Flags: near retracted/extended bands (based on calibration)
  uint8_t flags = 0;
  if (raw >= (maxAnalogReading - BAND)) flags |= 0x02; // near EXTENDED
  if (raw <= (minAnalogReading + BAND)) flags |= 0x01; // near RETRACTED

  // Pack status
  CAN_message_t m;
  m.id  = CAN_ID_STATUS;
  m.len = 8;
  // raw ADC (little-endian)
  m.buf[0] = (uint8_t)(raw & 0xFF);
  m.buf[1] = (uint8_t)((raw >> 8) & 0xFF);
  // length in 0.01 inch (little-endian)
  m.buf[2] = (uint8_t)(lenCenti & 0xFF);
  m.buf[3] = (uint8_t)((lenCenti >> 8) & 0xFF);
  m.buf[4] = currentCommand;   // 7/8/9
  m.buf[5] = flags;
  m.buf[6] = Speed;            // commanded PWM magnitude
  m.buf[7] = estop ? 1 : 0;    // estop state mirrored

  Can1.write(m);
}
