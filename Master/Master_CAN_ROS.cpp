// Teensy 4.1 — CAN Master controlled by ROS keyboard via Serial
// Expects lines like:  B:0,M:1,L:0.60,A:-0.25\n
// Sends CAN @ 250k on CAN1 (pins 23=RX, 22=TX) via FlexCAN_T4

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <ctype.h>

// ================= CAN config =================
#define CAN_BITRATE     250000
#define THROTTLE_ID     0x100
#define THROTTLE_STATUS 0x101
#define BRAKE_ID        0x300
#define BRAKE_STATUS    0x301
#define STEER_ID        0x200
#define STEER_STATUS    0x201

// Select the CAN controller that matches your wiring:
// CAN1 -> pins 23(RX)/22(TX), CAN2 -> pins 0/1, CAN3 -> bottom pads 30/31
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

// ================ State =================
static bool  g_estop       = false; // Not driven by ROS line; keep false unless you add it
static float g_throttle    = 0.0f;  // [0..1]
static char  g_mode        = 'N';   // 'N','D','S','R'
static float g_brake       = 0.0f;  // [0..1] (0 or 1 from ROS)
static float g_steer       = 0.0f;  // [-1..1]
static bool  g_centerPulse = false; // (not used by ROS line, but kept for compatibility)

// Match your Python: max_turn_speed = 0.7 -> scale to full [-1..1]
static constexpr float ROS_TURN_FULL_SCALE = 0.7f;

// ============== Helpers / clamp ==============
static inline float clamp01(float v){ return v < 0 ? 0 : (v > 1 ? 1 : v); }
static inline float clamp11(float v){ return v < -1 ? -1 : (v > 1 ? 1 : v); }

static inline uint8_t pack_f01_u8(float v){
  v = clamp01(v);
  return (uint8_t)(v * 255.0f + 0.5f);
}
static inline int8_t pack_f11_i8(float v){
  v = clamp11(v);
  float s = v * 127.0f;
  return (int8_t)(s + (s >= 0 ? 0.5f : -0.5f));
}

// ============== CAN senders ==============
static void sendThrottle(bool estop, float value01, char driveMode) {
  CAN_message_t m;
  m.id  = THROTTLE_ID;
  m.len = 3;
  m.buf[0] = estop ? 1 : 0;
  m.buf[1] = pack_f01_u8(value01);
  m.buf[2] = (uint8_t)driveMode;
  Can1.write(m);
}
static void sendBrake(bool estop, float value01) {
  CAN_message_t m;
  m.id  = BRAKE_ID;
  m.len = 2;
  m.buf[0] = estop ? 1 : 0;
  m.buf[1] = pack_f01_u8(value01);
  Can1.write(m);
}
static void sendSteer(bool center, float valueMinus1to1) {
  CAN_message_t m;
  m.id  = STEER_ID;
  m.len = 2;
  m.buf[0] = center ? 1 : 0; // not used by ROS; stays 0 unless you set g_centerPulse
  m.buf[1] = (uint8_t)pack_f11_i8(valueMinus1to1);
  Can1.write(m);
}

// ============== Serial ROS-line parser ==============
// Accepts lines like: "B:0,M:1,L:0.60,A:-0.25"
static void parseRosLine(char *line){
  // Defaults: keep last values unless token present
  int   brake_i = -1;     // -1 means "not provided"
  int   mode_i  = -1;     // 0..3 -> N,D,S,R
  float lin     = NAN;    // forward +, reverse -
  float ang     = NAN;    // left -, right +

  // Tokenize on commas
  for (char *tok = strtok(line, ","); tok; tok = strtok(NULL, ",")) {
    // skip spaces
    while (*tok==' '||*tok=='\t') ++tok;
    if (!*tok) continue;

    if (toupper(tok[0])=='B' && tok[1]==':') {
      brake_i = atoi(tok+2);
    } else if (toupper(tok[0])=='M' && tok[1]==':') {
      mode_i = atoi(tok+2);
    } else if (toupper(tok[0])=='L' && tok[1]==':') {
      lin = atof(tok+2);
    } else if (toupper(tok[0])=='A' && tok[1]==':') {
      ang = atof(tok+2);
    }
  }

  // Apply parsed values
  if (brake_i >= 0) {
    g_brake = brake_i ? 1.0f : 0.0f;
  }

  if (mode_i >= 0) {
    switch (mode_i) {
      case 0: g_mode = 'N'; break;
      case 1: g_mode = 'D'; break;
      case 2: g_mode = 'S'; break;
      case 3: g_mode = 'R'; break;
      default: /* ignore */ break;
    }
  }

  if (!isnan(lin)) {
    // Map linear to throttle magnitude; keep direction via mode=R if you prefer
    // If you want auto-reverse when lin<0, uncomment the block below.
    /*
    if (lin < 0 && g_mode!='R') g_mode='R';
    if (lin > 0 && g_mode=='R') g_mode='N'; // or 'D' if you want auto forward
    */
    g_throttle = clamp01(fabsf(lin)); // Teensy expects 0..1
  }

  if (!isnan(ang)) {
    // Scale ROS angular into full [-1..1] and clamp
    g_steer = clamp11(ang / ROS_TURN_FULL_SCALE);
  }
}

// Non-blocking line reader (CR, LF, or CRLF end-of-line)
static void readSerialRos(){
  static char buf[96];
  static uint8_t idx = 0;

  while (Serial.available()) {
    char c = Serial.read();

    if (c=='\r' || c=='\n') {
      if (idx > 0) {
        buf[idx] = '\0';
        parseRosLine(buf);
        idx = 0;
      }
      continue; // swallow extra CR/LF
    }

    if (idx < sizeof(buf)-1) {
      buf[idx++] = c;
    } else {
      // overflow: reset
      idx = 0;
    }
  }
}

// ================= Arduino =================
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2000) { /* give USB up to 2s */ }

  Can1.begin();
  Can1.setBaudRate(CAN_BITRATE);

  Serial.println("CAN Master (ROS-serial) up @ 250k");
}

void loop() {
  // 1) Ingest ROS keyboard line(s) from USB serial
  readSerialRos();

  // 2) Optionally read back any CAN status (debug print)
  CAN_message_t rx;
  while (Can1.read(rx)) {
    // Uncomment for debugging bus activity:
    // Serial.printf("RX id=0x%03X len=%d :", rx.id, rx.len);
    // for (int i=0;i<rx.len;i++) Serial.printf(" %02X", rx.buf[i]);
    // Serial.println();
  }

  // 3) Transmit commands at ~50–100 Hz
  static uint32_t last_tx = 0;
  uint32_t now = millis();
  if (now - last_tx >= 10) { // 100 Hz (match or exceed ROS 50 Hz)
    last_tx = now;

    // We don't auto-assert estop from B; brake is a service brake.
    // If you want spacebar to act as E-STOP, set g_estop = (g_brake >= 0.5f).
    sendThrottle(g_estop, g_throttle, g_mode);
    sendBrake(g_estop, g_brake);

    // No center one-shot from ROS; always 0 unless you set g_centerPulse elsewhere
    bool center = g_centerPulse;
    g_centerPulse = false;
    sendSteer(center, g_steer);
  }
}
