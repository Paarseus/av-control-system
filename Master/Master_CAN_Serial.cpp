// Teensy 4.1 — Serial-driven CAN Master (throttle/brake/steer + estop)
// Commands (type in Serial Monitor, press Enter; CR/LF/CRLF all OK):
//   E 1|0        -> estop on/off
//   T 0..1       -> throttle (normalized)
//   M N|D|S|R    -> drive mode
//   B 0..1       -> brake (normalized)
//   S -1..1      -> steer (left -, right +)
//   C            -> center (one-shot flag for steering)
//   P            -> print current state
//   H            -> help
//
// CAN frames (11-bit):
//   THROTTLE_ID 0x100: [0]=estop(0/1), [1]=throttle u8(0..255), [2]=mode 'N'/'D'/'S'/'R'
//   BRAKE_ID    0x300: [0]=estop(0/1), [1]=brake    u8(0..255)
//   STEER_ID    0x200: [0]=center(0/1), [1]=steer   i8(-127..127)

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <ctype.h>
#include <stdlib.h>

// ---- CAN settings & IDs ----
#define CAN_BITRATE     250000

#define THROTTLE_ID     0x100
#define THROTTLE_STATUS 0x101

#define BRAKE_ID        0x300
#define BRAKE_STATUS    0x301

#define STEER_ID        0x200
#define STEER_STATUS    0x201

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

// ---------------- State controlled via Serial ----------------
bool  g_estop       = false;
float g_throttle    = 0.0f;   // [0..1]
char  g_mode        = 'N';    // 'N','D','S','R'
float g_brake       = 0.0f;   // [0..1]
float g_steer       = 0.0f;   // [-1..1]
bool  g_centerPulse = false;  // one-shot center flag for steer

// ---- helpers: clamp & pack ----
static inline float clamp01(float v){ if(v<0) return 0; if(v>1) return 1; return v; }
static inline float clamp11(float v){ if(v<-1) return -1; if(v>1) return 1; return v; }

static inline uint8_t pack_f01_u8(float v) {      // 0..1 -> 0..255
  if (v < 0) v = 0; if (v > 1) v = 1;
  return (uint8_t)(v * 255.0f + 0.5f);
}
static inline int8_t pack_f11_i8(float v) {       // -1..1 -> -127..127
  if (v < -1) v = -1; if (v >  1) v =  1;
  float s = v * 127.0f;
  return (int8_t)(s + (s >= 0 ? 0.5f : -0.5f));
}

// ---- senders ----
void sendThrottle(bool estop, float value01, char driveMode) {
  CAN_message_t m;
  m.id  = THROTTLE_ID;
  m.len = 3;
  m.buf[0] = estop ? 1 : 0;            // E-STOP flag
  m.buf[1] = pack_f01_u8(value01);     // throttle 0..255
  m.buf[2] = (uint8_t)driveMode;       // 'N','D','S','R'
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
  m.buf[0] = center ? 1 : 0;           // "recenter" one-shot
  m.buf[1] = (uint8_t)pack_f11_i8(valueMinus1to1); // signed -127..127
  Can1.write(m);
}

void sendEstop() {
  sendBrake(true, 1.0f);
  sendThrottle(true, 0.0f, 'N');
  sendSteer(true, 0.0f);
}

// ---- Serial UI ----
void printHelp(){
  Serial.println(F("\nCommands:"));
  Serial.println(F("  E 1|0        -> estop on/off"));
  Serial.println(F("  T <0..1>     -> throttle"));
  Serial.println(F("  M N|D|S|R    -> mode"));
  Serial.println(F("  B <0..1>     -> brake"));
  Serial.println(F("  S <-1..1>    -> steer (left -, right +)"));
  Serial.println(F("  C            -> center (one-shot flag)"));
  Serial.println(F("  P            -> print current state"));
  Serial.println(F("  H            -> help\n"));
}

void printState(){
  Serial.print(F("E=")); Serial.print(g_estop);
  Serial.print(F("  T=")); Serial.print(g_throttle, 3);
  Serial.print(F("  M=")); Serial.print(g_mode);
  Serial.print(F("  B=")); Serial.print(g_brake, 3);
  Serial.print(F("  S=")); Serial.print(g_steer, 3);
  Serial.print(F("  Cflag=")); Serial.println(g_centerPulse ? 1 : 0);
}

void handleLine(char* line){
  // trim leading spaces
  while (*line==' ' || *line=='\t') ++line;
  if (!*line) return;

  char cmd = toupper(*line++);
  // skip spaces between cmd and arg
  while (*line==' ' || *line=='\t') ++line;

  switch(cmd){
    case 'E': { // E 1|0
      int v = atoi(line);
      g_estop = (v != 0);
      printState();
    } break;

    case 'T': { // T 0..1
      float v = atof(line);
      g_throttle = clamp01(v);
      printState();
    } break;

    case 'M': { // M N|D|S|R
      if (*line) {
        char m = toupper(*line);
        if (m=='N'||m=='D'||m=='S'||m=='R') g_mode = m;
      }
      printState();
    } break;

    case 'B': { // B 0..1
      float v = atof(line);
      g_brake = clamp01(v);
      printState();
    } break;

    case 'S': { // S -1..1
      float v = atof(line);
      g_steer = clamp11(v);
      printState();
    } break;

    case 'C': { // center one-shot
      g_centerPulse = true;
      printState();
    } break;

    case 'P': printState(); break;
    case 'H': default: printHelp(); break;
  }
}

void readSerialCommands(){
  static char buf[64];
  static uint8_t idx = 0;

  while (Serial.available()){
    char c = Serial.read();

    // Treat both CR and LF as end-of-line; handle CRLF too
    if (c == '\r' || c == '\n'){
      if (idx > 0) {          // only process non-empty lines
        buf[idx] = '\0';
        handleLine(buf);
        idx = 0;
      }
      continue;               // swallow extra CR/LF
    }

    if (idx < sizeof(buf) - 1){
      buf[idx++] = c;
    } else {
      // overflow: reset the buffer (optional behavior)
      idx = 0;
    }
  }
}

// ---- Arduino entry points ----
void setup() {
  Serial.begin(115200);
  // Give USB Serial up to 2s to come up (non-blocking if already open)
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2000) { /* wait */ }

  Can1.begin();
  Can1.setBaudRate(CAN_BITRATE);

  Serial.println(F("Master CAN up @ 250k"));
  printHelp();
  printState();
}

void loop() {
  // read & apply serial commands
  readSerialCommands();

  // send frames at ~100 Hz
  static uint32_t last_tx = 0;
  uint32_t now = millis();
  if (now - last_tx >= 10) {
    last_tx = now;

    if (g_estop) {
      sendEstop();
    } else {
      sendThrottle(g_estop, g_throttle, g_mode);
      sendBrake(g_estop, g_brake);

      // one-shot center flag: send once, then clear
      bool center = g_centerPulse;
      g_centerPulse = false;
      sendSteer(center, g_steer);
    }
  }
}
