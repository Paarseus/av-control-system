// Teensy 4.1 + MCP4728 — Throttle over CAN (0..1), with mode lines (N/D/S/R)
// ID 0x100 (CMD): Byte0 = E-STOP (0/1), Byte1 = throttle (0..255 -> 0..1), Byte2 = mode char {'N','D','S','R'}
// ID 0x101 (STATUS): we TX status; we also accept a request here: Byte0 == 0

#include <Arduino.h>
#include <Wire.h>
#include <FlexCAN_T4.h>
#include <Adafruit_MCP4728.h>

// ----------------- CAN Config -----------------
#define CAN_BITRATE      250000
#define THROTTLE_CMD_ID  0x100
#define THROTTLE_STAT_ID 0x101
const uint8_t CMD_STATUS_REQ = 0x00;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

// ===== Types =====
enum TargetMode : uint8_t { TM_NEUTRAL, TM_DRIVE, TM_SPORT, TM_REVERSE };

// ===== Globals =====
Adafruit_MCP4728 dac;
TargetMode currentMode = TM_NEUTRAL;

// 0..4.096 V full-scale helpers (internal 2.048 V ref × 2 gain)
static inline uint16_t v_to_counts_4096(float v) {
  if (v < 0) v = 0;
  if (v > 4.096f) v = 4.096f;
  return (uint16_t)(v * (4095.0f / 4.096f) + 0.5f);
}
static inline float clamp01(float x){ if(x<0) return 0; if(x>1) return 1; return x; }

// Throttle control
static constexpr float THROTTLE_IDLE_V = 0.30f;
static constexpr float THROTTLE_MIN_V  = 0.00f;  // set to THROTTLE_IDLE_V if you want 0.0→idle
static constexpr float THROTTLE_MAX_V  = 4.00f;  // raise toward 4.096 if needed
float throttleV = THROTTLE_IDLE_V;

// ===== Mode line writers (exact volts, 0..4.096 V range) =====
void setNeutralLines() {  // B=3.15V, C=1.91V, D=3.20V
  dac.setChannelValue(MCP4728_CHANNEL_B, v_to_counts_4096(3.15f), MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X, MCP4728_PD_MODE_NORMAL, false);
  dac.setChannelValue(MCP4728_CHANNEL_C, v_to_counts_4096(1.91f), MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X, MCP4728_PD_MODE_NORMAL, false);
  dac.setChannelValue(MCP4728_CHANNEL_D, v_to_counts_4096(3.20f), MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X, MCP4728_PD_MODE_NORMAL, true);
}
void setDriveLines() {    // B=0.00V, C=1.91V, D=3.18V
  dac.setChannelValue(MCP4728_CHANNEL_B, v_to_counts_4096(0.00f), MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X, MCP4728_PD_MODE_NORMAL, false);
  dac.setChannelValue(MCP4728_CHANNEL_C, v_to_counts_4096(1.91f), MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X, MCP4728_PD_MODE_NORMAL, false);
  dac.setChannelValue(MCP4728_CHANNEL_D, v_to_counts_4096(3.18f), MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X, MCP4728_PD_MODE_NORMAL, true);
}
void setSportLines() {    // B=0.55V, C=1.88V, D=0.00V
  dac.setChannelValue(MCP4728_CHANNEL_B, v_to_counts_4096(0.55f), MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X, MCP4728_PD_MODE_NORMAL, false);
  dac.setChannelValue(MCP4728_CHANNEL_C, v_to_counts_4096(1.88f), MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X, MCP4728_PD_MODE_NORMAL, false);
  dac.setChannelValue(MCP4728_CHANNEL_D, v_to_counts_4096(0.00f), MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X, MCP4728_PD_MODE_NORMAL, true);
}
void setReverseLines() {  // B=3.15V, C=0.00V, D=3.18V
  dac.setChannelValue(MCP4728_CHANNEL_B, v_to_counts_4096(3.15f), MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X, MCP4728_PD_MODE_NORMAL, false);
  dac.setChannelValue(MCP4728_CHANNEL_C, v_to_counts_4096(0.00f), MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X, MCP4728_PD_MODE_NORMAL, false);
  dac.setChannelValue(MCP4728_CHANNEL_D, v_to_counts_4096(3.18f), MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X, MCP4728_PD_MODE_NORMAL, true);
}

// ===== Throttle writer (A only) =====
void setThrottleVolts(float v) {
  if (v < THROTTLE_MIN_V) v = THROTTLE_MIN_V;
  if (v > THROTTLE_MAX_V) v = THROTTLE_MAX_V;
  throttleV = v;
  dac.setChannelValue(MCP4728_CHANNEL_A, v_to_counts_4096(throttleV), MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X, MCP4728_PD_MODE_NORMAL, true);
}
void setThrottle01(float x01){
  x01 = clamp01(x01);
  float v = THROTTLE_IDLE_V + (THROTTLE_MAX_V - THROTTLE_IDLE_V) * x01;
  setThrottleVolts(v);
}

// ===== Mode change (sequence once), throttle unchanged =====
void selectMode(TargetMode m) {
  if (m == currentMode) return;

  // Safety: momentary idle in Neutral, then switch mode, then leave throttle AS-IS
  setNeutralLines();
  setThrottleVolts(THROTTLE_IDLE_V);
  delay(300);

  switch (m) {
    case TM_NEUTRAL: setNeutralLines(); break;
    case TM_DRIVE:   setDriveLines();   break;
    case TM_SPORT:   setSportLines();   break;
    case TM_REVERSE: setReverseLines(); break;
  }
  currentMode = m;
  delay(200);

  // Restore the previously commanded throttle
  setThrottleVolts(throttleV);
}

// ===== Status TX =====
void sendStatusNow() {
  // pack: [0..1 throttle as u16 counts of A], mode, and raw volts*100
  uint16_t a_counts = v_to_counts_4096(throttleV);
  int16_t  v_centi  = (int16_t)lroundf(throttleV * 100.0f);

  CAN_message_t m;
  m.id  = THROTTLE_STAT_ID;
  m.len = 8;
  m.buf[0] = (uint8_t)(a_counts >> 8);
  m.buf[1] = (uint8_t)(a_counts & 0xFF);
  m.buf[2] = (uint8_t)currentMode;       // 0=N,1=D,2=S,3=R
  m.buf[3] = 0;                          // reserved
  m.buf[4] = (uint8_t)(v_centi & 0xFF);  // V*100 (LE)
  m.buf[5] = (uint8_t)((v_centi >> 8) & 0xFF);
  m.buf[6] = 0;
  m.buf[7] = 0;
  Can1.write(m);
}

// ===== Setup / Loop =====
static bool beginMCP4728_any() {
  if (dac.begin(0x60)) return true;
  if (dac.begin(0x64)) return true;
  return false;
}

uint32_t last_cmd_ms = 0;
const uint32_t WATCHDOG_MS = 200; // fail-safe to Neutral+Idle if no commands

void setup() {
  Serial.begin(115200);
  Wire.begin();                        // Teensy 4.1: SDA=18, SCL=19

  if (!beginMCP4728_any()) {
    Serial.println("ERROR: MCP4728 not found at 0x60/0x64.");
    while (1) delay(250);
  }

  // Put ALL channels into 0..4.096 V config (internal ref ×2)
  dac.setChannelValue(MCP4728_CHANNEL_A, 0, MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X, MCP4728_PD_MODE_NORMAL, false);
  dac.setChannelValue(MCP4728_CHANNEL_B, 0, MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X, MCP4728_PD_MODE_NORMAL, false);
  dac.setChannelValue(MCP4728_CHANNEL_C, 0, MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X, MCP4728_PD_MODE_NORMAL, false);
  dac.setChannelValue(MCP4728_CHANNEL_D, 0, MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X, MCP4728_PD_MODE_NORMAL, true);

  // Boot safe in Neutral + idle
  setNeutralLines();
  setThrottleVolts(THROTTLE_IDLE_V);
  currentMode = TM_NEUTRAL;

  // CAN up
  Can1.begin();
  Can1.setBaudRate(CAN_BITRATE);
}

void loop() {
  // --- CAN RX ---
  CAN_message_t msg;
  while (Can1.read(msg)) {
    // Status request
    if (msg.id == THROTTLE_STAT_ID && msg.len >= 1 && msg.buf[0] == CMD_STATUS_REQ) {
      sendStatusNow();
      continue;
    }

    // Throttle command
    if (msg.id == THROTTLE_CMD_ID && msg.len >= 3) {
      bool estop = (msg.buf[0] != 0);
      float thr01 = (float)msg.buf[1] / 255.0f; // 0..1
      char modeCh = (char)msg.buf[2];           // 'N','D','S','R'

      if (estop) {
        // Immediate safe: Neutral + 0V (or set THROTTLE_MIN_V to idle if desired)
        selectMode(TM_NEUTRAL);
        setThrottleVolts(0.0f);
      } else {
        // Mode change if needed
        TargetMode m = currentMode;
        if      (modeCh == 'N') m = TM_NEUTRAL;
        else if (modeCh == 'D') m = TM_DRIVE;
        else if (modeCh == 'S') m = TM_SPORT;
        else if (modeCh == 'R') m = TM_REVERSE;
        if (m != currentMode) selectMode(m);

        // Apply throttle
        setThrottle01(thr01);
      }
      last_cmd_ms = millis();
    }
  }

  // Watchdog: if commands stop arriving, fall back to Neutral + Idle
  if (millis() - last_cmd_ms > WATCHDOG_MS) {
    if (currentMode != TM_NEUTRAL) selectMode(TM_NEUTRAL);
    setThrottleVolts(THROTTLE_IDLE_V);
  }

  // (Optional) periodic status, e.g., 10 Hz
  static uint32_t last_status = 0;
  uint32_t now = millis();
  if (now - last_status >= 100) {
    last_status = now;
    sendStatusNow();
  }
}
