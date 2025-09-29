#include <Wire.h>
#include <Adafruit_MCP4728.h>

// ===== Choose target mode =====
enum TargetMode { TM_NEUTRAL, TM_DRIVE, TM_SPORT, TM_REVERSE };
static constexpr TargetMode MODE_TARGET = TM_SPORT;  // <-- change here if needed

Adafruit_MCP4728 dac;

// 0..4.096 V full-scale helpers (internal 2.048 V ref × 2 gain)
static inline uint16_t v_to_counts_4096(float v) {
  if (v < 0) v = 0;
  if (v > 4.096f) v = 4.096f;
  return (uint16_t)(v * (4095.0f / 4.096f) + 0.5f);
}

bool beginMCP4728() {
  if (dac.begin(0x60)) return true;
  if (dac.begin(0x64)) return true; // some boards ship as 0x64
  return false;
}

// ----- Mode line writers (exact volts, 0..4.096 V range) -----
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

void setThrottleVolts(float v) { // Channel A (Throttle)
  dac.setChannelValue(MCP4728_CHANNEL_A, v_to_counts_4096(v), MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X, MCP4728_PD_MODE_NORMAL, true);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("\nTeensy 4.1 + MCP4728 — Neutral -> Mode -> Throttle sequence");

  Wire.begin(); // SDA=18, SCL=19 on Teensy 4.1

  if (!beginMCP4728()) {
    Serial.println("ERROR: MCP4728 not found at 0x60/0x64. Check wiring & 3.3V power.");
    while (1) delay(250);
  }
  Serial.println("MCP4728 ready.");

  // Put ALL channels into 0..4.096 V config first (internal ref ×2)
  // (Zero them with the desired config to lock vref/gain across the board.)
  dac.setChannelValue(MCP4728_CHANNEL_A, 0, MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X, MCP4728_PD_MODE_NORMAL, false);
  dac.setChannelValue(MCP4728_CHANNEL_B, 0, MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X, MCP4728_PD_MODE_NORMAL, false);
  dac.setChannelValue(MCP4728_CHANNEL_C, 0, MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X, MCP4728_PD_MODE_NORMAL, false);
  dac.setChannelValue(MCP4728_CHANNEL_D, 0, MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X, MCP4728_PD_MODE_NORMAL, true);

  // --- 1) Explicit NEUTRAL first + idle throttle ---
  setNeutralLines();
  setThrottleVolts(0.30f);   // idle ~0.3 V
  Serial.println("Step 1: Neutral lines set, throttle idle (~0.30 V).");
  delay(1500);               // give controller time to register Neutral

  // --- 2) Change to target MODE (still idle throttle) ---
  switch (MODE_TARGET) {
    case TM_NEUTRAL: setNeutralLines(); Serial.println("Step 2: Stay in NEUTRAL"); break;
    case TM_DRIVE:   setDriveLines();   Serial.println("Step 2: Switched to DRIVE"); break;
    case TM_SPORT:   setSportLines();   Serial.println("Step 2: Switched to SPORT"); break;
    case TM_REVERSE: setReverseLines(); Serial.println("Step 2: Switched to REVERSE"); break;
  }
  delay(500); // brief settle in mode (controller often needs this)

  // --- 3) Apply throttle after mode is set ---
  setThrottleVolts(4.00f);   // request ~3.30 V on A
  Serial.println("Step 3: Throttle set to 3.30 V.");
}

void loop() {
  // hold outputs steady
}
