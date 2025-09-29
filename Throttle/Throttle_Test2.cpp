#include <Wire.h>
#include <Adafruit_MCP4728.h>

// ===== Types =====
enum TargetMode { TM_NEUTRAL, TM_DRIVE, TM_SPORT, TM_REVERSE };

// ===== Globals =====
Adafruit_MCP4728 dac;
TargetMode currentMode = TM_NEUTRAL;

// 0..4.096 V full-scale helpers (internal 2.048 V ref × 2 gain)
static inline uint16_t v_to_counts_4096(float v) {
  if (v < 0) v = 0;
  if (v > 4.096f) v = 4.096f;
  return (uint16_t)(v * (4095.0f / 4.096f) + 0.5f);
}

// Throttle control
static constexpr float THROTTLE_IDLE_V = 0.30f;
static constexpr float THROTTLE_STEP_V = 0.10f;
static constexpr float THROTTLE_MIN_V  = 0.00f;
static constexpr float THROTTLE_MAX_V  = 4.00f; // raise toward 4.096 if needed
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

// ===== Mode change (sequence once), throttle unchanged =====
void selectMode(TargetMode m) {
  // Safety: momentary idle in Neutral, then switch mode, then leave throttle AS-IS
  setNeutralLines();
  setThrottleVolts(THROTTLE_IDLE_V);
  Serial.println("Neutral + idle");
  delay(1200);

  switch (m) {
    case TM_NEUTRAL: setNeutralLines(); Serial.println("Mode: NEUTRAL"); break;
    case TM_DRIVE:   setDriveLines();   Serial.println("Mode: DRIVE");   break;
    case TM_SPORT:   setSportLines();   Serial.println("Mode: SPORT");   break;
    case TM_REVERSE: setReverseLines(); Serial.println("Mode: REVERSE"); break;
  }
  currentMode = m;
  delay(400);

  // Restore the user’s chosen throttle (could still be idle)
  setThrottleVolts(throttleV);
  Serial.print("Throttle = "); Serial.print(throttleV, 2); Serial.println(" V");
}

// ===== UI =====
void printMenu() {
  Serial.println("\nCommands:");
  Serial.println("  n d s r  = select mode (runs Neutral->Mode ONCE)");
  Serial.println("  + / -    = throttle +/- 0.10 V (applies IMMEDIATELY)");
  Serial.println("  0        = throttle idle (0.30 V)");
  Serial.println("  p        = print status");
  Serial.println("  m        = show this menu\n");
}

void printStatus() {
  const char* name = (currentMode==TM_NEUTRAL) ? "NEUTRAL" :
                     (currentMode==TM_DRIVE)   ? "DRIVE"   :
                     (currentMode==TM_SPORT)   ? "SPORT"   : "REVERSE";
  Serial.print("Mode="); Serial.print(name);
  Serial.print(" | ThrottleV="); Serial.print(throttleV, 2); Serial.println(" V");
}

// ===== Setup / Loop =====
static bool beginMCP4728_any() {
  if (dac.begin(0x60)) return true;
  if (dac.begin(0x64)) return true;
  return false;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("\nTeensy 4.1 + MCP4728 — pick mode once, live throttle");

  Wire.begin();  // Teensy 4.1: SDA=18, SCL=19
  if (!beginMCP4728_any()) {
    Serial.println("ERROR: MCP4728 not found at 0x60/0x64. Check wiring & 3.3V.");
    while (1) delay(250);
  }
  Serial.println("MCP4728 ready.");

  // Put ALL channels into 0..4.096 V config (internal ref ×2)
  dac.setChannelValue(MCP4728_CHANNEL_A, 0, MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X, MCP4728_PD_MODE_NORMAL, false);
  dac.setChannelValue(MCP4728_CHANNEL_B, 0, MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X, MCP4728_PD_MODE_NORMAL, false);
  dac.setChannelValue(MCP4728_CHANNEL_C, 0, MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X, MCP4728_PD_MODE_NORMAL, false);
  dac.setChannelValue(MCP4728_CHANNEL_D, 0, MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X, MCP4728_PD_MODE_NORMAL, true);

  // Boot safe in Neutral + idle
  setNeutralLines();
  setThrottleVolts(THROTTLE_IDLE_V);
  currentMode = TM_NEUTRAL;

  printMenu();
  printStatus();
}

void loop() {
  if (Serial.available()) {
    char c = (char)Serial.read();

    if (c=='n'||c=='N') selectMode(TM_NEUTRAL);
    else if (c=='d'||c=='D') selectMode(TM_DRIVE);
    else if (c=='s'||c=='S') selectMode(TM_SPORT);
    else if (c=='r'||c=='R') selectMode(TM_REVERSE);
    else if (c=='+') { setThrottleVolts(throttleV + THROTTLE_STEP_V); printStatus(); }
    else if (c=='-') { setThrottleVolts(throttleV - THROTTLE_STEP_V); printStatus(); }
    else if (c=='0') { setThrottleVolts(THROTTLE_IDLE_V);             printStatus(); }
    else if (c=='p'||c=='P') printStatus();
    else if (c=='m'||c=='M') printMenu();
  }
}
