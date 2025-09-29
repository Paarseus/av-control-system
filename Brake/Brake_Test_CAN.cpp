// Teensy calibration helper for linear actuator analog feedback + simple jog control.
// Controls H-bridge with two PWM pins and prints raw ADC + live min/max.

#include <Arduino.h>

//FULL EXTEND = 800-805
//MIN RETRACT = 742-747
//FULL RETRACT = 390-400

const int RPWM = 12;
const int LPWM = 11;
const int AIN  = A0;

int pwmSpeed = 180;              // start gentle
int rawMinSeen = 1023, rawMaxSeen = 0;

void stopMotor() {
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
}

void goExtend() {  // adjust direction if reversed
  analogWrite(RPWM, pwmSpeed);
  analogWrite(LPWM, 0);
}

void goRetract() {
  analogWrite(RPWM, 0);
  analogWrite(LPWM, pwmSpeed);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(AIN, INPUT);
  analogReadResolution(10);      // 0..1023 on Teensy default
  stopMotor();

  Serial.println("=== Actuator Calibration Helper ===");
  Serial.println("Keys: e=extend, r=retract, s=stop, +=faster, -=slower");
  Serial.println("Drive to each hard stop and record min/max raw values shown.");
}

void loop() {
  // Serial control
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'e') goExtend();
    else if (c == 'r') goRetract();
    else if (c == 's') stopMotor();
    else if (c == '+') { pwmSpeed = min(255, pwmSpeed + 10); Serial.printf("Speed=%d\n", pwmSpeed); }
    else if (c == '-') { pwmSpeed = max( 50, pwmSpeed - 10); Serial.printf("Speed=%d\n", pwmSpeed); }
  }

  // Read & track extremes (simple 8-sample average)
  long acc = 0;
  for (int i=0;i<8;i++) { acc += analogRead(AIN); delayMicroseconds(300); }
  int raw = (int)(acc / 8);

  rawMinSeen = min(rawMinSeen, raw);
  rawMaxSeen = max(rawMaxSeen, raw);

  // Print live info ~10 Hz
  static uint32_t last = 0;
  if (millis() - last > 100) {
    last = millis();
    Serial.printf("raw=%4d   minSeen=%4d   maxSeen=%4d\n", raw, rawMinSeen, rawMaxSeen);
  }
}
