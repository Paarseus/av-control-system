const int Rswitch = 21;  // NC to GND
void setup() {
  Serial.begin(115200);
  pinMode(Rswitch, INPUT_PULLDOWN);  // idle LOW, pressed HIGH
}
void loop() {
  if (digitalRead(Rswitch) == LOW) {
    Serial.println("Pressed");
  } else {
    Serial.println("Released");
  }
  delay(20);
}
