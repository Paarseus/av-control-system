#include <Arduino.h>
#include <FlexCAN_T4.h>

#define CAN_BITRATE 250000
#define ROLE_PIN    2            // strap to GND on the SENDER board
#define LED_PIN     13

// IDs
#define ID_HI    0x100
#define ID_HELLO 0x101

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
bool IS_SENDER = true;
uint32_t nextTx = 0;

void sendHI() {
  CAN_message_t m; m.id = ID_HI; m.len = 2; m.buf[0] = 'H'; m.buf[1] = 'I';
  Can1.write(m);
  Serial.println("TX T1: HI");
}
void sendHELLO() {
  CAN_message_t m; m.id = ID_HELLO; m.len = 5;
  m.buf[0]='H'; m.buf[1]='E'; m.buf[2]='L'; m.buf[3]='L'; m.buf[4]='O';
  Can1.write(m);
  Serial.println("TX: HELLO");
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(ROLE_PIN, INPUT_PULLUP);
  IS_SENDER = (digitalRead(ROLE_PIN) == LOW); // strap -> sender

  Serial.begin(115200);
  delay(200);

  Can1.begin();
  Can1.setBaudRate(CAN_BITRATE);

  Serial.println(IS_SENDER ? "Role: SENDER" : "Role: RESPONDER");
  if (IS_SENDER) nextTx = millis() + 1000; // start sending after 1s
}

void loop() {
  // Sender: periodically send HI
  if (IS_SENDER && (int32_t)(millis() - nextTx) >= 0) {
    nextTx += 1000;
    sendHI();
  }

  // RX handler
  CAN_message_t rx;
  while (Can1.read(rx)) {
    digitalWriteFast(LED_PIN, !digitalReadFast(LED_PIN)); // blink on traffic

    // If we receive HI and we're the responder -> reply HELLO
    if (!IS_SENDER && rx.id == ID_HI && rx.len >= 2 && rx.buf[0]=='H' && rx.buf[1]=='I') {
      Serial.println("RX: HI  -> replying HELLO");
      sendHELLO();
    }
    // If we receive HELLO (sender sees the reply)
    else if (IS_SENDER && rx.id == ID_HELLO && rx.len >= 5 &&
             rx.buf[0]=='H'&&rx.buf[1]=='E'&&rx.buf[2]=='L'&&rx.buf[3]=='L'&&rx.buf[4]=='O') {
      Serial.println("RX: HELLO");
    }
  }
}
