#include <Arduino.h>
#include <FlexCAN_T4.h>

// Create CAN bus object on CAN3 (adjust if needed)
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can;
CAN_message_t outgoingMsg;

volatile uint8_t currentValue = 1; // starting with value 1

// Callback for processing feedback messages (ID 0x300)
void leaderOnReceive(const CAN_message_t &msg) {
  if (msg.id == 0x300) {
    uint8_t feedback = msg.buf[0];
    Serial.print("Leader: Received follower1 feedback: ");
    Serial.println(feedback);
    // Update currentValue; if it exceeds 100, reset to 1
    if (feedback > 100) {
      currentValue = 1;
    } else {
      currentValue = feedback;
    }
  }
  else if (msg.id == 0x400) {
    uint8_t feedback = msg.buf[0];
    Serial.print("Leader: Received follower2 feedback: ");
    Serial.println(feedback);
    // Update currentValue; if it exceeds 100, reset to 1
    if (feedback > 100) {
      currentValue = 1;
    } else {
      currentValue = feedback;
    }    
  }
}

void setup() {
  Serial.begin(115200);

  Can.begin();
  Can.setBaudRate(250000);
  Can.setMaxMB(3);  // We'll use MB0 for RX and MB1 for TX

  // Configure MB0 for receiving feedback messages (filtered for ID 0x300)
  Can.setMB(MB0, RX, STD);
  Can.setMBFilter(MB0, 0x300, 0x400);
  
  // Configure MB1 for transmitting leader messages (ID 0x200)
  Can.setMB(MB1, TX, STD);

  // Enable mailbox interrupts and register the callback for MB0
  Can.enableMBInterrupts();
  Can.onReceive(MB0, leaderOnReceive);

  delay(200);
  Serial.println("Leader: CAN bus initialized.");
}

void loop() {
  Serial.println("----------------------");
  // Prepare a message with CAN ID 0x200 that carries the current value
  outgoingMsg.id = 0x200;
  outgoingMsg.len = 1;             // One byte for the value
  outgoingMsg.flags.extended = 0;  // Standard 11-bit identifier
  outgoingMsg.buf[0] = currentValue;
  
  Serial.print("Leader: Sending value: ");
  Serial.println(currentValue);
  
  int ret = Can.write(MB1, outgoingMsg);
  if (ret == 1) {
    Serial.println("Leader: Message sent successfully.");
  } else {
    Serial.print("Leader: Error sending message, code: ");
    Serial.println(ret);
  }
  
  delay(500); // upper limit for leader ti wait for follower manifold to response
}
