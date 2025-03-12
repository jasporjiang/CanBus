#include <Arduino.h>
#include <FlexCAN_T4.h>

// Create CAN bus object on CAN3 (adjust if needed)
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can;
CAN_message_t outgoingMsg;

// Callback for processing received leader messages (ID 0x200)
void followerOnReceive(const CAN_message_t &msg) {
  if (msg.id == 0x200) {
    uint8_t value = msg.buf[0];
    Serial.print("Follower: Received value: ");
    Serial.println(value);
    // Do the operation: add 1
    uint8_t newValue = value + 1;
    
    // Prepare feedback message with CAN ID 0x300
    outgoingMsg.id = 0x300;
    outgoingMsg.len = 1;             // One byte feedback
    outgoingMsg.flags.extended = 0;
    outgoingMsg.buf[0] = newValue;
    
    Serial.print("Follower: Sending feedback: ");
    Serial.println(newValue);
    
    int ret = Can.write(MB1, outgoingMsg);
    if (ret == 1) {
      Serial.println("Follower: Feedback sent successfully.");
    } else {
      Serial.print("Follower: Error sending feedback, code: ");
      Serial.println(ret);
    }
  }
}

void setup() {
  Serial.begin(115200);
//   while (!Serial) { } // wait for serial monitor

  Can.begin();
  Can.setBaudRate(250000);
  Can.setMaxMB(3);  // We'll use MB0 for RX and MB1 for TX

  // Configure MB0 for receiving leader messages (filter for ID 0x200)
  Can.setMB(MB0, RX, STD);
  Can.setMBFilter(MB0, 0x200);
  
  // Configure MB1 for transmitting feedback messages (ID 0x300)
  Can.setMB(MB1, TX, STD);

  // Enable mailbox interrupts and register the callback for MB0
  Can.enableMBInterrupts();
  Can.onReceive(MB0, followerOnReceive);

  delay(500);
  Serial.println("Follower: CAN bus initialized.");
}

void loop() {
  Serial.print("--------msg: ");
  Serial.print(outgoingMsg.buf[0]);
  Serial.println("--------------");
  // In event-driven mode, callbacks process incoming messages,
  // so the main loop can be kept minimal.
  delay(100); // upper limit for physical manifold hardware
}
