#include <Arduino.h>
#include <FlexCAN_T4.h>

// Create a CAN bus object on CAN3 (adjust as needed)
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can;

CAN_message_t incomingMsg;
CAN_message_t outgoingMsg;

void setup() {
  Serial.begin(115200);
  while (!Serial) { } // Wait for serial connection

  Can.begin();
  Can.setBaudRate(250000);
  Can.setMaxMB(2);  // Use two mailboxes (MB0 and MB1)

  // Configure mailbox MB0 as RX (receive mailbox)
  Can.setMB(MB0, RX, STD);
  // Set a filter so that only messages with ID 0x200 are accepted
  Can.setMBFilter(MB0, 0x200);
  // (Optionally, you can call enhanceFilter(MB0) for tighter filtering)
  Can.setMB(MB1, TX, STD);

  Serial.println("Follower1: CAN bus initialized.");
  delay(500);
}

void loop() {
  Serial.println("----------------------");
  Can.events();
    // Send leader message with ID 0x200
    outgoingMsg.id = 0x300;
    outgoingMsg.len = 6;
    outgoingMsg.flags.extended = 0;
    for (int i = 0; i < 6; ++i) {
      outgoingMsg.buf[i] = 60 - i * 10;  // Data: 10, 20, 30, 40, 50, 60
    }
  
    Serial.print("Follower1: Sending message with ID 0x");
    Serial.println(outgoingMsg.id, HEX);
    int ret = Can.write(MB1, outgoingMsg);
    if(ret == 1) {
      Serial.println("Follower1: Message sent successfully.");
      Serial.print("Data: ");
      for (int i = 0; i < outgoingMsg.len; i++) {
        Serial.print(outgoingMsg.buf[i], DEC);
        Serial.print(" ");
      }
      Serial.println();
    } else {
      Serial.print("Follower1: Error sending message. Error code: ");
      Serial.println(ret);
    }
    
    // Give some time for feedback to arrive
    delay(10);
    
    // Process events so that any incoming messages are queued
    
    // Poll for a feedback message (with ID 0x300)
    int r = Can.read(incomingMsg);
    if(r == 1) {
      Serial.print("Follower1: Received message with ID 0x");
      Serial.println(incomingMsg.id, HEX);
      Serial.print("Data: ");
      for (int i = 0; i < incomingMsg.len; i++) {
        Serial.print(incomingMsg.buf[i], DEC);
        Serial.print(" ");
      }
      Serial.println();
    } else {
      // If no message is available, read() returns 0.
      Serial.println("Follower1: No feedback message available.");
    }
    
    delay(990);  // Repeat every second
}
