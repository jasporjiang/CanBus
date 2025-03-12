#include <Arduino.h>
#include <FlexCAN_T4.h>

// Set follower identity: 1 for follower1, 2 for follower2
#define FOL_ID 1   // Change to 2 for follower2

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can;
CAN_message_t incomingMsg;  // Buffer for received broadcast message
CAN_message_t outgoingMsg;  // Buffer for sending feedback

// Callback for processing broadcast messages (ID 0x100)
void followerBroadcastCallback(const CAN_message_t &msg) {
  if (msg.id == 0x100) { // Broadcast from leader
    // Extract the two broadcast values
    uint8_t val1 = msg.buf[0];
    uint8_t val2 = msg.buf[1];
    uint8_t val3 = msg.buf[2];
    Serial.print("Follower ");
    Serial.print(FOL_ID);
    Serial.print(": Received broadcast [");
    Serial.print(val1);
    Serial.print(", ");
    Serial.print(val2);
    Serial.print(", ");
    Serial.print(val3);
    Serial.println("]");
    
    // Compute feedback based on follower identity
    uint8_t feedback;
    if (FOL_ID == 1) {
      // Follower1: feedback = first value - 1
      feedback = val1 - 1;
      outgoingMsg.id = 0x300;
    } else if (FOL_ID == 2) {
      // Follower2: feedback = second value + 1
      feedback = val2 + 1;
      outgoingMsg.id = 0x400;
    } else if (FOL_ID == 3) {
      feedback = val3 + 1; // Default
      outgoingMsg.id = 0x500;
    }
    
    // Prepare feedback message with unique ID
    // Use 0x201 for follower1 and 0x202 for follower2
    outgoingMsg.len = 1;
    outgoingMsg.flags.extended = 0;
    outgoingMsg.buf[0] = feedback;
    
    Serial.print("Follower ");
    Serial.print(FOL_ID);
    Serial.print(": Sending feedback: ");
    Serial.println(feedback);
    
    int ret = Can.write(MB1, outgoingMsg);
    if (ret == 1) {
      Serial.println("Follower: Feedback sent successfully.");
    } else {
      Serial.print("Follower: Error sending feedback. Code: ");
      Serial.println(ret);
    }
  }
}

void setup() {
  Serial.begin(115200);

  Can.begin();
  Can.setBaudRate(250000);
  Can.setMaxMB(2);  // We'll use MB0 for RX and MB1 for TX

  // Configure MB0 for receiving the broadcast message (ID 0x100)
  Can.setMB(MB0, RX, STD);
  Can.setMBFilter(MB0, 0x100);
  
  // Configure MB1 for transmitting feedback messages
  Can.setMB(MB1, TX, STD);
  
  Can.enableMBInterrupts();
  Can.onReceive(MB0, followerBroadcastCallback);
  
  Serial.print("Follower ");
  Serial.print(FOL_ID);
  Serial.println(": CAN bus initialized (event-driven).");
}

void loop() {
  Serial.print("--------- follower ");
  Serial.print(FOL_ID);
  Serial.println(" -------------");
  // In event-driven mode, the callback processes the incoming broadcast.
  // The main loop can be minimal.
  delay(50);
}
