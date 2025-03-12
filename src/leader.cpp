#include <Arduino.h>
#include <FlexCAN_T4.h>

// Use CAN3 (adjust if needed)
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can;
CAN_message_t broadcastMsg;  // Message to broadcast (ID 0x100)
CAN_message_t feedbackMsg;   // Temporary buffer for feedback

// Global variables for broadcast values
volatile uint8_t value1 = 100;
volatile uint8_t value2 = 1;
volatile uint8_t value3 = 1;

// Feedback storage (updated via callback)
volatile uint8_t fb1 = value1;  // from follower1
volatile uint8_t fb2 = value2;  // from follower2
volatile uint8_t fb3 = value3;  // from follower3

// Callback for receiving feedback messages on MB0
void leaderFeedbackCallback(const CAN_message_t &msg) {
  // Identify which follower sent feedback by CAN ID.
  if (msg.id == 0x300) {
    fb1 = msg.buf[0]; // Feedback from follower1
    Serial.print("Leader: Received feedback from Follower 1: ");
    Serial.println(fb1);
  } 
  else if (msg.id == 0x400) {
    fb2 = msg.buf[0]; // Feedback from follower2
    Serial.print("Leader: Received feedback from Follower 2: ");
    Serial.println(fb2);
  }
    
  else if (msg.id == 0x500) {
    fb3 = msg.buf[0]; // Feedback from follower2
    Serial.print("Leader: Received feedback from Follower 3: ");
    Serial.println(fb3);
  }
}

void setup() {
  Serial.begin(115200);
  Can.begin();
  Can.setBaudRate(250000);
  Can.setMaxMB(3);  // We'll use MB0 for RX (feedback) and MB1 for TX (broadcast)
  
  Can.setMB(MB0, RX, STD);
  Can.setMBFilter(MB0, 0x300, 0x400, 0x500);  
  
  // Configure MB1 for transmitting broadcast messages
  Can.setMB(MB1, TX, STD);
  
  Can.enableMBInterrupts();
  Can.onReceive(MB0, leaderFeedbackCallback);
  
  Serial.println("Leader: CAN bus initialized (event-driven).");
}

void loop() {
  Serial.println("----------------------");
  // Leader broadcasts its message on CAN ID 0x100 with two bytes: value1 and value2.
  broadcastMsg.id = 0x100;
  broadcastMsg.len = 3;
  broadcastMsg.flags.extended = 0;
  broadcastMsg.buf[0] = value1;
  broadcastMsg.buf[1] = value2;
  broadcastMsg.buf[2] = value3;
  
  Serial.print("Leader: Broadcasting [");
  Serial.print(value1);
  Serial.print(", ");
  Serial.print(value2);
  Serial.print(", ");
  Serial.print(value3);
  Serial.println("]");
  
  int ret = Can.write(MB1, broadcastMsg);
  if(ret == 1) {
    // Serial.println("Leader: Broadcast sent successfully.");
  } else {
    Serial.print("Leader: Error sending broadcast. Code: ");
    Serial.println(ret);
  }
  
  // Wait a short while to allow followers to respond.
  delay(1000);
  
  // Use received feedback to update broadcast values.
  // For this demo, we'll simply adopt the feedback values.
  value1 = fb1;  // e.g. follower1 sent feedback as (value1 - 1)
  value2 = fb2;  // e.g. follower2 sent feedback as (value2 + 1)
  value3 = fb3; 
  // Optional: add logic to reset values if needed (e.g., if value1 exceeds a threshold, reset to 100)
  if (value1 < 50) {
    value1 = 100;
  }
  if (value2 > 50) {
    value2 = 1;
  }
  if (value3 > 20) {
    value3 = 1;
  }
}
