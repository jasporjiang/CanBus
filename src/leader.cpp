#include <Arduino.h>
#include <FlexCAN_T4.h>
// fixed ID protocol, see readme for more details
#define P_LEADER 0x100 //pressure
#define D_LEADER 0x200 //displacement

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can;
CAN_message_t outgoingMsg;

volatile uint8_t p = 100;
volatile uint8_t d = 1;

// Feedback storage (updated via callback)
volatile uint8_t fb_p = p;  // from follower1
volatile uint8_t fb_d = d;  // from follower2

// Decode function: we need this function since multiple follower will send feedback together (8 kinds of ID thus require separet function for sorting.)
void decodeID(uint32_t id, uint8_t &agentID, uint8_t &msgType) {
  if (id >= 0x200 && id < 0x300) {
    msgType = 1; // Potentiometer message
    agentID = id - 0x200; 
  } else if (id >= 0x100 && id < 0x200) {
    msgType = 0; // Pressure message
    agentID = id - 0x100; // e.g., 0x101 gives  follower ID 1
  } else {
    msgType = 255;
    agentID = 255;
  }
}

// Callback for receiving feedback messages on MB0
void leaderCallback(const CAN_message_t &incomingMsg) {
  // Identify which follower sent feedback by CAN ID.
  uint8_t agentID, msgType;
  decodeID(incomingMsg.id, agentID, msgType);

  Serial.print("Leader: Received feedback from Follower ");
  Serial.print(agentID);
  Serial.print(" (");
  Serial.print((msgType == 0) ? "Pressure" : "Potentiometer");
  Serial.print("): Data = ");
  for (uint8_t i = 0; i < incomingMsg.len; i++) {
    Serial.print(incomingMsg.buf[i], DEC);
    Serial.print(" ");
  }
  Serial.println();

  if (msgType == 0) {
    fb_p = incomingMsg.buf[0]; // Pressure feedback from follower w/#agentID
  } 
  else if (msgType == 1) {
    fb_d = incomingMsg.buf[0]; // displacement feedback from follower
  }
}

void setup() {
  Serial.begin(115200);
  Can.begin();
  Can.setBaudRate(250000);
  Can.setMaxMB(3);  // We'll use MB0 for RX (feedback) and MB1 for TX (broadcast)
  
  Can.setMB(MB0, RX, STD);
  Can.setMBFilter(MB0, 0x101, 0x201);  
  
  // Configure MB1 for transmitting broadcast messages
  Can.setMB(MB1, TX, STD);
  
  Can.enableMBInterrupts();
  Can.onReceive(MB0, leaderCallback);
  
  Serial.println("Leader: CAN bus initialized (event-driven).");
}

void loop() {
  Serial.println("----------------------");
  // Leader broadcasts its message on CAN ID 0x100 with two bytes: value1 and value2.
  p = fb_p;
  d = fb_d;
  outgoingMsg.id = P_LEADER;
  outgoingMsg.len = 1;
  outgoingMsg.flags.extended = 0;
  outgoingMsg.buf[0] = p;
  
  Serial.print("Leader: Broadcasting pressure ");
  Serial.print ("[");
  Serial.print(p);
  Serial.println("]");
  
  int ret = Can.write(MB1, outgoingMsg);
  if(ret == 1) {
    Serial.println("Leader: Broadcast sent successfully.");
  } else {
    Serial.print("Leader: Error sending broadcast. Code: ");
    Serial.println(ret);
  }
  
  // Wait a short while to allow followers to respond.
  delay(500);

  outgoingMsg.id = D_LEADER;
  outgoingMsg.len = 1;
  outgoingMsg.flags.extended = 0;
  outgoingMsg.buf[0] = d;
  
  Serial.print("Leader: Broadcasting position ");
  Serial.print ("[");
  Serial.print(d);
  Serial.println("]");
  
  ret = Can.write(MB1, outgoingMsg);
  if(ret == 1) {
    Serial.println("Leader: Broadcast sent successfully.");
  } else {
    Serial.print("Leader: Error sending broadcast. Code: ");
    Serial.println(ret);
  }
  
  // Use received feedback to update broadcast values.
  // For this demo, we'll simply adopt the feedback values.
  if (p < 50) {
    p = 100;
    fb_p = p;
}
  if (d > 50) {
    d = 1;
    fb_d = d;
}

  delay(500);
}
