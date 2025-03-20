#include <Arduino.h>
#include <FlexCAN_T4.h>

// Set follower identity: 1 for follower1, 2 for follower2
#define FOL_ID 1   // Change to 2 for follower2

#define P_BASE       0x100
#define D_BASE  0x200

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can;
CAN_message_t outgoingMsg;

// Callback for processing broadcast messages (ID 0x100)
void followerCallback(const CAN_message_t &incomingMsg) {
  // debouncing: avoid too fast communication by adding a 10ms interval
  static unsigned long lastCallTime = 0;
  unsigned long now = millis();
  // Ignore duplicate events if they occur within 10ms
  if (now - lastCallTime < 10) return;
  lastCallTime = now;

  uint8_t receivedValue = incomingMsg.buf[0];
  Serial.print("Follower ");
  Serial.print(FOL_ID);
  Serial.print(": Received message with ID 0x");
  Serial.print(incomingMsg.id, HEX);
  Serial.print(", data: ");
  Serial.println(receivedValue, DEC);
  
  uint32_t responseID = 0;
  uint8_t responseValue = 0;
  if (incomingMsg.id == 0x100) { // Broadcast from leader
    // Extract the two broadcast values
    responseID = P_BASE + FOL_ID;
    responseValue = receivedValue - 1;
  }
  else if (incomingMsg.id == 0x200)  {
    responseID = D_BASE + FOL_ID;
    responseValue = receivedValue + 1;
  }
  
  outgoingMsg.id = responseID;
  outgoingMsg.len = 1;
  outgoingMsg.flags.extended = 0;
  outgoingMsg.buf[0] = responseValue;

  int ret = Can.write(MB1, outgoingMsg);

  if(ret == 1) {
    Serial.print("Follower ");
    Serial.print(FOL_ID);
    Serial.print(": Response with ");
    Serial.print( (incomingMsg.id == 0x100)? "pressure " : "position " ); 
    Serial.print(responseValue);
    Serial.print(", ID 0x");
    Serial.println(outgoingMsg.id, HEX);
  } 
  else {
    Serial.print("Follower ");
    Serial.print(FOL_ID);
    Serial.println(": Error sending response.");
  }  

}

void setup() {
  Serial.begin(115200);

  Can.begin();
  Can.setBaudRate(250000);
  Can.setMaxMB(2);  // We'll use MB0 for RX and MB1 for TX

  // Configure MB0 for receiving the broadcast message (ID 0x100)
  Can.setMB(MB0, RX, STD);
  Can.setMBFilter(MB0, 0x100, 0x200);
  
  // Configure MB1 for transmitting feedback messages
  Can.setMB(MB1, TX, STD);
  
  Can.enableMBInterrupts();
  Can.onReceive(MB0, followerCallback);
  
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
  delay(100);
}
