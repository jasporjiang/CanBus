#include <Arduino.h>
#include <FlexCAN_T4.h>

// Set follower identity: 1 for follower1, 2 for follower2
#define FOL_ID 1   // Change to 2 for follower2
#define P_BASE  0x100
#define D_BASE  0x200
#define BAUD_SERIAL                 115200    
#define BAUD_CANBUS                 250000
#define ANALOG_READ_RESOLUTION      12
#define ANALOG_WRITE_RESOLUTION     12

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can;
CAN_message_t outgoingMsg;

// Callback for processing broadcast messages (ID 0x100)
void followerCallback(const CAN_message_t &incomingMsg) {
  // debouncing: avoid too fast communication by adding a 10ms interval
  // static unsigned long lastCallTime = 0;
  // unsigned long now = millis();
  // // Ignore duplicate events if they occur within 10ms
  // if (now - lastCallTime < 10) return;
  // lastCallTime = now;

  Serial.print("Follower ");
  Serial.print(FOL_ID);
  Serial.print(": Received message with ID 0x");
  Serial.print(incomingMsg.id, HEX);
  Serial.print(", data: [");
  float stored_data[3] = {0,0,0};
  if (incomingMsg.len == 6) {
    uint16_t scaledSetpoints[3];

    // Unpack the scaled setpoints from the message buffer
    memcpy(&scaledSetpoints[0], &incomingMsg.buf[0], sizeof(uint16_t));
    memcpy(&scaledSetpoints[1], &incomingMsg.buf[2], sizeof(uint16_t));
    memcpy(&scaledSetpoints[2], &incomingMsg.buf[4], sizeof(uint16_t));

    // Convert scaled setpoints back to float and update PID controllers
    for (int i = 0; i < 3; ++i) {
        float setpoint = static_cast<float>(scaledSetpoints[i]) / 100.0f;
        // setPressureSetpoint(i, setpoint);
        stored_data[i] = setpoint;
        
        Serial.print(stored_data[i]);
        Serial.print(" ");
      }
      Serial.println("]");
    }
  else {
    Serial.println("Invalid setpoint message length.");
  }

  uint32_t responseID = 0;
  uint8_t responseValue = 0;
  // if (incomingMsg.id == 0x200) // potentiometer from leader 
  // {
  responseID = D_BASE + FOL_ID;
  responseValue = 50;
  // }
  
  outgoingMsg.id = responseID;
  outgoingMsg.len = 1;
  outgoingMsg.flags.extended = 0;
  outgoingMsg.buf[0] = responseValue;

  int ret = Can.write(MB1, outgoingMsg);

  if(ret == 1) {
    Serial.print("Follower ");
    Serial.print(FOL_ID);
    Serial.print(": Response with ");
    Serial.print(  "pressure "); 
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
  Serial.begin(BAUD_SERIAL);

  Can.begin();
  Can.setBaudRate(BAUD_CANBUS);
  Can.setMaxMB(2);  // We'll use MB0 for RX and MB1 for TX

  // Configure MB0 for receiving the broadcast message (ID 0x100)
  Can.setMB(MB0, RX, STD);
  // Can.setMBFilter(MB0, 0x100, 0x200);
  
  // Configure MB1 for transmitting feedback messages
  Can.setMB(MB1, TX, STD);
  
  Can.enableMBInterrupts();
  Can.onReceive(MB0, followerCallback);
  
  Serial.print("Follower ");
  Serial.print(FOL_ID);
  Serial.println(": CAN bus initialized (event-driven).");
}

void loop() {
  // Serial.print("--------- follower ");
  // Serial.print(FOL_ID);
  // Serial.println(" -------------");
  // In event-driven mode, the callback processes the incoming broadcast.
  // The main loop can be minimal.
  delay(10);
}
