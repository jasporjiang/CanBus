#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "origamiJoystick.h"
#define P_LEADER 0x100 //pressure
#define D_LEADER 0x200 //displacement
#define JOYSTICK_DOFS 3
#define BAUD_SERIAL                   115200    
#define BAUD_CANBUS                 250000
#define ANALOG_READ_RESOLUTION      12
#define ANALOG_WRITE_RESOLUTION     12

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can;
CAN_message_t outgoingMsg;

int joystickPins[JOYSTICK_DOFS] = {A7, A9, A10}; // changed the pins A0->A7, A1->A9, A2->A10
double measuredLegAngles[JOYSTICK_DOFS] = {0, 0, 0}; // [rad]
  
// Callback for receiving feedback messages from follower. currently assume that only receive pressure information.
void leaderCallback(const CAN_message_t &incomingMsg) {
// Identify which follower sent feedback by CAN ID.
// uint8_t agentID;
// decodeID(incomingMsg.id, agentID, msgType);

Serial.print("Leader: Received feedback from Follower ");
Serial.print(incomingMsg.id - 0x100);
Serial.print(" (");
Serial.print("Pressure Data = ");
for (uint8_t i = 0; i < incomingMsg.len; i++) {
    Serial.print(incomingMsg.buf[i], DEC);
    Serial.print(" ");
}
Serial.println();
}

void setup() {
    Serial.begin(BAUD_SERIAL);
    Can.begin();
    Can.setBaudRate(BAUD_CANBUS);
    Can.setMaxMB(2);  // We'll use MB0 for RX (feedback) and MB1 for TX (broadcast)
    
    Can.setMB(MB0, RX, STD);
    // Can.setMBFilter(MB0, 0x101, 0x201);  
    
    // Configure MB1 for transmitting broadcast messages
    Can.setMB(MB1, TX, STD);
    
    Can.enableMBInterrupts();
    Can.onReceive(MB0, leaderCallback);
    
    analogReadResolution(ANALOG_READ_RESOLUTION); //12 bits [0-4095]
    analogWriteResolution(ANALOG_WRITE_RESOLUTION); //12 bits [0-4095]
    // initialize the joystick pins
    for (size_t i = 0; i < JOYSTICK_DOFS; i++)
    {
        pinMode(joystickPins[i], INPUT);
    }

    Serial.println("Leader: CAN bus initialized (event-driven).");
  }
  
void loop() {
    outgoingMsg.id = D_LEADER;
    outgoingMsg.len = 6;
    outgoingMsg.flags.extended = 0;

    Serial.println("----------------------");
    Serial.print("Leader: Broadcasting position ");
    Serial.print ("[");
    // get the position values from potentiometer
    float test[3] = {0,0,0};
    for (size_t i = 0; i < JOYSTICK_DOFS; i++)
    {
      measuredLegAngles[i] = readJoystickLegAngle(joystickPins[i]); // [rad]

      uint16_t scaledSetpoint = static_cast<uint16_t>(measuredLegAngles[i] * 100);
      memcpy(&outgoingMsg.buf[i * 2], &scaledSetpoint, sizeof(uint16_t));
      Serial.print(measuredLegAngles[i]);
      Serial.print(" ");
    }
    Serial.println("]");

    int ret = Can.write(MB1, outgoingMsg);
    if(ret == 1) {
      Serial.println("Leader: Broadcast sent successfully.");
    } else {
      Serial.print("Leader: Error sending broadcast. Code: ");
      Serial.println(ret);
    }
    
    delay(100);
}