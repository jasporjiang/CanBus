#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "Joystick/origamiJoystick.h"
#include "Cancomm/CANcomm.h"
#include "parameters.h"

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> canBus;
CAN_message_t outgoingMsg;
CANCommunication canComm(canBus, ID_LEADER);

int joystickPins[JOYSTICK_DOFS] = {A7, A9, A10}; // changed the pins A0->A7, A1->A9, A2->A10
double measuredLegAngles[JOYSTICK_DOFS] = {0, 0, 0}; // [rad]
unsigned long sendTime = 0; //to measure the time delay

// Callback for receiving feedback messages from follower. currently assume that only receive pressure information.
void leaderCallback(const CAN_message_t &incomingMsg) {
// Identify which follower sent feedback by CAN ID.
// uint8_t agentID;
// decodeID(incomingMsg.id, agentID, msgType);

//measuring latency
unsigned long receiveTime = micros();  // <-- Get response timestamp
unsigned long latency = receiveTime - sendTime;
Serial.print("> Leader: Communication latency: ");
Serial.println(latency); //microsecond

// Serial.print("Leader: Received feedback from Follower ");
// Serial.println(incomingMsg.id - 0x100);
// Serial.print("Leader: read sample pressure data: [");
///////////// read pressure data //////////////
double pressure[3];
if (incomingMsg.len == 6) {
  uint16_t scaledData[3];

  // Unpack the scaled setpoints from the message buffer
  memcpy(&scaledData[0], &incomingMsg.buf[0], sizeof(uint16_t));
  memcpy(&scaledData[1], &incomingMsg.buf[2], sizeof(uint16_t));
  memcpy(&scaledData[2], &incomingMsg.buf[4], sizeof(uint16_t));

  // Convert scaled setpoints back to float and update PID controllers
  for (int i = 0; i < 3; ++i) {
      double data = static_cast<float>(scaledData[i]) / 100.0f;
      // setPressureSetpoint(i, setpoint);
      pressure[i] = data;
      Serial.print("> leader pressure in ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(pressure[i]);
      // Serial.print(" ");
    }
    // Serial.println("]");
  }
  else {
    Serial.println("Invalid setpoint message length.");
  }
}

void setup() {
    Serial.println("Leader: initialization starts.");
    Serial.begin(BAUD_SERIAL);
    canBus.begin();
    canComm.setup();
    canComm.onReceive(leaderCallback);
    canComm.canBus.setMBFilter(ACCEPT_ALL); //example to setup filter directly
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
    Serial.println("----------------------");
    // get the position values from potentiometer
    for (size_t i = 0; i < JOYSTICK_DOFS; i++)
    {
      measuredLegAngles[i] = readJoystickLegAngle(joystickPins[i]); // [rad]
    }

    sendTime = micros(); 
    int ret = canComm.sendMessage(D_LEADER, measuredLegAngles, 3);
    if(ret == 1) {
      Serial.println("Leader: Broadcast sent successfully.");
    } else {
      Serial.print("Leader: Error sending broadcast. Code: ");
      Serial.println(ret);
    }
    
    delay(50);
}