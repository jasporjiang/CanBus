#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "Cancomm/CANcomm.h" 
#include "Manifold/manifold.h"

#include "parameters.h"

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> canBus;
CAN_message_t outgoingMsg;
CANCommunication canComm(canBus, ID_FOLLOWER);

Manifold manifold;
const int analogPin0 = A0;
const int analogPin1 = A1;
const int analogPin2 = A3;

// Map joystick reading [0.4, 0.9] to pressure [20, 45] kPa
double mapJoystickToPressure(double joystickVal) {
  const double minJoy = 0.4;
  const double maxJoy = 0.9;
  const double minPressure = 10.0;
  const double maxPressure = 30.0;

  // Clamp value for safety
  if (joystickVal < minJoy) joystickVal = minJoy;
  if (joystickVal > maxJoy) joystickVal = maxJoy;

  // Linear mapping
  return minPressure + (joystickVal - minJoy) * (maxPressure - minPressure) / (maxJoy - minJoy);
}

// Callback for processing broadcast messages (ID 0x100)
void followerCallback(const CAN_message_t &incomingMsg) {
  Serial.print("--------- follower ");
  Serial.print(ID_FOLLOWER);
  Serial.println(" -------------");
  Serial.print("Received message with ID 0x");
  Serial.println(incomingMsg.id, HEX);

  double values[4];
  uint8_t numValues;
  if (canComm.readMessage(incomingMsg, values, numValues)) {
    // manifold actuating
    for (int i = 0; i < 3; ++i) {
      double targetPressure = mapJoystickToPressure(values[i]);
      manifold.setPressureSetpoint(i, targetPressure);
      Serial.print("Leg ");
      Serial.print(i);
      Serial.print(" joystick: ");
      Serial.print(values[i]);
      Serial.print(" → pressure setpoint: ");
      Serial.println(targetPressure);
  }     
  } else {
      Serial.println("CANcomm: Failed to decode message.");
  }

  double pressure[3];
  for (int i = 0; i < 3; i++) {
  pressure[i] = manifold.getPressure(i); 
  }//physically now using channel 3 as an example

  uint32_t responseID = D_BASE + ID_FOLLOWER; // module ID + message type ID
  int ret = canComm.sendMessage(responseID, pressure, 3);
  if(ret == 1) {
    Serial.print("Follower ");
    Serial.print(ID_FOLLOWER);
    Serial.print(": Response with ");
    Serial.print(  "pressure [");
    for (int i = 0; i < 3; i++) {
      Serial.print(pressure[i]);
      Serial.print(" "); 
    }    
    Serial.print("]");
    Serial.print(", ID 0x");
    Serial.println(outgoingMsg.id, HEX);
  } 
  else {
    Serial.print("Follower ");
    Serial.print(ID_FOLLOWER);
    Serial.println(": Error sending response.");
  }  
  delay(10);
}

void setup() {
  Serial.begin(BAUD_SERIAL);
  analogReadResolution(ANALOG_READ_RESOLUTION); //12 bits [0-4095]
  analogWriteResolution(ANALOG_WRITE_RESOLUTION); // TODO: fix PID resolution issue. now can only use res=8.
  canBus.begin();
  canComm.setup();
  canComm.onReceive(followerCallback);

  manifold.setup();
  double Kp = 150;
  double Ki = 5;
  double Kd = 0.0;
  double Alpha = 0.3;
  for(int i = 0; i < 3; i++) {
  manifold.setPIDTunings(i, Kp, Ki, Kd, Alpha);
  }
  manifold.purgeOutlets(3000);

  Serial.print("Follower ");
  Serial.print(ID_FOLLOWER);
  Serial.println(": CAN bus initialized (event-driven).");
}

void loop() {
  manifold.update();
  // In event-driven mode, the callback processes the incoming broadcast.
  // The main loop can be minimal.
  delay(50);
}
