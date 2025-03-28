#include "main.h"

#define JOYSTICK_DOFS 3
// create a pin array
int joystickPins[JOYSTICK_DOFS] = {A7, A9, A10}; // changed the pins A0->A7, A1->A9, A2->A10
double measuredLegAngles[JOYSTICK_DOFS] = {0, 0, 0}; // [rad]


MasterComm masterComm(0x00);

void setup() {
  // any reason why?
  analogReadResolution(ANALOG_READ_RESOLUTION); //12 bits [0-4095]
  analogWriteResolution(ANALOG_WRITE_RESOLUTION); //12 bits [0-4095]

  // initialize the joystick pins
  for (size_t i = 0; i < JOYSTICK_DOFS; i++)
  {
    pinMode(joystickPins[i], INPUT);
  }

  masterComm.setupMasterComm(); 
  Serial.begin(BAUD_RATE);
}

void loop() {
  float test[3] = {0,0,0};
  for (size_t i = 0; i < JOYSTICK_DOFS; i++)
  {
    measuredLegAngles[i] = readJoystickLegAngle(joystickPins[i]); // [rad]
    test[i] = (float)measuredLegAngles[i];
  }

  masterComm.sendSetpoints(0x01, test);

  Serial.print("> Leg1: ");
  Serial.println(measuredLegAngles[0], 2);
  Serial.print("> Leg2: ");
  Serial.println(measuredLegAngles[1], 2);
  Serial.print("> Leg3: ");
  Serial.println(measuredLegAngles[2], 2);
  delay(100);
}
