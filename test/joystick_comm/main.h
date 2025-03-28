#include <Arduino.h>
#include "origamiJoystick.h"
#include "Timing.cpp"
 #include "communication.h"
//#include "newComm.h"

const int controlMode = 0; // 0-> jotstick control; 1-> automated;

#define BAUD_RATE                   115200    
#define ANALOG_READ_RESOLUTION      12
#define ANALOG_WRITE_RESOLUTION      12