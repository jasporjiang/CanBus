#include "origamiJoystick.h"

double readJoystickLegAngle(int pin){
  double inputVoltage = 3.3; //5; // [v]
  double sensorVoltage = (inputVoltage*(double)analogRead(pin)/1023.0)/4; // 10 bit resolution, might need to divide by 4 in case 12 bit resolution
  double sensorDisplacement, x, sensorAngleCalculated;
  double xOffset= 7.50; // b - a  (mm) in the model!
  sensorDisplacement = 8*(inputVoltage-sensorVoltage)/inputVoltage;  //then from voltage to [0mm,8mm] range, // this is delta X,
  x = sensorDisplacement + xOffset; //but we need to give total X in the inv. kinematic model (or with its offset)
  // inverse kinematics
  sensorAngleCalculated = 2* atan2( -4*(4*x -sqrt(436.5*pow(x,2) - pow(x,4) -18473.0625) +24 ) , (48*x - 4*pow(x,2) + 521) );
  //clamp the angle between 0 and pi/2
    if(sensorAngleCalculated < 0){
        sensorAngleCalculated = 0;
    }
    else if(sensorAngleCalculated > PI/2){
        sensorAngleCalculated = PI/2;
    }
  return sensorAngleCalculated; //rad
}

