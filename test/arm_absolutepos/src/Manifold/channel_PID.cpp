// single channel control with a manifold
// Author:  Shaopeng Jiang <shaopeng.jiang@epfl.ch>

#include "channel_PID.h"
#include <Arduino.h>
#include "parameters_mnf.h"

Channel::Channel(int inletPin, int outletPin, int sensorPin)
  : inletPin(inletPin), outletPin(outletPin), sensorPin(sensorPin),
    pressure(0), filteredPressure(0), filterAlpha(0.0),setpoint(0), output(0),
    Kp(1.0), Ki(0.0), Kd(0.0), SampleTime(10), ControllerDirection(DIRECT),
    pidController(&pressure, &output, &setpoint, Kp, Ki, Kd, SampleTime, ControllerDirection)
{
  // Initialize PID controller parameters
  pidController.SetSampleTime(PRESSURE_CONTROL_PERIOD);  // Set sample time in milliseconds
  pidController.SetOutputLimits(-OUTPUT_LIMIT, OUTPUT_LIMIT);  // Adjust as needed
  pidController.IntegratorAntiWindUpLimits(-ANTI_WINDUP_LIMIT, ANTI_WINDUP_LIMIT);  // Adjust as needed
  // pidController.filterBeta = 0.1;  // Adjust filter coefficient as needed

  // TODO: test Read initial pressure 
  // readPressure();
  // Initialize setpoint to current pressure
  // setpoint = 0;
}

void Channel::setup() {
  // Initialize valve pins
  pinMode(inletPin, OUTPUT);
  pinMode(outletPin, OUTPUT);

  // Initialize sensor pin
  pinMode(sensorPin, INPUT);

  // Set PWM frequency if needed (depending on your board)
  analogWriteFrequency(inletPin, PWM_FREQUENCY);
  analogWriteFrequency(outletPin, PWM_FREQUENCY);
}

void Channel::update() {
  // Read the current pressure
  readPressure();

  // Compute PID output
  if (pidController.Compute()) {
    // Control the valves based on PID output
    controlValves();
  }
}

void Channel::setPressureSetpoint(double setpoint) {
  this->setpoint = setpoint;
  // pidController.Reset(); // TODO: avoid integrater wind-up in PID controller
}

double Channel::getPressure() {
  return pressure;
}

void Channel::setPIDTunings(double Kp, double Ki, double Kd, double Alpha) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->filterAlpha = Alpha;
  pidController.SetTunings(Kp, Ki, Kd);
}

void Channel::getValveStatus(double &inletStatus, double &outletStatus) {
  inletStatus = inletRate;
  outletStatus = outletRate;
}


// //////////////// PRIVATE MEMBERSHIP FUNCTIONS ///////////////////
// add low-pass filter here
void Channel::readPressure() {
  int adcValue = analogRead(sensorPin);
  double rawPressure = convertADCtoPressure(adcValue);

  // Initialize filteredPressure with the first reading
  if (filteredPressure == 0) {
    filteredPressure = rawPressure;
  } else {
    // Apply low-pass filter
    filteredPressure = filterAlpha * filteredPressure + (1.0 - filterAlpha) * rawPressure;
  }
  pressure = filteredPressure;
}

void Channel::controlValves() {
  int inletDutyCycle = MIN_PWM;
  int outletDutyCycle = MIN_PWM;

  // Normalize the PID output to [0, 1]
  // double maxDutyCycle = 255.0;
  // double normalizedOutput = output / maxDutyCycle; //check PID controller output range

  // double openness = output;

  // Determine whether to pressurize or depressurize based on PID output
  if (output > 0) {
    // Need to increase pressure - open inlet valve
    inletDutyCycle = constrain((int)(output) + MIN_PWM, MIN_PWM, MAX_PWM);
    outletDutyCycle = MIN_PWM;
  } else {
    // Need to decrease pressure - open outlet valve
    inletDutyCycle = MIN_PWM;
    outletDutyCycle = constrain((int)(-output) + MIN_PWM, MIN_PWM, MAX_PWM);
  }
  setValveStatus(inletDutyCycle, outletDutyCycle);
}

void Channel::setValveStatus(double inletDutyCycle, double outletDutyCycle) {
  // This function is responsible for actually setting the valve openness (PWM)
  this->inletRate = inletDutyCycle / (MAX_PWM - MIN_PWM);
  this->outletRate = outletDutyCycle / (MAX_PWM - MIN_PWM);

  // Write to the valves
  analogWrite(inletPin, inletDutyCycle);
  analogWrite(outletPin, outletDutyCycle);
  // Serial.print(">inletPin:");//TODO: testing
  // Serial.println(inletDutyCycle);
  // Serial.print(">OUTletPin:");//TODO: testing
  // Serial.println(outletDutyCycle);
}


double Channel::convertADCtoPressure(int adcValue) {
  double percent = adcValue / double(PRESSURE_READ_RANGE) * 8.4 / 5.1;
  double pressure = (percent - 0.1) * (103.42*2)/0.8 -  103.42 + 3.17; 
  return pressure;
}

