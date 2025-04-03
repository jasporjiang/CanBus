#ifndef CHANNEL_H
#define CHANNEL_H

#include <Arduino.h>
#include "PID.h"

class Channel {
  public:
    // Constructor
    Channel(int inletPin, int outletPin, int sensorPin);
    // Setup method
    void setup();
    // Update method to read sensor and control valves
    void update();
    // Set pressure setpoint
    void setPressureSetpoint(double setpoint);
    // Get current pressure
    double getPressure();
    // Set PID tunings
    void setPIDTunings(double Kp, double Ki, double Kd, double filterAlpha);
    void getValveStatus(double &inletStatus, double &outletStatus);
    void setValveStatus(double inletRate, double outletRate);


  private:
    // Valve pins
    int inletPin;
    int outletPin;
    int sensorPin;

    // Pressure variables
    double pressure;
    double filteredPressure;
    double filterAlpha; // Filter coefficient between 0 and 1

    double setpoint; // desired pressure

    // PID variables
    double output; 

    // PID tuning parameters
    double Kp;
    double Ki;
    double Kd;
    unsigned long SampleTime;
    int ControllerDirection;
    // PID output
    PID pidController;

    // valve status
    double inletRate;
    double outletRate;

    // Private methods
    void readPressure();
    void controlValves();
    double convertADCtoPressure(int adcValue);
};

#endif