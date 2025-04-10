/**********************************************************************************************
 * Arduino PID Libraryfor pneumagami
 * created by Mustafa Mete <mustafa.mete@epfl.ch> and modified by Shaopeng Jiang <shaopeng.jiang@epfl.ch>
 **********************************************************************************************/
#include "PID_custom.h"
#include "parameters_mnf.h"
#include "Arduino.h"
//Constructor
PID::PID(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd, unsigned long SampleTime, int ControllerDirection)
{
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint; 
    this->SampleTime = SampleTime;              //default Controller Sample Time is 0.01 seconds  // [ms]

    PID::SetOutputLimits(-OUTPUT_LIMIT/2, OUTPUT_LIMIT);				//default output limit corresponds to //the arduino pwm limits
    PID::IntegratorAntiWindUpLimits(-OUTPUT_LIMIT/2, OUTPUT_LIMIT); //min below 0 to allow vent

    PID::SetControllerDirection(ControllerDirection);
    PID::SetTunings(Kp, Ki, Kd);

    lastTime = millis()-SampleTime;
}


/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool PID::Compute()
{
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      double input = *myInput;
        myError = *mySetpoint - input; // should error be global?
        dInput = (input - lastInput);

         // ANTI_WINDUP via back calculation and clamping
         double integratorPre = integrator + (ki * myError);

         // Filter the derivative
         dInputFiltered = lowpassFilter(dInputFiltered, dInput, filterBeta);
         // -- Compute the unsaturated output using the provisional integrator.
         double unsatOutput = (kp * myError) + integratorPre - (kd * dInputFiltered);
         // -- Saturate the output.
         double saturatedOutput = unsatOutput;
         if (unsatOutput > outMax)
            saturatedOutput = outMax;
         else if (unsatOutput < outMin)
            saturatedOutput = outMin;

         // -- Back-calculation for anti-windup:
         // beta: anti-windup gain (a small factor, e.g., 0.1).
         double beta = 0.0;
         // Adjust the integrator by feeding back the saturation error.
         // This moves the integrator toward a value that would produce the clamped output.
         integrator = integratorPre + beta * (saturatedOutput - unsatOutput);  
         // Optional, clamp the integrator to ensure it stays within set bounds.
         if(integrator > outMaxIntegrator) integrator = outMaxIntegrator;
         else if(integrator < outMinIntegrator) integrator = outMinIntegrator;

        // PID
        double output;
        output = kp * myError + integrator - kd * dInputFiltered;

        //clamping the pid output
        if(output > outMax) output = outMax;
        else if(output < outMin) output = outMin;

        *myOutput = output;

        // Remember some variables for next time 
        lastInput = input;
        lastTime = now;
        return true;
   }
   else return false;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void PID::SetTunings(double Kp, double Ki, double Kd)
{
   if (Kp<0 || Ki<0 || Kd<0) return;

   double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;

  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}



/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void PID::SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID::SetOutputLimits(double Min, double Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;

   // if(*myOutput > outMax) *myOutput = outMax;
   // else if(*myOutput < outMin) *myOutput = outMin;

   // if(integrator > outMaxIntegrator) integrator = outMaxIntegrator;
   // else if(integrator < outMinIntegrator) integrator = outMinIntegrator;
}

void PID::IntegratorAntiWindUpLimits(double Min, double Max){
   if(Min >= Max) return;
   outMinIntegrator= Min;
   outMaxIntegrator= Max;
}


/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.) 
 ******************************************************************************/
void PID::SetControllerDirection(int Direction){
   if(Direction !=controllerDirection){
        kp = (0 - kp);
        ki = (0 - ki);
        kd = (0 - kd);
   }
   controllerDirection = Direction;
}

//change this to moving average
double PID::lowpassFilter(double previousFilteredValue, double input, double  beta){
  return beta * previousFilteredValue + (1 - beta) * input;
}