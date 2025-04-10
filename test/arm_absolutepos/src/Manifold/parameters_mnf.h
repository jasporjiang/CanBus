// parameters_mnf.h
#ifndef PARAMETERS_MNF_H
#define PARAMETERS_MNF_H

#define ANALOG_READ_RESOLUTION 12
#define ANALOG_WRITE_RESOLUTION 12

#define NUM_CHANNELS 3

// for PID controller limits
#define MAX_PWM               (1<<ANALOG_WRITE_RESOLUTION) - 1              // [8/12 bits]
#define MIN_PWM               (int)(0.03*MAX_PWM)           // [8 bits], minimal air outlet rate. setted to non-zero rate for reducing dead zone
#define PWM_FREQUENCY         20000 // for proportional valves

#define OUTPUT_LIMIT          (1<<ANALOG_WRITE_RESOLUTION) - 1         // [8 bits]

#define ANTI_WINDUP_LIMIT     (1<<ANALOG_WRITE_RESOLUTION) - 1         // [8 bits]

// for pressure sensor analog reading
#define MIN_PRESSURE          0                 // [kPa]
#define MAX_PRESSURE          100               // [kPa]
#define PRESSURE_READ_RANGE   (1<<ANALOG_READ_RESOLUTION) - 1.0
#define PRESSURE_CONTROL_PERIOD 1               // [ms]

// for serial baud rate setup
#define BAUD_SERIAL           115200
#endif