// parameters.h
#ifndef PARAMETERS_H
#define PARAMETERS_H

#define JOYSTICK_DOFS   3
#define ID_LEADER 0
#define ID_FOLLOWER 1

#define P_LEADER        0x100 //pressure
#define D_LEADER        0x200 //displacement

#define P_BASE  0x100
#define D_BASE  0x200
#define BAUD_SERIAL 115200    
#define BAUD_CANBUS 250000
#define ANALOG_READ_RESOLUTION 12
#define ANALOG_WRITE_RESOLUTION 12

#endif // PARAMETERS_H