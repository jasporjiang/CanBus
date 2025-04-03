// Manifold.h

#ifndef MANIFOLD_H
#define MANIFOLD_H

#include <vector>
#include "Channel.h"

class Manifold {
public:
    Manifold();
    ~Manifold(); // Destructor to clean up dynamically allocated channels

    void setup();
    void update();

    void setPressureSetpoint(int channelIndex, double setpoint);
    double getPressure(int channelIndex);
    void setPIDTunings(int channelIndex, double Kp, double Ki, double Kd, double Alpha);
    void getValveStatus(int channelIndex, double &inletStatus, double &outletStatus);
    int getChannelCount();

    void purgeOutlets(unsigned long purgeDuration);//used for intialization to eject air fully
private:
    std::vector<Channel*> channels; // Store pointers to Channel objects

    void initializeChannels();
};

#endif // MANIFOLD_H
