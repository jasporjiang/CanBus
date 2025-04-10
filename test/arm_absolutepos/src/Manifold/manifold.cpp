// Manifold.cpp

#include "Manifold.h"
#include "parameters_mnf.h"

// Define pin numbers for each channel
const int inletPins[NUM_CHANNELS] = {11, 9, 2};
const int outletPins[NUM_CHANNELS] = {13, 6, 3};
const int sensorPins[NUM_CHANNELS] = {A0, A1, A3};

Manifold::Manifold() {
    initializeChannels();
}

Manifold::~Manifold() {
    // Clean up allocated channels
    for (auto channel : channels) {
        delete channel;
    }
    channels.clear();
}

void Manifold::initializeChannels() {
    for (int i = 0; i < NUM_CHANNELS; ++i) {
        Channel* channel = new Channel(inletPins[i], outletPins[i], sensorPins[i]);
        channels.push_back(channel);
    }
}

void Manifold::setup() {
    for (auto channel : channels) {
        channel->setup();
    }
}

void Manifold::update() {
    for (auto channel : channels) {
        channel->update();
    }
}

void Manifold::setPressureSetpoint(int channelIndex, double setpoint) {
    if (channelIndex >= 0 && channelIndex < static_cast<int>(channels.size())) {
        channels[channelIndex]->setPressureSetpoint(setpoint);
    }
}

double Manifold::getPressure(int channelIndex) {
    if (channelIndex >= 0 && channelIndex < static_cast<int>(channels.size())) {
        return channels[channelIndex]->getPressure();
    }
    return 0.0;
}

void Manifold::setPIDTunings(int channelIndex, double Kp, double Ki, double Kd, double Alpha) {
    if (channelIndex >= 0 && channelIndex < static_cast<int>(channels.size())) {
        channels[channelIndex]->setPIDTunings(Kp, Ki, Kd, Alpha);
    }
}

void Manifold::getValveStatus(int channelIndex, double &inletStatus, double &outletStatus) {
    if (channelIndex >= 0 && channelIndex < static_cast<int>(channels.size())) {
        channels[channelIndex]->getValveStatus(inletStatus, outletStatus);
    }
}

int Manifold::getChannelCount() {
    return channels.size();
}

// method to purge outlet valves for a given duration
void Manifold::purgeOutlets(unsigned long purgeDuration) {
    // Open outlet valves (255 = fully open)
    for (int i = 0; i < NUM_CHANNELS; ++i) {
        analogWrite(outletPins[i], MAX_PWM);
    }
    delay(purgeDuration);
    // Close outlet valves (0 = closed)
    for (int i = 0; i < NUM_CHANNELS; ++i) {
        analogWrite(outletPins[i], 0);
    }
}