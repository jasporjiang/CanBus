// Manifold.cpp
// Author: Shaopeng Jiang
// This script creates a manifold class for the specific 3-channel manifold module.
// If you want to reduce the number of active channels, just change the NUM_CNANNELS variable below.

#include <iostream>
#include "Manifold.h"
#define NUM_CHANNELS 3
#define VALVE_HIGH 255 //
#define VALVE_LOW 0

// Fixed pin numbers by default for each channel
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

// this function set the desired pressure for specific channel. 
void Manifold::setPressureSetpoint(int channelIndex, double setpoint) {
    if (channelIndex >= 0 && channelIndex < static_cast<int>(channels.size())) {
        channels[channelIndex]->setPressureSetpoint(setpoint);
    } 
    else {
        std::cerr << "Error: Invalid index for manifold setPressureSetpoint()." << std::endl;
    }
}

// this function returns the actual pressure of specific channel. 
double Manifold::getPressure(int channelIndex) {
    if (channelIndex >= 0 && channelIndex < static_cast<int>(channels.size())) {
        return channels[channelIndex]->getPressure();
    }
    else {
        std::cerr << "Error: Invalid index for manifold getPressure()." << std::endl;
        return 0.0;
    }
}

// This function sets the PID parameters and lowpass filter weight ratio.
void Manifold::setPIDTunings(int channelIndex, double Kp, double Ki, double Kd, double Alpha) {
    if (channelIndex >= 0 && channelIndex < static_cast<int>(channels.size())) {
        channels[channelIndex]->setPIDTunings(Kp, Ki, Kd, Alpha);
    }
}

// This function extracts the last input to both inlet and outlet valve. 
void Manifold::getValveStatus(int channelIndex, double &inletStatus, double &outletStatus) {
    if (channelIndex >= 0 && channelIndex < static_cast<int>(channels.size())) {
        channels[channelIndex]->getValveStatus(inletStatus, outletStatus);
    }
}

int Manifold::getChannelCount() {
    return channels.size();
}

// method to purge outlet valves for a given duration. It should be called in every setup() loop.
void Manifold::purgeOutlets(unsigned long purgeDuration) {
    // Open outlet valves (255 = fully open)
    for (int i = 0; i < NUM_CHANNELS; ++i) {
        analogWrite(outletPins[i], VALVE_HIGH);
    }
    delay(purgeDuration);
    // Close outlet valves (0 = closed)
    for (int i = 0; i < NUM_CHANNELS; ++i) {
        analogWrite(outletPins[i], VALVE_LOW);
    }
}