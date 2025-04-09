// This is a sample usage of a manifold to perform swipping sinusoid for bandwidth test.
// Only a single channel is used, while the other 2 channels are inactive.

#include <Arduino.h>
#include "Manifold.h"  
#include "parameters.h"

Manifold manifold;

void setup() {
    Serial.begin(BAUD_SERIAL);
    analogReadResolution(ANALOG_READ_RESOLUTION);
    analogWriteResolution(12);
    // Setup the manifold (creates channels and performs channel setup)
    manifold.setup();
    double Kp = 25.0*16;
    double Ki = 5.0;
    double Kd = 0.0;
    double Alpha = 0.3;
    for(int i = 0; i < 3; i++) {
    manifold.setPIDTunings(i, Kp, Ki, Kd, Alpha);
    }
    // Perform an outlet purge (opens the outlets for 3 seconds) for a consistent initialization.
    manifold.purgeOutlets(3000);
}

void loop() {
    for(int i = 0; i < 3; i++) {
        manifold.setPressureSetpoint(i, (i+1)*10);
    }
    double pressure[3];
    for (int i = 0; i < 3; i++) {
        pressure[i] = manifold.getPressure(i); 
        Serial.print("> Channel ");
        Serial.print(i);
        Serial.print(":");
        Serial.println(pressure[i]);
    }   

    manifold.update();
    delay(10);
}
