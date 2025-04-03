#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "Cancomm/CANcomm.h" 
#include "Manifold/manifold.h"

#include "parameters.h"

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> canBus;
CAN_message_t outgoingMsg;
CANCommunication canComm(canBus, ID_FOLLOWER);

// Manifold manifold;
const int analogPin0 = A0;
const int analogPin1 = A1;
const int analogPin2 = A3;

double convertADCtoPressure(int adcValue) {
    double percent = adcValue / 4095.0 * 8.4 / 5.1;
    double pressure = (percent - 0.1) * (103.42*2)/0.8 -  103.42 + 3.17; 
    return pressure;
}

// Callback for processing broadcast messages (ID 0x100)
void followerCallback(const CAN_message_t &incomingMsg) {
  Serial.print("--------- follower ");
  Serial.print(ID_FOLLOWER);
  Serial.println(" -------------");
  Serial.print("Received message with ID 0x");
  Serial.print(incomingMsg.id, HEX);
  Serial.print(", data: [");

  // TODO: rewrite this part into separate function (decypher)
  double stored_data[3] = {0,0,0};
  if (incomingMsg.len == 6) {
    uint16_t scaledSetpoints[3];

    // Unpack the scaled setpoints from the message buffer
    memcpy(&scaledSetpoints[0], &incomingMsg.buf[0], sizeof(uint16_t));
    memcpy(&scaledSetpoints[1], &incomingMsg.buf[2], sizeof(uint16_t));
    memcpy(&scaledSetpoints[2], &incomingMsg.buf[4], sizeof(uint16_t));

    // Convert scaled setpoints back to float and update PID controllers
    for (int i = 0; i < 3; ++i) {
        double setpoint = static_cast<float>(scaledSetpoints[i]) / 100.0f;
        // setPressureSetpoint(i, setpoint);
        stored_data[i] = setpoint;
        
        Serial.print(stored_data[i]);
        Serial.print(" ");
      }
      Serial.println("]");
    }
  else {
    Serial.println("Invalid setpoint message length.");
  }

  uint32_t responseID = D_BASE + ID_FOLLOWER; // module ID + message type ID
  double pressure[3];
  int adcValue0 = analogRead(analogPin0);
  int adcValue1 = analogRead(analogPin1);
  int adcValue2 = analogRead(analogPin2);

  pressure[0] = convertADCtoPressure(adcValue0);
  pressure[1] = convertADCtoPressure(adcValue1);
  pressure[2] = convertADCtoPressure(adcValue2);

  // for (int i = 0; i < 3; i++) {
  // // pressure[i] = manifold.getPressure(i); 
  // Serial.println(pressure[i]);
  // }//physically now using channel 3 as an example
  int ret = canComm.sendMessage(responseID, pressure, 3);

  if(ret == 1) {
    Serial.print("Follower ");
    Serial.print(ID_FOLLOWER);
    Serial.print(": Response with ");
    Serial.print(  "pressure [");
    for (int i = 0; i < 3; i++) {
      Serial.print(pressure[i]);
      Serial.print(" "); 
    }    
    Serial.print("]");
    Serial.print(", ID 0x");
    Serial.println(outgoingMsg.id, HEX);
  } 
  else {
    Serial.print("Follower ");
    Serial.print(ID_FOLLOWER);
    Serial.println(": Error sending response.");
  }  
  delay(10);
}

void setup() {
  Serial.begin(BAUD_SERIAL);
  analogReadResolution(ANALOG_READ_RESOLUTION); //12 bits [0-4095]
  analogWriteResolution(ANALOG_WRITE_RESOLUTION); 
  canBus.begin();
  canComm.setup();
  canComm.onReceive(followerCallback);

  // manifold.setup();
  // manifold.purgeOutlets(3000);

  Serial.print("Follower ");
  Serial.print(ID_FOLLOWER);
  Serial.println(": CAN bus initialized (event-driven).");
}

void loop() {

  // In event-driven mode, the callback processes the incoming broadcast.
  // The main loop can be minimal.
  delay(10);
}
