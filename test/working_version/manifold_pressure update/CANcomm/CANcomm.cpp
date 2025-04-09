#include "Cancomm/CANcomm.h"


CANCommunication::CANCommunication(FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16>& canInstance, uint32_t idBase)
    : baseID(idBase), canBus(canInstance) {  // baseID first, then canBus
}

// This initialization should be run in void setup()
void CANCommunication::setup() {
    // canBus.begin();
    canBus.setBaudRate(BAUD_CANBUS);
    canBus.setMaxMB(2);  // Use MB0 for RX and MB1 for TX
    canBus.setMB(MB0, RX, STD);
    canBus.setMB(MB1, TX, STD);
    canBus.enableMBInterrupts();
}

// Set callback for receiving messages on MB0
void CANCommunication::onReceive(void (*callback)(const CAN_message_t&)) {
    canBus.onReceive(MB0, callback);  // TODO: make MB a parameter if needed
}

//currently send message via MB1. be careful about this.
// this function can send 1~4 sets of data in float format, loss of accuracy 1% due to data conversion is normal 
// return 1 if message send out successfully; -1 if failed; 0 if payload exceed limit before send out 
int CANCommunication::sendMessage(uint32_t id, const double* data, uint8_t numValues) {
    const uint8_t maxValues = 4; // Maximum payload of canbus: 8 bytes. for uint16_t it means 4 sets of data
    if (numValues > maxValues) {
        // Handle error: Too many values to send in one message
        return 0;
    }

    CAN_message_t outgoingMsg;
    outgoingMsg.id = id;
    outgoingMsg.len = numValues * sizeof(uint16_t); // Each uint16_t is 2 bytes
    
    Serial.print("CANcomm: Broadcasting data: [ ");

    for (uint8_t i = 0; i < numValues; ++i) {
        uint16_t scaledValue = static_cast<uint16_t>(data[i] * 100); // Scale float to preserve three decimal places
        memcpy(&outgoingMsg.buf[i * 2], &scaledValue, sizeof(uint16_t));
    Serial.print((double)scaledValue/100);
    Serial.print(" ");
    }
    Serial.println("]");
    return canBus.write(MB1, outgoingMsg);
}

bool CANCommunication::readMessage(const CAN_message_t &incomingMsg, double* decodedValues, uint8_t &numValues) {
    const uint8_t maxPayload = 8;
    const uint8_t valueSize = sizeof(uint16_t);
    if (incomingMsg.len > maxPayload || incomingMsg.len % valueSize != 0) {
        // Invalid message length
        return false;
    }

    numValues = incomingMsg.len / valueSize;

    for (uint8_t i = 0; i < numValues; ++i) {
        uint16_t raw;
        memcpy(&raw, &incomingMsg.buf[i * valueSize], valueSize);
        decodedValues[i] = static_cast<double>(raw) / 100.0;
    }

    Serial.print("CANcomm: Decoding received data: [ ");
    for (uint8_t i = 0; i < numValues; ++i) {
        Serial.print(decodedValues[i]);
        Serial.print(" ");
    }
    Serial.println("]");

    return true;
}
