#ifndef CANCOMM_H
#define CANCOMM_H

#include <FlexCAN_T4.h>
#include "parameters.h"

class CANCommunication {
private:
    CAN_message_t outgoingMsg; // incomingMsg is the trigger for event-driven method, so no need to store it seperately
    uint32_t baseID;

public:
    FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16>& canBus;

    CANCommunication(FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16>& canInstance, uint32_t idBase);

    void setup();
    void onReceive(void (*callback)(const CAN_message_t&));
    int sendMessage(uint32_t id, const double* data, uint8_t numValues) ;
    // double readMessage();
};


#endif