#include <Arduino.h>
#include <FlexCAN_T4.h>

// Create CAN bus object on CAN3 (adjust if needed)
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can;

CAN_message_t outgoingMsg;
CAN_message_t incomingMsg;

void setup() {
  Serial.begin(115200);
  while (!Serial) { } // Wait for serial monitor to connect

  Can.begin();
  Can.setBaudRate(250000);
  Can.setMaxMB(2);  // We use MB0 and MB1

  // Setup MB0 for receiving feedback messages with CAN ID 0x300
  Can.setMB(MB0, RX, STD);
//   Can.setMBFilter(MB0, 0x300, 0x400);
  Can.setMBFilter(MB0, ACCEPT_ALL);

  // Setup MB1 for transmitting leader messages
  Can.setMB(MB1, TX, STD);

  Can.enableMBInterrupts();
  
  Serial.println("Leader: CAN bus initialized.");
}

void loop() {
    Serial.println("----------------------");
    Can.events();
    // Send leader message with ID 0x200
    outgoingMsg.id = 0x200;
    outgoingMsg.len = 6;
    outgoingMsg.flags.extended = 0;
    for (int i = 0; i < 6; i++) {
    outgoingMsg.buf[i] = 10 + i * 10;  // Data: 10, 20, 30, 40, 50, 60
    }

    Serial.print("Leader: Sending message with ID 0x");
    Serial.println(outgoingMsg.id, HEX);
    int ret = Can.write(MB1, outgoingMsg);
    if(ret == 1) {
    Serial.println("Leader: Message sent successfully.");
    Serial.print("Data: ");
    for (int i = 0; i < incomingMsg.len; i++) {
        Serial.print(outgoingMsg.buf[i], DEC);
        Serial.print(" ");
    }
    Serial.println();
    } else {
    Serial.print("Leader: Error sending message. Error code: ");
    Serial.println(ret);
    }
  
    // Give some time for feedback to arrive
    delay(10);
  
    // Process events so that any incoming messages are queued

    // Poll for a feedback message (with ID 0x300)
    int r = Can.read(incomingMsg);
    if(r == 1) {
    Serial.print("Leader: Received feedback with ID 0x");
    Serial.println(incomingMsg.id, HEX);
    Serial.print("Data: ");
    for (int i = 0; i < incomingMsg.len; i++) {
        Serial.print(incomingMsg.buf[i], DEC);
        Serial.print(" ");
    }
    Serial.println();
    } else {
    // If no message is available, read() returns 0.
    Serial.println("Leader: No feedback message available.");
    }

    delay(490);  // Repeat every second
    // Poll for a feedback message (with ID 0x300)

    r = Can.read(incomingMsg);
    if(r == 1) {
    Serial.print("Leader: Received feedback with ID 0x");
    Serial.println(incomingMsg.id, HEX);
    Serial.print("Data: ");
    for (int i = 0; i < incomingMsg.len; i++) {
        Serial.print(incomingMsg.buf[i], DEC);
        Serial.print(" ");
    }
    Serial.println();
    } else {
    // If no message is available, read() returns 0.
    Serial.println("Leader: No feedback message available.");
    }
    
    delay(490);  // Repeat every second
}
