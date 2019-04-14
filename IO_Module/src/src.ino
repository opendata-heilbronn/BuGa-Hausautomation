// Enable debug prints to serial monitor
#define MY_DEBUG

#define MY_NODE_ID 2

// Enable RS485 transport layer
#define MY_RS485

// Set RS485 baud rate to use
#define MY_RS485_BAUD_RATE 9600

// USART1: TX=PA9 / RX=PA10
// USART2: TX=PA2 / RX=PA3
#define MY_RS485_HWSERIAL (Serial1)

// Define this to enables DE-pin management on defined pin
#define MY_RS485_DE_PIN PA8

#include <Arduino.h>
#include <MySensors.h>

#define cidOffsetAnalog 0x10
#define cidOffsetInput 0x20
#define cidOffsetOutput 0x30

const uint8_t analogPins[] = {}; // child id range 0x10-0x1F
const uint8_t inputPins[] = {};  //                0x20-0x2F
const uint8_t outputPins[] = {}; //                0x30-0x3F
const uint8_t numInputPins = sizeof(inputPins) / sizeof(inputPins[0]), numOutputPins = sizeof(outputPins) / sizeof(outputPins[0]),
              numAnalogPins = sizeof(analogPins) / sizeof(analogPins[0]);

// convert child id to pin on µC, returns 255 if not found
uint8_t cidToPin(uint8_t cid) {
    if(cid >= cidOffsetAnalog && cid < cidOffsetAnalog + 16) {
        return analogPins[cidOffsetAnalog - cid];
    }
    else if(cid >= cidOffsetInput && cid < cidOffsetInput + 16) {
        return inputPins[cidOffsetInput - cid];
    }
    else if(cid >= cidOffsetOutput && cid < cidOffsetOutput + 16) {
        return outputPins[cidOffsetOutput - cid];
    }
    else {
        return 255;
    }
}

void setup() {
    for (uint8_t i = 0; i < numInputPins; i++) {
        pinMode(inputPins[i], INPUT_PULLUP);
    }
    for (uint8_t i = 0; i < numOutputPins; i++) {
        pinMode(outputPins[i], OUTPUT);
    }
    for (uint8_t i = 0; i < numAnalogPins; i++) {
        pinMode(analogPins[i], INPUT_ANALOG);
    }
}

void presentation() {
    // Present locally attached sensors here
    sendSketchInfo("IO_Module", "1.0");

    for (uint8_t i = 0; i < numAnalogPins; i++) {
        present(cidOffsetAnalog + i, S_CUSTOM);
    }
    for (uint8_t i = 0; i < numInputPins; i++) {
        present(cidOffsetInput + i, S_BINARY);
    }
    for (uint8_t i = 0; i < numOutputPins; i++) {
        present(cidOffsetOutput + i, S_BINARY);
    }
}

void loop() {
    // Send locally attached sensors data here
}

void receive(const MyMessage &message) {
    if (message.getCommand() == C_SET) {
        switch (message.type) {}
    }
    else if (message.getCommand() == C_REQ) {
        switch (message.type) {
            case V_STATUS:      // digital value
                break;
            case V_VAR1:        // analog value
                break;
            case V_PERCENTAGE:  // scaled analog input
                break;
        }
    }
}