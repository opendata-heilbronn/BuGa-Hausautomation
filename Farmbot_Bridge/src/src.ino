// Enable debug prints to serial monitor
#define MY_DEBUG
// #define IO_DEBUG

#define MY_NODE_ID 3

// Enable RS485 transport layer
#define MY_RS485

// Set RS485 baud rate to use
#define MY_RS485_BAUD_RATE 9600

// USART1: TX=PA9 / RX=PA10
// USART2: TX=PA2 / RX=PA3
#define MY_RS485_HWSERIAL (Serial1)

// Define this to enables DE-pin management on defined pin
#define MY_RS485_DE_PIN PA8

// Timout when no gateway can be found
#define MY_TRANSPORT_WAIT_READY_MS 1000

#include <Arduino.h>
#include <MySensors.h>

// averaging settings for analog input
#define AVERAGING_FACTOR    50
#define INPUT_POLL_INTERVAL 10  // ms
#define REFERENCE_VOLTAGE   1.172   // determined by trial and error for this STM32, enable IO_DEBUG and tweak value
#define ADC_STEPS           4096

#define cidOffsetAnalog 0x20
#define cidOffsetInput 0x40
#define cidOffsetOutput 0x60

#define NODE_ROLLO  1
#define CID_ROLLO   1
#define NODE_IO     2
#define CID_IO_PUMP (cidOffsetOutput + 0)

#define PIN_COVER_CONTROL   PA6 // transition from high to low to issue "cover up" command
#define PIN_PUMP_CONTROL    PA7 // control water pump (low active)
#define PIN_EMERGENCY_STATE PB0 // read state of emergency button
#define PIN_COVER_STATE     PB5 // low when cover is upen
#define PIN_COME_HOME       PB4 // low to tell farmbot to get inside

const uint8_t inputPins[] = {PIN_COVER_CONTROL, PIN_PUMP_CONTROL, PIN_EMERGENCY_STATE};
const uint8_t inputChildIds[] = {32, 33, 34};
const uint8_t outputPins[] = {PIN_COVER_STATE, PIN_COME_HOME};
const uint8_t outputChildIds[] = {48, 49};
const uint8_t numInputPins = sizeof(inputPins) / sizeof(inputPins[0]), numOutputPins = sizeof(outputPins) / sizeof(outputPins[0]);
bool lastPinStates[numInputPins] = {1};

uint32_t lastInputPoll = 0;
float adcRef;

float getVCC() {
    return REFERENCE_VOLTAGE * ADC_STEPS / adcRef;
}

void printPinDebug() {
    Serial.print("Inputs: ");
    for (uint8_t i = 0; i < numInputPins; i++) {
        Serial.print(digitalRead(inputPins[i]));
    }
    Serial.print(" | Outputs: ");
    for (uint8_t i = 0; i < numOutputPins; i++) {
        Serial.print(digitalRead(outputPins[i]));
    }
    Serial.print(" | Ref: ");
    Serial.print(adcRef);
    Serial.print(" - ");
    Serial.print(getVCC(), 3);
    Serial.print("V");
    Serial.println();
}

// bool readDigitalChildId(uint8_t cid) {
//     return digitalRead(cidToPin(cid));
// }

// void writeDigitalChildId(uint8_t cid, bool state) {
//     digitalWrite(cidToPin(cid), state);
// }

// uint16_t readAnalogChildId(uint8_t cid) {
//     uint8_t pinId = cid - cidOffsetAnalog;
//     if (pinId < numAnalogPins) {
//         return (uint16_t)analogVals[pinId];
//     }
//     else {
//         return UINT16_MAX;
//     }
// }

void inputChangeCallback(uint8_t inputId, bool newState) {
    MyMessage msg(MY_NODE_ID, V_STATUS);
    send(msg.set(newState));

    switch(inputId) {
        case 0: {   // cover control
            if(newState == false) { // low active
                MyMessage msg(CID_ROLLO, V_UP);
                msg.setDestination(NODE_ROLLO);
                send(msg);
            }
        } break;
        case 1: {   // pump control
            MyMessage msg(CID_IO_PUMP, V_STATUS);
            msg.setDestination(NODE_IO);
            send(msg.set(!newState)); // send active low input to active high output
        } break;
        case 2: {   // emergency button state
            // nothing to do, state already sent to gateway above
        } break;
    }
}

void runInputPoll() {
    if(millis() - lastInputPoll >= INPUT_POLL_INTERVAL) {
        lastInputPoll = millis();

        for (uint8_t i = 0; i < numInputPins; i++) {
            bool curState = digitalRead(inputPins[i]);
            if (lastPinStates[i] != curState) {
                lastPinStates[i] = curState;
                inputChangeCallback(i, curState);
            }
        }
        adcRef -= adcRef / AVERAGING_FACTOR;
        adcRef += adc_read(ADC1, 17)/ AVERAGING_FACTOR;

        #ifdef IO_DEBUG
            printPinDebug();
        #endif
    }
}

void initVCCChannel() {
    adc_reg_map *regs = ADC1->regs;
    regs->CR2 |= ADC_CR2_TSVREFE;    // enable VREFINT and temp sensor
    regs->SMPR1 =  ADC_SMPR1_SMP17;  // sample rate for VREFINT ADC channel
}

void setup() {
    //disable superfluent debug pins, make PB3, PB4 and PA15 usable
    afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY); 

    Serial.begin(115200);
    initVCCChannel();

    for (uint8_t i = 0; i < numInputPins; i++) {
        pinMode(inputPins[i], INPUT_PULLUP);
    }
    for (uint8_t i = 0; i < numOutputPins; i++) {
        pinMode(outputPins[i], OUTPUT);
        digitalWrite(outputPins[i], 1);
    }
}


void presentation() {
    sendSketchInfo("Farmbot_Bridge", "1.0");

    for (uint8_t i = 0; i < numInputPins; i++) {
        present(inputChildIds[i], S_BINARY);
    }
    for (uint8_t i = 0; i < numOutputPins; i++) {
        present(outputChildIds[i], S_BINARY);
    }
}

// uint32_t lastTest = 0;
// uint8_t test = 0;

void loop() {
    runInputPoll();

    // activate each output in turn
    // if(millis() - lastTest > 1000) {
    //     lastTest = millis();
    //     writeDigitalChildId(cidOffsetOutput + (test + 1) % 10, 1);
    //     writeDigitalChildId(cidOffsetOutput + (test) % 10, 0);
    //     test++;
    //     if(test == 10) {
    //         test = 0;
    //     }
    // }
}

void receive(const MyMessage &message) {
    uint8_t cid = message.sensor;
    uint8_t command = message.getCommand();

    if (message.getCommand() == C_SET) {
        switch (message.type) {
            
        }
    }
    else if (message.getCommand() == C_REQ) {
        switch (message.type) {
            
        }
    }
}