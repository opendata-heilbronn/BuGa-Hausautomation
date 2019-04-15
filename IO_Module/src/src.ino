// Enable debug prints to serial monitor
#define MY_DEBUG
#define IO_DEBUG

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

// Timout when no gateway can be found
#define MY_TRANSPORT_WAIT_READY_MS 1000

#include <Arduino.h>
#include <MySensors.h>

// averaging settings for analog input
#define AVERAGING_FACTOR    50
#define AVERAGING_INTERVAL  10  // ms
#define REFERENCE_VOLTAGE   1.172   // determined by trial and error for this STM32, enable IO_DEBUG and tweak value
#define ADC_STEPS           4096

#define cidOffsetAnalog 0x10
#define cidOffsetInput 0x20
#define cidOffsetOutput 0x30

#define PINS_PER_TYPE 16
const uint8_t analogPins[] = {PA2, PA3, PA4, PA5};                                      // child id range 0x10-0x1F
const uint8_t inputPins[] = {PA6, PA7, PB0, PB1};                                       //                0x20-0x2F
const uint8_t outputPins[] = {PB5, PB4, PB3, PA15, PB7, PB6, PB15, PB14, PB13, PB12};   //                0x30-0x3F
const uint8_t numInputPins = sizeof(inputPins) / sizeof(inputPins[0]), numOutputPins = sizeof(outputPins) / sizeof(outputPins[0]),
              numAnalogPins = sizeof(analogPins) / sizeof(analogPins[0]);

uint32_t lastMeasure = 0;
float analogVals[numAnalogPins], adcRef;

// convert child id to pin on µC, returns 255 if not found
uint8_t cidToPin(uint8_t cid) {
    if(cid >= cidOffsetAnalog && cid < cidOffsetAnalog + PINS_PER_TYPE) {
        return analogPins[cid - cidOffsetAnalog];
    }
    else if(cid >= cidOffsetInput && cid < cidOffsetInput + PINS_PER_TYPE) {
        return inputPins[cid - cidOffsetInput];
    }
    else if(cid >= cidOffsetOutput && cid < cidOffsetOutput + PINS_PER_TYPE) {
        return outputPins[cid - cidOffsetOutput];
    }
    else {
        return 255;
    }
}

// uint8_t pinToId(uint8_t search, const uint8_t* arr, uint8_t length) {
//     for (uint8_t i = 0; i < length; i++) {
//         if(arr[i] == search) {
//             return i;
//         }
//     }
//     return 255;
// }

float getVCC() {
    return REFERENCE_VOLTAGE * ADC_STEPS / adcRef;
}

void printPinDebug() {
    Serial.print("Inputs: ");
    for (uint8_t i = cidOffsetInput; i < numInputPins + cidOffsetInput; i++) {
        Serial.print(readDigitalChildId(i));
    }
    Serial.print(" | Outputs: ");
    for (uint8_t c = 0, i = cidOffsetOutput; i < numOutputPins + cidOffsetOutput; c++, i++) {
        Serial.print(readDigitalChildId(i));
        if (c % 4 == 3)
            Serial.print(" ");
    }
    Serial.print(" | Analog: ");
    char buf[10];
    for (uint8_t i = cidOffsetAnalog; i < numAnalogPins + cidOffsetAnalog; i++) {
        snprintf(buf, 10, "%4d ", readAnalogChildId(i));
        Serial.print(buf);
    }
    Serial.print(" | Ref: ");
    Serial.print(adcRef);
    Serial.print(" - ");
    Serial.print(getVCC(), 3);
    Serial.print("V");
    Serial.println();
}

bool readDigitalChildId(uint8_t cid) {
    return digitalRead(cidToPin(cid));
}

void writeDigitalChildId(uint8_t cid, bool state) {
    digitalWrite(cidToPin(cid), state);
}

uint16_t readAnalogChildId(uint8_t cid) {
    uint8_t pinId = cid - cidOffsetAnalog;
    if (pinId < numAnalogPins) {
        return (uint16_t)analogVals[pinId];
    }
    else {
        return UINT16_MAX;
    }
}

void runAveraging() {
    if(millis() - lastMeasure >= AVERAGING_INTERVAL) {
        lastMeasure = millis();

        for (uint8_t i = 0; i < numAnalogPins; i++) {
            // do EWMA averaging
            analogVals[i] -= analogVals[i] / AVERAGING_FACTOR;
            float val = analogRead(analogPins[i]);
            analogVals[i] += val / AVERAGING_FACTOR;
        }
        adcRef -= adcRef / AVERAGING_FACTOR;
        adcRef += adc_read(ADC1, 17)/ AVERAGING_FACTOR;

        #ifdef IO_DEBUG
            printPinDebug();
        #endif
    }
}

void initAveraging() {
    for (uint8_t i = 0; i < numAnalogPins; i++) {
        analogVals[i] = analogRead(analogPins[i]);
    }
    adcRef = adc_read(ADC1, 17);
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
    initAveraging();
    initVCCChannel();

    for (uint8_t i = 0; i < numInputPins; i++) {
        pinMode(inputPins[i], INPUT_PULLUP);
    }
    for (uint8_t i = 0; i < numOutputPins; i++) {
        pinMode(outputPins[i], OUTPUT);
        digitalWrite(outputPins[i], 0);
    }
    for (uint8_t i = 0; i < numAnalogPins; i++) {
        pinMode(analogPins[i], INPUT_ANALOG);
    }
}

void presentation() {
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

// uint32_t lastTest = 0;
// uint8_t test = 0;

void loop() {
    runAveraging();

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
    if (message.getCommand() == C_SET) {
        switch (message.type) {
            case V_STATUS:
                bool state = message.getBool();
                writeDigitalChildId(message.sensor, state);
            break;
        }
    }
    else if (message.getCommand() == C_REQ) {
        switch (message.type) {
            case V_STATUS:      // digital value
                if (message.sensor >= cidOffsetInput && message.sensor < cidOffsetOutput + PINS_PER_TYPE) { // reading of inputs and outputs is supported
                    bool value = readDigitalChildId(message.sensor);
                    MyMessage msg(message.sensor, message.type);
                    send(msg.set(value));
                }
                break;
            case V_VAR1:        // analog value
                if (message.sensor >= cidOffsetAnalog && message.sensor < cidOffsetAnalog + PINS_PER_TYPE) {
                    uint16_t value = readAnalogChildId(message.sensor);
                    MyMessage msg(message.sensor, message.type);
                    send(msg.set(value));
                }
                break;
            case V_VOLTAGE:        // analog value in volts
                if (message.sensor >= cidOffsetAnalog && message.sensor < cidOffsetAnalog + PINS_PER_TYPE) {
                    uint16_t value = readAnalogChildId(message.sensor);
                    float voltage = getVCC() * (float)value / ADC_STEPS;
                    MyMessage msg(message.sensor, message.type);
                    send(msg.set(voltage, 3));
                }
                break;
        }
    }
}