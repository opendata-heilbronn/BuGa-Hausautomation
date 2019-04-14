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

#include <Ethernet_STM.h>
#include <MySensors.h>

const uint8_t gpioPins[] = {};
const uint8_t analogPins[] = {};

void setup()
{
    // Setup locally attached sensors
}

void presentation()
{
    // Present locally attached sensors here
}

void loop()
{
    // Send locally attached sensors data here
}