
/*
TODO
- Mode so that each strip on both sides of the cavity get assinged the same data
*/

/*
This example will receive multiple universes via Art-Net and control a strip of
WS2812 LEDs via the FastLED library: https://github.com/FastLED/FastLED
This example may be copied under the terms of the MIT license, see the LICENSE file for details
*/

#if defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif
#include <ArtnetWifi.h>
#include <WiFiUdp.h>
#define FASTLED_ALLOW_INTERRUPTS 0
#include <FastLED.h>

#include "config.h"
#include "ota.h"


// 4 18 3
#define GAP_WIDTH 8
#define GAP_POS 44
#define WIDTH (50 + GAP_WIDTH)
#define HEIGHT 10
int16_t ledMapping[HEIGHT][WIDTH];

// LED settings
#define ANIMATION_SPEED 20 // [ms] 460 LEDs take way too long to update anyways
const byte dataPin = D4;
const int numLeds = 460;                         // CHANGE FOR YOUR SETUP
const int numberOfChannels = WIDTH * HEIGHT * 3; // Total number of channels you want to receive (1 led = 3 channels)
CRGB leds[numLeds];

// Art-Net settings
#define ARTNET_TIMEOUT 5000 // start default animation after 5s of Artnet missing
ArtnetWifi artnet;
const int startUniverse = 1; // CHANGE FOR YOUR SETUP most software    this is 1, some software send out artnet first universe as 0.

// Check if we got all universes
const int maxUniverses = numberOfChannels / 512 + ((numberOfChannels % 512) ? 1 : 0);
bool universesReceived[maxUniverses];
bool sendFrame = 1;
int previousDataLength = 0;

// magic, do not touch
void generateLedMapping() {
    // generate mapping for small left counter segment
    for (uint8_t x = 0; x < 8; x++) {
        int16_t xOffset = x * HEIGHT / 2;
        for (uint8_t y = 0; y < HEIGHT; y++) {
            int16_t ledId = -1;
            if (y >= HEIGHT / 2) {
                ledId = xOffset + ((x % 2 == 1) ? y - (HEIGHT / 2) : (HEIGHT - y - 1));
            }
            ledMapping[y][x] = ledId;
        }
    }

    // generate for rest of counter
    for (uint8_t x = 8; x < WIDTH; x++) {
        int16_t xOffset = x * HEIGHT - 40;
        if (x >= GAP_POS + GAP_WIDTH) {
            xOffset -= GAP_WIDTH * HEIGHT;
        }
        // Serial.printf("%d %d \n", x, xOffset);
        for (uint8_t y = 0; y < HEIGHT; y++) {
            int16_t ledId = -1;
            if (x < GAP_POS || x >= GAP_POS + GAP_WIDTH) {
                ledId = xOffset + ((x % 2 == 1) ? y : (HEIGHT - y - 1));
            }
            ledMapping[y][x] = ledId;
        }
    }

    // for (uint8_t y = 0; y < HEIGHT; y++) {
    //     for (uint8_t x = 0; x < WIDTH; x++) {
    //         Serial.printf("%3d ", ledMapping[y][x]);
    //     }
    //     Serial.println();
    // }
}

// connect to wifi – returns true if successful or false if not
boolean ConnectWifi(void) {
    boolean state = true;
    int i = 0;

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.println("");
    Serial.println("Connecting to WiFi");

    // Wait for connection
    Serial.print("Connecting");
    while (WiFi.status() != WL_CONNECTED) {
        delay(100);
        Serial.print(".");
        if (i > 100) {
            state = false;
            break;
        }
        i++;
    }
    if (state) {
        Serial.println("");
        Serial.print("Connected to ");
        Serial.println(WIFI_SSID);
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("");
        Serial.println("Connection failed.");
    }

    return state;
}

void initTest() {
    for (int i = 0; i < numLeds; i++) {
        leds[i] = CRGB(127, 0, 0);
    }
    FastLED.show();
    delay(200);
    for (int i = 0; i < numLeds; i++) {
        leds[i] = CRGB(0, 127, 0);
    }
    FastLED.show();
    delay(200);
    for (int i = 0; i < numLeds; i++) {
        leds[i] = CRGB(0, 0, 127);
    }
    FastLED.show();
    delay(200);
    for (int i = 0; i < numLeds; i++) {
        leds[i] = CRGB(0, 0, 0);
    }
    FastLED.show();
}

int16_t xyToLedNum(uint8_t x, uint8_t y) {
    if (x < WIDTH && y < HEIGHT) {
        return ledMapping[y][x];
    } else {
        return -1;
    }
}

void setLed(uint8_t x, uint8_t y, CRGB color) {
    int16_t ledId = xyToLedNum(x, y);
    if (ledId > -1) {
        leds[ledId] = color;
    }
}

byte lastSequenceNum = 0;
uint32_t lastArtnetReceived = 0;

void onDmxFrame(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t *data) {
    // Serial.println(data[0]);

    sendFrame = 1;
    // set brightness of the whole strip

    // Store which universe has got in
    if ((universe - startUniverse) < maxUniverses) {
        universesReceived[universe - startUniverse] = 1;
        // Serial.println("inside store universe");
    }

    for (int i = 0; i < maxUniverses; i++) {
        // Serial.println(maxUniverses);
        if (universesReceived[i] == 0) {
            Serial.println("Broke");
            sendFrame = 0;
            break;
        }
    }

    int diff = sequence - lastSequenceNum;
    if (diff < 0 && diff > -200) {
        sendFrame = 0;
    } else {
        lastSequenceNum = sequence;
    }

    // read universe and put into the right part of the display buffer
    for (int i = 0; i < length / 3; i++) {
        // Serial.println(i);
        int led = i + (universe - startUniverse) * (previousDataLength / 3);
        if (led < numLeds) {
            // leds[led] = CRGB(data[i * 3], data[i * 3 + 1], data[i * 3 + 2]);
            setLed(led % WIDTH, led / WIDTH, CRGB(data[i * 3], data[i * 3 + 1], data[i * 3 + 2]));
        }
    }
    previousDataLength = length;

    if (sendFrame) {
        FastLED.show();
        // Serial.println(d);
        // Reset universeReceived to 0
        memset(universesReceived, 0, maxUniverses);
        lastArtnetReceived = millis();
    }
}

void setup() {
    Serial.begin(115200);
    Serial.print("ArtnetWifi init...");
    initOTA();
    ConnectWifi();
    generateLedMapping();
    artnet.begin();
    FastLED.addLeds<WS2811, dataPin, RGB>(leds, numLeds);
    initTest();

    // this will be called for each packet received
    artnet.setArtDmxCallback(onDmxFrame);
    Serial.println(" done.");
}

uint32_t lastAnimationUpdate = 0;
uint16_t animationStep = 0;
uint8_t animId = 0;

void loop() {
    // we call the read function inside the loop
    artnet.read();
    loopOTA();

    if (millis() - lastArtnetReceived >= ARTNET_TIMEOUT) { // run default animation
        if (millis() - lastAnimationUpdate >= ANIMATION_SPEED) {
            uint32_t waitTime = millis() - lastAnimationUpdate;
            lastAnimationUpdate = millis();
            switch (animId) {
            case 0: {
                uint8_t hueOffset = (animationStep >> 0) % 0xFF; // lower 8 bits of anim counter
                const uint8_t locationFactor = (256 / 32);       // complete rainbow over 32 pixels
                for (uint8_t y = 0; y < HEIGHT; y++) {
                    for (uint8_t x = 0; x < WIDTH; x++) {
                        uint8_t hue = (hueOffset + x * locationFactor + y * locationFactor) % 256;
                        setLed(x, y, CHSV(hue, 255, 255));
                        // Serial.printf("%3d ", hue);
                    }
                    // Serial.println();
                }
                // Serial.println("\n\n");
            } break;
            }
            FastLED.show();
            animationStep += waitTime / ANIMATION_SPEED;
            // Serial.println(waitTime);
        }
    }
}
