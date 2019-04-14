/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0 - Henrik Ekblad
 * 
 * DESCRIPTION
 * Example sketch for a "light switch" where you can control light or something 
 * else from both HA controller and a local physical button 
 * (connected between digital pin 3 and GND).
 * This node also works as a repeader for other nodes
 * http://www.mysensors.org/build/relay
 */

// // Enable debug prints to serial monitor
#include <Arduino.h>
#include <PinChangeInt.h>

// // Enable debug prints to serial
// #define MY_DEBUG

#define MY_NODE_ID 1

#define CHILD_ID 1 // Id of the sensor child

// Enable RS485 transport layer
#define MY_RS485

// Timout when no gateway can be found
#define MY_TRANSPORT_WAIT_READY_MS 1000

#define MY_DISABLED_SERIAL

// Define this to enables DE-pin management on defined pin
#define MY_RS485_DE_PIN 3

// Set RS485 baud rate to use
#define MY_RS485_BAUD_RATE 9600

// // Enable this if RS485 is connected to a hardware serial port
#define MY_RS485_HWSERIAL Serial

#define RELAY_UP 12 // Arduino Digital I/O pin number for relay
#define RELAY_DOWN A1
#define BUTTON_UP 13 // Arduino Digital I/O pin number for button
#define BUTTON_DOWN 11

#define RELAY_ON 0
#define RELAY_OFF 1

#include <MySensors.h>

MyMessage upMessage(CHILD_ID, V_UP);
MyMessage downMessage(CHILD_ID, V_DOWN);
MyMessage stopMessage(CHILD_ID, V_STOP);
MyMessage statusMessage(CHILD_ID, V_PERCENTAGE);

unsigned long lastDebounceTimeUp = 0;
unsigned long lastDebounceTimeDown = 0;
unsigned long debounceDelay = 50;


#define SHUTTER_TIME      10000 // How long the shutter takes to fully unroll from completely rolled up
#define SHUTTER_OVERSHOOT 2000  // How long to keep the motor running at the end so it always stays "calibrated"
int32_t currentShutterProgress = SHUTTER_TIME, shutterTarget = -SHUTTER_OVERSHOOT; // defaults, so shutter goes open on startup


#define DETECTION_TOLERANCE 50
inline bool isShutterMoving() {
  return (currentShutterProgress < (shutterTarget - DETECTION_TOLERANCE)) || (currentShutterProgress > (shutterTarget + DETECTION_TOLERANCE));
}

void sendStatus() {
  // Send current state and status to gateway.
  bool isMoving = isShutterMoving();
  uint8_t percentage = map(constrain(currentShutterProgress, 0, SHUTTER_TIME), 0, SHUTTER_TIME, 0, 100);
  send(upMessage.set(isMoving && shutterTarget < currentShutterProgress));
  send(downMessage.set(isMoving && shutterTarget > currentShutterProgress));
  send(stopMessage.set(!isMoving));
  send(statusMessage.set(percentage));
}

void shutterUp() {
  shutterTarget = -SHUTTER_OVERSHOOT;
  sendStatus();
}

void shutterDown() {
  shutterTarget = SHUTTER_TIME + SHUTTER_OVERSHOOT;
  sendStatus();
}

void shutterStop() {
  shutterTarget = currentShutterProgress;
  // send status is handled in shutter loop
}

bool lastShutterMoving = false;
uint32_t lastShutterLoop = 0;

void shutterLoop() {
  uint32_t now = millis();
  uint32_t diff = now - lastShutterLoop;
  lastShutterLoop = now;

  bool shutterMoving = isShutterMoving();

  if(shutterMoving) {
    if(shutterTarget < currentShutterProgress) {
      digitalWrite(RELAY_UP, RELAY_ON);
      currentShutterProgress -= diff;
    }
    else {
      digitalWrite(RELAY_DOWN, RELAY_ON);
      currentShutterProgress += diff;
      // Serial.println(digitalRead(RELAY_DOWN));
    }
  }
  else { // detect if movement should be stopped
    digitalWrite(RELAY_UP, RELAY_OFF);
    digitalWrite(RELAY_DOWN, RELAY_OFF);
  }
 
  if(lastShutterMoving != shutterMoving) { // status changed
    if(shutterMoving) { // shutter began moving

    }
    else { // shutter stopped moving
      // reset overshoot
      if(currentShutterProgress < 0) { 
        currentShutterProgress = 0;
        shutterTarget = 0;
      }
      else if(currentShutterProgress > SHUTTER_TIME) {
        currentShutterProgress = SHUTTER_TIME;
        shutterTarget = SHUTTER_TIME;
      }

      sendStatus();
    }
  }

  lastShutterMoving = shutterMoving;
}


void IRQ_UP()
{

  if (digitalRead(BUTTON_UP) && lastDebounceTimeUp + debounceDelay < millis())
  {
    lastDebounceTimeUp = millis();
    // Serial.println("UP");
    
    if(isShutterMoving()) {
      shutterStop();
    }
    else {
      shutterUp();
    }
  }
}

void IRQ_DOWN()
{
  if (digitalRead(BUTTON_DOWN) && lastDebounceTimeDown + debounceDelay < millis())
  {
    lastDebounceTimeDown = millis();
    // Serial.println("DOWN");

    if(isShutterMoving()) {
      shutterStop();
    }
    else {
      shutterDown();
    }
  }
}

void setup()
{
  Serial.begin(115200);
  // Setup the button
  pinMode(BUTTON_UP, INPUT_PULLUP);

  pinMode(BUTTON_DOWN, INPUT_PULLUP);

  // Then set relay pins in output mode
  pinMode(RELAY_UP, OUTPUT);
  pinMode(RELAY_DOWN, OUTPUT);
  digitalWrite(RELAY_UP, RELAY_OFF);
  digitalWrite(RELAY_DOWN, RELAY_OFF);

  PCintPort::attachInterrupt(BUTTON_UP, IRQ_UP, CHANGE);
  PCintPort::attachInterrupt(BUTTON_DOWN, IRQ_DOWN, CHANGE);
}

void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("RolloController", "1.0");

  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID, S_COVER);

  sendStatus();
}

uint32_t lastDebugTime = 0;
void loop()
{
  shutterLoop();

  if(millis() - lastDebugTime >= 100) {
    lastDebugTime = millis();
    char buf[50];
    sprintf(buf, "[%10lu] m:%-2d c:%-5ld t:%-5ld\n", millis(), isShutterMoving(), currentShutterProgress, shutterTarget);
    // Serial.print(buf);
  }
}

void receive(const MyMessage &message)
{
  if(message.getCommand() == C_SET) {
    switch(message.type) {
      case V_UP:
        shutterUp();
        break;
      case V_DOWN:
        shutterDown();
        break;
      case V_STOP:
        shutterStop();
        break;
      case V_PERCENTAGE:
        uint8_t target = constrain(message.getByte(), 0, 100);
        if(target == 0)
          shutterTarget = -SHUTTER_OVERSHOOT;
        else if(target == 100) 
          shutterTarget = SHUTTER_TIME + SHUTTER_OVERSHOOT;
        else
          shutterTarget = map(target, 0, 100, 0, SHUTTER_TIME);
        break;
    }
  }
}