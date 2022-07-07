// -----
// SimplePollRotator.ino - Example for the RotaryEncoder library.
// This class is implemented for use with the Arduino environment.
//
// Copyright (c) by Matthias Hertel, http://www.mathertel.de
// This work is licensed under a BSD 3-Clause License. See http://www.mathertel.de/License.aspx
// More information on: http://www.mathertel.de/Arduino
// -----
// 18.01.2014 created by Matthias Hertel
// 04.02.2021 conditions and settings added for ESP8266
// 17.05.2021 conditions and settings changed for ESP32 (ESP8266 deleted) by Ward Vandermeulen
// -----

// This example checks the state of the rotary encoder using interrupts and in the loop() function.
// The current position and direction is printed on output when changed.

// Hardware setup:
// Attach a rotary encoder with output pins to
// * 2 and 3 on Arduino UNO. (supported by attachInterrupt)
// * D5 and D6 on ESP8266 board (e.g. NodeMCU).>> deleted
// * 4 and 5 on ESP32 board.
// * Settings Microstep Driver > Microstep = 8 / Pulse/rev = 1600 / I = 2A
// Swap the pins when direction is detected wrong.
// The common contact should be attached to ground.
//
// Hints for using attachinterrupt see https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/

#include <Arduino.h>
#include <RotaryEncoder.h>
#include <AccelStepper.h>
#include "DFRobot_VEML7700.h"
#include <Wire.h>

// Top Level:
int DEBUG = 1; // Set to 1 to enable serial monitor debugging info

// Variabelen luxsensor

DFRobot_VEML7700 als;
bool autoOpen;
bool autoSluit;
bool donker = false;
float lux;
const long vertragingLuxmeting = 300000;
unsigned long vorigeMillis = 0;

// Variabelen stepper
const int maxSpeed = 1000;
const int acceleration = 250;

bool open = 0;
bool sluit = 0;
int statusStepper = 0;
int statusStepperVorig = 0;
int hoogstePositie = 7700;
int slotpositie = 250;
long reststappen;

#define stepPin 25
#define dirPin 26
#define enablePin 27
#define openPin 32
#define sluitPin 33
#define magneetslotPin 14

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

void setup()
{
  Serial.begin(115200);
  Serial.println("");
  Serial.println("------------------------");
  Serial.println("Automatische kippendeur");
  Serial.println("------------------------");
  Serial.println("");

  setCpuFrequencyMhz(20);

  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(acceleration);
  stepper.setEnablePin(enablePin);
  stepper.setPinsInverted(true, true, false); // dirPin, stepPin, enablePin
  // stepper.disableOutputs();

  pinMode(enablePin, OUTPUT);
  pinMode(magneetslotPin, OUTPUT);
  pinMode(openPin, INPUT_PULLUP);
  pinMode(sluitPin, INPUT_PULLUP);
  digitalWrite(magneetslotPin, HIGH);

  als.begin(); // Init
  // if DEBUG is on, print serial monitor
  if (DEBUG)
  {
    Serial.println("Debugging is ON");
  }
}

void SM_stap()
{
  open = digitalRead(openPin);
  sluit = digitalRead(sluitPin);
  statusStepperVorig = statusStepper;

  switch (statusStepper)
  {
  case 0: // Deur dicht
    stepper.disableOutputs();
    if (open == LOW || autoOpen == true)
    {
      autoOpen = false; // om automatisch openen te beletten na handmatig sluiten
      statusStepper = 1;
    }
    digitalWrite(magneetslotPin, HIGH);
    break;

  case 1: // Naar hoogste positie
    stepper.enableOutputs();
    stepper.moveTo(hoogstePositie);
    // stepper.run;
    if (stepper.distanceToGo() == 0)
    {
      statusStepper = 2;
    }
    if (sluit == LOW)
    {
      statusStepper = 5;
    }
    digitalWrite(magneetslotPin, LOW);
    break;

  case 2: // Naar slotpositie
    stepper.enableOutputs();
    stepper.moveTo(hoogstePositie - slotpositie);
    if (stepper.distanceToGo() == 0)
    {
      statusStepper = 3;
    }
    digitalWrite(magneetslotPin, HIGH);
    break;

  case 3: // Deur open
    stepper.disableOutputs();
    if (sluit == LOW || autoSluit == true)
    {
      autoSluit = false; // om automatisch sluiten te beletten na handmatig openen
      statusStepper = 4;
    }
    digitalWrite(magneetslotPin, HIGH);
    break;

  case 4: // Terug naar hoogste positie
    stepper.enableOutputs();
    stepper.moveTo(hoogstePositie);
    if (stepper.distanceToGo() == 0)
    {
      statusStepper = 5;
    }
    if (open == LOW)
    {
      statusStepper = 1;
    }
    digitalWrite(magneetslotPin, HIGH);
    break;

  case 5: // Naar laagste positie
    stepper.enableOutputs();
    stepper.moveTo(0);
    if (stepper.distanceToGo() == 0)
    {
      statusStepper = 0;
    }
    if (open == LOW)
    {
      statusStepper = 1;
    }
    digitalWrite(magneetslotPin, LOW);
    break;
  }
}

void loop()
{

  SM_stap(); // voer statemachine stepper uit
  stepper.run();

  if (DEBUG) // If DEBUG enabled >> print boodschappen
  {

    if (statusStepperVorig != statusStepper) // If statusStepper wijzigt, print status
    {
      Serial.print("targetPosition :");
      Serial.println(stepper.targetPosition());
      Serial.print("Stepper State: ");
      Serial.println(statusStepper);
    }
  }

  unsigned long huidigeMillis = millis();
  if (huidigeMillis - vorigeMillis >= vertragingLuxmeting)
  {
    vorigeMillis = huidigeMillis;

    als.getALSLux(lux); // Get the measured ambient light value
    if (DEBUG)
    {
      Serial.print("Lux:");
      Serial.print(lux);
      Serial.println(" lx");
    }

    if (lux < 20.0 && donker == false)
    {
      autoSluit = true;
      donker = true;
      Serial.println("Puls sluit");
    }

    else
    {
      autoSluit = false;
    }

    if (lux > 150.0 && donker == true)
    {
      autoOpen = true;
      donker = false;
      Serial.println("Puls open");
    }

    else
    {
      autoOpen = false;
    }
  }
}