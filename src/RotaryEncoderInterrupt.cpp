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
// * 4 and 15 on ESP32 board.
// Swap the pins when direction is detected wrong.
// The common contact should be attached to ground.
//
// Hints for using attachinterrupt see https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/

#include <Arduino.h>
#include <RotaryEncoder.h>

// Top Level:
int DEBUG = 1; //Set to 1 to enable serial monitor debugging info

// Variabelen rotary
int stappenteller = 0;
unsigned long currentTime = 0;
unsigned long click = 0;
unsigned long vertraging = 100;

int nulstap = 0;
int maxstap = 500;

//Variabelen keyRotary:
int statusKeyRotary = 0;
int statusKeyRotaryVorig = 0;
int valKeyRotary = 0;
unsigned long t0;
unsigned long t1;
unsigned long bounce_delay_s1 = 20;
unsigned long hold_delay_s1 = 1000;

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO_EVERY)
// Arduino UNO with input signals on pin 2 and 3 
#define s1Rotary 2
#define s2Rotary 3

#elif defined(ESP32)
// ESP32 with input signals on pin 4, 5 en 16
#define s1Rotary 4
#define s2Rotary 5
#define keyRotary 16
#endif

// http://www.mathertel.de/Arduino/RotaryEncoderLibrary.aspx
// Setup a RotaryEncoder with 4 steps per latch for the 2 signal input pins:
RotaryEncoder encoder(s1Rotary, s2Rotary, RotaryEncoder::LatchMode::FOUR3); //comment/uncomment

// Setup a RotaryEncoder with 2 steps per latch for the 2 signal input pins:
// * RotaryEncoder encoder(s1Rotary, s2Rotary, RotaryEncoder::LatchMode::TWO03); //uncomment/comment

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO_EVERY)
// This interrupt routine will be called on any change of one of the input signals

void checkPosition()
{
  encoder.tick(); // just call tick() to check the state.
}

#elif defined(ESP32)
// The interrupt service routine will be called on any change of one of the input signals.

ICACHE_RAM_ATTR void checkPosition()
{
  encoder.tick(); // just call tick() to check the state.
}

#endif

void setup()
{
  Serial.begin(115200);
  pinMode(keyRotary, INPUT);
  while (!Serial)
    ;
  Serial.println();
  Serial.println("InterruptRotator met KeyPress.");
  Serial.println();

  attachInterrupt(digitalPinToInterrupt(s1Rotary), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(s2Rotary), checkPosition, CHANGE);

  //if DEBUG is on, print serial monitor
  if (DEBUG)
  {
    Serial.println("Debugging is ON");
  }
}
void SM_s1()
{
  //Bijna alle statussen gebruiken deze lijnen, daarom staan ze buiten de State Machine
  valKeyRotary = digitalRead(keyRotary);
  statusKeyRotaryVorig = statusKeyRotary;

  //State Machine
  switch (statusKeyRotary)
  {
  case 0: //RESET!
    //Zet alles in beginsituatie
    statusKeyRotary = 1;
    break;

  case 1: //WACHT-op-ON
    //Wacht op keyRotary ingedrukt (=LOW)
    if (valKeyRotary == LOW)
    {
      statusKeyRotary = 2;
    }
    break;

  case 2: //INGEDRUKT!
    //Registreer de tijd en ga verder naar CHECK BOUNCE
    t0 = millis();
    statusKeyRotary = 3;
    break;

  case 3: //CHECK BOUNCE
    //Check bounce delay.  Bij bounce terug naar RESET
    t1 = millis();
    if (t1 - t0 > bounce_delay_s1)
    {
      statusKeyRotary = 4;
    }
    if (valKeyRotary == HIGH)
    {
      statusKeyRotary = 0;
    }
    break;

  case 4: //DEBOUNCED
    //keyRotary los? (=HIGH) > Korte puls. keyRotary langer dan hold_delay > Lange puls
    t1 = millis();
    if (valKeyRotary == HIGH)
    {
      statusKeyRotary = 5;
    }
    if (t1 - t0 > hold_delay_s1)
    {
      statusKeyRotary = 6;
    }
    break;

  case 5: //Korte Puls
    //doorgaan naar RESET
    statusKeyRotary = 0;
    break;

  case 6: //Lange Puls
    //Doorgaan naar WACHT-op-OFF
    statusKeyRotary = 7;
    break;

  case 7: //WACHT-op-OFF
    //wacht op keyRotary los laten >> RESET
    if (valKeyRotary == HIGH)
    {
      statusKeyRotary = 0;
    }
    break;
  }
}

void loop()
{
  // put your main code here, to run repeatedly:
  SM_s1(); //voer statemachine uit

  if (DEBUG) // If DEBUG enabled >> print boodschappen
  {
    if (statusKeyRotaryVorig != statusKeyRotary) // If status wijzigt, print status
    {
      Serial.print("KEY State: ");
      Serial.println(statusKeyRotary);
    }

    if (statusKeyRotary == 6) // keyRotary lang ingedrukt?
    {
      Serial.println("Lange puls!");
    }
    if (statusKeyRotary == 5) // keyRotary kort ingedrukt?
    {
      Serial.println("Korte puls!");
    }
  };

  static int pos = 0;
  static int dir = 0;
  encoder.tick();
  // Read de positie van de encoder en print bij wijziging.
  int newPos = encoder.getPosition();
  if (pos != newPos)
  {
    if (DEBUG)
    {
      Serial.print("pos:");
      Serial.print(newPos);
      Serial.print("  dir:"); /* code */
    }

    dir = (int(encoder.getDirection()));

    if (DEBUG)
    {
      Serial.println(dir);
    }

    pos = newPos;
    currentTime = millis();
    if (currentTime - click >= vertraging)
    {
      if (DEBUG)
      {
        Serial.println("Traag....");
      }
      stappenteller = stappenteller + dir;
    }
    else
    {
      if (DEBUG)
      {
        Serial.println("Snel....");
      }
      stappenteller = stappenteller + 10 * dir;
    }
    if (DEBUG)
    {
      Serial.print("tijdsverschil tussen kliks ");
      Serial.println(currentTime - click);
      Serial.print("stappenteller= ");
      Serial.println(stappenteller);
      Serial.println();
    }

    click = currentTime;
  }
}