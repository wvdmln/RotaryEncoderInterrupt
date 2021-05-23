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

unsigned long currentTime = 0;
unsigned long click = 0;
unsigned long vertraging = 100;
unsigned long t0;
unsigned long t1;
int stappenteller = 0;
bool statusDrukknop;
bool vergrendel;
int nulstap = 0;
int maxstap = 500;
int stapcase = 0;

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO_EVERY)
// Example for Arduino UNO with input signals on pin 2 and 3
#define PIN_IN1 2
#define PIN_IN2 3

#elif defined(ESP32)
// Example for ESP32 with input signals on pin 4 and 15
#define PIN_IN1 4
#define PIN_IN2 5
#define Drukknop 16
#endif

// http://www.mathertel.de/Arduino/RotaryEncoderLibrary.aspx
// Setup a RotaryEncoder with 4 steps per latch for the 2 signal input pins:
RotaryEncoder encoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::FOUR3); //comment/uncomment

// Setup a RotaryEncoder with 2 steps per latch for the 2 signal input pins:
// * RotaryEncoder encoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03); //uncomment/comment

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
  pinMode(Drukknop, INPUT);
  while (!Serial)
    ;
  Serial.println("InterruptRotator example for the RotaryEncoder library.");

  attachInterrupt(digitalPinToInterrupt(PIN_IN1), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_IN2), checkPosition, CHANGE);
}

// Read the current position of the encoder and print out when changed.
void loop()
{
  static int pos = 0;
  static int dir = 0;
  encoder.tick();
  statusDrukknop = digitalRead(Drukknop);
  if (statusDrukknop == LOW && vergrendel == 0)
  {
    vergrendel = 1;
    Serial.println("Drukknop ingedrukt");
  }
  if (statusDrukknop == HIGH)
  {
    vergrendel = 0;
  }

  int newPos = encoder.getPosition();

  if (pos != newPos)
  {
    Serial.print("pos:");
    Serial.print(newPos);
    Serial.print("  dir:");
    dir = (int(encoder.getDirection()));
    Serial.println(dir);
    pos = newPos;
    currentTime = millis();
    if (currentTime - click >= vertraging)
    {
      Serial.println("Traag....");
      stappenteller = stappenteller + dir;
    }
    else
    {
      Serial.println("Snel....");
      stappenteller = stappenteller + 10 * dir;
    }

    Serial.print("tijdsverschil tussen kliks ");
    Serial.println(currentTime - click);
    Serial.print("stappenteller= ");
    Serial.println(stappenteller);
    Serial.println();

    click = currentTime;
  }
}