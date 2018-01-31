/*RobotReceiver.ino
  Authors: Carlotta Berry, Ricky Rung
  modified: 11/23/16
  This program will set up the laptop to use a nRF24L01 wireless transceiver to
  communicate wirelessly with a mobile robot
  the transmitter is an Arduino Mega connected to the laptop
  the receiver is on an Arduino Mega mounted on the robot

  https://www.arduino.cc/en/Hacking/PinMapping2560
  Arduino MEGA nRF24L01 connections
    CE  pin 7         CSN   pin 8
    MOSI  pin 51      MISO  pin 50
    SCK pin 52        VCC 3.3 V
    GND GND
*/

#include <SPI.h>      //include serial peripheral interface library
#include <RF24.h>     //include wireless transceiver library
#include <nRF24L01.h> //include wireless transceiver library
#include <printf.h>
#include <RF24_config.h>
#include "PinDefinitions.h"


void setup() {
  Serial.begin(9600);//start serial communication
  radio.begin();//start radio
  radio.setChannel(team_channel);//set the transmit and receive channels to avoid interference
  radio.openReadingPipe(1, pipe);//open up reading pipe
  radio.startListening();;//start listening for data;
  pinMode(PIN_LED_TEST, OUTPUT);//set LED pin as an output
}

void loop() {
  while (radio.available()) {
    radio.read(&data, sizeof(data));
    digitalWrite(PIN_LED_TEST, data[0]);
    Serial.println("change the light");
  }
}
