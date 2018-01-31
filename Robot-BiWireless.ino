/*Robot-BiWireless.ino
  Authors: Carlotta Berry
  modified: 02/10/17
  This program will show how to use the Arduino Mega on the robot with the Arduino Uno
  attached to the robot to create bi-directional wireless communication.  This will be used
  to send topological path commands from the laptop to the robot, also to send start and goal positions
  from the laptop to the robot, to receive localization information from the robot to the laptop, and
  to receive a map from the robot to the laptop
  For testing:
    - You can connect both devices to your computer
    - open up 2 instances of the Arduino IDE and put the same program on both
    - upload Mega send code to the robot microcontroller
    - upload Uno receive code on the laptop microcontroller
    - open both serial monitors on your laptop and test the communication

*** HARDWARE CONNECTIONS *****
   https://www.arduino.cc/en/Hacking/PinMapping2560
   Arduino MEGA nRF24L01 connections  *********ROBOT CONNECTION ******************
    CE  pin 7         CSN   pin 8
    VCC 3.3 V         GND GND
    MISO  pin 50      MOSI  pin 51
    SCK pin 52

   http://arduino-info.wikispaces.com/Nrf24L01-2.4GHz-HowTo
   http://www.theengineeringprojects.com/2015/07/interfacing-arduino-nrf24l01.html

   Arduino Uno nRF24L01 connections *************LAPTOP CONNECTION **************
   CE  pin 7        CSN   pin 8
   VCC 3.3 V        GND GND
   MOSI pin 11      MISO pin 12
   SCK pin 13
*/

#include <SPI.h>//include serial peripheral interface library
#include <RF24.h>//include wireless transceiver library
#include <nRF24L01.h>//include wireless transceiver library
#include "PinDefinitions.h"

//variables
boolean transmit = true;              //set variable to send or receive data (use same code for both devices but change variable)
boolean uno = false;                   //set variable for type of microcontroller sending (uno-true-laptop,uno-false-robot)

void setup() {
  Serial.begin(baud_rate);//start serial communication
  radio.begin();//start radio
  radio.setChannel(team_channel);//set the transmit and receive channels to avoid interference
  if (transmit) {
    radio.openWritingPipe(pipe);//open up writing pipe
    //radio.openWritingPipe(addresses[1]);//open reading pipe
    //radio.openReadingPipe(1, addresses[0]);//open reading pipe
    Serial.println("***********************************");
    Serial.println("....Starting nRF24L01 Transmit.....");
    Serial.println("***********************************");
  } else {
    radio.openReadingPipe(1, pipe);//open up reading pipe
    radio.startListening();;//start listening for data;
    //radio.openReadingPipe(1, addresses[1]);//open up reading pipe
    //radio.openWritingPipe(addresses[0]);//open writing pipe
    Serial.println("***********************************");
    Serial.println("....Starting nRF24L01 Receive.....");
    Serial.println("***********************************");
  }
}

void loop() {
  if (transmit) {
    ////// Use this code to test sending with the serial port
    readSerial();
    if (uno) {
      if (data[0] > 0) {
        Serial.println(data[0]);
        radio.write(data, sizeof(data));
      }
    }
    else {
      //// Use this code to test sending from the robot
      Serial.println("\t\trobot_data ");
      for (int i = 0; i <= 3; i++) {
          data[0] = i;
          radio.write(data, sizeof(data));
          Serial.print(data[0]);
          Serial.print("\t");
        }
    }
    //else {
    // Serial.println("no transmit radio");
    //}//no transmit radio
  }//end of transmit if
  else if (!transmit) {
    /// Use this code to receive from the laptop or the robot
    while (radio.available()) {
      radio.read(&data, sizeof(data));
      Serial.println(data[0]);//print the data stream
      radio.read(&incoming, 1);
      if (incoming[0] > 0) {
        Serial.println(incoming[0]);
      }
    }//end while
  }
  delay(100);//wait so the data is readable
}//end of loop

void readSerial() {
  if (Serial.available() > 0) {
    data[0] = Serial.parseInt();
    if (data[0] > 1) {
      Serial.println(data[0]);
    }
  }
}

