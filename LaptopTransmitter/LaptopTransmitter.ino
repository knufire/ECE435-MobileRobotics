/*RobotTransmitter.ino
  Authors: Carlotta Berry, Ricky Rung
  modified: 11/23/16
  This program will set up the laptop to use a nRF24L01 wireless transceiver to
  communicate wirelessly with a mobile robot
  the transmitter is an Arduino Mega connected to the laptop
  the receiver is on an Arduino Mega mounted on the robot

  HARDWARE CONNECTIONS:

  https://www.arduino.cc/en/Hacking/PinMapping2560
  Arduino MEGA nRF24L01 connections
    CE  pin 7         CSN   pin 8
    MOSI  pin 51      MISO  pin 50
    SCK pin 52        VCC 3.3 V
    GND GND

    Arduino Uno nRF24L01 connections
     http://arduino-info.wikispaces.com/Nrf24L01-2.4GHz-HowTo
     http://www.theengineeringprojects.com/2015/07/interfacing-arduino-nrf24l01.html
   1 - GND
   2 - VCC 3.3V !!! NOT 5V
   3 - CE to Arduino pin 7
   4 - CSN to Arduino pin 8
   5 - SCK to Arduino pin 13
   6 - MOSI to Arduino pin 11
   7 - MISO to Arduino pin 12
   8 - UNUSED
*/

#include <SPI.h>//include serial peripheral interface library
#include <RF24.h>//include wireless transceiver library
#include <nRF24L01.h>
#include <printf.h>
#include <RF24_config.h>


// Set up the wireless transceiver pins
#define CE_PIN  7
#define CSN_PIN 8
#define test_LED 13
#define MAX_LEN 63		  //Serial buffer is only 64 bytes, and we need 1 for a null terminator.
#define team_channel 37   //transmitter and receiver on same channel between 1 & 125

const uint64_t pipe = 0xE8E8F0F0E1LL; //define the radio transmit pipe (5 Byte configurable)
RF24 radio(CE_PIN, CSN_PIN);          //create radio object
char buf[MAX_LEN];
uint8_t len;

void setup() {
  Serial.begin(9600);//start serial communication
  radio.begin();//start radio
  radio.setChannel(team_channel);//set the transmit and receive channels to avoid interference
  radio.openWritingPipe(pipe);//open up writing pipe
  radio.openReadingPipe(1, pipe);
  radio.startListening();
}

void loop() {
  //use serial monitor to send 0 and 1 to blink LED on digital pin 13 on robot microcontroller
  getSerialData();
  if (len > 0) {
	  Serial.println(buf);
	  delay(100);
	  radio.stopListening();
	  radio.write(buf, len);
	  radio.startListening();
  }
  if (radio.available()) {
	  	char buf[128];
  		radio.read(buf, 128);
  		Serial.println(buf);
  }

}

void getSerialData() {
	if(Serial.available() > 0) {
		len = Serial.readBytesUntil(' ', buf, MAX_LEN-1);
		buf[len] = '\0'; //Need to null terminate string
		len++;
	} else {
		len = 0;
	}
}
