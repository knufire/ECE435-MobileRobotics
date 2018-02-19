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

#include "Arduino.h"
#include "WirelessReceiver.h"
#include <SPI.h>      //include serial peripheral interface library
#include <RF24.h>     //include wireless transceiver library
#include <nRF24L01.h> //include wireless transceiver library
#include <printf.h>
#include <RF24_config.h>
#include "PinDefinitions.h"

const uint64_t pipe = 0xE8E8F0F0E1LL; //define the radio transmit pipe (5 Byte configurable)
RF24 radio(PIN_CE, PIN_CSN);          //create radio object
char command[64];                      //variable to hold transmit data

/**
 * Setup wireless communication.
 */
void wirelessSetup() {
	radio.begin();                      //start radio
	radio.setChannel(TEAM_CHANNEL); //set the transmit and receive channels to avoid interference
	radio.openReadingPipe(1, pipe); //open up reading pipe
	radio.openWritingPipe(pipe);	//open writing pipe
	radio.startListening();			//start listening for data;
}

/**
 * Receive a wireless command, if available.
 */
char* wirelessRecieve() {
	if (radio.available()) {
		radio.read(command, 64);
		return command;
	}
	return NULL;
}

/**
 * Send a string over the wireless interface
 */
void wirelessSend(String str) {
	//Convert string to char array
	char buf[str.length()+1];
	str.toCharArray(buf, str.length());
	buf[str.length()] = '\0';

	//Stop listening to put into write mode.
	radio.stopListening();

	//Send data
	radio.write(buf, str.length());

	//Start listening again
	radio.startListening();
}
