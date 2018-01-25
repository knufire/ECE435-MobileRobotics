/*	Arbib_Lab02.ino
 * 	Author: Rahul Yarlagadda, Ellie Honious
 * 	Date: January 20th, 2018
 *
 * 	This file contains the main loop for Airbib accomplishing the tasks laid out in Lab 3.
 * 	This includes aggressive kid behaviors, shy kid behaviors, random-wander, and go-to-goal
 * 	functionality. Additionally, random-wander and go-to-goal have integrated obstacle avoidance
 * 	and wall follow capabilities.
 */

#include <TimerOne.h>
#include "RobotDrive.h"
#include "IRSensor.h"
#include "PhotoSensor.h"
#include "PinDefinitions.h"

#define baud_rate 9600//set serial communication baud rate

volatile boolean test_state; //variable to hold test led state for timer interrupt

//flag byte to hold sensor data
volatile byte flag = 0; // Flag to hold obstacle - used to create the state machine

/**
 * State machine varable and states
 */
volatile int state = 0;   //state to hold robot states and motor motion
#define AVOID     				0   //avoid behavior
#define WANDER    				1   //wander behavior
#define FOLLOW_LEFT     		2   //follow left wall behavior
#define FOLLOW_RIGHT    		3   //follow right wall behavior
#define CENTER    				4   //follow hallway behavior
#define RIGHT_INSIDE_CORNER		5
#define LEFT_INSIDE_CORNER		6
#define RIGHT_OUTSIDE_CORNER 	7
#define LEFT_OUTSIDE_CORNER		8

/**
 * Enum for different control modes
 * Used as the parameter for the wallFollowing() and followWall() functions
 */
#define BANG_BANG				0
#define P_CONTROL				1
#define PD_CONTROL				2


/**
 * Enum for different braitenberg vehicles
 */
int braitenberg_type;
#define FEAR			0
#define AGGRESSION		1
#define LOVE			2
#define EXPLORER		3

#define timer_int 100000 // 1/10 second (100000 us) period for timer interrupt

float lastError = 0; //Previous error for derivative control

void setup() {
	//Serial setup
	Serial.begin(baud_rate); //start serial communication in order to debug the software while coding

	//Drive Setup
	driveSetup();

	//LED Setup
	pinMode(PIN_GREEN_LED, OUTPUT);
	pinMode(PIN_RED_LED, OUTPUT);
	pinMode(PIN_LED_ENABLE, OUTPUT);
	digitalWrite(PIN_LED_ENABLE, HIGH);
	digitalWrite(PIN_GREEN_LED, HIGH);
	digitalWrite(PIN_RED_LED, HIGH);

	//Timer Interrupt Set Up
	Timer1.initialize(timer_int); // initialize timer1, and set a period in microseconds
	Timer1.attachInterrupt(updateSensors); // attaches updateSensors() as a timer overflow interrupt

	delay(1500);  //wait 3 seconds before robot moves

	braitenberg_type = AGGRESSION;
}

void loop() {
	simpleBraiteinbergVehicle();
}

void simpleBraiteinbergVehicle() {
	switch (braitenberg_type) {
	case (LOVE):
		runAtSpeed(1000-photoLeft, 1000-photoRight);
		break;
	case (EXPLORER):
		runAtSpeed(photoRight, photoLeft);
		break;
	case (AGGRESSION):
		runAtSpeed(photoLeft, photoRight);
		break;
	case (FEAR):
		runAtSpeed(1000-photoRight, 1000-photoLeft);
		break;
	}
}

/*
 This is a sample updateSensors() function and it should be updated along with the description to reflect what you actually implemented
 to meet the lab requirements.
 */
void updateSensors() {
	test_state = !test_state; //LED to test the heartbeat of the timer interrupt routine
	digitalWrite(PIN_LED_TEST, test_state);	//Toggles the LED to let you know the timer is working
	updateIR();  //update IR readings and update flag variable and state machine
	updatePhoto();
}
