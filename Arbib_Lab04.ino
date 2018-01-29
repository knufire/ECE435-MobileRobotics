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
#include "WallFollowing.h"

#define baud_rate 9600//set serial communication baud rate

volatile boolean test_state; //variable to hold test led state for timer interrupt

//flag byte to hold sensor data
volatile byte flag = 0; // Flag to hold obstacle - used to create the state machine

/**
 * Enum for different braitenberg vehicles
 */
int braitenberg_type = 1;
#define FEAR			0
#define AGGRESSION		1
#define LOVE			2
#define EXPLORER		3

#define LEFT 0
#define RIGHT 1

volatile int robotState;
#define FOLLOW_WALL		0
#define DOCKING			1
#define RETURNING		2

#define timer_int 100000 // 1/10 second (100000 us) period for timer interrupt

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
//	goToLightAndAvoidObstacles();
	homingAndDocking();
	delay(100);
}

/*
 This is a sample updateSensors() function and it should be updated along with the description to reflect what you actually implemented
 to meet the lab requirements.
 */
void updateSensors() {
	test_state = !test_state; //LED to test the heartbeat of the timer interrupt routine
	updateIR();  //update IR readings and update flag variable and state machine
	updatePhoto();
}

/**
 * Lab Part 1
 */
void goToLightAndAvoidObstacles() {
	int wheelSpeed[2];

	//Layer 0 - Random Wander
	int randomSpeed[2];
	randomSpeed[LEFT] = rand() % 1000;
	randomSpeed[RIGHT] = rand() % 1000;

	//Layer 1 - Obstacle avoidance
	int* obstacleSpeeds = obstacleAvoidance();

	//Layer 2 - Follow Light
	int* lightSpeeds = simpleBraiteinbergVehicle();

	//Combine Follow Light and Obstacle Avoidance
	wheelSpeed[LEFT] = (obstacleSpeeds[LEFT] / 2) + (lightSpeeds[LEFT] / 2);
	wheelSpeed[RIGHT] = (obstacleSpeeds[RIGHT] / 2) + (lightSpeeds[RIGHT] / 2);

	//Supress random wander
	if (wheelSpeed[LEFT] == 0 && wheelSpeed[RIGHT] == 0) {
		wheelSpeed[LEFT] = randomSpeed[LEFT];
		wheelSpeed[RIGHT] = randomSpeed[RIGHT];
	}

	//Inhibit if already at light
	if (atLight()) {
		wheelSpeed[LEFT] = 0;
		wheelSpeed[RIGHT] = 0;
	}

	//Actually run the wheels
	setSpeed(wheelSpeed[LEFT], wheelSpeed[RIGHT]);
	runSpeed(500);

	//Free memory
	free(obstacleSpeeds);
	free(lightSpeeds);
}

bool atLight() {
	return photoLeft > 900 || photoRight > 900;
}

int* obstacleAvoidance() {
	int* speeds = calloc(2, sizeof(int));
	if (bitRead(flag, obFront)) {
		speeds[LEFT] = -600;
		speeds[RIGHT] = -600;
	}
	if (bitRead(flag, obRear)) {
		speeds[LEFT] = 600;
		speeds[RIGHT] = 600;
	}
	if (bitRead(flag, obLeft)) {
		speeds[LEFT] = 600;
	}
	if (bitRead(flag, obRight)) {
		speeds[RIGHT] = 600;
	}
	Serial.print("OSpeeds:\t");
	Serial.print(speeds[LEFT]);
	Serial.print("\t");
	Serial.println(speeds[RIGHT]);
	return speeds;
}

int* simpleBraiteinbergVehicle() {
	int* speeds = calloc(2, sizeof(int));
	if (photoLeft < 350 && photoRight < 350) {
		return speeds;
	}
	switch (braitenberg_type) {
	case (LOVE):
		speeds[LEFT] = 1000 - photoLeft;
		speeds[RIGHT] = 1000 - photoRight;
		break;
	case (EXPLORER):
		speeds[LEFT] = photoRight;
		speeds[RIGHT] = photoLeft;
		break;
	case (AGGRESSION):
		speeds[LEFT] = photoLeft;
		speeds[RIGHT] = photoRight;
		break;
	case (FEAR):
		speeds[LEFT] = 1000 - photoRight;
		speeds[RIGHT] = 1000 - photoLeft;
		break;
	}
	Serial.print("BSpeeds:\t");
	Serial.print(speeds[LEFT]);
	Serial.print("\t");
	Serial.println(speeds[RIGHT]);
	return speeds;
}

/**
 * Lab Part 2
 */

void homingAndDocking() {
	updateState();
	switch (robotState) {
	case FOLLOW_WALL:
		followWall();
		break;
	case DOCKING:
		goToLight();
		break;
	case RETURNING:
		if (goToGoal(0,0)) {
			goToAngle(0);
			setState(FOLLOW_WALL);
		}
		break;
	}
}

void goToLight() {
	int* wheelSpeed = simpleBraiteinbergVehicle();
	float magnitude = wheelSpeed[LEFT]/2 + wheelSpeed[RIGHT]/2;
	float turn = wheelSpeed[LEFT] - wheelSpeed[RIGHT];
	spinDegrees((turn*90)/1000);
	forward(quarter_rotation);
//	setSpeed(wheelSpeed[LEFT], wheelSpeed[RIGHT]);
//	runSpeed(250);
	free(wheelSpeed);
}

void setState(int newState) {
	switch (newState) {
	case FOLLOW_WALL:
		digitalWrite(PIN_LED_TEST, LOW);	//Toggles the LED to let you know the timer is working
		digitalWrite(PIN_GREEN_LED, LOW);
		digitalWrite(PIN_RED_LED, LOW);
		break;
	case DOCKING:
		robotX = 0;
		robotY = 0;
		robotAngle = 0;
		digitalWrite(PIN_LED_TEST, HIGH);
		digitalWrite(PIN_GREEN_LED, LOW);
		digitalWrite(PIN_RED_LED, HIGH);
		break;
	case RETURNING:
		digitalWrite(PIN_LED_TEST, HIGH);
		digitalWrite(PIN_GREEN_LED, HIGH);
		digitalWrite(PIN_RED_LED, LOW);
		break;
	}
	robotState = newState;
}

void updateState() {
	switch (robotState) {
	case FOLLOW_WALL:
		if (photoLeft > 500 || photoRight > 500) {
			setState(DOCKING);
		}
		break;
	case DOCKING:
		if (photoLeft > 900 || photoRight > 900) {
			setState(RETURNING);
		}
		break;
	case RETURNING:
		break;
	}
}
