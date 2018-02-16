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
#include "PinDefinitions.h"
#include "WallFollowing.h"
#include "WirelessReceiver.h"
#include "MetricPathPlanning.h"
#include "RobotLocalization.h"

#define baud_rate 9600//set serial communication baud rate

volatile boolean test_state; //variable to hold test led state for timer interrupt

//flag byte to hold sensor data
volatile byte flag = 0; // Flag to hold obstacle - used to create the state machine

volatile int robotState;
#define FOLLOW_WALL		0
#define DOCKING			1
#define RETURNING		2

#define STRAIGHT 		0
#define LEFT			1
#define RIGHT			2
#define TERMINATE		3

#define timer_int 100000 // 1/10 second (100000 us) period for timer interrupt

char* receivedCommand;
String parsedCommand;

int startX, startY;

void setup() {
	Serial.println("Setting up...");
	//Serial setup
	Serial.begin(baud_rate); //start serial communication in order to debug the software while coding

	//Drive Setup
	driveSetup();

	//Wireless setup
	wirelessSetup();

	//LED Setup
	pinMode(PIN_GREEN_LED, OUTPUT);
	pinMode(PIN_RED_LED, OUTPUT);
	pinMode(PIN_LED_ENABLE, OUTPUT);
	digitalWrite(PIN_LED_ENABLE, LOW);
	digitalWrite(PIN_GREEN_LED, LOW);
	digitalWrite(PIN_RED_LED, LOW);

	//Timer Interrupt Set Up
	Timer1.initialize(timer_int); // initialize timer1, and set a period in microseconds
	Timer1.attachInterrupt(updateSensors); // attaches updateSensors() as a timer overflow interrupt
	receivedCommand = NULL;

//	parsedCommand = getDirectionsToGoal(0, 0, 3, 0);
//	Serial.println(parsedCommand);
//	delay(100);
//	wirelessSend(parsedCommand);
//	executeCommands();

	delay(1000);
	makeMap();
//	int startPoint = localize();
//	startX = startPoint/10;
//	startY = startPoint%10;
//	wirelessSend("At: (" + String(startX) + "," + String(startY) +")\t");
//	waitForCommand();
//	int endPoint = parsedCommand.toInt();
//	int x = endPoint/10;
//	int y = endPoint%10;
//	parsedCommand = getDirectionsToGoal(startX, startY, x, y);
//	wirelessSend(parsedCommand);
//	executeCommands();

}

void loop() {

}


void waitForCommand() {
	wirelessSend("WAITING FOR COMMAND");
	while (receivedCommand == NULL) {
		delay(1000);
		receivedCommand = wirelessRecieve();
		parsedCommand = String(receivedCommand);
		Serial.println(parsedCommand);
	}
}

void executeCommands() {
	for (int i = 0; i < parsedCommand.length(); i++) {
		char command = parsedCommand.charAt(i);
		switch (command) {
		case 'S':
			wirelessSend("-------COMMAND: STRAIGHT----------");
			if (parsedCommand.charAt(i+1) == 'T') {
				forward(two_rotation);
			} else {
				executeMove(STRAIGHT);
			}
			break;
		case 'L':
			wirelessSend("-------COMMAND: LEFT----------");
			executeMove(LEFT);
			break;
		case 'R':
			wirelessSend("-------COMMAND: RIGHT----------");
			executeMove(RIGHT);
			break;
		case 'T':
			wirelessSend("-------COMMAND: STOP----------");
			executeMove(TERMINATE);
			return;
		}

	}
}

void executeMove(char move) {
	if (move == -1) {
		Serial.println("Error: bad move");
	}
	boolean successful = false;
	while (!successful) {
		switch (move) {
		case STRAIGHT:
			successful = moveForward();
			break;
		case LEFT:
			successful = moveLeft();
			break;
		case RIGHT:
			successful = moveRight();
			break;
		case TERMINATE:
			stop();
			successful = true;
			break;
		}
	}
}

void followUntilChange() {
	char leftState = bitRead(flag, obLeft);
	char rightState = bitRead(flag, obRight);
	while ((leftState == bitRead(flag, obLeft)) && (rightState == bitRead(flag, obRight))) {
		followWall();
	}
}


bool moveForward() {
	if (bitRead(flag, obFront)) {
		Serial.print("Tried to move foward, but obstacle. Stopping.");
		stop();
		return false;
	} else {
		Serial.print("Moving forward!");
		followUntilChange();
		return true;
	}
}

bool moveLeft() {
	if (bitRead(flag, obLeft)) {
		Serial.print("Tried to move left, but obstacle. Going forward");
		moveForward();
		return false;
	} else {
		Serial.print("Moving Left!");
		reverse(quarter_rotation);
		pivot(90*CONST_SPIN_DEGREES_TO_STEPS*1.95, 0);
		forward(one_rotation + quarter_rotation);
		return true;
	}
}

bool moveRight() {
	if (bitRead(flag, obRight)) {
		Serial.print("Tried to move right, but obstacle. Going forward");
		moveForward();
		return false;
	} else {
		Serial.print("Moving right!");
		reverse(quarter_rotation);
		pivot(90*CONST_SPIN_DEGREES_TO_STEPS*1.95, 1);
		forward(one_rotation + quarter_rotation);
		return true;
	}
}

/**
 * Updates all sensors. Bound to a timer-based interrupt.
 */
void updateSensors() {
	test_state = !test_state; //LED to test the heartbeat of the timer interrupt routine
	updateIR();  //update IR readings and update flag variable and state machine
}

