/*	Arbib_Main.ino
 * 	Author: Rahul Yarlagadda, Ellie Honious
 * 	Date: January 20th, 2018
 *
 * 	This file contains the main loop for Arbib accomplishing the tasks laid out in the final project.
 *  It also contains the code to follow topological paths.
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

	delay(1000);

	//Wander around the world and generate a map.
	makeMap();

	//Localize the robot within the world.
	int startPoint = localize();

	//Parse the (x,y) current location from the robot localizing code.
	startX = startPoint/10;
	startY = startPoint%10;

	//Sent the current robot location over the wireless interface.
	wirelessSend("At: (" + String(startX) + "," + String(startY) +")\t");

	//Do nothing until we get a goal location from the laptop.
	waitForCommand();

	//Parse the (x,y) goal location from the command sent wirelessly.
	int endPoint = parsedCommand.toInt();
	int x = endPoint/10;
	int y = endPoint%10;

	//Get the topological directions to the goal location.
	parsedCommand = getDirectionsToGoal(startX, startY, x, y);

	//Send those directions to the laptop.
	wirelessSend(parsedCommand);

	//Execute the topological directions.
	executeCommands();

}

void loop() {

}

/**
 * Blocks until a command is received from the wireless interface.
 */
void waitForCommand() {
	//Tell the laptop we are waiting for a command.
	wirelessSend("WAITING FOR COMMAND");

	//Block until we receive a command.
	while (receivedCommand == NULL) {
		//Check once every second.
		delay(1000);
		receivedCommand = wirelessRecieve();

		//Convert the received command into a string.
		parsedCommand = String(receivedCommand);
	}
}

/**
 * Executes a topological path plan.
 */
void executeCommands() {
	//For each letter in the string.
	for (int i = 0; i < parsedCommand.length(); i++) {
		//Get the letter.
		char command = parsedCommand.charAt(i);

		//Move straight, left, or right depending on the letter.
		switch (command) {
		case 'S':
			//If the last move is straight, just drive forward a bit since there might not be a wall to follow.
			if (parsedCommand.charAt(i+1) == 'T') {
				forward(two_rotation);
			} else {
				executeMove(STRAIGHT);
			}
			break;
		case 'L':
			executeMove(LEFT);
			break;
		case 'R':
			executeMove(RIGHT);
			break;
		case 'T':
			executeMove(TERMINATE);
			return;
		}

	}
}
/**
 * Execute a single topological move.
 */
void executeMove(char move) {
	if (move == -1) {
		Serial.println("Error: bad move");
	}
	boolean successful = false;

	//Keep trying to make the move until it's successful.
	//This way, if the move is a turn, it will keep driving forward until it can actually turn.
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

/**
 * Follow a wall until the wall disappears.
 */
void followUntilChange() {
	//Get the current state of left and right obstacles
	char leftState = bitRead(flag, obLeft);
	char rightState = bitRead(flag, obRight);

	//Until either of those obstacles change, keep following the wall (or hallway).
	while ((leftState == bitRead(flag, obLeft)) && (rightState == bitRead(flag, obRight))) {
		followWall();
	}
}

/**
 * Move one cell forward if possible. If it isn't, stop. Return whether the move was successful or not.
 */
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

/**
 * Move one cell left if possible. If it isn't, move forward. Return whether the move was successful or not.
 */
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

/**
 * Move one cell right if possible. If it isn't, move forward. Return whether the move was successful or not.
 */
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

