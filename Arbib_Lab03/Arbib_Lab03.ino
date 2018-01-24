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
}

void loop() {
	updateState();  	//update State Machine based upon sensor readings
	wallFollowing(PD_CONTROL);
}

/**
 * This function contains the specific actions the robot should take based on the current state.
 */
void wallFollowing(int mode) {
	Serial.print("\nWallBang: li_cerror ri_cerror\t");
	Serial.print(li_cerror);
	Serial.print("\t");
	Serial.println(ri_cerror);
	switch (state) {
	case (FOLLOW_RIGHT):
		followWall(mode, ri_cerror);
		break;
	case (FOLLOW_LEFT):
		followWall(mode, -li_cerror);
		break;
	case (CENTER):
		followWall(mode, derror);
		break;
	case (RIGHT_INSIDE_CORNER):
		Serial.print("right wall: front corner ");
		stop();
		pivot(-quarter_rotation, 0);
		reverse(quarter_rotation);
		pivot(-quarter_rotation, 1);
		spinDegrees(90);
		break;
	case (LEFT_INSIDE_CORNER):
		Serial.print("left wall: front corner ");
		stop();
		pivot(-quarter_rotation, 1);
		reverse(quarter_rotation);
		pivot(-quarter_rotation, 0);
		spinDegrees(-90);
		break;
	case (LEFT_OUTSIDE_CORNER):
		stop();
		reverse(quarter_rotation);
		spinDegrees(90);
		forward(one_rotation + half_rotation);
		break;
	case (RIGHT_OUTSIDE_CORNER):
		stop();
		spinDegrees(-90);
		forward(one_rotation + half_rotation);
		break;
	case (WANDER):
		Serial.println("nothing to see here, I need to look for a wall");
		randomWander();
		break;
	}
}


/**
 * This function contains the specific logic to follow a wall. It determines the amount of
 * error based on the control mode passed in, and uses pivots to shift the robot left or right.
 */
void followWall(int mode, float error) {
	if (error == 0) {                 //no error, robot in deadband
		forward(quarter_rotation);
	} else {
		//Calculate the amount to move based on the control method
		float output = 0;
		switch (mode) {
		case (BANG_BANG):
			output = quarter_rotation;
			break;
		case (P_CONTROL):
			output = fabs(PController(error));
			break;
		case (PD_CONTROL):
			output = fabs(PDController(error));
			break;
		}

		if (output < 0) {      	//negative error means too left
			pivot(output, 0);  	//pivot left
			pivot(output, 1);  	//pivot right to straighten up
		} else if (output > 0) { //positive error means too right
			pivot(output, 1);   //pivot right
			pivot(output, 0);  	//pivot left to straighten up
		}
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
}

/**
 * Updates the state based on the obstacle flags from sensors. Calls the setState() function to
 * actually change the state.
 */
void updateState() {
	if (flag == 0) { //no sensors triggered
		setState(WANDER);
	} else if (bitRead(flag,obRight) && !bitRead(flag, obLeft)) {
		if (bitRead(flag, obFront)) {
			setState(RIGHT_INSIDE_CORNER);
		} else {
			setState(FOLLOW_RIGHT);
		}
	} else if (bitRead(flag,obLeft) && !bitRead(flag, obRight)) {
		if (bitRead(flag, obFront)) {
			setState(LEFT_INSIDE_CORNER);
		} else {
			setState(FOLLOW_LEFT);
		}
	} else if (bitRead(flag,obLeft) && bitRead(flag, obRight)) {
		setState(CENTER);
	}
}

/**
 * Updates the current state with the specified new state. The new state may be intercepted
 * (e.g., going from following a wall to wandering will be intercepted with a corner state).
 * If the new state is different from the current state, the initNewState() function will be
 * called.
 */
void setState(int newState) {
	switch (state) {
	case (WANDER):
		break;
	case (FOLLOW_LEFT):
		if (newState == WANDER) {
			newState = LEFT_OUTSIDE_CORNER;
		}
		break;
	case (FOLLOW_RIGHT):
		if (newState == WANDER) {
			newState = RIGHT_OUTSIDE_CORNER;
		}
		break;
	case (CENTER):
		break;
	case (LEFT_INSIDE_CORNER):
		if (newState == LEFT_INSIDE_CORNER) {
			newState = FOLLOW_LEFT;
		}
		break;
	case (RIGHT_INSIDE_CORNER):
		if (newState == RIGHT_INSIDE_CORNER) {
			newState = FOLLOW_RIGHT;
		}
		break;
	case (LEFT_OUTSIDE_CORNER):
		if (state == newState) {
			newState = WANDER;
		}
		break;
	case (RIGHT_OUTSIDE_CORNER):
		if (state == newState) {
			newState = WANDER;
		}
		break;
	}
	if (state != newState) {
		initNewState(newState);
	}
	state = newState;
}

/**
 * Provides any initalization necessary when transitioning to a new state.
 */
void initNewState(int newState) {
	switch (newState) {
	case (WANDER):
		Serial.println("\tset random wander state");
		digitalWrite(PIN_GREEN_LED, LOW);
		digitalWrite(PIN_RED_LED, LOW);
		break;
	case (FOLLOW_LEFT):
		//We're possibly starting a new control loop, reset the last error.
		if (state != newState) {
			lastError = 0;
		}
		Serial.println("\tset follow left state");
		digitalWrite(PIN_GREEN_LED, HIGH);
		digitalWrite(PIN_RED_LED, LOW);
		break;
	case (FOLLOW_RIGHT):
		//We're possibly starting a new control loop, reset the last error.
		if (state != newState) {
			lastError = 0;
		}
		Serial.println("\tset follow right state");
		digitalWrite(PIN_GREEN_LED, HIGH);
		digitalWrite(PIN_RED_LED, LOW);
		break;
	case (CENTER):
		Serial.println("\tset follow hallway state");
		digitalWrite(PIN_GREEN_LED, LOW);
		digitalWrite(PIN_RED_LED, HIGH);
		break;
	case (LEFT_INSIDE_CORNER):
		Serial.println("\tset left corner state");
		digitalWrite(PIN_GREEN_LED, HIGH);
		digitalWrite(PIN_RED_LED, HIGH);
		break;
	case (RIGHT_INSIDE_CORNER):
		Serial.println("\tset right corner state");
		digitalWrite(PIN_GREEN_LED, HIGH);
		digitalWrite(PIN_RED_LED, HIGH);
		break;
	case (LEFT_OUTSIDE_CORNER):
		Serial.println("\tset left outside corner state");
		digitalWrite(PIN_GREEN_LED, HIGH);
		digitalWrite(PIN_RED_LED, HIGH);
		break;
	case (RIGHT_OUTSIDE_CORNER):
		Serial.println("\tset right outside corner state");
		digitalWrite(PIN_GREEN_LED, HIGH);
		digitalWrite(PIN_RED_LED, HIGH);
		break;
	}
}

/**
 * Generic proportional controller.
 */
float PController(float error) {
	float kp = 200;
	return kp * error;
}

/**
 * Generic proportional-derivative controller
 */
float PDController(float error) {
	float kp = 100;
	float kd = 20;
	float output = kp * error - kd * (error - lastError);
	lastError = error;
	return output;
}
