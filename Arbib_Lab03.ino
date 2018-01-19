/*	Arbib_Lab02.ino
 * 	Author: Rahul Yarlagadda, Ellie Honious
 * 	Date: December 13th, 2017
 *
 * 	This file contains the main loop for Airbib accomplishing the tasks laid out in Lab 2.
 * 	This includes aggressive kid behaviors, shy kid behaviors, random-wander, and go-to-goal
 * 	functionality. Additionally, random-wander and go-to-goal have integrated obstacle avoidance.
 */

#include <TimerOne.h>
#include "RobotDrive.h"
#include "IRSensor.h"
#include "PinDefinitions.h"
#include "SonarSensor.h"

#define stopThresh  150 // If the robot has been stopped for this threshold move
#define baud_rate 9600//set serial communication baud rate

volatile boolean test_state; //variable to hold test led state for timer interrupt

int goalX = 5;
int goalY = 2;

//flag byte to hold sensor data
volatile byte flag = 0; // Flag to hold IR & Sonar data - used to create the state machine

//bit definitions for sensor data flag byte
#define obFront   0 // Front IR trip
#define obRear    1 // Rear IR trip
#define obRight   2 // Right IR trip
#define obLeft    3 // Left IR trip
#define obFLeft   4 // Left Sonar trip
#define obFRight  5 // Right Sonar trip

int count; //count number of times collide has tripped
#define max_collide 250 //maximum number of collides before robot reverses

//state byte to hold robot motion and state data
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

//define layers of subsumption architecture that are active [hallway Wall Wander Avoid]
byte layers = 4;
#define aLayer 0      //avoid obstacle layer
#define wLayer 1      //wander layer
#define fwLayer 2     //follow wall layer
#define fhLayer 3     //follow hallway layer

#define timer_int 100000 // 1/10 second (250000 us) period for timer interrupt

float lastError = 0;
float outputLog = 0;

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
	wallPD();
}

float PController(float error) {
	float kp = 200;
	outputLog = kp * error;
	return outputLog;
}

float PDController(float error) {
	float kp = 100;
	float kd = 20;
	outputLog = kp*error - kd * (error - lastError);
	lastError = error;
	return outputLog;
}

void wallPD() {
	Serial.print("\nWallBang: li_cerror ri_cerror\t");
	Serial.print(li_cerror);
	Serial.print("\t");
	Serial.println(ri_cerror);
	switch (state) {
	case (FOLLOW_RIGHT):
		Serial.println("right wall found");
		if (ri_cerror == 0) {                 //no error, robot in deadband
			/* THIS WORKS */
			Serial.println("right wall detected, drive forward");
			forward(quarter_rotation); //move robot forward. If i replace this with pivot, the robot starts freezing again (weird).
		} else {
			Serial.println("rt wall: adjust turn angle based upon error");
			float output = fabs(PDController(ri_cerror));
			if (ri_cerror < 0) {          //negative error means too close
				Serial.println("\trt wall: too close turn left");
				pivot(output, 0);      //pivot left
				pivot(output, 1);  //pivot right to straighten up
			} else if (ri_cerror > 0) {     //positive error means too far
				Serial.println("\trt wall: too far turn right");
				pivot(output, 1);      //pivot right
				pivot(output, 0);   //pivot left to straighten up
			}
		}
		break;
	case (FOLLOW_LEFT):
		if (li_cerror == 0) {       //no error robot in dead band drives forward
			Serial.println("lt wall detected, drive forward");
			forward(quarter_rotation);      //move robot forward
		} else {
			Serial.println(
					"lt wall detected: adjust turn angle based upon error");
			float output = fabs(PDController(li_cerror));
			if (li_cerror < 0) { //negative error means too close
				Serial.println("\tlt wall: too close turn right");
				pivot(output, 1);   //pivot right
				pivot(output, 0);   //pivot left to straighten up
			} else if (li_cerror > 0) { //positive error means too far
				Serial.println("\tlt wall: too far turn left");
				pivot(output, 0);   //pivot left
				pivot(output, 1);  //pivot right to straighten up
			}
		}
		break;
	case (CENTER):
		if (derror == 0) {
			//Serial.println("hallway detected, drive forward");
			forward(half_rotation);          //drive robot forward
		} else {
			//Serial.println("hallway detected: adjust turn angle based upon error");
			//try to average the error between the left and right to CENTER the robot
			float output = fabs(PDController(derror));
			if (derror > 0) {
				pivot(output, 0); //spin right, the left error is larger
				pivot(output, 1);       //pivot left to adjust forward
			} else {
				pivot(output, 1); //spin left the right error is larger
				pivot(output, 0);      //pivot right to adjust forward
			}
		}
		break;
	default:
		wanderAndCorners();
	}
}

void wallP() {
	Serial.print("\nWallBang: li_cerror ri_cerror\t");
	Serial.print(li_cerror);
	Serial.print("\t");
	Serial.println(ri_cerror);
	switch (state) {
	case (FOLLOW_RIGHT):
		Serial.println("right wall found");
		if (ri_cerror == 0) {                 //no error, robot in deadband
			/* THIS WORKS */
			Serial.println("right wall detected, drive forward");
			forward(quarter_rotation); //move robot forward. If i replace this with pivot, the robot starts freezing again (weird).
		} else {
			Serial.println("rt wall: adjust turn angle based upon error");
			float output = fabs(PController(ri_cerror));
			if (ri_cerror < 0) {          //negative error means too close
				Serial.println("\trt wall: too close turn left");
				pivot(output, 0);      //pivot left
				pivot(output, 1);  //pivot right to straighten up
			} else if (ri_cerror > 0) {     //positive error means too far
				Serial.println("\trt wall: too far turn right");
				pivot(output, 1);      //pivot right
				pivot(output, 0);   //pivot left to straighten up
			}
		}
		break;
	case (FOLLOW_LEFT):
		if (li_cerror == 0) {       //no error robot in dead band drives forward
			Serial.println("lt wall detected, drive forward");
			forward(quarter_rotation);      //move robot forward
		} else {
			Serial.println(
					"lt wall detected: adjust turn angle based upon error");
			float output = fabs(PController(li_cerror));
			if (li_cerror < 0) { //negative error means too close
				Serial.println("\tlt wall: too close turn right");
				pivot(output, 1);   //pivot right
				pivot(output, 0);   //pivot left to straighten up
			} else if (li_cerror > 0) { //positive error means too far
				Serial.println("\tlt wall: too far turn left");
				pivot(output, 0);   //pivot left
				pivot(output, 1);  //pivot right to straighten up
			}
		}
		break;
	case (CENTER):
		if (((ri_cerror == 0) && (li_cerror == 0)) || (derror == 0)) {
			//Serial.println("hallway detected, drive forward");
			forward(half_rotation);          //drive robot forward
		} else {
			//Serial.println("hallway detected: adjust turn angle based upon error");
			//try to average the error between the left and right to CENTER the robot
			float output = fabs(PController(derror));
			if (derror > 0) {
				spin(output, 1); //spin right, the left error is larger
				pivot(output, 0);       //pivot left to adjust forward
			} else {
				spin(output, 0); //spin left the right error is larger
				pivot(output, 1);      //pivot right to adjust forward
			}
		}
		break;
	default:
		wanderAndCorners();
	}
}

/*
 This is a sample wallBang() function, the description and code should be updated to reflect the actual robot motion function that you will implement
 based upon the the lab requirements.  Some things to consider, you cannot use a blocking motor function because you need to use sensor data to update
 movement.  You also need to continue to poll    the sensors during the motion and update flags and state because this will serve as your interrupt to
 stop or change movement. This function will have the robot follow the wall if it is within 4 to 6 inches from the wall by moving forward and turn on the
 controller if it is outside that band to make an adjustment to get back within the band.
 */
void wallBang() {
	Serial.print("\nWallBang: li_cerror ri_cerror\t");
	Serial.print(li_cerror);
	Serial.print("\t");
	Serial.println(ri_cerror);
	switch (state) {
	case (FOLLOW_RIGHT):
		Serial.println("right wall found");
		if (ri_cerror == 0) {                 //no error, robot in deadband
			/* THIS WORKS */
			Serial.println("right wall detected, drive forward");
			forward(quarter_rotation); //move robot forward. If i replace this with pivot, the robot starts freezing again (weird).
		} else {
			Serial.println("rt wall: adjust turn angle based upon error");
			if (ri_cerror < 0) {          //negative error means too close
				Serial.println("\trt wall: too close turn left");
				pivot(quarter_rotation, 0);      //pivot left
				pivot(quarter_rotation, 1);  //pivot right to straighten up
			} else if (ri_cerror > 0) {     //positive error means too far
				Serial.println("\trt wall: too far turn right");
				pivot(quarter_rotation, 1);      //pivot right
				pivot(quarter_rotation, 0);   //pivot left to straighten up
			}
		}
		break;
	case (FOLLOW_LEFT):
		if (li_cerror == 0) {       //no error robot in dead band drives forward
			Serial.println("lt wall detected, drive forward");
			forward(quarter_rotation);      //move robot forward
		} else {
			Serial.println(
					"lt wall detected: adjust turn angle based upon error");
			if (li_cerror < 0) { //negative error means too close
				Serial.println("\tlt wall: too close turn right");
				pivot(quarter_rotation, 1);   //pivot right
				pivot(quarter_rotation, 0);   //pivot left to straighten up
			} else if (li_cerror > 0) { //positive error means too far
				Serial.println("\tlt wall: too far turn left");
				pivot(quarter_rotation, 0);   //pivot left
				pivot(quarter_rotation, 1);  //pivot right to straighten up
			}
		}
		break;
	case (CENTER):
		if (((ri_cerror == 0) && (li_cerror == 0)) || (derror == 0)) {
			//Serial.println("hallway detected, drive forward");
			forward(half_rotation);          //drive robot forward
		} else {
			//Serial.println("hallway detected: adjust turn angle based upon error");
			//try to average the error between the left and right to CENTER the robot
			if (derror > 0) {
				spin(quarter_rotation, 1); //spin right, the left error is larger
				pivot(quarter_rotation, 0);       //pivot left to adjust forward
			} else {
				spin(quarter_rotation, 0); //spin left the right error is larger
				pivot(quarter_rotation, 1);      //pivot right to adjust forward
			}
		}
		break;
	default:
		wanderAndCorners();
	}
}

void wanderAndCorners() {
	switch (state) {
	case (RIGHT_INSIDE_CORNER):
		Serial.print("right wall: front corner ");
		//make left turn if wall found
		stop();
		pivot(-quarter_rotation, 0);
		reverse(quarter_rotation);              //back up
		pivot(-quarter_rotation, 1);
		spinDegrees(90);              //turn left
		break;
	case (LEFT_INSIDE_CORNER):
		//make right turn if wall found
		Serial.print("left wall: front corner ");
		//make left turn if wall found
		stop();
		pivot(-quarter_rotation, 1);
		reverse(quarter_rotation);              //back up
		pivot(-quarter_rotation, 0);
		spinDegrees(-90);              //turn right
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

void randomWander() {
	int randomAngle = random(0, 360);
	spinDegrees((random(0, 1) ? -1 : 1) * randomAngle);
	forward((random(0, 1) ? -1 : 1) * half_rotation);
}

/*
 This is a sample updateSensors() function and it should be updated along with the description to reflect what you actually implemented
 to meet the lab requirements.
 */
void updateSensors() {
	test_state = !test_state; //LED to test the heartbeat of the timer interrupt routine
	digitalWrite(PIN_LED_TEST, test_state);	//Toggles the LED to let you know the timer is working
	updateIR();  //update IR readings and update flag variable and state machine
	//Serial.println(outputLog);
}

/*
 This is a sample updateState() function, the description and code should be updated to reflect the actual state machine that you will implement
 based upon the the lab requirements.
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

void initNewState(int newState) {
	switch (newState) {
	case (WANDER):
		Serial.println("\tset random wander state");
		digitalWrite(PIN_GREEN_LED, LOW);
		digitalWrite(PIN_RED_LED, LOW);
		break;
	case (FOLLOW_LEFT):
		if (state != newState) {
			lastError = 0;
		}
		Serial.println("\tset follow left state");
		digitalWrite(PIN_GREEN_LED, HIGH);
		digitalWrite(PIN_RED_LED, LOW);
		break;
	case (FOLLOW_RIGHT):
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
