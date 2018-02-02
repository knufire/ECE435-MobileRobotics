/**
 * WallFollowing.ino
 * 	Author: Rahul Yarlagadda, Ellie Honious
 * 	Date: December 13th, 2017
 *
 * 	This file contains behaviors for following walls and navigating corners. It also maintains it's own internal state
 * 	machine to execute corner manuvers and continue following.
 */

#include "WallFollowing.h"

int wallState = 0;

float lastError = 0; //Previous error for derivative control

/**
 * Follows a wall, including hallways and corners. This is a state machine.
 */
void followWall() {
	updateWallState();
//	if (wallState != WANDER) {
//		Serial.print("\nWallBang: li_cerror ri_cerror\t");
//		Serial.print(li_cerror);
//		Serial.print("\t");
//		Serial.println(ri_cerror);
//	} else {
//		Serial.print("Random wander");
//	}
	switch (wallState) {
	case (FOLLOW_RIGHT):
		Serial.println("right wall found");
		if (ri_cerror == 0) {                 //no error, robot in deadband
			Serial.println("right wall detected, drive forward");
			forward (quarter_rotation); //move robot forward. If i replace this with pivot, the robot starts freezing again (weird).
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
			forward (quarter_rotation);      //move robot forward
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
			forward (quarter_rotation);          //drive robot forward
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
//	case (RIGHT_INSIDE_CORNER):
//		Serial.print("right wall: front corner ");
//		//make left turn if wall found
//		stop();
//		pivot(-quarter_rotation, 0);
//		reverse (quarter_rotation);              //back up
//		pivot(-quarter_rotation, 1);
//		spinDegrees(-90);              //turn left
//		break;
//	case (LEFT_INSIDE_CORNER):
//		//make right turn if wall found
//		Serial.print("left wall: front corner ");
//		//make left turn if wall found
//		stop();
//		pivot(-quarter_rotation, 1);
//		reverse(quarter_rotation);              //back up
//		pivot(-quarter_rotation, 0);
//		spinDegrees(90);              //turn right
//		break;
//	case (LEFT_OUTSIDE_CORNER):
//		stop();
//		reverse(quarter_rotation);
//		spinDegrees(-90);
//		forward(one_rotation + half_rotation);
//		break;
//	case (RIGHT_OUTSIDE_CORNER):
//		stop();
//		spinDegrees(90);
//		forward(one_rotation + half_rotation);
//		break;
//	case (WANDER):
//		randomWander();
//		break;
	}
}

/**
 * A generic PD controller to maintain a set distance away from the wall.
 */
float PDController(float error) {
	float kp = 100;
	float kd = 20;
	float output = kp * error - kd * (error - lastError);
	lastError = error;
	return output;
}

/**
 * Automatically updates the wall following state based on sensor feedback.
 */
void updateWallState() {
	if (bitRead(flag, obRight) && !bitRead(flag, obLeft)) {
//		if (bitRead(flag, obFront)) {
//			setWallState(RIGHT_INSIDE_CORNER);
//		} else {
//			setWallState(FOLLOW_RIGHT);
//		}
		setWallState(FOLLOW_RIGHT);
	} else if (bitRead(flag, obLeft) && !bitRead(flag, obRight)) {
//		if (bitRead(flag, obFront)) {
//			setWallState(LEFT_INSIDE_CORNER);
//		} else {
//			setWallState(FOLLOW_LEFT);
//		}
		setWallState(FOLLOW_LEFT);
	} else if (bitRead(flag, obLeft) && bitRead(flag, obRight)) {
		setWallState(CENTER);
	}
}

/**
 * Updates the current wall state to the newState, checking for illegal state transitions and correcting them.
 */
void setWallState(int newState) {
//	switch (wallState) {
//	case (WANDER):
//		break;
//	case (FOLLOW_LEFT):
//		if (newState == WANDER) {
//			newState = LEFT_OUTSIDE_CORNER;
//		}
//		break;
//	case (FOLLOW_RIGHT):
//		if (newState == WANDER) {
//			newState = RIGHT_OUTSIDE_CORNER;
//		}
//		break;
//	case (CENTER):
//		break;
//	case (LEFT_INSIDE_CORNER):
//		if (newState == LEFT_INSIDE_CORNER) {
//			newState = FOLLOW_LEFT;
//		}
//		break;
//	case (RIGHT_INSIDE_CORNER):
//		if (newState == RIGHT_INSIDE_CORNER) {
//			newState = FOLLOW_RIGHT;
//		}
//		break;
//	case (LEFT_OUTSIDE_CORNER):
//		if (wallState == newState) {
//			newState = WANDER;
//		}
//		break;
//	case (RIGHT_OUTSIDE_CORNER):
//		if (wallState == newState) {
//			newState = WANDER;
//		}
//		break;
//	}
	if (wallState != newState) {
		initNewWallState(newState);
	}
	wallState = newState;
}

/**
 * Performs any initalization necessray when changing states.
 */
void initNewWallState(int newState) {
	switch (newState) {
	case (WANDER):
		Serial.println("\tset random wander wallState");
//		digitalWrite(PIN_GREEN_LED, LOW);
//		digitalWrite(PIN_RED_LED, LOW);
		break;
	case (FOLLOW_LEFT):
		if (wallState != newState) {
			lastError = 0;
		}
		Serial.println("\tset follow left wallState");
//		digitalWrite(PIN_GREEN_LED, HIGH);
//		digitalWrite(PIN_RED_LED, LOW);
		break;
	case (FOLLOW_RIGHT):
		if (wallState != newState) {
			lastError = 0;
		}
		Serial.println("\tset follow right wallState");
//		digitalWrite(PIN_GREEN_LED, HIGH);
//		digitalWrite(PIN_RED_LED, LOW);
		break;
	case (CENTER):
		Serial.println("\tset follow hallway wallState");
//		digitalWrite(PIN_GREEN_LED, LOW);
//		digitalWrite(PIN_RED_LED, HIGH);
		break;
	case (LEFT_INSIDE_CORNER):
		Serial.println("\tset left corner wallState");
//		digitalWrite(PIN_GREEN_LED, HIGH);
//		digitalWrite(PIN_RED_LED, HIGH);
		break;
	case (RIGHT_INSIDE_CORNER):
		Serial.println("\tset right corner wallState");
//		digitalWrite(PIN_GREEN_LED, HIGH);
//		digitalWrite(PIN_RED_LED, HIGH);
		break;
	case (LEFT_OUTSIDE_CORNER):
		Serial.println("\tset left outside corner wallState");
//		digitalWrite(PIN_GREEN_LED, HIGH);
//		digitalWrite(PIN_RED_LED, HIGH);
		break;
	case (RIGHT_OUTSIDE_CORNER):
		Serial.println("\tset right outside corner wallState");
//		digitalWrite(PIN_GREEN_LED, HIGH);
//		digitalWrite(PIN_RED_LED, HIGH);
		break;
	}
}
