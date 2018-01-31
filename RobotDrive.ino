/**
 * RobotDrive.ino
 * 	Author: Rahul Yarlagadda, Ellie Honious
 * 	Date: December 13th, 2017
 *
 * 	This file contains the basic motor control and odometry code for the robot. Basic functions for driving forward, reverse
 * 	and going to an angle are written.
 */

#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library
#include "RobotDrive.h"
#include "PinDefinitions.h"

AccelStepper stepperRight(AccelStepper::DRIVER, PIN_RT_STEP, PIN_RT_DIR); //create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, PIN_LT_STEP, PIN_LT_DIR); //create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers; //create instance to control multiple steppers at the same time

/**
 * NOTE: FUNCTION DOCUMENTATION IN RobotDrive.h
 */

void driveSetup() {
	Serial.println("Setting up drive");

	pinMode(PIN_RT_STEP, OUTPUT); //sets pin as outputr
	pinMode(PIN_RT_DIR, OUTPUT); //sets pin as output
	pinMode(PIN_LT_STEP, OUTPUT); //sets pin as output
	pinMode(PIN_LT_DIR, OUTPUT); //sets pin as output
	pinMode(PIN_STEP_ENABLE, OUTPUT); //sets pin as output

	digitalWrite(PIN_STEP_ENABLE, false); 	//turns on the stepper motor drivers

	stepperRight.setMaxSpeed(max_spd); //set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
	stepperRight.setAcceleration(max_accel); //set desired acceleration in steps/s^2
	stepperLeft.setMaxSpeed(max_spd); //set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
	stepperLeft.setAcceleration(max_accel); //set desired acceleration in steps/s^2

	stepperRight.setSpeed(robot_spd); //set right motor speed
	stepperLeft.setSpeed(robot_spd); //set left motor speed

	steppers.addStepper(stepperRight); //add right motor to MultiStepper
	steppers.addStepper(stepperLeft); //add left motor to MultiStepper
}

void forward(int rot) {
	stepperRight.move(rot);	//move right motor to position
	stepperLeft.move(rot); 	//move left motor to position
	runToStop();   			//run until the robot reaches the target
	updateRobotPosition(rot, rot);
}

void pivot(int rot, int dir) {
	if (dir > 0) {
		stepperLeft.move(rot);
		updateRobotPosition(rot, 0);
	} else {
		stepperRight.move(rot);
		updateRobotPosition(0, rot);
	}
	runToStop();
	//TODO: Update robot postition variables.
}

void spin(int rot) {
	stepperRight.move(-rot);
	stepperLeft.move(rot);
	runToStop();
	updateRobotPosition(rot, -rot);
}

void stop() {
	stepperRight.stop();
	stepperLeft.stop();
}

void reverse(int rot) {
	forward(-rot);
}

void spinDegrees(float degrees) {
	//Convert change in angle into motor steps
	long numSteps = -degrees * CONST_SPIN_DEGREES_TO_STEPS;
	spin(numSteps);
}

void goToAngle(float degrees) {
	//Use the robot's current angle to figure out the needed change in angle
	float robotAngle = robotPose(2) / PI * 180;
	float dAngle = degrees - robotAngle;

	//Ensure the change in angle is between -180 and 180 for the shortest spin possible
	while (dAngle > 180) {
		dAngle = dAngle - 360;
	}
	while (dAngle < -180) {
		dAngle = dAngle + 360;
	}
	spinDegrees(dAngle);

}

void runToStop(void) {
	steppers.runSpeedToPosition();
}

void setSpeed(int leftSpeed, int rightSpeed) {
	stepperLeft.setSpeed(leftSpeed);
	stepperRight.setSpeed(rightSpeed);
}

void runSpeed(unsigned int ms) {
	int leftPos = stepperLeft.currentPosition();
	int rightPos = stepperRight.currentPosition();
	unsigned long startTime = millis();

	while (millis() < startTime + ms) {
		stepperLeft.runSpeed();
		stepperRight.runSpeed();
	}
	leftPos = stepperLeft.currentPosition() - leftPos;
	rightPos = stepperRight.currentPosition() - rightPos;
	updateRobotPosition(leftPos, rightPos);
}

void randomWander() {
	int randomAngle = random(0, 360);
	spinDegrees((random(0, 1) ? -1 : 1) * randomAngle);
	forward((random(0, 1) ? -1 : 1) * half_rotation);
}

bool goToGoal(int goalX, int goalY) {
	//Get current robot pose
	float robotX = robotPose(0);
	float robotY = robotPose(1);
	float robotAngle = robotPose(2);

	//Get vector towards the current goal
	float goalAngle = atan2((goalY - robotY), (goalX - robotX));
	float robotRadAngle = robotAngle * PI / 180;
	float dAngle = goalAngle - robotRadAngle;

	//Find obstacles and add vectors to avoid them.
	float obstacleX = cos(dAngle);
	float obstacleY = sin(dAngle);
	int multiplier = 1;
	if (bitRead(flag, obFront)) {
		forward(-1 * half_rotation);
		multiplier = 2;
		obstacleX--;
	}
	if (bitRead(flag, obRear)) {
		multiplier = 2;
		obstacleX++;
	}
	if (bitRead(flag, obLeft)) {
		multiplier = 2;
		obstacleY++;
	}
	if (bitRead(flag, obRight)) {
		obstacleY--;
		multiplier = 2;
	}

	//Calculate angle resulting from vector sum.
	float angleInRad = atan2(obstacleY, obstacleX);
	float angleInDeg = angleInRad / PI * 180;
	goToAngle(angleInDeg + robotAngle);
	forward(quarter_rotation * multiplier);

	//Return whether the robot is close enough to the goal.
	return (abs(robotX - goalX) < 0.25 && abs(robotY - goalY) < 0.25);
}

