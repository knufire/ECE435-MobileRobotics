/*	RobotDrive.ino
 * 	Author: Rahul Yarlagadda, Ellie Honious
 * 	Date: December 13th, 2017
 *
 * 	This file contains the basic motor control and odometry code for the robot. Basic functions for driving forward, reverse
 * 	and going to an angle are written. Additionally, the robot uses encoders to keep track of it's position and poise relative
 * 	to it's starting location.
 */

#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library
#include "RobotDrive.h"
#include "PinDefinitions.h"

float robotX = 0;
float robotY = 0;
float robotAngle = 0;

AccelStepper stepperRight(AccelStepper::DRIVER, PIN_RT_STEP, PIN_RT_DIR);//create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, PIN_LT_STEP, PIN_LT_DIR);//create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;//create instance to control multiple steppers at the same time

void driveSetup() {
	//stepper Motor set up
	Serial.println("Setting up drive");

	pinMode(PIN_RT_STEP, OUTPUT);//sets pin as outputr
	pinMode(PIN_RT_DIR, OUTPUT);//sets pin as output
	pinMode(PIN_LT_STEP, OUTPUT);//sets pin as output
	pinMode(PIN_LT_DIR, OUTPUT);//sets pin as output
	pinMode(PIN_STEP_ENABLE, OUTPUT);//sets pin as output

	digitalWrite(PIN_STEP_ENABLE, false);//turns on the stepper motor drivers

	stepperRight.setMaxSpeed(max_spd);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
	stepperRight.setAcceleration(max_accel);//set desired acceleration in steps/s^2
	stepperLeft.setMaxSpeed(max_spd);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
	stepperLeft.setAcceleration(max_accel);//set desired acceleration in steps/s^2

	stepperRight.setSpeed(robot_spd);//set right motor speed
	stepperLeft.setSpeed(robot_spd);//set left motor speed

	steppers.addStepper(stepperRight);//add right motor to MultiStepper
	steppers.addStepper(stepperLeft);//add left motor to MultiStepper

}

void forward(int rot) {
	stepperRight.move(rot);  //move right motor to position
	stepperLeft.move(rot);   //move left motor to position
	runToStop();//run until the robot reaches the target
//	float dx = cos(robotAngle * PI / 180) * rot / CONST_FEET_TO_STEPS;
//	float dy = sin(robotAngle * PI / 180) * rot / CONST_FEET_TO_STEPS;
//	robotX = robotX + dx;
//	robotY = robotY + dy;
}

/*robot pivot function */
void pivot(int rot, int dir) {
  if (dir > 0) {//pivot right
	  stepperLeft.move(rot);
  }
  else//pivot left
  {
    stepperRight.move(rot);
  }
  runToStop();                       //run until the robot reaches the target
}

/*robot spin function */
void spin(int rot, int dir) {
  if (dir > 0) {//spin right
	 stepperRight.move(-rot);
     stepperLeft.move(rot);
  }
  else//spin left
  {
	 stepperRight.move(-rot);
	 stepperLeft.move(rot);
  }
  runToStop();                        //run until the robot reaches the target
}

void stop() {
	stepperRight.stop();
	stepperLeft.stop();
}

/*robot move reverse function */
void reverse(int rot) {
	forward(-rot);
}

void spinDegrees(float degrees) {
	goToAngle(degrees + robotAngle);
}

void goToAngle(float degrees) {
	float dAngle = degrees - robotAngle;
	while(dAngle > 180){
		dAngle = dAngle - 360;
	}
	while(dAngle < -180){
		dAngle = dAngle + 360;
	}
	long numSteps = dAngle * CONST_SPIN_DEGREES_TO_STEPS;
	stepperRight.move(numSteps);
	stepperLeft.move(numSteps * -1);
	runToStop();
	//TODO: Reimplement navigation
//	while (degrees < 0) {
//		degrees += 360;
//	}
//	while (degrees > 360) {
//		degrees -= 360;
//	}
//	robotAngle = degrees;
}

/*This function, runToStop(), will run the robot until the target is achieved and
   then stop it
 */
void runToStop ( void ) {
	steppers.runSpeedToPosition();
}

void runAtSpeed(float leftSpeed, float rightSpeed) {
	stepperLeft.setSpeed(leftSpeed);
	stepperRight.setSpeed(rightSpeed);
	stepperLeft.runSpeed();
	stepperRight.runSpeed();
}
