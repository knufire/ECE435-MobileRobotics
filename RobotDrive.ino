/*	RobotDrive.ino
 * 	Author: Rahul Yarlagadda, Ellie Honious
 * 	Date: December 13th, 2017
 *
 * 	This file defines all hardware connections on the robot, so pin assignments can be refered to
 *  by name instead of by pin number
 * */

#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library
#include "RobotDrive.h"

float robotX = 0;
float robotY = 0;
float robotAngle = 0;

AccelStepper stepperRight(AccelStepper::DRIVER, PIN_RT_STEP, PIN_RT_DIR);//create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, PIN_LT_STEP, PIN_LT_DIR);//create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;//create instance to control multiple steppers at the same time

void driveSetup() {
	//stepper Motor set up
	pinMode(PIN_RT_STEP, OUTPUT);//sets pin as outputr
	pinMode(PIN_RT_DIR, OUTPUT);//sets pin as output
	pinMode(PIN_LT_STEP, OUTPUT);//sets pin as output
	pinMode(PIN_LT_DIR, OUTPUT);//sets pin as output
	pinMode(PIN_STEP_ENABLE, OUTPUT);//sets pin as output

	digitalWrite(PIN_STEP_ENABLE, HIGH);//turns on the stepper motor driver

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
	long positions[2]; // Array of desired stepper positions
	stepperRight.setCurrentPosition(0);
	stepperLeft.setCurrentPosition(0);
	positions[0] = stepperRight.currentPosition() + rot;  //right motor absolute position
	positions[1] = stepperLeft.currentPosition() + rot;   //left motor absolute position
	stepperRight.move(positions[0]);  //move right motor to position
	stepperLeft.move(positions[1]);   //move left motor to position
	Serial.print("Going forward: ");
	Serial.println(rot);
	runToStop();//run until the robot reaches the target
	float dx = cos(robotAngle * PI / 180) * rot / CONST_FEET_TO_STEPS;
	float dy = sin(robotAngle * PI / 180) * rot / CONST_FEET_TO_STEPS;
	robotX = robotX + dx;
	robotY = robotY + dy;
	Serial.print("Robot X: ");
	Serial.print(robotX);
	Serial.print("\t Robot Y: ");
	Serial.print(robotY);
	Serial.print("\t Robot T: ");
	Serial.println(robotAngle);
}

void stop() {
	stepperRight.stop();
	stepperLeft.stop();
}

void goToAngle(float degrees) {
	Serial.print("Going to Angle: ");
	Serial.print(degrees);
	Serial.print("\n");
	float dAngle = degrees - robotAngle;
	while(dAngle > 180){
		dAngle = dAngle - 360;
	}
	while(dAngle < -180){
		dAngle = dAngle + 360;
	}
	//  Serial.print("delta Angle: ");
	//  Serial.print(dAngle);
	//  Serial.print("\n");
	long numSteps = dAngle * CONST_SPIN_DEGREES_TO_STEPS;
	stepperRight.move(numSteps);
	stepperLeft.move(numSteps * -1);
	runToStop();
	while (degrees < 0) {
		degrees += 360;
	}
	while (degrees > 360) {
		degrees -= 360;
	}
	robotAngle = degrees;
}

/*This function, runToStop(), will run the robot until the target is achieved and
   then stop it
 */
void runToStop ( void ) {
	int runNow = 1;
	while (runNow) {
		if (!stepperRight.run() ) {
			runNow = 0;
			stop();
		}
		if (!stepperLeft.run()) {
			runNow = 0;
			stop();
		}
	}
}
