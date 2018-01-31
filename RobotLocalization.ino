/**
 * RobotLocalization.ino
 * 	Author: Rahul Yarlagadda, Ellie Honious
 * 	Date: December 13th, 2017
 *
 * 	This file contains a forward kinematics model of the robot to continuously
 * 	keep track of the robot's pose.
 */

#include "RobotLocalization.h"
#include "RobotDrive.h"
#include <BasicLinearAlgebra.h>

using namespace BLA;

Matrix<3> robotPose;

/**
 * Updates the robot pose based on the number of steps each wheel has spun.
 */
void updateRobotPosition(int leftSteps, int rightSteps) {
	float vl = leftSteps / CONST_FEET_TO_STEPS;
	float vr = rightSteps / CONST_FEET_TO_STEPS;

	Serial.print(vl);
	Serial.print("\t");
	Serial.print(vr);

	//Edge case: robot isn't moving
	if (vl == 0 && vr == 0) {
		//Do nothing
	}

	//Edge case: robot is moving straight
	else if (fabs(vl - vr) < 0.01) {
		updatePositionStraight(vl, vr);
	}

	//Edge case: robot is spinning
	else if (fabs(fabs(vl) - fabs(vr)) < 0.01) {
		updatePostionSpin(vl, vr);
		return;
	}

	//No edge cases, use forward kinematics
	else {
		updatePositionGeneral(vl, vr);
	}

	clampAngle();
	printPose();
}

/**
 * Update the robot pose using the specified left and right wheel velocities.
 * Assumes the robot is driving in a straight line.
 */
void updatePositionStraight(float vl, float vr) {
	float dx = vl * cos(robotPose(2));
	float dy = vl * sin(robotPose(2));
	robotPose(0) = robotPose(0) + dx;
	robotPose(1) = robotPose(1) + dy;
}

/**
 * Update the robot pose using the specified left and right wheel velocities.
 * Assuems the robot is performing a spin about the center of the robot.
 */
void updatePostionSpin(float vl, float vr) {
	float w = (vr - vl) / 2;
	robotPose(2) = robotPose(2) + w;
}

/**
 * Update the robot pose using the specified left and right wheel velocities.
 * Uses a differential drive foward kinematic model.
 */
void updatePositionGeneral(float vl, float vr) {
	//Calculate angular velocity
	float w = (vr - vl) / 2;

	//Calculate radius of curvature
	float r = ((8.5 / 12) / 2) * (vl + vr) / (vr - vl);

	//Get current pose
	float x = robotPose(0);
	float y = robotPose(1);
	float theta = robotPose(2);

	//Find coordinates of instantaneous center of rotation
	float ICCx = x - r * sin(theta);
	float ICCy = y + r * cos(theta);

	//Calculate new pose
	Matrix<3, 3> M;
	M << cos(w), -sin(w), 0,
		 sin(w), cos(w), 0,
		 0, 0, 1;

	Matrix<3> X;
	X << x - ICCx,
		 y - ICCy,
		 theta;

	Matrix<3> B;
	B << ICCx,
		 ICCy,
		 w;

	robotPose = M * X + B;
}

/**
 * Ensure the robot angle is within (0,2*PI).
 */
void clampAngle() {
	while (robotPose(2) > 2 * PI) {
		robotPose(2) = robotPose(2) - 2 * PI;
	}
	while (robotPose(2) < 0) {
		robotPose(2) = robotPose(2) + 2 * PI;
	}
}

/**
 * Print the current robot pose to the serial monitor.
 */
void printPose() {
	Serial.print("X: ");
	Serial.print(robotPose(0));
	Serial.print("\tY: ");
	Serial.print(robotPose(1));
	Serial.print("\tT: ");
	Serial.println(robotPose(2) / PI * 180);
}

