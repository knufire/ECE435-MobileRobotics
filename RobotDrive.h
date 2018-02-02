/**
 * RobotDrive.h
 * 	Author: Rahul Yarlagadda, Ellie Honious
 * 	Date: December 13th, 2017
 *
 * 	This file contains the basic motor control and odometry code for the robot. Basic functions for driving forward, reverse
 * 	and going to an angle are written.
 */

#pragma once
#ifndef ROBOTDRIVE_H_
#define ROBOTDRIVE_H_

/**
 * These constants define kinematic traits of the robot, to be used by the AccelStepper library.
 */
#define robot_spd 	400 	//Desired robot speed in steps/s
#define max_accel 	5000	//Maximum robot acceleration in steps/s^2
#define max_spd 	1000	//Maximum robot speed in steps/s

/**
 * These constants are the number of motor steps for the specified number of rotations.
 */
#define quarter_rotation 	200
#define half_rotation 		400
#define one_rotation  		800
#define two_rotation  		1600
#define three_rotation 		2400
#define four_rotation 		3200
#define five_rotation 		4000

/**
 * These are experimentally defined constants to convert motor steps to real-world units.
 */
#define CONST_FEET_TO_STEPS 880.0
#define CONST_SPIN_DEGREES_TO_STEPS 5.635

/**
 * Contains the initalizaiton routines for the stepper motor drivers and the AccelStepper library.
 * This should be called in the init() function in the main program file.
 */
void driveSetup();

/**
 * Moves the robot forward by the specified number of steps.
 */
void forward(int rot);

/**
 * Pivots the robot by moving one wheel for the specified number of steps.
 * Pivots left if dir = 0, pivots right otherwise.
 */
void pivot(int rot, int dir);

/**
 * Spins the robot using both wheels by the specified number of steps.
 * Positive number of steps spins the robot clockwise.
 */
void spin(int rot);
/**
 * Spins the robot by the specified number of degrees. Utilizes spin().
 */
void spinDegrees(float degrees);

/**
 * Stops both stepper motors.
 */
void stop();

/**
 * Moves the robot backwards by the specified number of steps.
 */
void reverse(int rot);

/**
 * Moves the robot to a specified angle. The robot's angle is 0 during startup.
 */
void goToAngle(float degrees);

/**
 * Blocks program execution until stepper movements have been completed.
 */
void runToStop();

/**
 * Performs a random wander manuever. Robot will spin to a random angle and drive forward.
 */
void randomWander();

/**
 * Sets the speed of both wheels.
 */
void setSpeed(int leftSpeed, int rightSpeed);

/**
 * Runs both wheels at constant speed. Blocks for the specified number of milliseconds.
 */
void runSpeed(unsigned int ms);

/**
 * Drives towards a goal location using odometry.
 */
bool goToGoal(int goalX, int goalY);

#endif /* ROBOTDRIVE_H_ */
