/*
 * RobotDrive.h
 *
 *  Created on: Dec 20, 2017
 *      Author: Rahul
 */

#pragma once
#ifndef ROBOTDRIVE_H_
#define ROBOTDRIVE_H_

#define robot_spd 800 //set robot speed
#define max_accel 5000//maximum robot acceleration
#define max_spd 1000//maximum robot speed

#define quarter_rotation 200  //stepper quarter rotation
#define half_rotation 400     //stepper half rotation
#define one_rotation  800     //stepper motor runs in 1/4 steps so 800 steps is one full rotation
#define two_rotation  1600    //stepper motor 2 rotations
#define three_rotation 2400   //stepper rotation 3 rotations
#define four_rotation 3200    //stepper rotation 3 rotations
#define five_rotation 4000    //stepper rotation 3 rotations

#define CONST_FEET_TO_STEPS 912.2
#define CONST_SPIN_DEGREES_TO_STEPS 5.35

extern float robotX;
extern float robotY;
extern float robotAngle;

void driveSetup();
void forward();
void stop();
void goToAngle();
void runToStop();




#endif /* ROBOTDRIVE_H_ */
