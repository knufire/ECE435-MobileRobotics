/*
 * IRSensor.h
 *
 *  Created on: Dec 20, 2017
 *      Author: Rahul
 */

#pragma once
#ifndef IRSENSOR_H_
#define IRSENSOR_H_

#define irThresh    5 // The IR threshold for presence of an obstacle

extern float irFront;  //variable to hold average of current front IR reading
extern float irLeft;   //variable to hold average of current left IR reading
extern float irRear;   //variable to hold average of current rear IR reading
extern float irRight;   //variable to hold average of current right IR reading

extern int li_cerror;    //left ir current error
extern int ri_cerror;    //right ir current error
extern int derror; //difference between left and right error to center robot in the hallway

extern volatile byte irFlag;    // Flag to hold IR & Sonar data - used to create the state machine

void updateIR();
int updateError();

#endif /* IRSENSOR_H_ */
