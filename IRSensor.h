/*
 * IRSensor.h
 *
 *  Created on: Dec 20, 2017
 *      Author: Rahul
 */

#pragma once
#ifndef IRSENSOR_H_
#define IRSENSOR_H_

extern float irFront;  //variable to hold average of current front IR reading
extern float irLeft;   //variable to hold average of current left IR reading
extern float irRear;   //variable to hold average of current rear IR reading
extern float irRight;   //variable to hold average of current right IR reading

extern volatile byte irFlag;    // Flag to hold IR & Sonar data - used to create the state machine

void updateIR();



#endif /* IRSENSOR_H_ */
