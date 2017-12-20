/*
 * IRSensor.h
 *
 *  Created on: Dec 20, 2017
 *      Author: Rahul
 */

#pragma once
#ifndef IRSENSOR_H_
#define IRSENSOR_H_

extern float irFrontAvg;  //variable to hold average of current front IR reading
extern float irLeftAvg;   //variable to hold average of current left IR reading
extern float irRearAvg;   //variable to hold average of current rear IR reading
extern float irRightAvg;   //variable to hold average of current right IR reading

extern volatile byte irFlag;    // Flag to hold IR & Sonar data - used to create the state machine

void updateIR();



#endif /* IRSENSOR_H_ */
