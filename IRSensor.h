/*
 * IRSensor.h
 *
 *  Created on: Dec 20, 2017
 *      Author: Rahul
 */

#pragma once
#ifndef IRSENSOR_H_
#define IRSENSOR_H_

//bit definitions for sensor data flag byte
#define obFront   0 // Front IR trip
#define obRear    2 // Rear IR trip
#define obRight   1 // Right IR trip
#define obLeft    3 // Left IR trip

extern float li_cerror;    //left ir current error
extern float ri_cerror;    //right ir current error
extern float derror; //difference between left and right error to center robot in the hallway

extern volatile byte irFlag;    // Flag to hold IR data - used to create the state machine

/**
 * Updates IR sensors and all assocated data.
 */
void updateIR();

#endif /* IRSENSOR_H_ */
