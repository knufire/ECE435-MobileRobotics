/*
 * SonarSensor.h
 *
 *  Created on: Dec 20, 2017
 *      Author: Rahul
 *
 */
#pragma once
#ifndef SONARSENSOR_H_
#define SONARSENSOR_H_

#define snrThresh   5   // The sonar threshold for presence of an obstacle

extern unsigned int srLeft;   //variable to hold average of left sonar current reading
extern unsigned int srRight;  //variable to hold average or right sonar current reading
extern volatile byte snrFlag;    // Flag to hold IR & Sonar data - used to create the state machine




#endif /* SONARSENSOR_H_ */
