/*
 * SonarSensor.h
 *
 *  Created on: Dec 20, 2017
 *      Author: Rahul

 */
#pragma once
#ifndef SONARSENSOR_H_
#define SONARSENSOR_H_

#define snrThresh   5   // The sonar threshold for presence of an obstacle in inches

extern unsigned int srLeft;   //variable to hold the crrent left sonar reading
extern unsigned int srRight;  //variable to hold the current right sonar reading
extern volatile byte snrFlag;    // Flag to hold whether a sonar sensor sees a obstacle

#endif /* SONARSENSOR_H_ */
