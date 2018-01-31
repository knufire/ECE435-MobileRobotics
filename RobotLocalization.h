/*
 * RobotLocalization.h
 *
 *  Created on: Jan 29, 2018
 *      Author: Rahul
 */

#ifndef ROBOTLOCALIZATION_H_
#define ROBOTLOCALIZATION_H_

#include <BasicLinearAlgebra.h>

using namespace BLA;

extern BLA::Matrix<3> robotPose;

#define WHEELBASE_LENGTH = 8.5;

/**
 * Updates the robot pose based on the number of steps each wheel has spun.
 */
void updateRobotPosition(int leftSteps, int rightSteps);

#endif /* ROBOTLOCALIZATION_H_ */
