/*
 * MetricPathPlanning.h
 *
 *  Created on: Feb 9, 2018
 *      Author: Rahul
 */


#pragma once
#ifndef METRICPATHPLANNING_H_
#define METRICPATHPLANNING_H_

#include "Arduino.h"

#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

String getDirectionsToGoal(int startX, int startY, int goalX, int goalY);

#endif /* METRICPATHPLANNING_H_ */
