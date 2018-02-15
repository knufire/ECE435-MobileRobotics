/*
 * RobotLocalization.h
 *
 *  Created on: Jan 29, 2018
 *      Author: Rahul
 */

#ifndef ROBOTLOCALIZATION_H_
#define ROBOTLOCALIZATION_H_

#define WHEELBASE_LENGTH = 8.5;
#define n 16

struct Cell {
    int x;
    int y;
};

struct Cell cells[n];

//bitDefinitions
#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

void localize();

#endif /* ROBOTLOCALIZATION_H_ */
