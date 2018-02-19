/**
 * RobotLocalization.h
 * 	Author: Rahul Yarlagadda, Ellie Honious
 * 	Date: December 13th, 2017
 *
 * 	This file contains the code necessary for the robot to localize itself within the world.
 */

#ifndef ROBOTLOCALIZATION_H_
#define ROBOTLOCALIZATION_H_

#define WHEELBASE_LENGTH = 8.5;

//Number of cells in the world.
#define n 16

//Enumeration of directions
#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

//Current direction the robot is facing.
extern int currentDirection;

//A data structure to hold a (x,y) position.
struct Cell {
    int x;
    int y;
};

//A list of positions used for keeping track of possible locations while localizing
struct Cell cells[n];

//Functions needed outside this file.
int localize();
void moveNorth();
void moveSouth();
void moveEast();
void moveWest();

#endif /* ROBOTLOCALIZATION_H_ */
