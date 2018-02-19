/**
 * RobotLocalization.ino
 * 	Author: Rahul Yarlagadda, Ellie Honious
 * 	Date: December 13th, 2017
 *
 * 	This file contains the code necessary for the robot to localize itself within the world.
 */

#include "RobotLocalization.h"
#include "RobotDrive.h"

int numPossibilities = 0;
int currentDirection = NORTH;

//Hardcoded map of the world.
int wall_map[4][4] = { { 11, 15, 15, 11 },
						{ 8, 5, 5, 2 },
						{ 10, 15, 15, 10 },
						{ 14, 15, 15, 14 } };
/**
 * Localizes the robot by wandering around the world. Needs a map of the world.
 */
int localize() {
	//Add all locations in the world to the list of possibilties.
	addAllPossibilites();

	//Try to localize
	while (numPossibilities > 1) {

		//Update the list of possibilites based on the walls at the robot's current location.
		updatePossibilties();

		//If there's 1 or 0 possibities left, stop trying to localize.
		if (numPossibilities <= 1) break;

		//Get the obstacles around the robot
		int ob = getCurrentValue();

		//Find a direction to move so we can keep trying to localize
		if (!bitRead(ob, NORTH)) {
			moveNorth();
		} else if (!bitRead(ob, EAST)) {
			moveEast();
		} else if (!bitRead(ob, WEST)) {
			moveWest();
		} else if (!bitRead(ob, SOUTH)) {
			moveSouth();
		}
	}

	//---------------DONE LOCALIZING----------

	//Search through all the cells to find the one remaining possibilty.
	for (int i = 0; i < n; i++) {
		if (cells[i].x != -1) {
			//Reset the robot's orientation north
			turnTo(NORTH);
			//Tu
			return 10*cells[i].x + cells[i].y;
		}
	}
	//Reset the robot's orientation north
	turnTo(NORTH);
	//Return (0,0) since there are no remaining possibilties.
	return 0;


}

/**
 * Move the robot one cell north. Also update the list of possible
 * robot locations to also be one cell north.
 */
void moveNorth() {
	Serial.println("Moving north.");
	//Turn north
	turnTo(NORTH);

	//Drive forward one cell
	forward(1360);

	//Move all possibile locations one cell east.
	for (int i = 0; i < n; i++) {
		cells[i].y = cells[i].y - 1;
	}
}

/**
 * Move the robot one cell south. Also update the list of possible
 * robot locations to also be one cell west.
 */
void moveSouth() {
	Serial.println("Moving south.");
	//Turn south
	turnTo(SOUTH);
	//Drive forward one cell.
	forward(1360);
	//Move all possibile locations one cell east.
	for (int i = 0; i < n; i++) {;
		cells[i].y = cells[i].y + 1;
	}
}

/**
 * Move the robot one cell east. Also update the list of possible
 * robot locations to also be one cell east.
 */
void moveEast() {
	Serial.println("Moving east.");
	//Turn east
	turnTo(EAST);

	//Drive forward one cell.
	forward(1360);

	//Move all possibile locations one cell east.
	for (int i = 0; i < n; i++) {
		cells[i].x = cells[i].x + 1;
	}
}

/**
 * Move the robot one cell west. Also update the list of possible
 * robot locations to also be one cell west.
 */
void moveWest() {
	Serial.println("Moving west.");
	//Turn west
	turnTo(WEST);

	//Drive forward one cell.
	forward(1360);

	//Move all possibile locations one cell west.
	for (int i = 0; i < n; i++) {
		cells[i].x = cells[i].x - 1;
	}
}

/**
 * Spins to the given direction. Just figures out the degrees difference between
 * two directions.
 */
void turnTo(int dir) {
	switch (currentDirection) {
	case (NORTH):
		switch (dir) {
		case (NORTH):
			break;
		case (SOUTH):
			spinDegrees(180);
			break;
		case (EAST):
			spinDegrees(-90);
			break;
		case (WEST):
			spinDegrees(90);
			break;
		}
		break;
	case (SOUTH):
		switch (dir) {
		case (NORTH):
			spinDegrees(180);
			break;
		case (SOUTH):
			break;
		case (EAST):
			spinDegrees(90);
			break;
		case (WEST):
			spinDegrees(-90);
			break;
		}
		break;
	case (EAST):
		switch (dir) {
		case (NORTH):
			spinDegrees(90);
			break;
		case (SOUTH):
			spinDegrees(-90);
			break;
		case (EAST):
			break;
		case (WEST):
			spinDegrees(180);
			break;
		}
		break;
	case (WEST):
		switch (dir) {
		case (NORTH):
			spinDegrees(90);
			break;
		case (SOUTH):
			spinDegrees(-90);
			break;
		case (EAST):
			spinDegrees(180);
			break;
		case (WEST):
			break;
		}
		break;
	}
	currentDirection = dir;
}

/**
 * Based on the value of the robot's current location, remove all the cells from
 * the list of possibilities who's values do not match. Also updates the global variable containing the
 * remaining number of possible locations.
 */
void updatePossibilties() {
	numPossibilities = 0;

	//Search through the list of possibitlies.
	for (int i = 0; i < n; i++) {
		//For this possiblity, get the value of the cell from the world map.
		int valueAtCell = wall_map[cells[i].y][cells[i].x];

		//If the value of the robot's current location matches the value from the map
		if (valueAtCell == getCurrentValue()) {
			numPossibilities++; //Increase the number of remaining possibities.
		} else { //Otherwise, set the cell to (-1,-1) so we know it's not a possibility anymore.
			cells[i].x = -1;
			cells[i].y = -1;
		}
	}
}

/**
 * Gets the current value of the cell the robot is in based on the IR sensors.
 * This factors in the direction the robot is facing to correctly identify which wall
 * is which.
 */
int getCurrentValue() {
	int value = 0;
	switch (currentDirection) {
	case (NORTH):
		value = flag;
		break;
	case (SOUTH):
		bitWrite(value, NORTH, bitRead(flag, obRear));
		bitWrite(value, SOUTH, bitRead(flag, obFront));
		bitWrite(value, EAST, bitRead(flag, obLeft));
		bitWrite(value, WEST, bitRead(flag, obRight));
		break;
	case (EAST):
		bitWrite(value, NORTH, bitRead(flag, obLeft));
		bitWrite(value, SOUTH, bitRead(flag, obRight));
		bitWrite(value, EAST, bitRead(flag, obFront));
		bitWrite(value, WEST, bitRead(flag, obRear));
		break;
	case (WEST):
		bitWrite(value, NORTH, bitRead(flag, obRight));
		bitWrite(value, SOUTH, bitRead(flag, obLeft));
		bitWrite(value, EAST, bitRead(flag, obRear));
		bitWrite(value, WEST, bitRead(flag, obFront));
		break;
	}
	return value;
}

/**
 * Add a cell for every location in the world to the list of possibilties. This needs to be done
 * when the robot starts localizing.
 */
void addAllPossibilites() {
	numPossibilities = 0;
	for (int y = 0; y < 4; y++) {
		for (int x = 0; x < 4; x++) {
			cells[numPossibilities].x = x;
			cells[numPossibilities].y = y;
			numPossibilities++;
		}
	}
}

