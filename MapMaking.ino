/*
 * MapMaking.ino
 *
 *  Created on: Feb 9, 2018
 *      Author: Rahul
 */

#include "WirelessReceiver.h"
#include "RobotDrive.h"
#include "RobotLocalization.h"

//Matrix to hold the map of the world
int working_map[4][4] = { { 0, 50, 50, 50 },
				  	  	  { 50, 50, 50, 50 },
						  { 50, 50, 50, 50 },
						  { 50, 50, 50, 50 } };

//Matrix to hold last direction we left the cell in
int priority_map[4][4] = {{ 50, 50, 50, 50 },
				  	  	  { 50, 50, 50, 50 },
						  { 50, 50, 50, 50 },
						  { 50, 50, 50, 50 } };

int currX = 0;
int currY = 0;

/**
 * Make a map of the world.
 */
void makeMap() {
	while (!completedMap()) {
		mapMakingStep();
		setUnaccessableLocations();
		delay(1000);
	}
	printWorkingMap();

}

/**
 * Check if any of the corners are surrounded by 99s. If they are, set the corner to 99,
 * since we know it's unereachable.
 */
void setUnaccessableLocations() {
	if (working_map[1][0] == 99 && working_map[0][1] == 99) working_map[0][0] = 99;
	if (working_map[2][0] == 99 && working_map[3][1] == 99) working_map[3][0] = 99;
	if (working_map[3][2] == 99 && working_map[2][3] == 99) working_map[3][3] = 99;
	if (working_map[0][2] == 99 && working_map[1][3] == 99) working_map[0][3] = 99;
}

/**
 * Check if the map is complete by seeing that there's is a value at every location.
 */
bool completedMap() {
	for (int y = 0; y < 4; y++) {
		for (int x = 0; x < 4; x++) {
			if (working_map[y][x] == 50) return false;
		}
	}
	wirelessSend("Map complete!");
	delay(100);
	return true;
}

/**
 * Run this continuously until the map is filled to create a map.
 */
void mapMakingStep() {
	wirelessSend("\nAt: (" + String(currX) + "," + String(currY) +")\t");
	delay(100);

	//Map current cell
	mapCurrentCell();

	//Find neighbor with lowest priotiy;
	int lowestPriority = 100;
	int directionToGo = 5;
	if (getPriority(currX, currY+1) < lowestPriority) {
		lowestPriority = getPriority(currX, currY+1);
		directionToGo = SOUTH;
	}
	if (getPriority(currX, currY-1) < lowestPriority) {
		lowestPriority = getPriority(currX, currY-1);
		directionToGo = NORTH;
	}
	if (getPriority(currX+1, currY) < lowestPriority) {
		lowestPriority = getPriority(currX+1, currY);
		directionToGo = EAST;
	}
	if (getPriority(currX-1, currY) < lowestPriority) {
		lowestPriority = getPriority(currX-1, currY);
		directionToGo = WEST;
	}

	//Go to the neighbor with the lowest priority. Update the priority_map with the direction that we're leaving this cell in.
	//If we're backtracking, set this cell in the priotity_map to a 5 instead of a direction. Also update the robot's current location.
	switch(directionToGo) {
	case (NORTH):
		priority_map[currY][currX] = 1; //Set this cell's priority to north, since we are leaving it going north.
		if (lowestPriority > 0) priority_map[currY][currX] = 5; //If we're backtracking, set the priority to 5, since we don't want to check any neighboring cells again.
		moveNorth();
		currY--; //Update robot's current position.
		break;
	case (SOUTH):
		priority_map[currY][currX] = 3; //Set this cell's priority to south, since we are leaving it going south.
		if (lowestPriority > 0) priority_map[currY][currX] = 5;  //If we're backtracking, set the priority to 5, since we don't want to check any neighboring cells again.
		moveSouth();
		currY++; //Update robot's current position.
		break;
	case (EAST):
		priority_map[currY][currX] = 4; //Set this cell's priority to east, since we are leaving it going east.
		if (lowestPriority > 0) priority_map[currY][currX] = 5;  //If we're backtracking, set the priority to 5, since we don't want to check any neighboring cells again.
		moveEast();
		currX++; //Update robot's current position.
		break;
	case (WEST):
		priority_map[currY][currX] = 2; //Set this cell's priority to west, since we are leaving it going west.
		if (lowestPriority > 0) priority_map[currY][currX] = 5;  //If we're backtracking, set the priority to 5, since we don't want to check any neighboring cells again.
		moveWest();
		currX--; //Update robot's current position.
		break;
	default:
		wirelessSend("ERROR: NO VALID NEIGHBOR");
		break;
	}

}

/**
 * Get the value of (x,y) from the priority map.
 */
int getPriority(int x, int y) {
	if (x >= 0 && x < 4 && y >= 0 && y < 4) {
		return priority_map[y][x];
	}
	return 100;
}

/**
 * Based on the robot's sensors and current location, check if any of the neighbors are open or closed.
 */
void mapCurrentCell() {
	//Get walls in each direction
	int northVal, southVal, eastVal, westVal;
	switch(currentDirection) {
	case (NORTH):
		northVal = bitRead(flag, obFront);
		southVal = bitRead(flag, obRear);
		eastVal = bitRead(flag, obRight);
		westVal = bitRead(flag, obLeft);
		break;
	case (SOUTH):
		northVal = bitRead(flag, obRear);
		southVal = bitRead(flag, obFront);
		eastVal = bitRead(flag, obLeft);
		westVal = bitRead(flag, obRight);
		break;
	case (EAST):
		northVal = bitRead(flag, obLeft);
		southVal = bitRead(flag, obRight);
		eastVal = bitRead(flag, obFront);
		westVal = bitRead(flag, obRear);
		break;
	case (WEST):
		northVal = bitRead(flag, obRight);
		southVal = bitRead(flag, obLeft);
		eastVal = bitRead(flag, obRear);
		westVal = bitRead(flag, obFront);
		break;
	}

	//Map all four neighbors.
	mapCell(currX, currY-1, northVal);
	mapCell(currX, currY+1, southVal);
	mapCell(currX+1, currY, eastVal);
	mapCell(currX-1, currY, westVal);

	//Send both maps to the
	printWorkingMap();
	wirelessSend("\n");
	printPriorityMap();


}

/**
 * Update both maps with
 */
void mapCell(int x, int y, int val) {
	if (x >= 0 && x < 4 && y >= 0 && y < 4) { //Cell is in bounds
		if (working_map[y][x] == 50) { //Cell isn't already mapped
			//Set working and priority map to 0 or 99.
			working_map[y][x] = val*99;
			priority_map[y][x] = val; //0 = unvisited, 99 = can't get to it, don't ever visit.
		}
	}
}

/**
 * Send working map over wireless interface.
 */
void printWorkingMap() {
	for (int y = 0; y < 4; y++) {
		String output = "";
		for (int x = 0; x < 4; x++) {
			output += (working_map[y][x]);
			output += "\t";
		}
		wirelessSend(output);
		delay(100);
	}
	delay(100);
}

/**
 * Send priority map over wireless interface.
 */
void printPriorityMap() {
	for (int y = 0; y < 4; y++) {
		String output = "";
		for (int x = 0; x < 4; x++) {
			output += (priority_map[y][x]);
			output += "\t";
		}
		wirelessSend(output);
		delay(100);
	}
	delay(100);
}
