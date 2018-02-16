/*
 * MapMaking.ino
 *
 *  Created on: Feb 9, 2018
 *      Author: Rahul
 */

#include "WirelessReceiver.h"
#include "RobotDrive.h"
#include "RobotLocalization.h"

int working_map[4][4] = { { 0, 50, 50, 50 },
				  	  	  { 50, 50, 50, 50 },
						  { 50, 50, 50, 50 },
						  { 50, 50, 50, 50 } };

int priority_map[4][4] = {{ 50, 50, 50, 50 },
				  	  	  { 50, 50, 50, 50 },
						  { 50, 50, 50, 50 },
						  { 50, 50, 50, 50 } };

int currX = 0;
int currY = 0;

void makeMap() {
	while (!completedMap()) {
		mapMakingStep();
		setUnaccessableLocations();
		delay(1000);
	}
	printWorkingMap();

}

void setUnaccessableLocations() {
	if (working_map[1][0] == 99 && working_map[0][1] == 99) working_map[0][0] = 99;
	if (working_map[2][0] == 99 && working_map[3][1] == 99) working_map[3][0] = 99;
	if (working_map[3][2] == 99 && working_map[2][3] == 99) working_map[3][3] = 99;
	if (working_map[0][2] == 99 && working_map[1][3] == 99) working_map[0][3] = 99;
}

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
	//Check if we're backtracking, update priority accordingly.
	//Set the new priority of this cell.

	switch(directionToGo) {
	case (NORTH):
		priority_map[currY][currX] = 1;
		if (lowestPriority > 0) priority_map[currY][currX] = 5;
		moveNorth();
		currY--;
		break;
	case (SOUTH):
		priority_map[currY][currX] = 3;
		if (lowestPriority > 0) priority_map[currY][currX] = 5;
		moveSouth();
		currY++;
		break;
	case (EAST):
		priority_map[currY][currX] = 4;
		if (lowestPriority > 0) priority_map[currY][currX] = 5;
		moveEast();
		currX++;
		break;
	case (WEST):
		priority_map[currY][currX] = 2;
		if (lowestPriority > 0) priority_map[currY][currX] = 5;
		moveWest();
		currX--;
		break;
	default:
		wirelessSend("ERROR: NO VALID NEIGHBOR");
		break;
	}

}

int getPriority(int x, int y) {
	if (x >= 0 && x < 4 && y >= 0 && y < 4) {
		return priority_map[y][x];
	}
	return 100;
}


void mapCurrentCell() {
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
	mapCell(currX, currY-1, northVal);
	mapCell(currX, currY+1, southVal);
	mapCell(currX+1, currY, eastVal);
	mapCell(currX-1, currY, westVal);
	printWorkingMap();
	wirelessSend("\n");
	printPriorityMap();


}

void mapCell(int x, int y, int val) {
	if (x >= 0 && x < 4 && y >= 0 && y < 4) { //Cell is in bounds
		if (working_map[y][x] == 50) { //Cell isn't already mapped
			working_map[y][x] = val*99;
			priority_map[y][x] = working_map[y][x];
		}
	}
}

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
