/**
 * RobotLocalization.ino
 * 	Author: Rahul Yarlagadda, Ellie Honious
 * 	Date: December 13th, 2017
 *
 * 	This file contains a forward kinematics model of the robot to continuously
 * 	keep track of the robot's pose.
 */

#include "RobotLocalization.h"
#include "RobotDrive.h"

int numPossibilities = 0;
int currentDirection = NORTH;


int wall_map[4][4] = { { 11, 15, 15, 11 },
						{ 8, 5, 5, 2 },
						{ 10, 15, 15, 10 },
						{ 14, 15, 15, 14 } };

int localize() {
	addAllPossibilites();
	while (numPossibilities > 1) {
		Serial.print("Value here: ");
		Serial.println(getCurrentValue());
		updatePossibilties();
		delay(1000);
		Serial.print("Num Possibilities: ");
		Serial.println(numPossibilities);
		if (numPossibilities <= 1) break;
		int ob = getCurrentValue();
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
	for (int i = 0; i < n; i++) {
		if (cells[i].x != -1) {
			turnTo(NORTH);
			return 10*cells[i].x + cells[i].y;
		}
	}
	turnTo(NORTH);
	return 0;


}

void moveNorth() {
	Serial.println("Moving north.");
	turnTo(NORTH);
	forward(1360);
	for (int i = 0; i < n; i++) {
		cells[i].y = cells[i].y - 1;
	}
}

void moveSouth() {
	Serial.println("Moving south.");
	turnTo(SOUTH);
	forward(1360);
	for (int i = 0; i < n; i++) {;
		cells[i].y = cells[i].y + 1;
	}
}

void moveEast() {
	Serial.println("Moving east.");
	turnTo(EAST);
	forward(1360);
	for (int i = 0; i < n; i++) {
		cells[i].x = cells[i].x + 1;
	}
}

void moveWest() {
	Serial.println("Moving west.");
	turnTo(WEST);
	forward(1360);
	for (int i = 0; i < n; i++) {
		cells[i].x = cells[i].x - 1;
	}
}

void turnTo(int dir) {
	switch (currentDirection) {
	case (NORTH):
		switch (dir) {
		case (NORTH):
			break;
		case (SOUTH):
			spinDegrees(180);
			break;
			;
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

void updatePossibilties() {
	numPossibilities = 0;
	Serial.println("------POSSIBILITIES-------");
	for (int i = 0; i < n; i++) {
		int valueAtCell = wall_map[cells[i].y][cells[i].x];
		if (valueAtCell == getCurrentValue()) {
			Serial.print("(");
			Serial.print(cells[i].x);
			Serial.print(",");
			Serial.print(cells[i].y);
			Serial.println(")");
			numPossibilities++;
		} else {
			cells[i].x = -1;
			cells[i].y = -1;
		}
	}
}

int getCurrentValue() {
	//Find north bit
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

