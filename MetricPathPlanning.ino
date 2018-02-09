/*
 * MetricPathPlanning.c
 *
 *  Created on: Feb 9, 2018
 *      Author: Rahul
 */

#include "MetricPathPlanning.h"

int world_map[4][4] = { { 0, 99, 99, 0 },
						{ 0, 0,  0,  0 },
						{ 0, 99, 99, 0 },
						{ 0, 99, 99, 0 } };

String getDirectionsToGoal(int startX, int startY, int goalX, int goalY) {
	Serial.print("Start = (");
	Serial.print(startX);
	Serial.print(",");
	Serial.print(startY);
	Serial.print(")\t");
	Serial.print("End = (");
	Serial.print(goalX);
	Serial.print(",");
	Serial.print(goalY);
	Serial.println(")");
	populateMap(startX, startY);
	//printMap();
//	String path = getPath(goalX, goalY);
//	return convertPathToDirections(path);
}

bool isValidNeighbor(int x, int y) {
	if (!cellExists(x, y))
		if (world_map[x][y] != 0)
			return false;
	return true;
}

bool cellExists(int x, int y) {
	if (x < 0 || x > 3)
		return false;
	if (y < 0 || y > 3)
		return false;
	return true;
}

void populateMap(int x, int y) {
	Serial.print("Start = (");
	Serial.print(x);
	Serial.print(",");
	Serial.print(y);
	//Check neighbor to the north
	if (isValidNeighbor(x, y - 1)) {
		world_map[x][y - 1] = world_map[x][y] + 1;
		Serial.println("Valid neighbor north.");
		populateMap(x, y - 1);
	}

	//Check neighbor to the south
	if (isValidNeighbor(x, y + 1)) {
		world_map[x][y + 1] = world_map[x][y] + 1;
		Serial.println("Valid neighbor south.");
		populateMap(x, y + 1);
	}

	//Check neighbor to the east
	if (isValidNeighbor(x + 1, y)) {
		world_map[x + 1][y] = world_map[x][y] + 1;
		Serial.println("Valid neighbor east.");
		populateMap(x + 1, y);
	}

	//Check neighbor to the west
	if (isValidNeighbor(x - 1, y)) {
		world_map[x - 1][y] = world_map[x][y] + 1;
		Serial.println("Valid neighbor west.");
		populateMap(x - 1, y);
	}
}

void printMap() {
	for (int y = 0; y < 4; y++) {
		for (int x = 0; x < 4; x++) {
			Serial.print(world_map[x][y]);
			Serial.print("\t");
		}
		Serial.println();
	}
}

String getPath(int x, int y) {
	int smallestValue = world_map[x][y];
	int smallestNeighbor = -1;

	if (smallestValue == 0) return "";

	//Check neighbor to the north
	if (cellExists(x, y-1)) {
		if (world_map[x][y - 1] < smallestValue) {
			smallestNeighbor = NORTH;
			smallestValue = world_map[x][y - 1];
		}
	}

	//Check neighbor to the south
	if (cellExists(x, y+1)) {
		if (world_map[x][y+1] < smallestValue) {
			smallestNeighbor = SOUTH;
			smallestValue = world_map[x][y+1];
		}
	}

	//Check neighbor to the east
	if (cellExists(x+1, y)) {
		if (world_map[x+1][y] < smallestValue) {
			smallestNeighbor = EAST;
			smallestValue = world_map[x+1][y];
		}
	}

	//Check neighbor to the west
	if (cellExists(x-1, y)) {
		if (world_map[x-1][y] < smallestValue) {
			smallestNeighbor = WEST;
			smallestValue = world_map[x-1][y];
		}
	}

	switch (smallestNeighbor) {
		case (NORTH):
			return getPath(x, y-1) + "S";
		case (SOUTH):
			return getPath(x, y+1) + "N";
		case (EAST):
			return getPath(x+1,y) + "W";
		case (WEST):
			return getPath(x-1, y) + "E";
		default:
			return "error";
	}
}

String convertPathToDirections(String path) {
	String result = "S";
	for (int i = 0; i < path.length()-1; i++) {
		char current = path.charAt(i);
		char next = path.charAt(i+1);
		result = result + computeDirection(current, next);
	}
	result = result + "T";
	return result;
}

char computeDirection(char current, char next) {
	if (current == next) return 'S';
	if (current == 'N') {
		if (next == 'E') return 'R';
		if (next == 'W') return 'L';
	}
	if (current == 'S') {
		if (next == 'E') return 'L';
		if (next == 'W') return 'R';
	}
	if (current == 'E') {
		if (next == 'N') return 'L';
		if (next == 'S') return 'R';
	}
	if (current == 'W') {
		if (next == 'N') return 'R';
		if (next == 'S') return 'L';
	}
	return 'X';
}


