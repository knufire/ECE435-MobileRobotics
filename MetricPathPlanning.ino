/*
 * MetricPathPlanning.ino
 *
 *  Created on: Feb 9, 2018
 *      Author: Rahul
 */

#include "MetricPathPlanning.h"
#include "WirelessReceiver.h"
#include "RobotDrive.h"

int world_map[4][4] = { { 0, 99, 99, 0 },
						{ 0, 0,  0,  0 },
						{ 0, 99, 99, 0 },
						{ 0, 99, 99, 0 } };

String getDirectionsToGoal(int startX, int startY, int goalX, int goalY) {
//	Serial.print("Start = (");
//	Serial.print(startX);
//	Serial.print(",");
//	Serial.print(startY);
//	Serial.print(")\t");
//	Serial.print("End = (");
//	Serial.print(goalX);
//	Serial.print(",");
//	Serial.print(goalY);
//	Serial.println(")");
	world_map[startY][startX] = 1;
	populateMap(startX, startY);
	printMap();
	String path = getPath(goalX, goalY);
	Serial.println(path);
	spinToDirection(path.charAt(0));
	String directions = convertPathToDirections(path);
	return directions;
}

void spinToDirection(char c) {
	if (c == 'S') spinDegrees(180);
	if (c == 'E') spinDegrees(-90);
	if (c == 'W') spinDegrees(90);
	if (!bitRead(flag, obRight) && !bitRead(flag, obLeft)) {
		forward(one_rotation + half_rotation);
	}
}

bool isValidNeighbor(int x, int y) {
	return cellExists(x, y) && world_map[y][x] == 0;
}

bool cellExists(int x, int y) {
	if (x < 0 || x > 3)
		return false;
	if (y < 0 || y > 3)
		return false;
	return true;
}

void populateMap(int x, int y) {
//	Serial.print("Start = (");
//	Serial.print(x);
//	Serial.print(",");
//	Serial.print(y);
//	Serial.println(")");
	//printMap();
	//Check neighbor to the north
	if (isValidNeighbor(x, y - 1)) {
		world_map[y-1][x] = world_map[y][x] + 1;
//		Serial.println("Valid neighbor north.");
		populateMap(x, y - 1);
	}

	//Check neighbor to the south
	if (isValidNeighbor(x, y + 1)) {
		world_map[y+1][x] = world_map[y][x] + 1;
//		Serial.println("Valid neighbor south.");
		populateMap(x, y + 1);
	}

	//Check neighbor to the east
	if (isValidNeighbor(x + 1, y)) {
		world_map[y][x+1] = world_map[y][x] + 1;
//		Serial.println("Valid neighbor east.");
		populateMap(x + 1, y);
	}

	//Check neighbor to the west
	if (isValidNeighbor(x - 1, y)) {
		world_map[y][x-1] = world_map[y][x] + 1;
//		Serial.println("Valid neighbor west.");
		populateMap(x - 1, y);
	}
}

void printMap() {
	for (int y = 0; y < 4; y++) {
		String output = "";
		for (int x = 0; x < 4; x++) {
			output += (world_map[y][x]);
			output += "\t";
		}
		wirelessSend(output);
		delay(100);
	}
}

String getPath(int x, int y) {
	int smallestValue = world_map[y][x];
	int smallestNeighbor = -1;

	if (smallestValue == 1) return "";

	//Check neighbor to the north
	if (cellExists(x, y-1)) {
		if (world_map[y-1][x] < smallestValue) {
//			Serial.println("Neighbor north smaller");
			smallestNeighbor = NORTH;
			smallestValue = world_map[y-1][x];
		}
	}

	//Check neighbor to the south
	if (cellExists(x, y+1)) {
		if (world_map[y+1][x] < smallestValue) {
//			Serial.println("Neighbor south smaller");
			smallestNeighbor = SOUTH;
			smallestValue = world_map[y+1][x];
		}
	}

	//Check neighbor to the east
	if (cellExists(x+1, y)) {
		if (world_map[y][x+1] < smallestValue) {
//			Serial.println("Neighbor east smaller");
			smallestNeighbor = EAST;
			smallestValue = world_map[y][x+1];
		}
	}

	//Check neighbor to the west
	if (cellExists(x-1, y)) {
		if (world_map[y][x-1] < smallestValue) {
//			Serial.println("Neighbor west smaller");
			smallestNeighbor = WEST;
			smallestValue = world_map[y][x-1];
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



