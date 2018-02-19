/**
 * MetricPathPlanning.ino
 * 	Author: Rahul Yarlagadda, Ellie Honious
 * 	Date: Feburary 9th, 2018
 *
 * 	This file contains code to create paths between any two points within the world.
 */

#include "MetricPathPlanning.h"
#include "WirelessReceiver.h"
#include "RobotDrive.h"

int world_map[4][4] = { { 0, 99, 99, 0 },
						{ 0, 0,  0,  0 },
						{ 0, 99, 99, 0 },
						{ 0, 99, 99, 0 } };
/**
 * Gets topological directions (A list of S, L, and R for straight, left, and right) from
 * (startX, startY) to (goalX, goalY), avoiding obstacles based on the world map.
 */
String getDirectionsToGoal(int startX, int startY, int goalX, int goalY) {
	//Set the starting location to a 1 in the map.
	world_map[startY][startX] = 1;

	//Use the wavefront algorithim to find the distance from all other cells to the start location.
	populateMap(startX, startY);

	//Send the map to the laptop.
	printMap();

	//Get a metric path from the goal location to the start location.
	String path = getPath(goalX, goalY);

	//Spin the robot towards the start of the path.
	spinToDirection(path.charAt(0));

	//Convert the metric path to topological directions.
	String directions = convertPathToDirections(path);
	return directions;
}

/**
 * Spin to the given direction, assuming the robot is facing north.
 * This preps the robot to follow the generated path.
 */
void spinToDirection(char c) {
	//Turn to the appropriate direction.
	if (c == 'S') spinDegrees(180);
	if (c == 'E') spinDegrees(-90);
	if (c == 'W') spinDegrees(90);

	//If there's no left or right wall, drive forward a bit to reach them.
	//Then the robot can start wall following.
	if (!bitRead(flag, obRight) && !bitRead(flag, obLeft)) {
		forward(one_rotation + half_rotation);
	}
}

/**
 * Figure out whether (x,y) is an open cell in the world.
 */
bool isValidNeighbor(int x, int y) {
	return cellExists(x, y) && world_map[y][x] == 0;
}

/**
 * Figure out whether (x,y) is a cell in the world
 */
bool cellExists(int x, int y) {
	if (x < 0 || x > 3)
		return false;
	if (y < 0 || y > 3)
		return false;
	return true;
}

/**
 * Recursive function to perform the wavefront algorithim on the map.
 * Puts the current cells value + 1 to all neighboring cells that haven't already been visited.
 * Then recursively calls the function on all those neighbors.
 */
void populateMap(int x, int y) {
	//Check neighbor to the north
	if (isValidNeighbor(x, y - 1)) {
		world_map[y-1][x] = world_map[y][x] + 1;
		populateMap(x, y - 1);
	}

	//Check neighbor to the south
	if (isValidNeighbor(x, y + 1)) {
		world_map[y+1][x] = world_map[y][x] + 1;
		populateMap(x, y + 1);
	}

	//Check neighbor to the east
	if (isValidNeighbor(x + 1, y)) {
		world_map[y][x+1] = world_map[y][x] + 1;
		populateMap(x + 1, y);
	}

	//Check neighbor to the west
	if (isValidNeighbor(x - 1, y)) {
		world_map[y][x-1] = world_map[y][x] + 1;
		populateMap(x - 1, y);
	}
}

/**
 * Send a string representation of the map over the wireless interface.
 */
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


/**
 * Recursive function to find a path from (x,y) to the starting location using results
 * from the wavefront algorithim. Builds the path from (x,y) by always taking the smallest-valued neighbor.
 */
String getPath(int x, int y) {
	int smallestValue = world_map[y][x];
	int smallestNeighbor = -1;

	//(x,y) is the starting location, so return an empty string.
	if (smallestValue == 1) return "";

	//Check neighbor to the north
	if (cellExists(x, y-1)) {
		if (world_map[y-1][x] < smallestValue) {
			smallestNeighbor = NORTH;
			smallestValue = world_map[y-1][x];
		}
	}

	//Check neighbor to the south
	if (cellExists(x, y+1)) {
		if (world_map[y+1][x] < smallestValue) {
			smallestNeighbor = SOUTH;
			smallestValue = world_map[y+1][x];
		}
	}

	//Check neighbor to the east
	if (cellExists(x+1, y)) {
		if (world_map[y][x+1] < smallestValue) {
			smallestNeighbor = EAST;
			smallestValue = world_map[y][x+1];
		}
	}

	//Check neighbor to the west
	if (cellExists(x-1, y)) {
		if (world_map[y][x-1] < smallestValue) {
			smallestNeighbor = WEST;
			smallestValue = world_map[y][x-1];
		}
	}

	//Add the direction of the smallest neighbor to the path from that neighbor.
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

/**
 * Converts a metric path (north, south, east west) to a topological path
 * (straight, left, right).
 */
String convertPathToDirections(String path) {
	//Topological paths always start with a straight.
	String result = "S";

	//Build the topological path
	for (int i = 0; i < path.length()-1; i++) {
		char current = path.charAt(i);
		char next = path.charAt(i+1);
		result = result + computeDirection(current, next);
	}

	//Topological paths always end with a terminate.
	result = result + "T";
	return result;
}

/**
 * Given two metric directions (north, south, east, west), find the
 * direction to turn.
 */
char computeDirection(char current, char next) {
	//If both directions are the same, go straight.
	if (current == next) return 'S';

	//Otherwise we need to turn
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

	//Error? Shouldn't ever get here.
	return 'X';
}



