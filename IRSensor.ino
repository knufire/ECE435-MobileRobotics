/*	IRSensor.ino
 * 	Author: Rahul Yarlagadda, Ellie Honious
 * 	Date: December 13th, 2017
 *
 * 	This file defines all hardware connections on the robot, so pin assignments can be refered to
 *  by name instead of by pin number
 * */

#include "IRSensor.h"
#include <NewPing.h>

#define irThresh    5 // The IR threshold for presence of an obstacle
#define minThresh 	0

int irFrontArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 front IR readings
int irRearArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 back IR readings
int irRightArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 right IR readings
int irLeftArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 left IR readings
int irIdx = 0;//index for 5 IR readings to take the average

//bit definitions for sensor data flag byte
#define obFront   0 // Front IR trip
#define obRear    1 // Rear IR trip
#define obRight   2 // Right IR trip
#define obLeft    3 // Left IR trip

float irFrontAvg = 0;
float irLeftAvg = 0;
float irRearAvg = 0;
float irRightAvg = 0;
volatile byte irFlag = 0;

void updateIR() {
	int front, back, left, right;
	front = analogRead(PIN_IR_FRONT);
	back = analogRead(PIN_IR_REAR);
	left = analogRead(PIN_IR_LEFT);
	right = analogRead(PIN_IR_RIGHT);
	irFrontAvg = (1280 / ((float)front + 18)) - 0.5;
	irRearAvg = (1100 / ((float)back + 16));
	irLeftAvg = (3000 / ((float)left + 22)) - 2;
	irRightAvg = (1950 / ((float)right - 34));

	if (irRightAvg < 0) irRightAvg = irThresh + 1;

	//  print IR data
	//  Serial.println("frontIR\tbackIR\tleftIR\trightIR");
	//  Serial.print(front); Serial.print("\t");
	//  Serial.print(back); Serial.print("\t");
	//  Serial.print(left); Serial.print("\t");
	//  Serial.println(right);
	if (irRightAvg < irThresh && !bitRead(irFlag, obRight)) {
		Serial.print("set right obstacle bit");
		Serial.println(irRightAvg);
		bitSet(irFlag, obRight);//set the right obstacle
	}
	else {
		bitClear(irFlag, obRight);//clear the right obstacle
	}
	if (irLeftAvg < irThresh && !bitRead(irFlag, obLeft)) {
		Serial.print("set left obstacle bit");
		Serial.println(irLeftAvg);
		bitSet(irFlag, obLeft);//set the left obstacle
	}
	else {
		bitClear(irFlag, obLeft);//clear the left obstacle
	}
	if (irFrontAvg < irThresh && !bitRead(irFlag, obFront)) {
		Serial.print("set front obstacle bit");
		Serial.println(irFrontAvg);
		bitSet(irFlag, obFront);//set the front obstacle
	}
	else {
		bitClear(irFlag, obFront);//clear the front obstacle
	}
	if (irRearAvg < irThresh && !bitRead(irFlag, obRear)) {
		Serial.print("set back obstacle bit");
		Serial.println(irRearAvg);
		bitSet(irFlag, obRear);//set the back obstacle
	}
	else {
		bitClear(irFlag, obRear);//clear the back obstacle
	}
}
