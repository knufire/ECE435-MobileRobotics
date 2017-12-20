/*	IRSensor.ino
 * 	Author: Rahul Yarlagadda, Ellie Honious
 * 	Date: December 13th, 2017
 *
 * 	This file contains the updateIR() function, which will poll the IR sensors and
 * 	convert their readings to inches. A moving average filter is applied to the readings to
 * 	reduce noise. The readings are available externally through the irFrontAvg, irLeftAvg, irRearAvg, irRightAvg, and irFlag variables.
 */

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

float irFront = 0;
float irLeft = 0;
float irRear = 0;
float irRight = 0;
volatile byte irFlag = 0;

void updateIR() {
	int front, rear, left, right;

	//Get all IR values
	front = analogRead(PIN_IR_FRONT);
	rear = analogRead(PIN_IR_REAR);
	left = analogRead(PIN_IR_LEFT);
	right = analogRead(PIN_IR_RIGHT);

	//Add IR values to array
	irFrontArray[irIdx] = front;
	irRearArray[irIdx] = rear;
	irLeftArray[irIdx] = left;
	irRightArray[irIdx] = right;

	//Increment array index
	irIdx++;
	if (irIdx == 5) {
		irIdx = 0;
	}

	//Find average of last 5 IR values
	for (int i = 0; i < 5; i++) {
		irFront += irFrontArray[i];
		irLeft += irLeftArray[i];
		irRear += irRearArray[i];
		irRight += irRightArray[i];
	}

	irFront = irFront / 5;
	irLeft = irLeft / 5;
	irRear = irRear / 5;
	irRight = irRight / 5;

	//Convert IR values to inches
	irFront = (1280 / (irFront + 18)) - 0.5;
	irRear = (1100 / (irRear + 16));
	irLeft = (3000 / (irLeft + 22)) - 2;
	irRight = (1950 / (irRight - 34));

	if (irRight < 0) irRight = irThresh + 1;

	//  print IR data
	//  Serial.println("frontIR\tbackIR\tleftIR\trightIR");
	//  Serial.print(front); Serial.print("\t");
	//  Serial.print(back); Serial.print("\t");
	//  Serial.print(left); Serial.print("\t");
	//  Serial.println(right);
	if (irRight < irThresh && !bitRead(irFlag, obRight)) {
		Serial.print("set right obstacle bit");
		Serial.println(irRight);
		bitSet(irFlag, obRight);//set the right obstacle
	}
	else {
		bitClear(irFlag, obRight);//clear the right obstacle
	}
	if (irLeft < irThresh && !bitRead(irFlag, obLeft)) {
		Serial.print("set left obstacle bit");
		Serial.println(irLeft);
		bitSet(irFlag, obLeft);//set the left obstacle
	}
	else {
		bitClear(irFlag, obLeft);//clear the left obstacle
	}
	if (irFront < irThresh && !bitRead(irFlag, obFront)) {
		Serial.print("set front obstacle bit");
		Serial.println(irFront);
		bitSet(irFlag, obFront);//set the front obstacle
	}
	else {
		bitClear(irFlag, obFront);//clear the front obstacle
	}
	if (irRear < irThresh && !bitRead(irFlag, obRear)) {
		Serial.print("set back obstacle bit");
		Serial.println(irRear);
		bitSet(irFlag, obRear);//set the back obstacle
	}
	else {
		bitClear(irFlag, obRear);//clear the back obstacle
	}
}
