/*	IRSensor.ino
 * 	Author: Rahul Yarlagadda, Ellie Honious
 * 	Date: December 13th, 2017
 *
 * 	This file contains the updateIR() function, which will poll the IR sensors and
 * 	convert their readings to inches. A moving average filter is applied to the readings to
 * 	reduce noise.
 */

#include "IRSensor.h"
#include <NewPing.h>

int irFrontArray[5] = { 0, 0, 0, 0, 0 }; //array to hold 5 front IR readings
int irRearArray[5] = { 0, 0, 0, 0, 0 }; //array to hold 5 back IR readings
int irRightArray[5] = { 0, 0, 0, 0, 0 }; //array to hold 5 right IR readings
int irLeftArray[5] = { 0, 0, 0, 0, 0 }; //array to hold 5 left IR readings
int irIdx = 0; //index for 5 IR readings to take the average

//define sensor constants and variables
#define irMin    2               // IR minimum threshold for wall (use a deadband of 4 to 6 inches)
#define irMax    9              // IR maximum threshold for wall (use a deadband of 4 to 6 inches)

//define error variables
float li_curr;    //left ir current reading
float ri_curr;    //right ir current reading

float li_cerror;    //left ir current error
float ri_cerror;    //right ir current error

float derror; //difference between left and right error to center robot in the hallway

//Define IR data variables
float irFront = 0;
float irLeft = 0;
float irRear = 0;
float irRight = 0;
volatile byte irFlag = 0;

int front, rear, left, right;

/**
 * This function is bound to an interrupt and called every 100ms.
 */
void updateIR() {
	updateIRValues();
	updateObstacles();
	updateError();
}

/**
 *	Gets value from IR sensors and converts to inches.
 */
void updateIRValues() {
	//Get all IR values
	front = analogRead(PIN_IR_FRONT);
	rear = analogRead(PIN_IR_REAR);
	left = analogRead(PIN_IR_LEFT);
	right = analogRead(PIN_IR_RIGHT);
	//Serial.print(rear); Serial.print("\t");


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

	//	Convert IR values to inches based on calibration
	irFront = (1280 / (irFront + 17)) - 0.5;
	irRear = (1100 / (irRear + 16));
	irLeft = (3000 / (irLeft + 10)) - 2.5;
	irRight = (1950 / (irRight - 32));

	//  print IR data
	//	Serial.println("frontIR\tbackIR\tleftIR\trightIR");
	//	Serial.print(irFront); Serial.print("\t");
	//	Serial.print(irRear); Serial.print("\t");
	//	Serial.print(irLeft); Serial.print("\t");
	//	Serial.println(irRight);

//		Serial.print(left); Serial.print("\t");
//		Serial.print(right); Serial.print("\t");
//		Serial.print(front); Serial.print("\t");
//		Serial.println(rear);
}

/**
 *	Updates the obstacle flag based on the latest IR data.
 */
void updateObstacles() {
	//Set obstacle flags
	if (irRight > irMin && irRight < irMax) {
		if (!bitRead(flag, obRight)) {
//			Serial.print("\t\tset right obstacle: ");
//			Serial.println(irRight);
			bitSet(flag, obRight);            //set the right obstacle
		}
	} else
		bitClear(flag, obRight);          //clear the right obstacle
	if (irLeft > irMin && irLeft < irMax) {
		if (!bitRead(flag, obLeft)) {
//			Serial.print("\t\tset left obstacle: ");
//			Serial.println(irLeft);
			bitSet(flag, obLeft);             //set the left obstacle
		}
	} else
		bitClear(flag, obLeft);           //clear the left obstacle
	if (irFront < 6) {
		if (!bitRead(flag, obFront)) {
			//Serial.print("\t\tset front obstacle bit: ");
			//Serial.println(irFront);
			bitSet(flag, obFront);            //set the front obstacle
		}
	} else {
		bitClear(flag, obFront);          //clear the front obstacle
	}
}

/**
 * Updates all of the error constants to be used for P and PD controls
 */
void updateError() {
	//Calculate error
	ri_curr = irRight;             //log current sensor reading [right IR]
	ri_cerror = ri_curr - 7; //calculate current error (too far positive, too close negative)
	if (ri_cerror < 0.5 && ri_cerror > -0.5)
		ri_cerror = 0;

	li_curr = irLeft;                  //log current sensor reading [left sonar]
	li_cerror = li_curr - 7;   //calculate current error
	if (li_cerror < 0.5 && li_cerror > -0.5)
		li_cerror = 0;
	derror = li_cerror - ri_cerror; //use IR data for difference error
	if (derror < 0.5 && derror > -0.5) {
		derror = 0;
	}

}
