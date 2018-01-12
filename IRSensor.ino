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

int irFrontArray[5] = { 0, 0, 0, 0, 0 }; //array to hold 5 front IR readings
int irRearArray[5] = { 0, 0, 0, 0, 0 }; //array to hold 5 back IR readings
int irRightArray[5] = { 0, 0, 0, 0, 0 }; //array to hold 5 right IR readings
int irLeftArray[5] = { 0, 0, 0, 0, 0 }; //array to hold 5 left IR readings
int irIdx = 0; //index for 5 IR readings to take the average

//bit definitions for sensor data flag byte
#define obFront   0 // Front IR trip
#define obRear    1 // Rear IR trip
#define obRight   2 // Right IR trip
#define obLeft    3 // Left IR trip

//define sensor constants and variables
#define irMin    150               // IR minimum threshold for wall (use a deadband of 4 to 6 inches)
#define irMax    300               // IR maximum threshold for wall (use a deadband of 4 to 6 inches)

//define error variables
int li_curr;    //left ir current reading
int ri_curr;    //right ir current reading

int li_cerror;    //left ir current error
int ri_cerror;    //right ir current error

int li_perror;    //left ir previous error
int ri_perror;    //right ir previous error

int li_derror;  //left ir delta error
int ri_derror;  //right ir delta error

int left_derror; //difference between left front and back sensor, this may be useful for adjusting the turn angle
int right_derror; //difference between right front and back sensor, this may be useful for adjusting the turn angle

int derror; //difference between left and right error to center robot in the hallway

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

//	//Convert IR values to inches based on calibration
//	irFront = (1280 / (irFront + 18)) - 0.5;
//	irRear = (1100 / (irRear + 16));
//	irLeft = (3000 / (irLeft + 22)) - 2;
//	irRight = (1950 / (irRight - 34));

	//  print IR data
//	  Serial.println("frontIR\tbackIR\tleftIR\trightIR");
//	  Serial.print(front); Serial.print("\t");
//	  Serial.print(rear); Serial.print("\t");
//	  Serial.print(left); Serial.print("\t");
//	  Serial.println(right);

	//Set obstacle flags
	if (right > irMin - 50) {
		//Serial.println("\t\tset right obstacle");
		bitSet(flag, obRight);            //set the right obstacle
	} else
		bitClear(flag, obRight);          //clear the right obstacle

	if (left > irMin - 50) {
		//Serial.println("\t\tset left obstacle");
		bitSet(flag, obLeft);             //set the left obstacle
	} else
		bitClear(flag, obLeft);           //clear the left obstacle

	if (front > irMax - 50) {
		//Serial.println("set front obstacle bit");
		bitSet(flag, obFront);            //set the front obstacle
	} else {
		bitClear(flag, obFront);          //clear the front obstacle
	}

	//Calculate error
	ri_curr = right;             //log current sensor reading [right IR]
	if ((ri_curr > irMax) | (ri_curr < irMin))
		ri_cerror = irMax - ri_curr; //calculate current error (too far positive, too close negative)
	else
		ri_cerror = 0;              //set error to zero if robot is in dead band
	ri_derror = ri_cerror - ri_perror; //calculate change in error
	ri_perror = ri_cerror;    //log current error as previous error [left sonar]

	li_curr = left;                   //log current sensor reading [left sonar]
	if ((li_curr > irMax) | (li_curr < irMin))
		li_cerror = irMax - li_curr;   //calculate current error
	else
		li_cerror = 0;                  //error is zero if in deadband
	li_derror = li_cerror - li_perror; //calculate change in error
	li_perror = li_cerror;                //log reading as previous error

}

/*
 This function will update all of the error constants to be used for P and PD control
 store previous error to calculate derror = curr_sensor-prev_sensor, side_derror = side front sensor - side back sensor
 */
int updateError() {
	derror = li_cerror - ri_cerror; //use IR data for difference error
	return derror;
}
