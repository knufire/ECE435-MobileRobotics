/*	SonarSensor.ino
 * 	Author: Rahul Yarlagadda, Ellie Honious
 * 	Date: December 13th, 2017
 *
 * 	This file defines all hardware connections on the robot, so pin assignments can be refered to
 *  by name instead of by pin number
 */

#include "SonarSensor.h"

NewPing sonarLt(PIN_SNR_LEFT, PIN_SNR_LEFT);    //create an instance of the left sonar
NewPing sonarRt(PIN_SNR_RIGHT, PIN_SNR_RIGHT);  //create an instance of the right sonar

//flag byte to hold sensor data
volatile byte snrFlag = 0;    // Flag to hold IR & Sonar data - used to create the state machine
#define obFLeft   4 // Left Sonar trip
#define obFRight  5 // Right Sonar trip

int srLeftArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 left sonar readings
int srRightArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 right sonar readings
int srIdx = 0;//index for 5 sonar readings to take the average
int srLeftAvg;  //variable to holde left sonar data
int srRightAvg; //variable to hold right sonar data
int snrLeft;
int snrRight;



/*
  This is a sample updateSonar2() function, the description and code should be updated to take an average, consider all sensors and reflect
  the necesary changes for the lab requirements.
 */
void updateSonar() {
	srRight =  sonarRt.ping_in(); //right sonara in inches
	srLeft = sonarLt.ping_in(); //left sonar in inches
	srRightAvg = srRight;
	srLeftAvg = srLeft;

	//  Serial.print("lt snr:\t");
	//  Serial.print(srLeftAvg);
	//  Serial.print("rt snr:\t");
	//  Serial.println(srRightAvg);
	if (srRightAvg < snrThresh && srRightAvg > minThresh) {
		//    Serial.println("set front right obstacle bit");
		bitSet(snrFlag, obFRight);//set the front right obstacle
	}
	else {
		bitClear(snrFlag, obFRight);//clear the front right obstacle
	}
	if (srLeftAvg < snrThresh && srLeftAvg > minThresh) {
		//    Serial.println("set front left obstacle bit");
		bitSet(snrFlag, obFLeft);//set the front left obstacle
	}
	else {
		bitClear(snrFlag, obFLeft);//clear the front left obstacle
	}
}

