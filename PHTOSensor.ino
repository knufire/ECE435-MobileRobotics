/*	PHTOSensor.ino
 * 	Author: Rahul Yarlagadda, Ellie Honious
 * 	Date: January 23th, 2018
 *
 * 	This file contains the updatePHTO() function, which will pull the Photoresistor sensors and
 * 	convert their readings to inches.
 */

#include "PHTOSensor.h"

//Define PHOTO data variables

float photoLeft = 0;
float photoRight = 0;


/**
 * This function is bound to an interrupt and called every 100ms.
 */
void updatePHTO() {
	updatePHTOValues();
}

/**
 *	Gets value from Photoresistor sensors.
 */
void updatePHTOValues() {
	int left, right;

	//Get all IR values
	left = analogRead(PIN_PHTO_LEFT);
	right = analogRead(PIN_PHTO_RIGHT);

	//  print Photoresistor data
	//	Serial.println("leftPHTO\trightPHTO");
	//	Serial.print(photoLeft); Serial.print("\t");
	//	Serial.println(photoRight);
}
