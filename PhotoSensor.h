/*	PHTOSensor.h
 * 	Author: Rahul Yarlagadda, Ellie Honious
 * 	Date: January 23th, 2018
 *
 * 	This file contains the updatePHTO() function, which will pull the Photoresistor sensors and
 * 	convert their readings to inches.
 */

#pragma once
#ifndef PHOTOSENSOR_H_
#define PHOTOSENSOR_H_


extern float photoLeft;
extern float photoRight;
/**
 * Updates Photoresistor sensors and all assocated data.
 */
void updatePhoto();



#endif /* PHOTOSENSOR_H_ */
