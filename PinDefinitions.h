/*	PinDefinitions.ino
 * 	Author: Rahul Yarlagadda, Ellie Honious
 * 	Date: December 13th, 2017
 *
 * 	This file defines all hardware connections on the robot, so pin assignments can be refered to
 *  by name instead of by pin number
 * */

#pragma once

/*------------------------Digital IO---------------------------------*/
//		Name		   Pin #	Description
#define PIN_LT_STEP		50 		//right stepper motor step pin
#define PIN_LT_DIR 		51 		//right stepper motor direction pin
#define PIN_RT_STEP 	52		//left stepper motor step pin
#define PIN_RT_DIR 		53		//left stepper motor direction pin
#define PIN_GREEN_LED 	6		//Green LED on the breadboard
#define PIN_RED_LED 	7		//Red LED on the breadboard
#define PIN_STEP_ENABLE	48		//Stepper enable pin
#define PIN_LED_ENABLE	13
#define PIN_LED_TEST 	13
#define PIN_ENCODER_LEFT 2
#define PIN_ENCODER_RIGHT 3

/*------------------------Analog IO---------------------------------*/
//		Name		   Pin #	Description
#define PIN_IR_FRONT   	A8    	//front IR analog pin
#define PIN_IR_REAR    	A9    	//back IR analog pin
#define PIN_IR_RIGHT   	A10   	//right IR analog pin
#define PIN_IR_LEFT    	A11   	//left IR analog pin
#define PIN_SNR_LEFT   	A12   	//front left sonar
#define PIN_SNR_RIGHT  	A13  	//front right sonar
#define PIN_BUTTON    	A15    	//pushbutton
#define PIN_PHTO_LEFT	A6
#define PIN_PHTO_RIGHT	A7





