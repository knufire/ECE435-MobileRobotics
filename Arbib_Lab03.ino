/*	Arbib_Lab02.ino
 * 	Author: Rahul Yarlagadda, Ellie Honious
 * 	Date: December 13th, 2017
 *
 * 	This file contains the main loop for Airbib accomplishing the tasks laid out in Lab 2.
 * 	This includes aggressive kid behaviors, shy kid behaviors, random-wander, and go-to-goal
 * 	functionality. Additionally, random-wander and go-to-goal have integrated obstacle avoidance.
 */

#include <TimerOne.h>
#include "RobotDrive.h"
#include "IRSensor.h"
#include "PinDefinitions.h"
#include "SonarSensor.h"

#define stopThresh  150 // If the robot has been stopped for this threshold move
#define baud_rate 9600//set serial communication baud rate

volatile boolean test_state; //variable to hold test led state for timer interrupt

int goalX = 5;
int goalY = 2;

//flag byte to hold sensor data
volatile byte flag = 0;    // Flag to hold IR & Sonar data - used to create the state machine

//bit definitions for sensor data flag byte
#define obFront   0 // Front IR trip
#define obRear    1 // Rear IR trip
#define obRight   2 // Right IR trip
#define obLeft    3 // Left IR trip
#define obFLeft   4 // Left Sonar trip
#define obFRight  5 // Right Sonar trip

int count; //count number of times collide has tripped
#define max_collide 250 //maximum number of collides before robot reverses

//state byte to hold robot motion and state data
volatile byte state = 0;   //state to hold robot states and motor motion
#define avoid     0   //avoid behavior
#define wander    1   //wander behavior
#define fleft     2   //follow left wall behavior
#define fright    3   //follow right wall behavior
#define center    4   //follow hallway behavior
#define movingL   6   //robot left wheel moving
#define movingR   7   //robot right wheel moving


//define layers of subsumption architecture that are active [hallway Wall Wander Avoid]
byte layers = 4;
#define aLayer 0      //avoid obstacle layer
#define wLayer 1      //wander layer
#define fwLayer 2     //follow wall layer
#define fhLayer 3     //follow hallway layer


#define timer_int 250000 // 1/4 second (250000 us) period for timer interrupt

void setup()
{
	//Serial setup
	Serial.begin(baud_rate);				//start serial communication in order to debug the software while coding

	//Drive Setup
	driveSetup();

	//LED Setup
	pinMode(PIN_GREEN_LED, OUTPUT);
	pinMode(PIN_RED_LED, OUTPUT);
	pinMode(PIN_LED_ENABLE, OUTPUT);
	digitalWrite(PIN_LED_ENABLE, HIGH);
	digitalWrite(PIN_GREEN_LED, HIGH);
	digitalWrite(PIN_RED_LED, HIGH);


	//Timer Interrupt Set Up
	Timer1.initialize(timer_int);         	// initialize timer1, and set a period in microseconds
	Timer1.attachInterrupt(updateSensors);  // attaches updateSensors() as a timer overflow interrupt


	delay(1500);//wait 3 seconds before robot moves
}

void loop()
{
	wallBang();  //execute robot motions based upon sensor data and current state
}

/*
   This is a sample wallBang() function, the description and code should be updated to reflect the actual robot motion function that you will implement
   based upon the the lab requirements.  Some things to consider, you cannot use a blocking motor function because you need to use sensor data to update
   movement.  You also need to continue to poll    the sensors during the motion and update flags and state because this will serve as your interrupt to
   stop or change movement. This function will have the robot follow the wall if it is within 4 to 6 inches from the wall by moving forward and turn on the
   controller if it is outside that band to make an adjustment to get back within the band.
*/
void wallBang() {
  Serial.print("\nWallBang: li_cerror ri_cerror\t");
  Serial.print(li_cerror); Serial.print("\t");
  Serial.println(ri_cerror);
  if (bitRead(state, fright)) {
    Serial.println("right wall found");
    if (bitRead(flag, obFront)) { //check for a front wall before moving
      Serial.print("right wall: front corner ");
      //make left turn if wall found
      reverse(two_rotation);              //back up
      spin(three_rotation, 0);              //turn left
    }
    if (ri_cerror == 0) {                 //no error, robot in deadband
      Serial.println("right wall detected, drive forward");
      forward(three_rotation);            //move robot forward
    }
    else {
      //Serial.println("rt wall: adjust turn angle based upon error");
      if (ri_cerror < 0) {          //negative error means too close
        Serial.println("\trt wall: too close turn left");
        pivot(quarter_rotation, 1);      //pivot left
        pivot(quarter_rotation, 0);   //pivot right to straighten up
      }
      else if (ri_cerror > 0) {     //positive error means too far
        Serial.println("\trt wall: too far turn right");
        pivot(quarter_rotation, 1);      //pivot right
        pivot(quarter_rotation, 0);   //pivot left to straighten up
      }
    }
  }

  else if (bitRead(state, fleft)  ) {
    if (bitRead(flag, obFront)) { //check for a front wall before moving forward
      //make right turn if wall found
      Serial.print("left wall: front corner ");
      //make left turn if wall found
      reverse(two_rotation);              //back up
      spin(three_rotation, 1);              //turn right
    }
    if (li_cerror == 0) {           //no error robot in dead band drives forward
      //Serial.println("lt wall detected, drive forward");
      forward(two_rotation);      //move robot forward
    }
    else {
      //Serial.println("lt wall detected: adjust turn angle based upon error");
      if (li_cerror < 0) { //negative error means too close
        //Serial.println("\tlt wall: too close turn right");
        pivot(quarter_rotation, 1);      //pivot right
        pivot(quarter_rotation, 0);   //pivot left
      }
      else if (li_cerror > 0)  { //positive error means too far
        //Serial.println("\tlt wall: too far turn left");
        pivot(quarter_rotation, 1);      //pivot left
        pivot(quarter_rotation, 0);   //pivot right
      }
    }
  }
  else if (bitRead(state, center) ) {//follow hallway
    if (((ri_cerror == 0) && (li_cerror == 0)) || (derror == 0)) {
      //Serial.println("hallway detected, drive forward");
      forward(two_rotation);          //drive robot forward
    }
    else {
      //Serial.println("hallway detected: adjust turn angle based upon error");
      //try to average the error between the left and right to center the robot
      if (derror > 0) {
        spin(quarter_rotation, 1);        //spin right, the left error is larger
        pivot(quarter_rotation, 0);       //pivot left to adjust forward
      }
      else
      {
        spin(quarter_rotation, 0);        //spin left the right error is larger
        pivot(quarter_rotation, 1);       //pivot right to adjust forward
      }
    }
  }
  else if (bitRead(state, wander)) {
    Serial.println("nothing to see here, I need to look for a wall");
    stop();
    delay(500);
    //reverse(half_rotation);
    spin(half_rotation, 0);
    forward(one_rotation);
    pivot(quarter_rotation,1);
  }
}

/*
  This is a sample updateSensors() function and it should be updated along with the description to reflect what you actually implemented
  to meet the lab requirements.
 */
void updateSensors() {
	test_state = !test_state;					//LED to test the heartbeat of the timer interrupt routine
	digitalWrite(PIN_LED_TEST, test_state);		//Toggles the LED to let you know the timer is working
	flag = 0;       	//clear all sensor flags
	state = 0;      	//clear all state flags
	updateIR();     	//update IR readings and update flag variable and state machine
	updateSonar();  	//update Sonar readings and update flag variable and state machine
	updateState();  	//update State Machine based upon sensor readings
}

/*
   This is a sample updateState() function, the description and code should be updated to reflect the actual state machine that you will implement
   based upon the the lab requirements.
*/
void updateState() {
  if (!(flag)) { //no sensors triggered
    //set random wander bit
    Serial.println("\tset random wander state");
    bitSet(state, wander);//set the wander state
    //clear all other bits
    bitClear(state, fright);//clear follow wall state
    bitClear(state, fleft);//clear follow wall state
    bitClear(state, center);//clear follow wall state
  }
  else if (bitRead(flag, obRight) && !bitRead(flag, obLeft) ) {
    Serial.println("\tset follow right state");
    bitSet(state, fright);    //set RIGHT WALL state
    //clear all other bits
    bitClear(state, wander);  //clear wander state
    bitClear(state, fleft);   //clear follow wall state
    bitClear(state, center);  //clear follow wall state
  }
  else if (bitRead(flag, obLeft) && !bitRead(flag, obRight) ) {
    Serial.println("\tset follow left state");
    bitSet(state, fleft);     //set left wall state
    //clear all other bits
    bitClear(state, fright);  //clear follow wall state
    bitClear(state, wander);  //clear wander state
    bitClear(state, center);  //clear follow wall state
  }
  else if (bitRead(flag, obLeft) && bitRead(flag, obRight) ) {
    Serial.println("\tset follow hallway state");
    bitSet(state, center);      //set the hallway state
    //clear all other bits
    bitClear(state, fright);    //clear follow wall state
    bitClear(state, wander);    //clear wander state
    bitClear(state, fleft);     //clear follow wall state
  }
}





