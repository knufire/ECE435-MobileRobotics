/*StateMachine.ino
  Author: Carlotta. A. Berry
  Date: December 3, 2016
  This program will provide a template for an example of implementing a behavior-based control architecture
  for a mobile robot to implement obstacle avoidance and random wander. There are many ways to create a state machine
  and this is just one. It is to help get you started and brainstorm ideas, you are not required to use it.
  Feel free to create your own version of state machine.

  The flag byte (8 bits) variable will hold the IR and sonar data [X X snrRight snrLeft irLeft irRight irRear irFront]
  The state byte (8 bits) variable will hold the state information as well as motor motion [X X X wander runAway collide rev fwd]

  Use the following functions to read, clear and set bits in the byte
  bitRead(state, wander)) { // check if the wander state is active
  bitClear(state, wander);//clear the the wander state
  bitSet(state, wander);//set the wander state
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

//bit definitions for robot motion and state byte
#define fwd     0
#define rev     1
#define collide 2
#define runAway 3
#define wander  4

//define layers of subsumption architecture that are active
byte layers = 2; //[wander runAway collide]
//bit definitions for layers
#define cLayer 0
#define rLayer 1
#define wLayer 2

#define timer_int 250000 // 1/2 second (500000 us) period for timer interrupt

void setup()
{
	driveSetup();

	pinMode(PIN_GREEN_LED, OUTPUT);
	pinMode(PIN_RED_LED, OUTPUT);
	pinMode(PIN_LED_ENABLE, OUTPUT);//set LED as output
	digitalWrite(PIN_LED_ENABLE, HIGH);//turn off enable LED
	digitalWrite(PIN_GREEN_LED, HIGH);
	digitalWrite(PIN_RED_LED, HIGH);
	//Timer Interrupt Set Up
	Timer1.initialize(timer_int);         // initialize timer1, and set a period in microseconds
	Timer1.attachInterrupt(updateSensors);  // attaches updateSensors() as a timer overflow interrupt

	Serial.begin(baud_rate);//start serial communication in order to debug the software while coding
	delay(3000);//wait 3 seconds before robot moves
}

void loop()
{
	robotMotion();  //execute robot motions based upon sensor data and current state
	//delay(100);
}


/*
  This is a sample updateSensors() function and it should be updated along with the description to reflect what you actually implemented
  to meet the lab requirements.
 */
void updateSensors() {
	//  Serial.print("updateSensors\t");
	//  Serial.println(test_state);
	//test_state = !test_state;//LED to test the heartbeat of the timer interrupt routine
	//digitalWrite(enableLED, test_state);  // Toggles the LED to let you know the timer is working
	test_state = !test_state;
	digitalWrite(PIN_LED_TEST, test_state);
	flag = 0;       //clear all sensor flags
	state = 0;      //clear all state flags
	updateIR();     //update IR readings and update flag variable and state machine
	//updateSonar();  //update Sonar readings and update flag variable and state machine
	updateState();  //update State Machine based upon sensor readings
	//delay(1000);     //added so that you can read the data on the serial monitor
}

/*
   This is a sample updateState() function, the description and code should be updated to reflect the actual state machine that you will implement
   based upon the the lab requirements.
 */
void updateState() {
	if (!(flag)) { //no sensors triggered
		bitSet(state, fwd); //set forward motion
		bitClear(state, collide);//clear collide state
		count--;//decrement collide counter
	}
	else if (flag & 0b1) { //front sensors triggered
		bitClear(state, fwd); //clear reverse motion
		bitSet(state, collide);//set collide state
		count++;
	}
}

/*
   This is a sample robotMotion() function, the description and code should be updated to reflect the actual robot motion function that you will implement
   based upon the the lab requirements.  Some things to consider, you cannot use a blocking motor function because you need to use sensor data to update
   movement.  You also need to continue to poll the sensors during the motion and update flags and state because this will serve as your interrupt to
   stop or change movement.
 */
void robotMotion() {
	flag = irFlag & snrFlag;
	if (layers == 0) {
		if ((flag & 0b1) || bitRead(state, collide)) { //check for a collide state
			stop();
		}
		else{
			forward(quarter_rotation);//move forward as long as all sensors are clear
		}
	} else if (layers == 1) {
		Serial.println("-----------Wander with obstacles--------------");
		int randomAngle = random(0 ,360);
		//Serial.println(randomAngle);
		float obstacleX = cos(randomAngle * PI / 180);
		float obstacleY = sin(randomAngle * PI / 180);
		//    float obstacleX = 0;
		//    float obstacleY = 0;
		int mult = 1;
		if (bitRead(flag, obFront)) {
			Serial.println("Front obstacle");
			obstacleX--;
			mult = 4;
		}
		if (bitRead(flag, obRear)) {
			Serial.println("Rear obstacle");
			obstacleX++;
			mult = 4;
		}
		if (bitRead(flag, obLeft)) {
			Serial.println("Left obstacle");
			obstacleY++;
			mult = 4;
		}
		if (bitRead(flag, obRight)) {
			Serial.println("Right obstacle");
			obstacleY--;
			mult = 4;
		}
//		Serial.print("OX: ");
//		Serial.print(obstacleX);
//		Serial.print("\t OY: ");
//		Serial.println(obstacleY);
		float angleInRad = atan2(obstacleY, obstacleX);
		float angleInDeg = angleInRad / PI * 180;
		if (mult == 1) {
			digitalWrite(PIN_GREEN_LED, HIGH);
			digitalWrite(PIN_RED_LED, LOW);
		} else {
			digitalWrite(PIN_GREEN_LED, LOW);
			digitalWrite(PIN_RED_LED, HIGH);
		}
		goToAngle(angleInDeg + robotAngle);
		forward(half_rotation * mult);
	} else if (layers == 2) {
		if (abs(robotX - goalX) < 0.2 && abs(robotY - goalY) < 0.2) {
			stop();
		}
		else {
			//Serial.println(randomAngle);
			float goalAngle = atan2((goalY-robotY),(goalX-robotX));
			float robotRadAngle = robotAngle * PI / 180;
			float dAngle = goalAngle - robotRadAngle;
			float obstacleX = cos(dAngle);
			float obstacleY = sin(dAngle);
			int multiplier = 1;
			if (bitRead(flag, obFront)) {
				Serial.println("Front obstacle");
				forward(-1 * half_rotation);
				multiplier = 2;
				obstacleX--;
			}
			if (bitRead(flag, obRear)) {
				Serial.println("Rear obstacle");
				multiplier = 2;
				obstacleX++;
			}
			if (bitRead(flag, obLeft)) {
				Serial.println("Left obstacle");
				multiplier = 2;
				obstacleY++;
			}
			if (bitRead(flag, obRight)) {
				Serial.println("Right obstacle");
				obstacleY--;
				multiplier = 2;
			}
//			Serial.print("OX: ");
//			Serial.print(obstacleX);
//			Serial.print("\t OY: ");
//			Serial.println(obstacleY);
			if (multiplier == 1) {
				digitalWrite(PIN_GREEN_LED, HIGH);
				digitalWrite(PIN_RED_LED, LOW);
			} else {
				digitalWrite(PIN_GREEN_LED, LOW);
				digitalWrite(PIN_RED_LED, HIGH);
			}
			float angleInRad = atan2(obstacleY, obstacleX);
			float angleInDeg = angleInRad / PI * 180;
			goToAngle(angleInDeg + robotAngle);
			forward(quarter_rotation * multiplier);
		}
	}
}



