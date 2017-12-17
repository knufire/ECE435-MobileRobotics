/*&StateMachine.ino
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

  Hardware Connections:
  Stepper Enable          Pin 48
  Right Stepper Step      Pin  46
  Right Stepper Direction Pin 53
  Left Stepper Step       Pin 44
  Left Stepper Direction  Pin 49

  Front IR    A8
  Back IR     A9
  Right IR    A10
  Left IR     A11
  Left Sonar  A12
  Right Sonar A13
  Button      A15
*/

#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library
#include <NewPing.h> //include sonar library
#include <TimerOne.h>

//define stepper motor pin numbers
const int rtStepPin = 50; //right stepper motor step pin
const int rtDirPin = 51;  // right stepper motor direction pin
const int ltStepPin = 52; //left stepper motor step pin
const int ltDirPin = 53;  //left stepper motor direction pin

const int greenLED = 6;
const int redLED = 7;

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin);//create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);//create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;//create instance to control multiple steppers at the same time

//define stepper motor constants
#define stepperEnable 48    //stepper enable pin on stepStick
#define enableLED 13 //stepper enabled LED
#define stepperEnTrue false //variable for enabling stepper motor
#define stepperEnFalse true //variable for disabling stepper motor
#define test_led 13 //test led to test interrupt heartbeat

#define robot_spd 800 //set robot speed
#define max_accel 5000//maximum robot acceleration
#define max_spd 1000//maximum robot speed

#define quarter_rotation 200  //stepper quarter rotation
#define half_rotation 400     //stepper half rotation
#define one_rotation  800     //stepper motor runs in 1/4 steps so 800 steps is one full rotation
#define two_rotation  1600    //stepper motor 2 rotations
#define three_rotation 2400   //stepper rotation 3 rotations
#define four_rotation 3200    //stepper rotation 3 rotations
#define five_rotation 4000    //stepper rotation 3 rotations


#define irFront   A7    //front IR analog pin
#define irRear    A9    //back IR analog pin
#define irRight   A10   //right IR analog pin
#define irLeft    A11   //left IR analog pin
#define snrLeft   A12   //front left sonar
#define snrRight  A13  //front right sonar
#define button    A15    //pushbutton

NewPing sonarLt(snrLeft, snrLeft);    //create an instance of the left sonar
NewPing sonarRt(snrRight, snrRight);  //create an instance of the right sonar

#define irThresh    5 // The IR threshold for presence of an obstacle
#define snrThresh   5   // The sonar threshold for presence of an obstacle
#define minThresh   0   // The sonar minimum threshold to filter out noise
#define stopThresh  150 // If the robot has been stopped for this threshold move
#define baud_rate 9600//set serial communication baud rate

int irFrontArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 front IR readings
int irRearArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 back IR readings
int irRightArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 right IR readings
int irLeftArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 left IR readings
float irFrontAvg;  //variable to hold average of current front IR reading
float irLeftAvg;   //variable to hold average of current left IR reading
float irRearAvg;   //variable to hold average of current rear IR reading
float irRightAvg;   //variable to hold average of current right IR reading
int irIdx = 0;//index for 5 IR readings to take the average

int srLeftArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 left sonar readings
int srRightArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 right sonar readings
int srIdx = 0;//index for 5 sonar readings to take the average
unsigned int srLeft;   //variable to hold average of left sonar current reading
unsigned int srRight;  //variable to hold average or right sonar current reading
float srLeftAvg;  //variable to holde left sonar data
float srRightAvg; //variable to hold right sonar data

volatile boolean test_state; //variable to hold test led state for timer interrupt

#define CONST_FEET_TO_STEPS 912.2
#define CONST_SPIN_DEGREES_TO_STEPS 5.35


float robotX = 0;
float robotY = 0;
int robotAngle = 0;
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
byte layers = 1; //[wander runAway collide]
//bit definitions for layers
#define cLayer 0
#define rLayer 1
#define wLayer 2

#define timer_int 250000 // 1/2 second (500000 us) period for timer interrupt

void setup()
{
  //stepper Motor set up
  pinMode(rtStepPin, OUTPUT);//sets pin as outputr
  pinMode(rtDirPin, OUTPUT);//sets pin as output
  pinMode(ltStepPin, OUTPUT);//sets pin as output
  pinMode(ltDirPin, OUTPUT);//sets pin as output
  pinMode(stepperEnable, OUTPUT);//sets pin as output
  pinMode(greenLED, OUTPUT);
  pinMode(redLED, OUTPUT);
  digitalWrite(stepperEnable, stepperEnFalse);//turns off the stepper motor driver
  pinMode(enableLED, OUTPUT);//set LED as output
  digitalWrite(enableLED, HIGH);//turn off enable LED
  digitalWrite(greenLED, HIGH);
  digitalWrite(redLED, HIGH);
  stepperRight.setMaxSpeed(max_spd);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(max_accel);//set desired acceleration in steps/s^2
  stepperLeft.setMaxSpeed(max_spd);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperLeft.setAcceleration(max_accel);//set desired acceleration in steps/s^2

  stepperRight.setSpeed(robot_spd);//set right motor speed
  stepperLeft.setSpeed(robot_spd);//set left motor speed

  steppers.addStepper(stepperRight);//add right motor to MultiStepper
  steppers.addStepper(stepperLeft);//add left motor to MultiStepper
  digitalWrite(stepperEnable, stepperEnTrue);//turns on the stepper motor driver
  digitalWrite(enableLED, HIGH);//turn on enable LED
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
  digitalWrite(test_led, test_state);
  flag = 0;       //clear all sensor flags
  state = 0;      //clear all state flags
  updateIR();     //update IR readings and update flag variable and state machine
  //updateSonar();  //update Sonar readings and update flag variable and state machine
  updateState();  //update State Machine based upon sensor readings
  //delay(1000);     //added so that you can read the data on the serial monitor
}

/*
   This is a sample updateIR() function, the description and code should be updated to take an average, consider all sensor and reflect
   the necesary changes for the lab requirements.
*/

void updateIR() {
  int front, back, left, right;
  front = analogRead(irFront);
  back = analogRead(irRear);
  left = analogRead(irLeft);
  right = analogRead(irRight);
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
  if (irRightAvg < irThresh && !bitRead(flag, obRight)) {
    Serial.print("set right obstacle bit");
    Serial.println(irRightAvg);
    bitSet(flag, obRight);//set the right obstacle
  }
  else {
    bitClear(flag, obRight);//clear the right obstacle
  }
  if (irLeftAvg < irThresh && !bitRead(flag, obLeft)) {
    Serial.print("set left obstacle bit");
    Serial.println(irLeftAvg);
    bitSet(flag, obLeft);//set the left obstacle
  }
  else {
    bitClear(flag, obLeft);//clear the left obstacle
  }
  if (irFrontAvg < irThresh && !bitRead(flag, obFront)) {
    Serial.print("set front obstacle bit");
    Serial.println(irFrontAvg);
    bitSet(flag, obFront);//set the front obstacle
  }
  else {
    bitClear(flag, obFront);//clear the front obstacle
  }
  if (irRearAvg < irThresh && !bitRead(flag, obRear)) {
    Serial.print("set back obstacle bit");
    Serial.println(irRearAvg);
    bitSet(flag, obRear);//set the back obstacle
  }
  else {
    bitClear(flag, obRear);//clear the back obstacle
  }
}

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
    bitSet(flag, obFRight);//set the front right obstacle
  }
  else {
    bitClear(flag, obFRight);//clear the front right obstacle
  }
  if (srLeftAvg < snrThresh && srLeftAvg > minThresh) {
//    Serial.println("set front left obstacle bit");
    bitSet(flag, obFLeft);//set the front left obstacle
  }
  else {
    bitClear(flag, obFLeft);//clear the front left obstacle
  }
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
//    Serial.print("OX: ");
//    Serial.print(obstacleX);
//    Serial.print("\t OY: ");
//    Serial.println(obstacleY);
    float angleInRad = atan2(obstacleY, obstacleX);
    float angleInDeg = angleInRad / PI * 180;
    if (mult == 1) {
        digitalWrite(greenLED, HIGH);
        digitalWrite(redLED, LOW);
      } else {
        digitalWrite(greenLED, LOW);
        digitalWrite(redLED, HIGH);
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
  //    Serial.print("OX: ");
  //    Serial.print(obstacleX);
  //    Serial.print("\t OY: ");
  //    Serial.println(obstacleY);
      if (multiplier == 1) {
        digitalWrite(greenLED, HIGH);
        digitalWrite(redLED, LOW);
      } else {
        digitalWrite(greenLED, LOW);
        digitalWrite(redLED, HIGH);
      }
      float angleInRad = atan2(obstacleY, obstacleX);
      float angleInDeg = angleInRad / PI * 180;
      goToAngle(angleInDeg + robotAngle);
      forward(quarter_rotation * multiplier);
    }
 }
}

void forward(int rot) {
  long positions[2]; // Array of desired stepper positions
  stepperRight.setCurrentPosition(0);
  stepperLeft.setCurrentPosition(0);
  positions[0] = stepperRight.currentPosition() + rot;  //right motor absolute position
  positions[1] = stepperLeft.currentPosition() + rot;   //left motor absolute position
  stepperRight.move(positions[0]);  //move right motor to position
  stepperLeft.move(positions[1]);   //move left motor to position
  Serial.print("Going forward: ");
  Serial.println(rot);
  runToStop();//run until the robot reaches the target
  float dx = cos(robotAngle * PI / 180) * rot / CONST_FEET_TO_STEPS;
  float dy = sin(robotAngle * PI / 180) * rot / CONST_FEET_TO_STEPS;
  robotX = robotX + dx;
  robotY = robotY + dy;
  Serial.print("Robot X: ");
  Serial.print(robotX);
  Serial.print("\t Robot Y: ");
  Serial.print(robotY);
  Serial.print("\t Robot T: ");
  Serial.println(robotAngle);
}

void stop() {
  stepperRight.stop();
  stepperLeft.stop();
}

void goToAngle(float degrees) {
  Serial.print("Going to Angle: ");
  Serial.print(degrees);
  Serial.print("\n");
  float dAngle = degrees - robotAngle;
  while(dAngle > 180){
    dAngle = dAngle - 360;
  }
  while(dAngle < -180){
    dAngle = dAngle + 360;
  }
//  Serial.print("delta Angle: ");
//  Serial.print(dAngle);
//  Serial.print("\n");
  long numSteps = dAngle * CONST_SPIN_DEGREES_TO_STEPS;
  stepperRight.move(numSteps);
  stepperLeft.move(numSteps * -1);
  runToStop();
  while (degrees < 0) {
    degrees += 360;
  }
  while (degrees > 360) {
    degrees -= 360;
  }
  robotAngle = degrees;
}

/*This function, runToStop(), will run the robot until the target is achieved and
   then stop it
*/
void runToStop ( void ) {
  int runNow = 1;
  while (runNow) {
    if (!stepperRight.run() ) {
      runNow = 0;
      stop();
    }
    if (!stepperLeft.run()) {
      runNow = 0;
      stop();
    }
  }
}

