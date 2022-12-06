/**
 * Sum numbers in a vector.
 *
 * This sum is the arithmetic sum, not some other kind of sum that only
 * mathematicians have heard of.
 *
 * @param values Container whose values are summed.
 * @return sum of `values`, or 0.0 if `values` is empty.
 */

/**
 * Author Names: Sal Altobellis & Chirag Sirigere
 * Date Created: 12/2/2022
 * Program Name: Walter_main.cpp
 * Program Layout:
 *  ECE425
 *  |- .pio
 *  |- .vscode
 *  |- include
 *  |- lib
 *  |- src
 *  |-- main.cpp
 *  |- test
 *  |- .gitignore
 *  |- platformio.ini
 *  |- README.md
 * 
 * Program Description:
 *  This program will introduce using the stepper motor library to create motion algorithms for the robot.
 *  The motions will be go to angle, go to goal, move in a circle, square, figure eight and teleoperation (stop, forward, spin, reverse, turn)
 *  It will also include wireless commmunication for remote control of the robot by using a game controller or serial monitor.
 * 
 * Key Functions:
 *  moveCircle - given the diameter in inches and direction of clockwise or counterclockwise, move the robot in a circle with that diameter
 *  moveFigure8 - given the diameter in inches, use the moveCircle() function with direction input to create a Figure 8
 *  forward, reverse - both wheels move with same velocity, same direction
 *  pivot - one wheel stationary, one wheel moves forward or back
 *  spin - both wheels move with same velocity opposite direction
 *  turn - both wheels move with same direction different velocity
 *  stop - both wheels stationary
 *  
 * Hardware Connections:
 *  Arduino pin mappings: https://www.arduino.cc/en/Hacking/PinMapping2560
 * 
 *  A4988 Stepper Motor Driver Pinout: https://www.pololu.com/product/1182
 *    digital pin 48 - enable PIN on A4988 Stepper Motor Driver StepSTICK
 *    digital pin 50 - right stepper motor step pin
 *    digital pin 51 - right stepper motor direction pin
 *    digital pin 52 - left stepper motor step pin
 *    digital pin 53 - left stepper motor direction pin
 *    digital pin 13 - enable LED on microcontroller
 * 
 *  LED Pinout:
 *    digital pin 6 - red LED in series with 220 ohm resistor
 *    digital pin 7 - green LED in series with 220 ohm resistor
 *    digital pin 8 - yellow LED in series with 220 ohm resistor
 * 
 *  Encoder Pinout:
 *    digital pin 18 - left encoder pin
 *    digital pin 19 - right encoder pin
 * 
 *  IMU Pinout:
 *    digital pin 2 - IMU INT
 *    digital pin 20 - IMU SDA
 *    digital pin 21 - IMU SCL
 * 
 * Resources:
 *  Interrupts:
 *    https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
 *    https://www.arduino.cc/en/Tutorial/CurieTimer1Interrupt
 *    https://playground.arduino.cc/code/timer1
 *    https://playground.arduino.cc/Main/TimerPWMCheatsheet
 *    http://arduinoinfo.mywikis.net/wiki/HOME
*/

//includew all necessary libraries
#include <Arduino.h>          //include for PlatformIO Ide
#include <AccelStepper.h>     //include the stepper motor library
#include <MultiStepper.h>     //include multiple stepper motor library
#include <Adafruit_MPU6050.h> //Include library for MPU6050 IMU
#include <SoftwareSerial.h>   //include Bluetooth module

//state LEDs connections
#define blueLED 5             //blue LED for displaying states
#define grnLED 6              //green LED for displaying states
#define ylwLED 7              //yellow LED for displaying states
#define enableLED 13          //stepper enabled LED

//define motor pin numbers
#define stepperEnable 48      //stepper enable pin on stepStick 
#define rtStepPin 50          //right stepper motor step pin 
#define rtDirPin 51           // right stepper motor direction pin 
#define ltStepPin 52          //left stepper motor step pin 
#define ltDirPin 53           //left stepper motor direction pin 

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin); //create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin); //create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers; //create instance to control multiple steppers at the same time

#define stepperEnTrue false   //variable for enabling stepper motor
#define stepperEnFalse true   //variable for disabling stepper motor

int pauseTime = 2500;         //time before robot moves
int stepTime = 500;           //delay time between high and low on step pin
int wait_time = 1000;         //delay for printing data

float widthBot = 23.3; //cm


//define encoder pins
#define LEFT 0                      //left encoder
#define RIGHT 1                     //right encoder
const int ltEncoder = 18;           //left encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
const int rtEncoder = 19;           //right encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
volatile long encoder[2] = {0, 0};  //interrupt variable to hold number of encoder counts (left, right)
int lastSpeed[2] = {0, 0};          //variable to hold encoder speed (left, right)
int accumTicks[2] = {0, 0};         //variable to hold accumulated ticks since last reset

//IMU object
Adafruit_MPU6050 mpu;

//Bluetooth module connections
#define BTTX 10               // TX on chip to pin 10 on Arduino Mega
#define BTRX 11               // RX on chip to pin 11 on Arduino Mega
SoftwareSerial BTSerial(BTTX, BTRX);

// Helper Functions
/**
 * Interrupt function to count left encoder ticks.
 */
void LwheelSpeed()
{
  encoder[LEFT] ++;  //count the left wheel encoder interrupts
}

/**
 * Interrupt function to count right encoder ticks
 */
void RwheelSpeed()
{
  encoder[RIGHT] ++; //count the right wheel encoder interrupts
}

// Initialization Functions
/**
 * Function to initialize Bluetooth
 */
void init_BT(){
  Serial.println("Goodnight moon!");
  BTSerial.println("Hello, world?");
}

/**
 * Function to initialize IMU
 */
void init_IMU(){
  Serial.println("Adafruit MPU6050 init!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
}

//function to set all stepper motor variables, outputs and LEDs
void init_stepper(){
  pinMode(rtStepPin, OUTPUT);//sets pin as output
  pinMode(rtDirPin, OUTPUT);//sets pin as output
  pinMode(ltStepPin, OUTPUT);//sets pin as output
  pinMode(ltDirPin, OUTPUT);//sets pin as output
  pinMode(stepperEnable, OUTPUT);//sets pin as output
  digitalWrite(stepperEnable, stepperEnFalse);//turns off the stepper motor driver
  pinMode(enableLED, OUTPUT);//set enable LED as output
  digitalWrite(enableLED, LOW);//turn off enable LED
  pinMode(blueLED, OUTPUT);//set red LED as output
  pinMode(grnLED, OUTPUT);//set green LED as output
  pinMode(ylwLED, OUTPUT);//set yellow LED as output
  digitalWrite(blueLED, HIGH);//turn on red LED
  digitalWrite(ylwLED, HIGH);//turn on yellow LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  delay(pauseTime / 5); //wait 0.5 seconds
  digitalWrite(blueLED, LOW);//turn off red LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  digitalWrite(grnLED, LOW);//turn off green LED

  stepperRight.setMaxSpeed(1500);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(10000);//set desired acceleration in steps/s^2
  stepperLeft.setMaxSpeed(1500);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperLeft.setAcceleration(10000);//set desired acceleration in steps/s^2
  steppers.addStepper(stepperRight);//add right motor to MultiStepper
  steppers.addStepper(stepperLeft);//add left motor to MultiStepper
  digitalWrite(stepperEnable, stepperEnTrue);//turns on the stepper motor driver
  digitalWrite(enableLED, HIGH);//turn on enable LED
}

//function prints encoder data to serial monitor
void print_encoder_data() {
  static unsigned long timer = 0;                           //print manager timer
  if (millis() - timer > 100) {                             //print encoder data every 100 ms or so
    lastSpeed[LEFT] = encoder[LEFT];                        //record the latest left speed value
    lastSpeed[RIGHT] = encoder[RIGHT];                      //record the latest right speed value
    accumTicks[LEFT] = accumTicks[LEFT] + encoder[LEFT];    //record accumulated left ticks
    accumTicks[RIGHT] = accumTicks[RIGHT] + encoder[RIGHT]; //record accumulated right ticks
    Serial.println("Encoder value:");
    Serial.print("\tLeft:\t");
    Serial.print(encoder[LEFT]);
    Serial.print("\tRight:\t");
    Serial.println(encoder[RIGHT]);
    Serial.println("Accumulated Ticks: ");
    Serial.print("\tLeft:\t");
    Serial.print(accumTicks[LEFT]);
    Serial.print("\tRight:\t");
    Serial.println(accumTicks[RIGHT]);
    encoder[LEFT] = 0;                          //clear the left encoder data buffer
    encoder[RIGHT] = 0;                         //clear the right encoder data buffer
    timer = millis();                           //record current time since program started
  }
}

//function to print IMU data to the serial monitor
void print_IMU_data(){
    /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
}

//function to send and receive data with the Bluetooth
void Bluetooth_comm(){
  String data="";
  if (Serial.available()) {
     while (Serial.available()){
      char nextChar = Serial.read();
      data = data + String(nextChar); 
      if (nextChar == ';') {
        break;
      }
     }
    Serial.println(data);
    BTSerial.println(data);
  }
  
  if (BTSerial.available()) {
    while (BTSerial.available()){
      char nextChar = BTSerial.read();
      data = data + String(nextChar); 
      if (nextChar == ';') {
        break;
      }
    }
    Serial.println(data);
    BTSerial.println(data);
  }
}
  
  
/*function to run both wheels to a position at speed*/
void runAtSpeedToPosition() {
  stepperRight.runSpeedToPosition();
  stepperLeft.runSpeedToPosition();
}

/*function to run both wheels continuously at a speed*/
void runAtSpeed ( void ) {
  while (stepperRight.runSpeed() || stepperLeft.runSpeed()) {
  }
}

/*This function, runToStop(), will run the robot until the target is achieved and
   then stop it
*/
void runToStop ( void ) {
  int runNow = 1;
  int rightStopped = 0;
  int leftStopped = 0;

  while (runNow) {
    if (!stepperRight.run()) {
      rightStopped = 1;
      stepperRight.stop();//stop right motor
    }
    if (!stepperLeft.run()) {
      leftStopped = 1;
      stepperLeft.stop();//stop ledt motor
    }
    if (rightStopped && leftStopped) {
      runNow = 0;
    }
  }
}


/*
   The move1() function will move the robot forward one full rotation and backwared on
   full rotation.  Recall that that there 200 steps in one full rotation or 1.8 degrees per
   step. This function uses setting the step pins high and low with delays to move. The speed is set by
   the length of the delay.
*/
void move1() {
  digitalWrite(blueLED, HIGH);//turn on red LED
  digitalWrite(grnLED, LOW);//turn off green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  digitalWrite(ltDirPin, HIGH); // Enables the motor to move in a particular direction
  digitalWrite(rtDirPin, HIGH); // Enables the motor to move in a particular direction
  // Makes 800 pulses for making one full cycle rotation
  for (int x = 0; x < 800; x++) {
    digitalWrite(rtStepPin, HIGH);
    digitalWrite(ltStepPin, HIGH);
    delayMicroseconds(stepTime);
    digitalWrite(rtStepPin, LOW);
    digitalWrite(ltStepPin, LOW);
    delayMicroseconds(stepTime);
  }
  delay(1000); // One second delay
  digitalWrite(ltDirPin, LOW); // Enables the motor to move in opposite direction
  digitalWrite(rtDirPin, LOW); // Enables the motor to move in opposite direction
  // Makes 800 pulses for making one full cycle rotation
  for (int x = 0; x < 800; x++) {
    digitalWrite(rtStepPin, HIGH);
    digitalWrite(ltStepPin, HIGH);
    delayMicroseconds(stepTime);
    digitalWrite(rtStepPin, LOW);
    digitalWrite(ltStepPin, LOW);
    delayMicroseconds(stepTime);
  }
  delay(1000); // One second delay
}

/*
   The move2() function will use AccelStepper library functions to move the robot
   move() is a library function for relative movement to set a target position
   moveTo() is a library function for absolute movement to set a target position
   stop() is a library function that causes the stepper to stop as quickly as possible
   run() is a library function that uses accel and decel to achieve target position, no blocking
   runSpeed() is a library function that uses constant speed to achieve target position, no blocking
   runToPosition() is a library function that uses blocking with accel/decel to achieve target position
   runSpeedToPosition() is a library function that uses constant speed to achieve target posiiton, no blocking
   runToNewPosition() is a library function that uses blocking with accel/decel to achieve target posiiton
*/
void move2() {
  digitalWrite(blueLED, LOW);//turn off red LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  stepperRight.moveTo(800);//move one full rotation forward relative to current position
  stepperLeft.moveTo(800);//move one full rotation forward relative to current position
  stepperRight.setSpeed(1000);//set right motor speed
  stepperLeft.setSpeed(1000);//set left motor speed
  stepperRight.runSpeedToPosition();//move right motor
  stepperLeft.runSpeedToPosition();//move left motor
  runToStop();//run until the robot reaches the target
  delay(1000); // One second delay
  stepperRight.moveTo(0);//move one full rotation backward relative to current position
  stepperLeft.moveTo(0);//move one full rotation backward relative to current position
  stepperRight.setSpeed(1000);//set right motor speed
  stepperLeft.setSpeed(1000);//set left motor speed
  stepperRight.runSpeedToPosition();//move right motor
  stepperLeft.runSpeedToPosition();//move left motor
  runToStop();//run until the robot reaches the target
  delay(1000); // One second delay
}

/*
   The move3() function will use the MultiStepper() class to move both motors at once
   move() is a library function for relative movement to set a target position
   moveTo() is a library function for absolute movement to set a target position
   stop() is a library function that causes the stepper to stop as quickly as possible
   run() is a library function that uses accel and decel to achieve target position, no blocking
   runSpeed() is a library function that uses constant speed to achieve target position, no blocking
   runToPosition() is a library function that uses blocking with accel/decel to achieve target position
   runSpeedToPosition() is a library function that uses constant speed to achieve target posiiton, no blocking
   runToNewPosition() is a library function that uses blocking with accel/decel to achieve target posiiton
*/
void move3() {
  digitalWrite(blueLED, LOW);//turn off red LED
  digitalWrite(grnLED, LOW);//turn off green LED
  digitalWrite(ylwLED, HIGH);//turn on yellow LED
  long positions[2]; // Array of desired stepper positions
  positions[0] = 800;//right motor absolute position
  positions[1] = 800;//left motor absolute position
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1000);//wait one second
  // Move to a different coordinate
  positions[0] = 0;//right motor absolute position
  positions[1] = 0;//left motor absolute position
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1000);//wait one second
}

/*this function will move to target at 2 different speeds*/
void move4() {

  int leftPos = 5000;//right motor absolute position
  int rightPos = 1000;//left motor absolute position
  int leftSpd = 5000;//right motor speed
  int rightSpd = 1000; //left motor speed

  digitalWrite(blueLED, HIGH);//turn on red LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED

  //Uncomment the next 4 lines for absolute movement
  stepperLeft.setCurrentPosition(0);//set left wheel position to zero
  stepperRight.setCurrentPosition(0);//set right wheel position to zero
  stepperLeft.moveTo(leftPos);//move left wheel to absolute position
  stepperRight.moveTo(rightPos);//move right wheel to absolute position

  //Unomment the next 2 lines for relative movement
  stepperLeft.move(leftPos);//move left wheel to relative position
  stepperRight.move(rightPos);//move right wheel to relative position

  //Uncomment the next two lines to set the speed
  stepperLeft.setSpeed(leftSpd);//set left motor speed
  stepperRight.setSpeed(rightSpd);//set right motor speed
  runAtSpeedToPosition();//run at speed to target position
}

/*This function will move continuously at 2 different speeds*/
void move5() {
  digitalWrite(blueLED, LOW);//turn off red LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  digitalWrite(ylwLED, HIGH);//turn on yellow LED
  int leftSpd = 5000;//right motor speed
  int rightSpd = 1000; //left motor speed
  stepperLeft.setSpeed(leftSpd);//set left motor speed
  stepperRight.setSpeed(rightSpd);//set right motor speed
  runAtSpeed();
}




/*
  INSERT DESCRIPTION HERE, what are the inputs, what does it do, functions used

  If direction is greater than zero, pivot clockwise
  If direction is less than zero, pivot counterclockwise
  currently set to 90 degrees
*/
void pivot(int direction) {
  stepperRight.setCurrentPosition(0);
  stepperLeft.setCurrentPosition(0);
  if (direction > 0) {
    stepperRight.moveTo(1000);
    stepperRight.setSpeed(500);//set right motor speed
    stepperRight.runSpeedToPosition();//move right motor
  } else {
    stepperLeft.moveTo(1000);
    stepperLeft.setSpeed(500);//set left motor speed
    stepperLeft.runSpeedToPosition();//move left motor
  }
  runToStop();//run until the robot reaches the target
}

/*
  INSERT DESCRIPTION HERE, what are the inputs, what does it do, functions used

  If direction is greater than zero, spin clockwise
  If direction is less than zero, spin counterclockwise
*/
void spin(int direction) { //Currently overshoots sometimes
  if (direction > 0) {
    stepperRight.moveTo(-2000);
    stepperLeft.moveTo(2000);
  } else {
    stepperRight.moveTo(2000);
    stepperLeft.moveTo(-2000);
  }
  stepperRight.setSpeed(500);//set right motor speeda
  stepperLeft.setSpeed(500);//set left motor speed
  stepperRight.runSpeedToPosition();//move right motor
  stepperLeft.runSpeedToPosition();//move left motor
  runToStop();//run until the robot reaches the target
}

/*
  INSERT DESCRIPTION HERE, what are the inputs, what does it do, functions used
  If input is greater than zero, turn right, else turn left
*/
void turn(int direction) {
  stepperRight.setCurrentPosition(0);
  stepperLeft.setCurrentPosition(0);
  stepperLeft.setMaxSpeed(400);
  stepperRight.setMaxSpeed(400);

  float diam = 60.0;
  float inner = 2.0 * 3.14 * ((diam/2.0) - widthBot/2.0) * (90.0/360.0);
  float outer = 2.0 * 3.14 * ((diam/2.0) + widthBot/2.0) * (90.0/360.0);

  Serial.println(inner);
  Serial.println(outer);

  int innerSteps = inner * 29.958;
  int outerSteps = outer * 29.958;

  float outerSpeed = 100;
  float time = 1/(outerSpeed/outer);
  float innerSpeed = inner/time;
  //float innerSpeed = (250 * outer)/inner; //Works?
  Serial.println(innerSteps);
  Serial.println(outerSteps);
  long positions[2]; // Array of desired stepper positions
  
  //delay(1000);//wait one second

  if (direction > 0) {
    positions[0] = innerSteps;//right motor absolute position
    positions[1] = outerSteps;//left motor absolute position
    steppers.moveTo(positions);
    //stepperRight.moveTo(innerSteps);
    //stepperLeft.moveTo(outerSteps);
    //stepperRight.setSpeed(innerSpeed);//set right motor speed
    //stepperLeft.setSpeed(outerSpeed);//set left motor speed
  } else {
    positions[0] = outerSteps;//right motor absolute position
    positions[1] = innerSteps;//left motor absolute position
    steppers.moveTo(positions);
    //stepperRight.moveTo(outerSteps);
    //stepperLeft.moveTo(innerSteps);
    //stepperRight.setSpeed(outerSpeed);//set right motor speed
    //stepperLeft.setSpeed(innerSpeed);//set left motor speed
  }
  steppers.runSpeedToPosition(); // Blocks until all are in position

  /*
  stepperRight.setCurrentPosition(0);
  stepperLeft.setCurrentPosition(0);
  if (direction > 0) {
    stepperRight.moveTo(500);
    stepperLeft.moveTo(1000);
    stepperRight.setSpeed(250);//set right motor speed
    stepperLeft.setSpeed(500);//set left motor speed
    stepperRight.runSpeedToPosition();//move right motor
     stepperLeft.runSpeedToPosition();//move left motor
  } else {
    stepperRight.moveTo(1000);
    stepperLeft.moveTo(500);
    stepperRight.setSpeed(500);//set right motor speed
    stepperLeft.setSpeed(250);//set left motor speed
    stepperRight.runSpeedToPosition();//move right motor
     stepperLeft.runSpeedToPosition();//move left motor
  }
  runToStop();//run until the robot reaches the target
  */
}
/*
  INSERT DESCRIPTION HERE, what are the inputs, what does it do, functions used
  Moves Robot forward distance based on input
  Input is in cm
*/
void forward(int distance) {
  float steps = distance * 29.9586;
  float ticks = steps * (40/800);
  Serial.print(steps);
  stepperRight.moveTo(steps);//move one full rotation forward relative to current position
  stepperLeft.moveTo(steps);//move one full rotation forward relative to current position
  stepperRight.setSpeed(500);//set right motor speed
  stepperLeft.setSpeed(500);//set left motor speed
  stepperRight.runSpeedToPosition();//move right motor
  stepperLeft.runSpeedToPosition();//move left motor

  runToStop();//run until the robot reaches the target

  int errorLeft;
  int errorRight;
  if (ltEncoder < ticks) {
    errorLeft = ticks-ltEncoder;
  }
  if (rtEncoder < ticks) {
    errorRight = ticks-rtEncoder;
  }
  if (rtEncoder < ticks) {
    errorLeft = ticks-ltEncoder;
  }

  int correction = 40* max(errorLeft, errorRight);
  stepperRight.setCurrentPosition(0);
  stepperLeft.setCurrentPosition(0);
  stepperRight.moveTo(correction);//move one full rotation forward relative to current position
  stepperLeft.moveTo(correction);//move one full rotation forward relative to current position
  stepperRight.setSpeed(250);//set right motor speed
  stepperLeft.setSpeed(250);//set left motor speed
  stepperRight.runSpeedToPosition();//move right motor
  stepperLeft.runSpeedToPosition();//move left motor
  runToStop();

}
/*
  Moves Robot backwards distance based on input
  Input is in cm
*/
void reverse(int distance) {
  float steps = distance * 29.9586;
  Serial.print(steps);
  stepperRight.moveTo(-steps);//move one full rotation forward relative to current position
  stepperLeft.moveTo(-steps);//move one full rotation forward relative to current position
  stepperRight.setSpeed(500);//set right motor speed
  stepperLeft.setSpeed(500);//set left motor speed
  stepperRight.runSpeedToPosition();//move right motor
  stepperLeft.runSpeedToPosition();//move left motor
  runToStop();//run until the robot reaches the target
}
/*
  INSERT DESCRIPTION HERE, what are the inputs, what does it do, functions used
  Calls stop on both steppers
*/
void stop() {
  stepperRight.stop();
  stepperLeft.stop();
}




/*
  INSERT DESCRIPTION HERE, what are the inputs, what does it do, functions used
  If direction is greater than zero, spin clockwise
  If direction is less than zero, spin counterclockwise
  //Blue Led
*/
void moveCircle(int diam, int dir) {
  digitalWrite(blueLED, HIGH);//turn off red LED
  stepperRight.setCurrentPosition(0);
  stepperLeft.setCurrentPosition(0);
  stepperLeft.setMaxSpeed(500);
  stepperRight.setMaxSpeed(500);

  float dist = 2 * 3.14 * (diam/2);
  float inner = 2 * 3.14 * ((diam/2) - widthBot/2) * 0.9;
  float outer = 2 * 3.14 * ((diam/2) + widthBot/2) * 0.9;

  int innerSteps = inner * 29.958;
  int outerSteps = outer * 29.958;
  
  Serial.println(innerSteps);
  Serial.println(outerSteps);
  Serial.println(outerSteps*(40.0/800.0));
  Serial.println(innerSteps*(40.0/800.0));
  long positions[2]; // Array of desired stepper positions
  
  //delay(1000);//wait one second

  if (dir > 0) {
    positions[0] = innerSteps;//right motor absolute position
    positions[1] = outerSteps;//left motor absolute position
    steppers.moveTo(positions);
    //stepperRight.moveTo(innerSteps);
    //stepperLeft.moveTo(outerSteps);
    //stepperRight.setSpeed(innerSpeed);//set right motor speed
    //stepperLeft.setSpeed(outerSpeed);//set left motor speed
  } else {
    positions[0] = outerSteps;//right motor absolute position
    positions[1] = innerSteps;//left motor absolute position
    steppers.moveTo(positions);
    //stepperRight.moveTo(outerSteps);
    //stepperLeft.moveTo(innerSteps);
    //stepperRight.setSpeed(outerSpeed);//set right motor speed
    //stepperLeft.setSpeed(innerSpeed);//set left motor speed
  }
  steppers.runSpeedToPosition(); // Blocks until all are in position
  //runToStop();
  //stepperRight.runSpeedToPosition();//move right motor
  //stepperLeft.runSpeedToPosition();//move left motor
  //runToStop();
  digitalWrite(blueLED, LOW);//turn off red LED
}

/*
  The moveFigure8() function takes the diameter in inches as the input. It uses the moveCircle() function
  twice with 2 different direcitons to create a figure 8 with circles of the given diameter.
  Blue and Yellow
*/
void moveFigure8(int diam) {
  digitalWrite(blueLED, HIGH);//turn off red LED
  digitalWrite(ylwLED, HIGH);//turn on yellow LED

  float dist = 2 * 3.14 * (diam/2);
  float inner = 2 * 3.14 * ((diam/2) - widthBot/2) * 0.9;
  float outer = 2 * 3.14 * ((diam/2) + widthBot/2) * 0.9;

  float innerSteps = inner * 29.958;
  float outerSteps = outer * 29.958;
  int innerTicks = innerSteps * 40;
  int outerTicks = outerSteps * 40;

  float outerSpeed = 250;
  float time = 1/(outerSpeed/outer);
  float innerSpeed = inner/time;

  moveCircle(diam, -1);
  int errorInner;
  int errorOuter;
  if (innerTicks < ltEncoder) {
    errorInner = innerTicks - ltEncoder;
  }
  if (outerTicks < rtEncoder) {
    errorOuter = outerTicks - rtEncoder;
  }
  
  moveCircle(diam, 1);
  digitalWrite(blueLED, LOW);//turn off red LED
  digitalWrite(ylwLED, LOW);//turn on yellow LED
}



//// MAIN
void setup()
{
  int baudrate = 9600; //serial monitor baud rate'
  int BTbaud = 9600;  // HC-05 default speed in AT command more
  init_stepper(); //set up stepper motor

  attachInterrupt(digitalPinToInterrupt(ltEncoder), LwheelSpeed, CHANGE);    //init the interrupt mode for the left encoder
  attachInterrupt(digitalPinToInterrupt(rtEncoder), RwheelSpeed, CHANGE);   //init the interrupt mode for the right encoder

  BTSerial.begin(BTbaud);     //start Bluetooth communication
  Serial.begin(baudrate);     //start serial monitor communication

  while (!Serial)
    delay(10); // will pause until serial console opens
  
  //init_BT(); //initialize Bluetooth

  //init_IMU(); //initialize IMU
  
  Serial.println("Robot starting...");
  Serial.println("");
  delay(pauseTime); //always wait 2.5 seconds before the robot moves
}


void loop()
{
  //uncomment each function one at a time to see what the code does
  //move1();//call move back and forth function
  //move2();//call move back and forth function with AccelStepper library functions
  //move3();//call move back and forth function with MultiStepper library functions
  //move4(); //move to target position with 2 different speeds
  //move5(); //move continuously with 2 different speeds

  //Uncomment to read Encoder Data (uncomment to read on serial monitor)
  //print_encoder_data();   //prints encoder data

  //Uncomment to read IMU Data (uncomment to read on serial monitor)
  //print_IMU_data();         //print IMU data

  //Uncomment to Send and Receive with Bluetooth
  //Bluetooth_comm();
  /*
  delay(3000);
  forward(30);
  delay(1000);
  reverse(30);
  delay(1000);
  spin(1);
  delay(1000);
  pivot(1);
  delay(1000);
  turn(1);
  */
  moveCircle(100, 1);
  delay(1000);
  moveFigure8(100);
  
  print_encoder_data();

  //delay(wait_time);               //wait to move robot or read data
  delay(15000); 
}
