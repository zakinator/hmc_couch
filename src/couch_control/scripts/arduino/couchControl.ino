/*
 * Author: Zakkai Davidson
 * Date: 7/11/13
 * For use with the robotic couch in Prof. Dodds' lab. Note that
 * ESCs may need to be recalibrated using the EscCalibration file
 * 
 * Designed for use at Harvery Mudd College
 */

#include "Arduino.h"
#include <Servo.h> 
#include <ros.h>
#include <couch_control/MotorCommand.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

// pin constants
#define PINR 9
#define PINL 10
#define LEDR 3
#define LEDG 5
#define LEDB 6

// motor power constants
#define START_THRESH  5
#define NEUTRAL       90
#define MAX_FORWARD   160
#define MAX_BACKWARD  20
#define MIN_FORWARD   (NEUTRAL + START_THRESH)
#define MIN_BACKWARD  (NEUTRAL - START_THRESH)

// motor input constants
#define MAX_INPUT  300
#define MIN_INPUT -300


ros::NodeHandle nh; // creates node object to interact with ROS

Servo motorL; // Servo objects to control the ESCs
Servo motorR;

// motor power variables
int powerL = 90; // 90 is neutral throttle
int powerR = 90;


/*
 * callback function for commands sent over the couchMotors topic.
 * This function sends commands to the motors to drive at the
 * requested speed
 */
void motor_cb( const couch_control::MotorCommand& motor_cmd){
  // take in published message and select the needed data
  int left  = motor_cmd.left;
  int right = motor_cmd.right;
  
  // balance motor power depending on
  setMotors(left, right);
  
  // for debugging
  char charLeft  [5];
  char charRight [5];
  itoa(powerL, charLeft, 10);
  itoa(powerR, charRight, 10);
  nh.loginfo(charLeft);
  nh.loginfo(charRight);
  
  // write to the ESCs
  motorL.write(powerL);
  motorR.write(powerR);
}


/*
 * callback function to set LED values. Input is one integer containing all
 * all three R, G, and B values.
 */
void led_cb( const std_msgs::Int32& led_input){
  // take given integer and select RGB values
  int led_val = led_input.data;
  int led_r = (led_val>>16)&0xff; // read third byte
  int led_g = (led_val>> 8)&0xff; // read second byte
  int led_b = led_val      &0xff; // read first byte
  
  // write values to the LED pins
  analogWrite(led_r, LEDR);
  analogWrite(led_g, LEDG);
  analogWrite(led_b, LEDB);
}


// initalize ROS subscription object
ros::Subscriber<couch_control::MotorCommand> sub("couchMotors", &motor_cb);
ros::Subscriber<std_msgs::Int32> led_sub("couchLEDs", &led_cb);

/*
 * setMotors converts the received callback values to valid motor
 * powers and updates the global motor powers to the correct values
 */
void setMotors(int left, int right){
  // clips input to between -300 and 300
  left  = clipInput(left);
  right = clipInput(right);
  
  // go faster if just going forward
  if (left > 0 && right > 0){
    // scale values proportionally
    left  = NEUTRAL + ( left*(MAX_FORWARD - NEUTRAL))/MAX_INPUT;
    right = NEUTRAL + (right*(MAX_FORWARD - NEUTRAL))/MAX_INPUT;
    
    // set the power levels to valid power levels
    powerL = validateForwardPower(left);
    powerR = validateForwardPower(right);
  }
  // if a stop is requested
  else if (left == 0 && right == 0){
    powerL = NEUTRAL;
    powerR = NEUTRAL;
  }
  // otherwise, go at the slower reversing/turning speed
  else{
    // scale values proportionally
    left  = NEUTRAL + ( left*(NEUTRAL - MAX_BACKWARD))/MAX_INPUT;
    right = NEUTRAL + (right*(NEUTRAL - MAX_BACKWARD))/MAX_INPUT;
    powerL = validateBackwardPower(left);
    powerR = validateBackwardPower(right);
    if (left > NEUTRAL){
      powerL = validateForwardPower(left);
      powerR = validateBackwardPower(right);
    }
    else if (right > NEUTRAL){
      powerL = validateBackwardPower(left);
      powerR = validateForwardPower(right);
    }
    else{
      powerL = validateBackwardPower(left);
      powerR = validateBackwardPower(right);
    }
  }
}


/*
 * clips given value to between -300 and 300
 */
int clipInput(int input){
  if (input > MAX_INPUT){
    input = MAX_INPUT;
  }
  else if (input < MIN_INPUT){
    input = MIN_INPUT;
  }
  return input;
}


/*
 * validateForwardPower sets an individual motor power value 
 * to within the possible values for driving forward
 */
int validateForwardPower(int power){
  if (power > MAX_FORWARD){
    power = MAX_FORWARD;
  }
  else if (power < MIN_FORWARD){
    power = MIN_FORWARD;
  }
  return power;
}


/*
 * validateBackwardPower sets an individual motor power value 
 * to within the possible values for driving backward
 */
int validateBackwardPower(int power){
  if (power < MAX_BACKWARD){
    power = MAX_BACKWARD;
  }
  else if (power > MIN_BACKWARD){
    power = MIN_BACKWARD;
  }
  return power;
}


/*
 * main function to initialze all needed variables and subscriptions
 */
void setup(){
  // initialize pins for correct output
  motorL.attach(PINL);
  motorR.attach(PINR);
  
  // set initial power as neutral to start motor controllers
  motorL.write(powerL);
  motorR.write(powerR);
  
  // initialize LED pins
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  
  // initialize ROS node and subscriber
  nh.initNode();
  nh.subscribe(sub);
  
  // wait until setup is complete so ROS info can be printed
  while(!nh.connected()) nh.spinOnce();
  
  nh.loginfo("Preparation complete.");
  nh.loginfo("Wait for ESCs to beep to begin driving");
}


/*
 * required loop function that serves to wait for more callbacks
 */
void loop(){
  nh.spinOnce();
  delay(1);
}
