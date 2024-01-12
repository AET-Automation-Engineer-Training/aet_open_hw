/**
 ******************************************************************************
 * @file    motor.c
 * @author  parkhoon1609
 * @version V1.0.0
 * @date    09-01-2024
 * @brief
 ******************************************************************************
 */

#include "motor.h"
#include "Arduino.h"
#include "AccelStepper.h"

/*******************************************************************************
* Define pin number of step motor
*******************************************************************************/

#define dir_pin_1               38
#define step_pin_1              27

#define dir_pin_2               39
#define step_pin_2              40

#define dir_pin_3               5
#define step_pin_3              3

#define dir_pin_4               4
#define step_pin_4              2

#define motorInterfaceType      1
#define enable_shield           7

// Stepper Motor control parameters
// Max speed of the steppers in steps per second
#define MAX_SPEED               1500
// Max acceleration of the steppers in steps per second ^2
#define MAX_ACCELERATION        1000
// Speed levels for the robot mouvements
#define SUPERSONIC              1500
#define INSANE                  1000
#define SUPERFAST               800
#define FAST                    600
#define NORMAL                  400
#define SLOW                    300

//Track the current action on the stepper
int action = 0;

//Function that is called once a message is published on the topic on which this Arduino is subscribed
void trigger(const std_msgs::String &msg){
    String ms = msg.data;
    if((ms == "A") || (ms == "a")){
        action = 1;
    }
    else if((ms == "Q") || (ms == "q")){
        action = 0;
    }
}

// Create an instance of the class AccelStepper for each stepper motor connected to the Arduino board
AccelStepper motor_right_front  = AccelStepper(motorInterfaceType, step_pin_1, dir_pin_1);
AccelStepper motor_left_front   = AccelStepper(motorInterfaceType, step_pin_2, dir_pin_2);
AccelStepper motor_right_back   = AccelStepper(motorInterfaceType, step_pin_3, dir_pin_3);
AccelStepper motor_left_back    = AccelStepper(motorInterfaceType, step_pin_4, dir_pin_4);

ros::NodeHandle nh;


void setup_motor(){

    // Enable the CNC shield
    pinMode(enable_shield, OUTPUT);
    digitalWrite(enable_shield, LOW);
  
    // Set the maximum speed 
    motor_right_front.setMaxSpeed(MAX_SPEED);
    motor_left_front.setMaxSpeed(MAX_SPEED);
    motor_right_back.setMaxSpeed(MAX_SPEED);
    motor_left_back.setMaxSpeed(MAX_SPEED);

    // Set the maximum acceleration
    motor_right_front.setAcceleration(MAX_ACCELERATION);
    motor_left_front.setAcceleration(MAX_ACCELERATION);
    motor_right_back.setAcceleration(MAX_ACCELERATION);
    motor_left_back.setAcceleration(MAX_ACCELERATION);

    // cerate and initialize a ROS node on this Arduino controller
    nh.initNode();
    nh.subscribe(sub);
}
void main_loop_motor(){

    // According to the last received message control the stepper
    switch(action){

        case 0:
          stopRobot();
          break;
        case 1:
          goForward(NORMAL); //Test
          break;
        default:
          stopRobot();
          break;
    }
    
    // Keep ROS Node Up & Running
    nh.spinOnce();
}


// Omnidirectional wheels mouvements
// SPEED CONTROL
void goForward(int velocity){
    // Set the speed in steps per second:
    motor_right_front.setSpeed(velocity);
    motor_left_front.setSpeed(velocity);
    motor_right_back.setSpeed(velocity);
    motor_left_back.setSpeed(velocity);
    // Step the motor with a constant speed as set by setSpeed():
    motor_right_front.runSpeed();
    motor_left_front.runSpeed();
    motor_right_back.runSpeed();
    motor_left_back.runSpeed();  
}

void goBackward(int velocity){
    // Set the speed in steps per second:
    motor_right_front.setSpeed(-velocity);
    motor_left_front.setSpeed(-velocity);
    motor_right_back.setSpeed(-velocity);
    motor_left_back.setSpeed(-velocity);
    // Step the motor with a constant speed as set by setSpeed():
    motor_right_front.runSpeed();
    motor_left_front.runSpeed();
    motor_right_back.runSpeed();
    motor_left_back.runSpeed();
}

void goRight(int velocity){
    // Set the speed in steps per second:
    motor_right_front.setSpeed(velocity);
    motor_left_front.setSpeed(-velocity);
    motor_right_back.setSpeed(-velocity);
    motor_left_back.setSpeed(velocity);
    // Step the motor with a constant speed as set by setSpeed():
    motor_right_front.runSpeed();
    motor_left_front.runSpeed();
    motor_right_back.runSpeed();
    motor_left_back.runSpeed();
}

void goLeft(int velocity){
    // Set the speed in steps per second:
    motor_right_front.setSpeed(-velocity);
    motor_left_front.setSpeed(velocity);
    motor_right_back.setSpeed(velocity);
    motor_left_back.setSpeed(-velocity);
    // Step the motor with a constant speed as set by setSpeed():
    motor_right_front.runSpeed();
    motor_left_front.runSpeed();
    motor_right_back.runSpeed();
    motor_left_back.runSpeed();
}

void goForwardRight(int velocity){
    // Set the speed in steps per second:
    motor_right_front.setSpeed(velocity);
    motor_left_front.setSpeed(0);
    motor_right_back.setSpeed(0);
    motor_left_back.setSpeed(velocity);
    // Step the motor with a constant speed as set by setSpeed():
    motor_right_front.runSpeed();
    motor_left_front.runSpeed();
    motor_right_back.runSpeed();
    motor_left_back.runSpeed();
}

void goForwardLet(int velocity){
    // Set the speed in steps per second:
    motor_right_front.setSpeed(0);
    motor_left_front.setSpeed(velocity);
    motor_right_back.setSpeed(velocity);
    motor_left_back.setSpeed(0);
    // Step the motor with a constant speed as set by setSpeed():
    motor_right_front.runSpeed();
    motor_left_front.runSpeed();
    motor_right_back.runSpeed();
    motor_left_back.runSpeed();
}

void goBackwardRight(int velocity){
    // Set the speed in steps per second:
    motor_right_front.setSpeed(0);
    motor_left_front.setSpeed(-velocity);
    motor_right_back.setSpeed(-velocity);
    motor_left_back.setSpeed(0);
    // Step the motor with a constant speed as set by setSpeed():
    motor_right_front.runSpeed();
    motor_left_front.runSpeed();
    motor_right_back.runSpeed();
    motor_left_back.runSpeed();
}

void goBackwardLeft(int velocity){
    // Set the speed in steps per second:
    motor_right_front.setSpeed(-velocity);
    motor_left_front.setSpeed(0);
    motor_right_back.setSpeed(0);
    motor_left_back.setSpeed(-velocity);
    // Step the motor with a constant speed as set by setSpeed():
    motor_right_front.runSpeed();
    motor_left_front.runSpeed();
    motor_right_back.runSpeed();
    motor_left_back.runSpeed();
}

// POSITION CONTROL
void moveRobotTo(int steps_1, int steps_2, int steps_3, int steps_4){
    
    // set a target position in steps for each motor
    motor_right_front.moveTo(steps_1);
    motor_left_front.moveTo(steps_2);
    motor_right_back.moveTo(steps_3);
    motor_left_back.moveTo(steps_4);
    // Reach the position goal
    motor_right_front.runToPosition();
    motor_left_front.runToPosition();
    motor_right_back.runToPosition();
    motor_left_back.runToPosition();
}

// Stop all the motors
void stopRobot(){
    // Set the speed in steps per second:
    motor_right_front.setSpeed(0);
    motor_left_front.setSpeed(0);
    motor_right_back.setSpeed(0);
    motor_left_back.setSpeed(0);
    // Step the motor with a constant speed as set by setSpeed():
    motor_right_front.runSpeed();
    motor_left_front.runSpeed();
    motor_right_back.runSpeed();
    motor_left_back.runSpeed();
}
