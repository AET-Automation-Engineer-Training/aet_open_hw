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
#include "Stepper.h"

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

#define STEP_PER_REVOLUTION     200

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

double linearVelocityX;
double linearVelocityY;
double angularVelocityZ;

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

Stepper motor_right_front_1(STEP_PER_REVOLUTION, step_pin_1, dir_pin_1);
Stepper motor_left_front_1(STEP_PER_REVOLUTION,  step_pin_2, dir_pin_2);
Stepper motor_right_back_1(STEP_PER_REVOLUTION,  step_pin_3, dir_pin_3);
Stepper motor_left_back_1 (STEP_PER_REVOLUTION,  step_pin_4, dir_pin_4);


void setup_motor(){

    // Enable the CNC shield
    pinMode(enable_shield, OUTPUT);
    digitalWrite(enable_shield, LOW);
  
    // Set the maximum speed 
    motor_right_front.setMaxSpeed(MAX_SPEED);
    motor_left_front.setMaxSpeed(MAX_SPEED);
    motor_right_back.setMaxSpeed(MAX_SPEED);
    motor_left_back.setMaxSpeed(MAX_SPEED);

    /* Step lib */
    motor_right_front_1.setSpeed(MAX_SPEED);
    motor_left_front_1.setSpeed(MAX_SPEED);
    motor_right_back_1.setSpeed(MAX_SPEED);
    motor_left_back_1.setSpeed(MAX_SPEED); 

    // Set the maximum acceleration
    motor_right_front.setAcceleration(MAX_ACCELERATION);
    motor_left_front.setAcceleration(MAX_ACCELERATION);
    motor_right_back.setAcceleration(MAX_ACCELERATION);
    motor_left_back.setAcceleration(MAX_ACCELERATION);

    // cerate and initialize a ROS node on this Arduino controller
    // nh.initNode();
    // nh.subscribe(sub);
}

void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg){
    linearVelocityX         = cmd_vel_msg.linear.x;
    linearVelocityY         = cmd_vel_msg.linear.y;
    angularVelocityZ        = cmd_vel_msg.angular.z;

    int wheel_front_left    = (linearVelocityX - linearVelocityY - angularVelocityZ) * 100; 
    int wheel_front_right   = (linearVelocityX + linearVelocityY + angularVelocityZ) * 100; 
    int wheel_rear_left     = (linearVelocityX + linearVelocityY - angularVelocityZ) * 100; 
    int wheel_rear_right    = (linearVelocityX - linearVelocityY + angularVelocityZ) * 100; 

    // Map wheel speeds to motors
    motor_left_front.setSpeed(map(wheel_front_left, -100, 100, -MAX_SPEED, MAX_SPEED));
    motor_left_back.setSpeed(map(wheel_rear_left, -100, 100, -MAX_SPEED, MAX_SPEED));
    motor_right_front.setSpeed(map(wheel_front_right, -100, 100, -MAX_SPEED, MAX_SPEED));
    motor_right_back.setSpeed(map(wheel_rear_right, -100, 100, -MAX_SPEED, MAX_SPEED));


    /* Step lib */
    double linear_x                     = cmd_vel_msg.linear.x;
    double linear_y                     = cmd_vel_msg.linear.y;
    double angular_z                    = cmd_vel_msg.angular.z;
    
    double wheel_front_left_1           = (linear_x - linear_y - (WHEEL_GEOMETRY) * angular_z);
    double wheel_front_right_1          = (linear_x + linear_y + (WHEEL_GEOMETRY) * angular_z);
    double wheel_rear_left_1            = (linear_x + linear_y - (WHEEL_GEOMETRY) * angular_z);
    double wheel_rear_right_1           = (linear_x - linear_y + (WHEEL_GEOMETRY) * angular_z);

    double wheel_front_left_step        = wheel_front_left_1   * STEP_PER_REVOLUTION / (2 * PI * WHEEL_RADIUS);
    double wheel_front_right_step       = wheel_front_right_1  * STEP_PER_REVOLUTION / (2 * PI * WHEEL_RADIUS);
    double wheel_rear_left_step         = wheel_rear_left_1    * STEP_PER_REVOLUTION / (2 * PI * WHEEL_RADIUS);  
    double wheel_rear_right_step        = wheel_rear_right_1   * STEP_PER_REVOLUTION / (2 * PI * WHEEL_RADIUS);  

    double wheel_front_left_mps         = (wheel_front_left_mps  * 0.0314) * WHEEL_RADIUS;
    double wheel_front_right_mps        = (wheel_front_right_mps * 0.0314) * WHEEL_RADIUS;
    double wheel_rear_left_mps          = (wheel_rear_left_mps   * 0.0314) * WHEEL_RADIUS;
    double wheel_rear_right_mps         = (wheel_rear_right_mps  * 0.0314) * WHEEL_RADIUS;

    motor_right_front_1.step(wheel_front_left_step);
    motor_left_front_1.step(wheel_front_right_step);
    motor_right_back_1.step(wheel_rear_left_step);
    motor_left_back_1.step(wheel_rear_right_step);

    /* https://forum.arduino.cc/t/arduino-omnidirectional-ros-driver/1091990 */
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
//     nh.spinOnce();
// }


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
