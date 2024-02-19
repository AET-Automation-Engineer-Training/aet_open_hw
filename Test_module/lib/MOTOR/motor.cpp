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

#define motorInterfaceType      34
#define enable_shield           35

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

Stepper motor_right_front_1(STEP_PER_REVOLUTION, step_pin_1, dir_pin_1);
Stepper motor_left_front_1(STEP_PER_REVOLUTION,  step_pin_2, dir_pin_2);
Stepper motor_right_back_1(STEP_PER_REVOLUTION,  step_pin_3, dir_pin_3);
Stepper motor_left_back_1 (STEP_PER_REVOLUTION,  step_pin_4, dir_pin_4);


void setup_motor(){

    // Enable the CNC shield
    pinMode(enable_shield, OUTPUT);
    digitalWrite(enable_shield, LOW);
    Serial.begin(9600);

    /* Step lib */
    motor_right_front_1.setSpeed(MAX_SPEED);
    motor_left_front_1.setSpeed(MAX_SPEED);
    motor_right_back_1.setSpeed(MAX_SPEED);
    motor_left_back_1.setSpeed(MAX_SPEED); 
}

// void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg){
//     // linearVelocityX         = cmd_vel_msg.linear.x;
//     // linearVelocityY         = cmd_vel_msg.linear.y;
//     // angularVelocityZ        = cmd_vel_msg.angular.z;

//     // int wheel_front_left    = (linearVelocityX - linearVelocityY - angularVelocityZ) * 100; 
//     // int wheel_front_right   = (linearVelocityX + linearVelocityY + angularVelocityZ) * 100; 
//     // int wheel_rear_left     = (linearVelocityX + linearVelocityY - angularVelocityZ) * 100; 
//     // int wheel_rear_right    = (linearVelocityX - linearVelocityY + angularVelocityZ) * 100; 

//     // // Map wheel speeds to motors
//     // motor_left_front.setSpeed(map(wheel_front_left, -100, 100, -MAX_SPEED, MAX_SPEED));
//     // motor_left_back.setSpeed(map(wheel_rear_left, -100, 100, -MAX_SPEED, MAX_SPEED));
//     // motor_right_front.setSpeed(map(wheel_front_right, -100, 100, -MAX_SPEED, MAX_SPEED));
//     // motor_right_back.setSpeed(map(wheel_rear_right, -100, 100, -MAX_SPEED, MAX_SPEED));


//     /* Step lib */
//     double linear_x                     = cmd_vel_msg.linear.x;
//     double linear_y                     = cmd_vel_msg.linear.y;
//     double angular_z                    = cmd_vel_msg.angular.z;
    
//     double wheel_front_left_1           = (linear_x - linear_y - (WHEEL_GEOMETRY) * angular_z);
//     double wheel_front_right_1          = (linear_x + linear_y + (WHEEL_GEOMETRY) * angular_z);
//     double wheel_rear_left_1            = (linear_x + linear_y - (WHEEL_GEOMETRY) * angular_z);
//     double wheel_rear_right_1           = (linear_x - linear_y + (WHEEL_GEOMETRY) * angular_z);

//     double wheel_front_left_step        = wheel_front_left_1   * STEP_PER_REVOLUTION / (2 * PI * WHEEL_RADIUS);
//     double wheel_front_right_step       = wheel_front_right_1  * STEP_PER_REVOLUTION / (2 * PI * WHEEL_RADIUS);
//     double wheel_rear_left_step         = wheel_rear_left_1    * STEP_PER_REVOLUTION / (2 * PI * WHEEL_RADIUS);  
//     double wheel_rear_right_step        = wheel_rear_right_1   * STEP_PER_REVOLUTION / (2 * PI * WHEEL_RADIUS);  

//     double wheel_front_left_mps         = (wheel_front_left_step  * 0.0314) * WHEEL_RADIUS;
//     double wheel_front_right_mps        = (wheel_front_right_step * 0.0314) * WHEEL_RADIUS;
//     double wheel_rear_left_mps          = (wheel_rear_left_step   * 0.0314) * WHEEL_RADIUS;
//     double wheel_rear_right_mps         = (wheel_rear_right_step  * 0.0314) * WHEEL_RADIUS;

//     motor_right_front_1.step(wheel_front_left_step);
//     motor_left_front_1.step(wheel_front_right_step);
//     motor_right_back_1.step(wheel_rear_left_step);
//     motor_left_back_1.step(wheel_rear_right_step);

//     Serial.print("linear_x:");
//     Serial.println(linear_x);
//     Serial.print("linear_x:");
//     Serial.println(linear_y);


//     /* https://forum.arduino.cc/t/arduino-omnidirectional-ros-driver/1091990 */
// }




