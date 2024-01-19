/**
 ******************************************************************************
 * @file    motor.h
 * @author  parkhoon1609
 * @version V1.0.0
 * @date    09-01-2024
 * @brief
 ******************************************************************************
 */


#ifndef _MOTOR_H_
#define _MOTOR_H_

#include"ros.h"
#include <Arduino.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#define NAME                                "Mecanum"

#define WHEEL_RADIUS                        0.03
#define WHEEL_SEPARATION_X                  0.1005           // meter 
#define WHEEL_SEPARATION_Y                  0.085            // meter 

#define WHEEL_GEOMETRY                      (WHEEL_SEPARATION_X + WHEEL_SEPARATION_Y)

#define TURNING_RADIUS                      0.080            // meter 
#define ROBOT_RADIUS                        0.105            // meter 
#define ENCODER_MIN                         -2147483648      // raw
#define ENCODER_MAX                         2147483648       // raw
    
#define MAX_LINEAR_VELOCITY                 0.22             // m/s  
#define MAX_ANGULAR_VELOCITY                2.84             // rad/s
    
#define MIN_LINEAR_VELOCITY                 -MAX_LINEAR_VELOCITY  
#define MIN_ANGULAR_VELOCITY                -MAX_ANGULAR_VELOCITY 

#define WHEEL_NUM                           4
#define REAR_LEFT                           0
#define REAR_RIGHT                          1
#define FRONT_LEFT                          2
#define FRONT_RIGHT                         3

#define LINEAR_X                            0
#define MAX_LINEAR_VELOCITY                 1
#define ANGULAR_Z                           5

#define DEG2RAD(x)                          (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                          (x * 57.2957795131)  // *180/PI

#define TICK2RAD                            0.001533981  // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f

#define TEST_DISTANCE                       0.300        // meter
#define TEST_RADIAN                         3.14         // 180 degree

void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);
void publishSensorStateMsg(void);

void updateVariable(bool isConnected);
void updateMotorInfo(int32_t left_rear_tick, int32_t right_rear_tick, int32_t left_front_tick, int32_t right_front_tick);
void updateOdometry(void);
void updateGoalVelocity(void);
bool calcOdometry(double diff_time);
void trigger(const std_msgs::String& msg);
void goForward(int velocity);
void goBackward(int velocity);
void goRight(int velocity);
void goLeft(int velocity);
void goForwardRight(int velocity);
void goForwardLet(int velocity);
void goBackwardRight(int velocity);
void goBackwardLeft(int velocity);
void moveRobotTo(int steps_1, int steps_2, int steps_3, int steps_4);
void stopRobot(void);

void setup_motor(void);
void main_loop_motor(void);


/*******************************************************************************
* Subscriber
*******************************************************************************/
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);
ros::Subscriber<std_msgs::String> sub("/stepper_trigger", trigger);

/*******************************************************************************
* Publisher
*******************************************************************************/
nav_msgs::Odometry odom;
std_msgs::Float32 msg_sensor;
ros::Publisher odom_pub("odom", &odom);
ros::Publisher sensor_pub("sensor", &msg_sensor);


/*******************************************************************************
* Calculation for odometry
*******************************************************************************/
bool init_encoder = true;
int32_t last_diff_tick[WHEEL_NUM] = {0, 0, 0, 0};
double  last_rad[WHEEL_NUM]       = {0.0, 0.0, 0.0, 0.0};


/*******************************************************************************
* Update Joint State
*******************************************************************************/
double  last_velocity[WHEEL_NUM]  = {0.0, 0.0, 0.0, 0.0};


/*******************************************************************************
* Declaration for controllers
*******************************************************************************/
float goal_velocity[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float goal_velocity_from_button[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float goal_velocity_from_cmd[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float goal_velocity_from_rc100[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


/*******************************************************************************
* Declaration for SLAM and navigation
*******************************************************************************/
unsigned long prev_update_time;
float odom_pose[3];
double odom_vel[3];

#endif