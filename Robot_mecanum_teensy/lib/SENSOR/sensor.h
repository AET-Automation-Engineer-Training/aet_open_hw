/**
 ******************************************************************************
 * @file    sensor.h
 * @author  parkhoon1609
 * @version V1.0.0
 * @date    09-01-2024
 * @brief
 ******************************************************************************
 */

#ifndef _SENSOR_H_
#define _SENSOR_H_

#include <Arduino.h>
#include "ros.h"
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>
#include <NewPing.h>


/*******************************************************************************
* Define 
*******************************************************************************/

#define SONAR_NUM                   8
#define PI                          3.1416
#define MIN_DISTANCE                0
#define MAX_DISTANCE                200
#define FIELD_OF_VIEW               0.26
#define PING_INTERVAL               50


/*******************************************************************************
* Declare pin sonar
*******************************************************************************/

#define TRIGGER_FRONT_RIGHT_SONAR   30           //sonar1
#define ECHO_FRONT_RIGHT_SONAR      33

#define TRIGGER_FRONT_LEFT_SONAR    31           //sonar2
#define ECHO_FRONT_LEFT_SONAR       16

#define TRIGGER_RIGHT_RIGHT_SONAR   32           //sonar3
#define ECHO_RIGHT_RIGHT_SONAR      26

#define TRIGGER_RIGHT_LEFT_SONAR    6            //sonar4
#define ECHO_RIGHT_LEFT_SONAR       9

#define TRIGGER_LEFT_RIGHT_SONAR    36           //sonar5
#define ECHO_LEFT_RIGHT_SONAR       37

#define TRIGGER_LEFT_LEFT_SONAR     24           //sonar6
#define ECHO_LEFT_LEFT_SONAR        25

#define TRIGGER_BACK_RIGHT_SONAR    17           //sonar7
#define ECHO_BACK_RIGHT_SONAR       12

#define TRIGGER_BACK_LEFT_SONAR     10           //sonar8
#define ECHO_BACK_LEFT_SONAR        11


/*******************************************************************************
* Topic names
*******************************************************************************/

#define TOPIC_SONAR_FRONT_RIGHT     "/sonar_front_right"
#define TOPIC_SONAR_FRONT_LEFT      "/sonar_front_left"
#define TOPIC_SONAR_RIGHT_RIGHT     "/sonar_right_right"
#define TOPIC_SONAR_RIGHT_LEFT      "/sonar_right_left"
#define TOPIC_SONAR_LEFT_RIGHT      "/sonar_left_right"
#define TOPIC_SONAR_LEFT_LEFT       "/sonar_left_left"
#define TOPIC_SONAR_BACK_RIGHT      "/sonar_back_right"
#define TOPIC_SONAR_BACK_LEFT       "/sonar_back_left"


/*******************************************************************************
* Publisher
*******************************************************************************/

sensor_msgs::Range sonar_front_right;
sensor_msgs::Range sonar_front_left;
sensor_msgs::Range sonar_right_right;
sensor_msgs::Range sonar_right_left;
sensor_msgs::Range sonar_left_right;
sensor_msgs::Range sonar_left_left;
sensor_msgs::Range sonar_back_right;
sensor_msgs::Range sonar_back_left;

std_msgs::String debug;

ros::Publisher pub_sonar_front_right(TOPIC_SONAR_FRONT_RIGHT, &sonar_front_right);
ros::Publisher pub_sonar_front_left(TOPIC_SONAR_FRONT_RIGHT, &sonar_front_left);
ros::Publisher pub_sonar_right_right(TOPIC_SONAR_FRONT_RIGHT, &sonar_right_right);
ros::Publisher pub_sonar_right_left(TOPIC_SONAR_FRONT_RIGHT, &sonar_right_left);
ros::Publisher pub_sonar_left_right(TOPIC_SONAR_FRONT_RIGHT, &sonar_left_right);
ros::Publisher pub_sonar_left_left(TOPIC_SONAR_FRONT_RIGHT, &sonar_left_left);
ros::Publisher pub_sonar_back_right(TOPIC_SONAR_FRONT_RIGHT, &sonar_back_right);
ros::Publisher pub_sonar_back_left(TOPIC_SONAR_FRONT_RIGHT, &sonar_back_left);

ros::Publisher pub_debug("/debug", &debug);

void initRangeMessage(sensor_msgs::Range &range_name);

void setup_sensor(void);
void main_loop_sensor(void);