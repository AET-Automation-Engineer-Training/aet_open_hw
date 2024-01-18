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
#include <Ultrasonic.h>
#include <std_msgs/Int8.h>
#include <stdint.h>


/*******************************************************************************
* Define 
*******************************************************************************/

#define SONAR_NUM                   8
// #define PI                          3.1416
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
#define TOPIC_SONAR_DATA            "/sonar_data"

/*******************************************************************************
* Publisher
*******************************************************************************/

void setup_sensor(ros::NodeHandle &nh);

void main_loop_sensor(ros::Publisher &pub_sonar_data);

String create_message(uint8_t front_right, uint8_t front_left, 
                    uint8_t right_right, uint8_t right_left, 
                    uint8_t left_right, uint8_t left_left, 
                    uint8_t back_right, uint8_t back_left);

#endif


