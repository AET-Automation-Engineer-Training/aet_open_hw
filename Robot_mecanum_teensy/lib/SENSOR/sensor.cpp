/**
 ******************************************************************************
 * @file    sensor.c
 * @author  parkhoon1609
 * @version V1.0.0
 * @date    09-01-2024
 * @brief
 ******************************************************************************
 */

#include "sensor.h"

NewPing sonars[SONAR_NUM] = {
    //sonar1
    NewPing(TRIGGER_FRONT_RIGHT_SONAR, ECHO_FRONT_RIGHT_SONAR, MAX_DISTANCE),
    //sonar2
    NewPing(TRIGGER_FRONT_LEFT_SONAR, ECHO_FRONT_LEFT_SONAR, MAX_DISTANCE),
    //sonar3
    NewPing(TRIGGER_RIGHT_RIGHT_SONAR, ECHO_RIGHT_RIGHT_SONAR, MAX_DISTANCE),
    //sonar4
    NewPing(TRIGGER_RIGHT_LEFT_SONAR, ECHO_RIGHT_LEFT_SONAR, MAX_DISTANCE),
    //sonar5
    NewPing(TRIGGER_LEFT_RIGHT_SONAR, ECHO_LEFT_RIGHT_SONAR, MAX_DISTANCE),
    //sonar6
    NewPing(TRIGGER_LEFT_LEFT_SONAR, ECHO_LEFT_LEFT_SONAR, MAX_DISTANCE),
    //sonar7
    NewPing(TRIGGER_BACK_RIGHT_SONAR, ECHO_BACK_RIGHT_SONAR, MAX_DISTANCE),
    //sonar8
    NewPing(TRIGGER_BACK_LEFT_SONAR, ECHO_BACK_LEFT_SONAR, MAX_DISTANCE)
};

ros::NodeHandle nh;

uint8_t sonar_front_right;
uint8_t sonar_front_left;
uint8_t sonar_right_right;
uint8_t sonar_right_left;
uint8_t sonar_left_right;
uint8_t sonar_left_left;
uint8_t sonar_back_right;
uint8_t sonar_back_left;

char frameid[] = "/sonar_ranger";

unsigned long pingTimer;

void initRangeMessage(sensor_msgs::Range &range_name){
    range_name.radiation_type   = sensor_msgs::Range::ULTRASOUND;
    range_name.header.frame_id  = frameid;
    range_name.field_of_view    = FIELD_OF_VIEW;
    range_name.min_range        = MIN_DISTANCE;
    range_name.max_range        = MAX_DISTANCE/100;
}

void create_message(uint8_t sonar_front_right, uint8_t sonar_front_left, uint8_t sonar_right_right, uint8_t sonar_right_left, uint8_t sonar_left_right, uint8_t sonar_left_left, uint8_t sonar_back_right, uint8_t sonar_back_left){
    uint8_t json_shaw = {
        "sonar1": {
            "radiation_type": sensor_msgs::Range::ULTRASOUND,
            "frame_id" : frameid,
            "field_of_view" : FIELD_OF_VIEW,
            "min_range" : MIN_DISTANCE,
            "max_range" : MAX_DISTANCE/100,
            "range"     : sonar_front_right,
        },
        "sonar2": {
            "radiation_type": sensor_msgs::Range::ULTRASOUND,
            "frame_id" : frameid,
            "field_of_view" : FIELD_OF_VIEW,
            "min_range" : MIN_DISTANCE,
            "max_range" : MAX_DISTANCE/100,
            "range"     : sonar_front_left,
        },
        "sonar3": {
            "radiation_type": sensor_msgs::Range::ULTRASOUND,
            "frame_id" : frameid,
            "field_of_view" : FIELD_OF_VIEW,
            "min_range" : MIN_DISTANCE,
            "max_range" : MAX_DISTANCE/100,
            "range"     : sonar_right_right,
        },
        "sonar4": {
            "radiation_type": sensor_msgs::Range::ULTRASOUND,
            "frame_id" : frameid,
            "field_of_view" : FIELD_OF_VIEW,
            "min_range" : MIN_DISTANCE,
            "max_range" : MAX_DISTANCE/100,
            "range"     : sonar_right_left,
        },
        "sonar5": {
            "radiation_type": sensor_msgs::Range::ULTRASOUND,
            "frame_id" : frameid,
            "field_of_view" : FIELD_OF_VIEW,
            "min_range" : MIN_DISTANCE,
            "max_range" : MAX_DISTANCE/100,
            "range"     : sonar_left_right,
        },
        "sonar6": {
            "radiation_type": sensor_msgs::Range::ULTRASOUND,
            "frame_id" : frameid,
            "field_of_view" : FIELD_OF_VIEW,
            "min_range" : MIN_DISTANCE,
            "max_range" : MAX_DISTANCE/100,
            "range"     : sonar_left_left,
        },
        "sonar7": {
            "radiation_type": sensor_msgs::Range::ULTRASOUND,
            "frame_id" : frameid,
            "field_of_view" : FIELD_OF_VIEW,
            "min_range" : MIN_DISTANCE,
            "max_range" : MAX_DISTANCE/100,
            "range"     : sonar_back_right,
        },
        "sonar8": {
            "radiation_type": sensor_msgs::Range::ULTRASOUND,
            "frame_id" : frameid,
            "field_of_view" : FIELD_OF_VIEW,
            "min_range" : MIN_DISTANCE,
            "max_range" : MAX_DISTANCE/100,
            "range"     : sonar_back_left,
        },
        
    }
}

void setup_sensor(void){
    nh.advertise(pub_sonar_front_right);
    nh.advertise(pub_sonar_front_left);
    nh.advertise(pub_sonar_right_right);
    nh.advertise(pub_sonar_right_left);
    nh.advertise(pub_sonar_left_right);
    nh.advertise(pub_sonar_left_left);
    nh.advertise(pub_sonar_back_right);
    nh.advertise(pub_sonar_back_left);

    initRangeMessage(sonar_front_right);
    initRangeMessage(sonar_front_left);
    initRangeMessage(sonar_right_right);
    initRangeMessage(sonar_right_left);
    initRangeMessage(sonar_left_right);
    initRangeMessage(sonar_left_left);
    initRangeMessage(sonar_back_right);
    initRangeMessage(sonar_back_left);

    pingTimer = millis();
}

void main_loop_sensor(void){
    if(millis()>=pingTimer){
        // Read From sensors
        sonar_front_right   = sonars[0].ping_cm(); 
        sonar_front_left    = sonars[1].ping_cm(); 
        sonar_right_right   = sonars[2].ping_cm(); 
        sonar_right_left    = sonars[3].ping_cm(); 
        sonar_left_right    = sonars[4].ping_cm();
        sonar_left_left     = sonars[5].ping_cm();
        sonar_back_right    = sonars[6].ping_cm();
        sonar_back_left     = sonars[7].ping_cm();
        String debug_msg    = "readings: sonar_front_right: "+String(sonar_front_right) + " sonar_front_left: "+String(sonar_front_left) + " sonar_right_right: "+String(sonar_right_right) + " sonar_right_left: "+String(sonar_right_left) + " sonar_left_right: "+String(sonar_left_right) + " sonar_left_left: "+String(sonar_left_left) + " sonar_back_right: "+String(sonar_back_right) + " sonar_back_left: "+String(sonar_back_left);
        
        int length          = debug_msg.length();
        char data_final[length+1];
        debug_msg.toCharArray(data_final, length + 1);
        debug.data = data_final;
        pub_debug.publish(&debug);

        // compose the sensor_msgs/Range message

        sonar_front_right   = (float)sonar_front_right/100;
        sonar_front_left    = (float)sonar_front_left /100;
        sonar_right_right   = (float)sonar_right_right/100;
        sonar_right_left    = (float)sonar_right_left /100;
        sonar_left_right    = (float)sonar_left_right /100;
        sonar_left_left     = (float)sonar_left_left  /100;
        sonar_back_right    = (float)sonar_back_right /100;
        sonar_back_left     = (float)sonar_back_left  /100;

        sonar_front_right.header.stamp  = nh.now();
        sonar_front_left.header.stamp   = nh.now();
        sonar_right_right.header.stamp  = nh.now();
        sonar_right_left.header.stamp   = nh.now();
        sonar_left_right.header.stamp   = nh.now();
        sonar_left_left.header.stamp    = nh.now();
        sonar_back_right.header.stamp   = nh.now();
        sonar_back_left.header.stamp    = nh.now();


        // publish each sonar reading on each topic

        pub_sonar_front_right.publish(&sonar_front_right);
        pub_sonar_front_left.publish(&sonar_front_left);
        pub_sonar_right_right.publish(&sonar_right_right);
        pub_sonar_right_left.publish(&sonar_right_left);
        pub_sonar_left_right.publish(&sonar_left_right);
        pub_sonar_left_left.publish(&sonar_left_left);
        pub_sonar_back_right.publish(&sonar_back_right);
        pub_sonar_back_left.publish(&sonar_back_left);

        pingTimer = millis() + PING_INTERVAL;
}