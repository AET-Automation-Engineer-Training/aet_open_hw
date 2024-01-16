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

uint8_t front_right;
uint8_t front_left;
uint8_t right_right;
uint8_t right_left;
uint8_t left_right;
uint8_t left_left;
uint8_t back_right;
uint8_t back_left;

char frameid[] = "/sonar_ranger";

unsigned long pingTimer;

void initRangeMessage(sensor_msgs::Range &range_name){
    range_name.radiation_type   = sensor_msgs::Range::ULTRASOUND;
    range_name.header.frame_id  = frameid;
    range_name.field_of_view    = FIELD_OF_VIEW;
    range_name.min_range        = MIN_DISTANCE;
    range_name.max_range        = MAX_DISTANCE/100;
}

// void create_message(uint8_t front_right, uint8_t front_left, uint8_t ight_right, uint8_t right_left, uint8_t left_right, uint8_t left_left, uint8_t back_right, uint8_t back_left){
//     uint8_t json_shaw = {
//         "sonar1": {
//             "radiation_type": sensor_msgs::Range::ULTRASOUND,
//             "frame_id" : frameid,
//             "field_of_view" : FIELD_OF_VIEW,
//             "min_range" : MIN_DISTANCE,
//             "max_range" : MAX_DISTANCE/100,
//             "range"     : sonar_front_right,
//         },
//         "sonar2": {
//             "radiation_type": sensor_msgs::Range::ULTRASOUND,
//             "frame_id" : frameid,
//             "field_of_view" : FIELD_OF_VIEW,
//             "min_range" : MIN_DISTANCE,
//             "max_range" : MAX_DISTANCE/100,
//             "range"     : sonar_front_left,
//         },
//         "sonar3": {
//             "radiation_type": sensor_msgs::Range::ULTRASOUND,
//             "frame_id" : frameid,
//             "field_of_view" : FIELD_OF_VIEW,
//             "min_range" : MIN_DISTANCE,
//             "max_range" : MAX_DISTANCE/100,
//             "range"     : sonar_right_right,
//         },
//         "sonar4": {
//             "radiation_type": sensor_msgs::Range::ULTRASOUND,
//             "frame_id" : frameid,
//             "field_of_view" : FIELD_OF_VIEW,
//             "min_range" : MIN_DISTANCE,
//             "max_range" : MAX_DISTANCE/100,
//             "range"     : sonar_right_left,
//         },
//         "sonar5": {
//             "radiation_type": sensor_msgs::Range::ULTRASOUND,
//             "frame_id" : frameid,
//             "field_of_view" : FIELD_OF_VIEW,
//             "min_range" : MIN_DISTANCE,
//             "max_range" : MAX_DISTANCE/100,
//             "range"     : sonar_left_right,
//         },
//         "sonar6": {
//             "radiation_type": sensor_msgs::Range::ULTRASOUND,
//             "frame_id" : frameid,
//             "field_of_view" : FIELD_OF_VIEW,
//             "min_range" : MIN_DISTANCE,
//             "max_range" : MAX_DISTANCE/100,
//             "range"     : sonar_left_left,
//         },
//         "sonar7": {
//             "radiation_type": sensor_msgs::Range::ULTRASOUND,
//             "frame_id" : frameid,
//             "field_of_view" : FIELD_OF_VIEW,
//             "min_range" : MIN_DISTANCE,
//             "max_range" : MAX_DISTANCE/100,
//             "range"     : sonar_back_right,
//         },
//         "sonar8": {
//             "radiation_type": sensor_msgs::Range::ULTRASOUND,
//             "frame_id" : frameid,
//             "field_of_view" : FIELD_OF_VIEW,
//             "min_range" : MIN_DISTANCE,
//             "max_range" : MAX_DISTANCE/100,
//             "range"     : sonar_back_left,
//         },
        
//     };
// }

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

void main_loop_sensor(void)
{
    setup_sensor();
    if(millis()>=pingTimer){
        // Read From sensors
        front_right   = sonars[0].ping_cm(); 
        front_left    = sonars[1].ping_cm(); 
        right_right   = sonars[2].ping_cm(); 
        right_left    = sonars[3].ping_cm(); 
        left_right    = sonars[4].ping_cm();
        left_left     = sonars[5].ping_cm();
        back_right    = sonars[6].ping_cm();
        back_left     = sonars[7].ping_cm();
        String debug_msg    = "readings: sonar_front_right: "+String(front_right) + " sonar_front_left: "+String(front_left) + " sonar_right_right: "+String(right_right) + " sonar_right_left: "+String(right_left) + " sonar_left_right: "+String(left_right) + " sonar_left_left: "+String(left_left) + " sonar_back_right: "+String(back_right) + " sonar_back_left: "+String(back_left);
        
        // String debug_msg    = create_message();

        int length          = debug_msg.length();
        char data_final[length+1];
        debug_msg.toCharArray(data_final, length + 1);
        debug.data = data_final;
        pub_debug.publish(&debug);

        // compose the sensor_msgs/Range message

        sonar_front_right.range   = (float)front_right/100;
        sonar_front_left.range    = (float)front_left /100;
        sonar_right_right.range   = (float)right_right/100;
        sonar_right_left.range    = (float)right_left /100;
        sonar_left_right.range    = (float)left_right /100;
        sonar_left_left.range     = (float)left_left  /100;
        sonar_back_right.range    = (float)back_right /100;
        sonar_back_left.range     = (float)back_left  /100;

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
}

// #include <ros.h>
// #include <ros/time.h>
// #include <sensor_msgs/Range.h>

// const int echoPin = 5;  //Echo pin
// const int trigPin = 6;  //Trigger pin

// const int maxRange = 400.0;   //Maximum range in centimeters
// const int minRange = 0.0;     //Minimum range

// unsigned long range_timer;    //Used to measure 50 ms interval

// // instantiate node handle and publisher for 
// //  a sensor_msgs/Range message (topic name is /ultrasound)
// ros::NodeHandle  nh;
// sensor_msgs::Range range_msg;
// ros::Publisher pub_range( "ultrasound", &range_msg);

// /*
//  * getRange() - This function reads the time duration of the echo
//  *              and converts it to centimeters.
//  */
// float getRange(){
//     int sample;      //Holds time in microseconds
    
//     // Trigger pin goes low then high for 10 us then low
//     //  to initiate the ultrasonic burst
//     digitalWrite(trigPin, LOW);
//     delayMicroseconds(2);
    
//     digitalWrite(trigPin, HIGH);
//     delayMicroseconds(10);
//     digitalWrite(trigPin, LOW);
    
//     // read pulse length in microseconds on the Echo pin
//     sample = pulseIn(echoPin, HIGH);
    
//     // sample in microseconds converted to centimeters
//     // 343 m/s speed of sound;  time divided by 2
//     return sample/58.3;
// }

// char frameid[] = "/ultrasound";   // global frame id string

// void setup_sensor()
// {
//   // initialize the node and message publisher
//   nh.initNode();
//   nh.advertise(pub_range);
  
//   // fill the description fields in the range_msg
//   range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
//   range_msg.header.frame_id =  frameid;
//   range_msg.field_of_view = 0.26;
//   range_msg.min_range = minRange;
//   range_msg.max_range = maxRange;
  
//   // set the digital I/O pin modes
//   pinMode(echoPin, INPUT);
//   pinMode(trigPin, OUTPUT);  
// }

// void main_loop_sensor()
// {
//     setup_sensor();
//   // sample the range data from the ultrasound sensor and
//   // publish the range value once every 50 milliseconds
//   if ( (millis()-range_timer) > 50){
//     range_msg.range = getRange();
//     range_msg.header.stamp = nh.now();
//     pub_range.publish(&range_msg);
//     range_timer =  millis() + 50;
//   }
//   nh.spinOnce();
// }