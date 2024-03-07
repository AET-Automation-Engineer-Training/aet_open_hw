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
#include "led.h"


Ultrasonic sonars[SONAR_NUM] = {
    //sonar1
    Ultrasonic(TRIGGER_FRONT_RIGHT_SONAR, ECHO_FRONT_RIGHT_SONAR),
    //sonar2
    Ultrasonic(TRIGGER_FRONT_LEFT_SONAR, ECHO_FRONT_LEFT_SONAR),
    //sonar3
    Ultrasonic(TRIGGER_RIGHT_RIGHT_SONAR, ECHO_RIGHT_RIGHT_SONAR),
    //sonar4
    Ultrasonic(TRIGGER_RIGHT_LEFT_SONAR, ECHO_RIGHT_LEFT_SONAR),
    //sonar5
    Ultrasonic(TRIGGER_LEFT_RIGHT_SONAR, ECHO_LEFT_RIGHT_SONAR),
    //sonar6
    Ultrasonic(TRIGGER_LEFT_LEFT_SONAR, ECHO_LEFT_LEFT_SONAR),
    //sonar7
    Ultrasonic(TRIGGER_BACK_RIGHT_SONAR, ECHO_BACK_RIGHT_SONAR),
    //sonar8
    Ultrasonic(TRIGGER_BACK_LEFT_SONAR, ECHO_BACK_LEFT_SONAR)
};



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

// TODO: Add timestamp 
String create_message(uint8_t front_right, uint8_t front_left, 
                    uint8_t right_right, uint8_t right_left, 
                    uint8_t left_right, uint8_t left_left, 
                    uint8_t back_right, uint8_t back_left)
{
    // String temp = "{";
    // // Sensor data
    // temp += "\"sonar_1\":";
    // temp += "\"";
    // temp += String(front_right);
    // temp += "\"";
    // temp += ",";
    // temp += "\"sonar_2\":";
    // temp += "\"";
    // temp += String(front_left);
    // temp += "\"";
    // temp += ",";
    // temp += "\"sonar_3:\":";
    // temp += "\"";
    // temp += String(right_right);
    // temp += "\"";
    // temp += ",";
    // temp += "\"sonar_4\":";
    // temp += "\"";
    // temp += String(right_left);
    // temp += "\"";
    // temp += ",";
    // temp += "\"sonar_5\":";
    // temp += "\"";
    // temp += String(left_right);
    // temp += "\"";
    // temp += ",";
    // temp += "\"sonar_6\":";
    // temp += "\"";
    // temp += String(left_left);
    // temp += "\"";
    // temp += ",";
    // temp += "\"sonar_7\":";
    // temp += "\"";
    // temp += String(back_right);
    // temp += "\"";
    // temp += ",";
    // temp += "\"sonar_8\":";
    // temp += "\"";
    // temp += String(back_left);
    // temp += "\"";
    // // End data object
    // temp += "}";
    // // End
    String temp = String(front_right) + ":" + String(front_left)+ ":"+String(right_right)+ ":"+String(right_left)+ ":"+String(left_right)+ ":"+String(left_left)+ ":"+String(back_right)+ ":"+String(back_left);

    return temp;
}

void setup_sensor(ros::NodeHandle &nh){
    pingTimer = millis();
}

void main_loop_sensor(ros::Publisher &pub_sonar_data)
{
    // setup_sensor();
    if(millis()>=pingTimer){
        // Read From sensors
        front_right   = sonars[0].read(); 
        front_left    = sonars[1].read(); 
        right_right   = sonars[2].read(); 
        right_left    = sonars[3].read(); 
        left_right    = sonars[4].read();
        left_left     = sonars[5].read();
        back_right    = sonars[6].read();
        back_left     = sonars[7].read();


        if(sonars[0].read() < 7 || sonars[1].read() < 7 || sonars[2].read() < 7 || sonars[3].read() < 7 || sonars[4].read() < 7 || sonars[5].read() < 7 || sonars[6].read() < 7 || sonars[7].read() < 7)
        {
            Warning_state();
        }
        else
        {
            Blink_state();
        }

        
        String debug_msg    = create_message(front_right, front_left, right_right, right_left, left_right, left_left, back_right, back_left);


        // Convert debug_msg to std_msgs::String
        std_msgs::String sonars_msg;
        sonars_msg.data = debug_msg.c_str();
        pub_sonar_data.publish(&sonars_msg);

        pingTimer = millis() + PING_INTERVAL;
    }
}


