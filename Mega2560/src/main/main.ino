#include <Arduino.h>
#include <ros.h>
#include "sensor.h"
#include <std_msgs/String.h>
#include "geometry_msgs/Twist.h"
#include "Stepper.h"
#include "AccelStepper.h"
#include "led.h"


#define USE_USBCON

#define WHEEL_RADIUS                        0.150
#define WHEEL_SEPARATION_X                  0.6527           // meter 
#define WHEEL_SEPARATION_Y                  0.392            // meter 

#define WHEEL_GEOMETRY                      (WHEEL_SEPARATION_X + WHEEL_SEPARATION_Y)

#define TURNING_RADIUS                      0.080            // meter 
#define ROBOT_RADIUS                        0.105            // meter 
#define ENCODER_MIN                         -2147483648      // raw
#define ENCODER_MAX                         2147483648       // raw
    
#define MAX_LINEAR_VELOCITY                 0.22             // m/s  
#define MAX_ANGULAR_VELOCITY                2.84             // rad/s
    
#define MIN_LINEAR_VELOCITY                 -MAX_LINEAR_VELOCITY  
#define MIN_ANGULAR_VELOCITY                -MAX_ANGULAR_VELOCITY 

#define dir_pin_2               23
#define step_pin_2              22

#define dir_pin_1               26
#define step_pin_1              25

#define dir_pin_3               29
#define step_pin_3              28

#define dir_pin_4               32
#define step_pin_4              31

#define enable_1                24
#define enable_2                27
#define enable_3                30
#define enable_4                33

#define motorInterfaceType      1

#define STEP_PER_REVOLUTION     1500

const int MOTOR_SPEED = 1500;

int wheel_front_left_step_2   = 0;
int wheel_front_right_step_2  = 0;
int wheel_back_left_step_2    = 0;
int wheel_back_right_step_2   = 0;


AccelStepper motor_right_front_1(motorInterfaceType, step_pin_1, dir_pin_1); //22 23
AccelStepper motor_left_front_1(motorInterfaceType,  step_pin_2, dir_pin_2); //25 26
AccelStepper motor_right_back_1(motorInterfaceType,  step_pin_3, dir_pin_3); //28 29
AccelStepper motor_left_back_1 (motorInterfaceType,  step_pin_4, dir_pin_4); //31 32


// put function declarations here:
ros::NodeHandle nh;

std_msgs::String debug;
std_msgs::String debug_2;

//nav_msgs::Odometry odom;
//std_msgs::Float32 msg_sensor;

// ros::Publisher pub_sonar_data(TOPIC_SONAR_DATA, &debug);
//ros::Publisher odom_pub("odom", &odom);
//ros::Publisher sensor_pub("sensor", &msg_sensor);
ros::Publisher pub_debug("/debug", &debug_2);
//
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg){

    /* Step lib */
    double linear_x             = cmd_vel_msg.linear.x;
    double linear_y             = cmd_vel_msg.linear.y;
    double angular_z            = cmd_vel_msg.angular.z;

    /* Debug */

//    double wheel_back_right_1     = (linear_x - linear_y -(WHEEL_GEOMETRY / 2) * angular_z); //front right
//    double wheel_front_left_1     = (linear_x + linear_y +(WHEEL_GEOMETRY / 2) * angular_z); //front left
//    double wheel_back_left_1      = (linear_x + linear_y -(WHEEL_GEOMETRY / 2) * angular_z); //back left
//    double wheel_front_right_1    = (linear_x - linear_y +(WHEEL_GEOMETRY / 2) * angular_z); //back right

    double wheel_back_left_1     = (linear_x - linear_y -(WHEEL_GEOMETRY / 2) * angular_z); //front right
    double wheel_front_right_1   = (linear_x + linear_y +(WHEEL_GEOMETRY / 2) * angular_z); //front left
    double wheel_back_right_1    = (linear_x + linear_y -(WHEEL_GEOMETRY / 2) * angular_z); //back left
    double wheel_front_left_1    = (linear_x - linear_y +(WHEEL_GEOMETRY / 2) * angular_z); //back right
    
    
    wheel_front_left_step_2   = wheel_front_left_1  * STEP_PER_REVOLUTION / (2 * PI * WHEEL_RADIUS);
    wheel_front_right_step_2  = wheel_front_right_1 * STEP_PER_REVOLUTION / (2 * PI * WHEEL_RADIUS);
    wheel_back_left_step_2    = wheel_back_left_1   * STEP_PER_REVOLUTION / (2 * PI * WHEEL_RADIUS);
    wheel_back_right_step_2   = wheel_back_right_1  * STEP_PER_REVOLUTION / (2 * PI * WHEEL_RADIUS);

    motor_right_front_1.setSpeed(wheel_front_right_step_2);
    motor_left_front_1.setSpeed(wheel_front_left_step_2);
    motor_right_back_1.setSpeed(wheel_back_right_step_2);
    motor_left_back_1.setSpeed(wheel_back_left_step_2);
    
    String debug_msg = String(wheel_front_left_step_2) + ":" + String(wheel_front_right_step_2) + ":" + String(wheel_back_left_step_2) + ":" + String(wheel_back_right_step_2);
  
    debug_2.data = debug_msg.c_str();

    pub_debug.publish(&debug_2);


    /* https://forum.arduino.cc/t/arduino-omnidirectional-ros-driver/1091990 */
}

/* Debugging motor */
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("/cmd_vel", &commandVelocityCallback);

void setup() {
  /// Setup ROS
//  nh.getHardware()->setBaud(57600);
  nh.initNode();
  
  // // Publisher configuration
  //  nh.advertise(pub_sonar_data);
   nh.advertise(pub_debug);

   pinMode(enable_1, OUTPUT);
   digitalWrite(enable_1, LOW);

   pinMode(enable_2, OUTPUT);
   digitalWrite(enable_2, LOW);

   pinMode(enable_3, OUTPUT);
   digitalWrite(enable_3, LOW);

   pinMode(enable_4, OUTPUT);
   digitalWrite(enable_4, LOW);

  // // Subscriber configuration
   nh.subscribe(sub_cmd_vel);

   motor_right_front_1.setMaxSpeed(MOTOR_SPEED);
   motor_left_front_1.setMaxSpeed(MOTOR_SPEED);
   motor_right_back_1.setMaxSpeed(MOTOR_SPEED);
   motor_left_back_1.setMaxSpeed(MOTOR_SPEED);

}

void loop() {

  // put your main code here, to run repeatedly:
//  main_loop_sensor(pub_sonar_data);
//  pub_debug.publish(&debug_2);
  motor_right_front_1.runSpeed();
  motor_left_front_1.runSpeed();
  motor_right_back_1.runSpeed();
  motor_left_back_1.runSpeed();
  
  nh.spinOnce(); 
  
}
