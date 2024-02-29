#include <Arduino.h>
#include "sensor.h"
#include "motor.h"
#include "TeensyThreads.h"
#include <sensor_msgs/Range.h>
#include "std_msgs/Float32.h"
#include "led.h"
#include "motor.cpp"
#include "AccelStepper.h"

// put function declarations here:
ros::NodeHandle nh;

std_msgs::String debug;
std_msgs::String debug_2;

nav_msgs::Odometry odom;
std_msgs::Float32 msg_sensor;



ros::Publisher pub_sonar_data(TOPIC_SONAR_DATA, &debug);
ros::Publisher odom_pub("odom", &odom);
ros::Publisher sensor_pub("sensor", &msg_sensor);
ros::Publisher pub_debug("/debug", &debug_2);

int wheel_front_left_step_2   = 0;
int wheel_front_right_step_2  = 0;
int wheel_back_left_step_2    = 0;
int wheel_back_right_step_2   = 0;

AccelStepper motor_right_front_1(motorInterfaceType, step_pin_1, dir_pin_1);
AccelStepper motor_left_front_1(motorInterfaceType,  step_pin_2, dir_pin_2);
AccelStepper motor_right_back_1(motorInterfaceType,  step_pin_3, dir_pin_3);
AccelStepper motor_left_back_1 (motorInterfaceType,  step_pin_4, dir_pin_4);


void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg){
    // linearVelocityX         = cmd_vel_msg.linear.x;
    // linearVelocityY         = cmd_vel_msg.linear.y;
    // angularVelocityZ        = cmd_vel_msg.angular.z;

    // int wheel_front_left    = (linearVelocityX - linearVelocityY - angularVelocityZ) * 100; 
    // int wheel_front_right   = (linearVelocityX + linearVelocityY + angularVelocityZ) * 100; 
    // int wheel_rear_left     = (linearVelocityX + linearVelocityY - angularVelocityZ) * 100; 
    // int wheel_rear_right    = (linearVelocityX - linearVelocityY + angularVelocityZ) * 100; 

    // // Map wheel speeds to motors
    // motor_left_front.setSpeed(map(wheel_front_left, -100, 100, -MAX_SPEED, MAX_SPEED));
    // motor_left_back.setSpeed(map(wheel_rear_left, -100, 100, -MAX_SPEED, MAX_SPEED));
    // motor_right_front.setSpeed(map(wheel_front_right, -100, 100, -MAX_SPEED, MAX_SPEED));
    // motor_right_back.setSpeed(map(wheel_rear_right, -100, 100, -MAX_SPEED, MAX_SPEED));


    /* Step lib */
    double linear_x                     = cmd_vel_msg.linear.x;
    double linear_y                     = cmd_vel_msg.linear.y;
    double angular_z                    = cmd_vel_msg.angular.z;
    
    // double wheel_front_left_1           = (linear_x - linear_y - (WHEEL_GEOMETRY) * angular_z);
    // double wheel_front_right_1          = (linear_x + linear_y + (WHEEL_GEOMETRY) * angular_z);
    // double wheel_rear_left_1            = (linear_x + linear_y - (WHEEL_GEOMETRY) * angular_z);
    // double wheel_rear_right_1           = (linear_x - linear_y + (WHEEL_GEOMETRY) * angular_z);

    // double wheel_front_left_step        = wheel_front_left_1   * STEP_PER_REVOLUTION / (2 * PI * WHEEL_RADIUS);
    // double wheel_front_right_step       = wheel_front_right_1  * STEP_PER_REVOLUTION / (2 * PI * WHEEL_RADIUS);
    // double wheel_rear_left_step         = wheel_rear_left_1    * STEP_PER_REVOLUTION / (2 * PI * WHEEL_RADIUS);  
    // double wheel_rear_right_step        = wheel_rear_right_1   * STEP_PER_REVOLUTION / (2 * PI * WHEEL_RADIUS);  

    // double wheel_front_left_mps         = (wheel_front_left_step  * 0.0314) * WHEEL_RADIUS;
    // double wheel_front_right_mps        = (wheel_front_right_step * 0.0314) * WHEEL_RADIUS;
    // double wheel_rear_left_mps          = (wheel_rear_left_step   * 0.0314) * WHEEL_RADIUS;
    // double wheel_rear_right_mps         = (wheel_rear_right_step  * 0.0314) * WHEEL_RADIUS;

    // motor_right_front_1.step(1500);
    // motor_left_front_1.step(1500);
    // motor_right_back_1.step(1500);
    // motor_left_back_1.step(1500);

    double wheel_front_left_1   = (linear_x - linear_y -(WHEEL_GEOMETRY / 2) * angular_z);
    double wheel_front_right_1  = (linear_x + linear_y +(WHEEL_GEOMETRY / 2) * angular_z);
    double wheel_back_left_1    = (linear_x + linear_y -(WHEEL_GEOMETRY / 2) * angular_z);
    double wheel_back_right_1   = (linear_x - linear_y +(WHEEL_GEOMETRY / 2) * angular_z);

    wheel_front_left_step_2   = wheel_front_left_1  * STEP_PER_REVOLUTION / (2 * PI * WHEEL_RADIUS);
    wheel_front_right_step_2  = wheel_front_right_1 * STEP_PER_REVOLUTION / (2 * PI * WHEEL_RADIUS);
    wheel_back_left_step_2    = wheel_back_left_1   * STEP_PER_REVOLUTION / (2 * PI * WHEEL_RADIUS);
    wheel_back_right_step_2   = wheel_back_right_1  * STEP_PER_REVOLUTION / (2 * PI * WHEEL_RADIUS);

    motor_right_front_1.setSpeed(wheel_front_right_step_2);
    motor_left_front_1.setSpeed(wheel_front_left_step_2);
    motor_right_back_1.setSpeed(wheel_back_right_step_2);
    motor_left_back_1.setSpeed(wheel_back_left_step_2);

    // Serial.print("linear_x:");
    // Serial.println(linear_x);
    // Serial.print("linear_x:");
    // Serial.println(linear_y);

    String debug_msg = String(wheel_front_right_step_2);
    debug_2.data = debug_msg.c_str();

    pub_debug.publish(&debug_2);


    /* https://forum.arduino.cc/t/arduino-omnidirectional-ros-driver/1091990 */
}

/* Debugging motor */
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("/cmd_vel", &commandVelocityCallback);

void setup() {
  /// Setup ROS
  Serial.begin(9600);
  nh.initNode();
  nh.getHardware()->setBaud(57600);

  // Setup sensor
  setup_sensor(nh);
  setup_motor();
  motor_right_front_1.setMaxSpeed(MAX_SPEED);
  motor_left_front_1.setMaxSpeed(MAX_SPEED);
  motor_right_back_1.setMaxSpeed(MAX_SPEED);
  motor_left_back_1.setMaxSpeed(MAX_SPEED); 
  // threads.addThread(main_led);
  
  // Publisher configuration
  nh.advertise(pub_sonar_data);
  nh.advertise(pub_debug);

  // Subscriber configuration
  nh.subscribe(sub_cmd_vel);

}

void loop() {
  // put your main code here, to run repeatedly:
  main_loop_sensor(pub_sonar_data);
  // pub_debug.publish(&debug_2);
  motor_right_front_1.runSpeed();
  motor_left_front_1.runSpeed();
  motor_right_back_1.runSpeed();
  motor_left_back_1.runSpeed();
  threads.yield();
  nh.spinOnce();
  
}
