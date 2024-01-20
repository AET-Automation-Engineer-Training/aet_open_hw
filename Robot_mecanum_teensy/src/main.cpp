#include <Arduino.h>
#include "sensor.h"
#include "motor.h"
#include "TeensyThreads.h"
#include <sensor_msgs/Range.h>
#include "std_msgs/Float32.h"
#include "led.h"

// put function declarations here:
ros::NodeHandle nh;

std_msgs::String debug;

nav_msgs::Odometry odom;
std_msgs::Float32 msg_sensor;

ros::Publisher pub_sonar_data(TOPIC_SONAR_DATA, &debug);
ros::Publisher odom_pub("odom", &odom);
ros::Publisher sensor_pub("sensor", &msg_sensor);
ros::Publisher pub_debug("/debug", &debug);

/* Debugging motor */
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel", &commandVelocityCallback);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  nh.getHardware()->setBaud(57600);
  
  // Setup ROS
  nh.initNode();

  // Setup sensor
  setup_sensor(nh);
  setup_motor();
  
  // Publisher configuration
  nh.advertise(pub_sonar_data);

  // Subscriber configuration
  nh.subscribe(sub_cmd_vel);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  main_loop_sensor(pub_sonar_data);
  threads.delay(100);
  main_led();

  threads.yield();

  nh.spinOnce();
}
