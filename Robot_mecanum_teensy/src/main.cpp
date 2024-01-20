#include <Arduino.h>
#include "sensor.h"
#include "motor.h"
#include "TeensyThreads.h"
#include <sensor_msgs/Range.h>
#include "led.h"

// put function declarations here:
ros::NodeHandle nh;

std_msgs::String debug;

ros::Publisher pub_sonar_data(TOPIC_SONAR_DATA, &debug);

/* Debugging motor */
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel", &commandVelocityCallback);

ros::Publisher pub_debug("/debug", &debug);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  setup_sensor(nh);
  
  /* Debugging motor */
  setup_motor();
  
  nh.advertise(pub_sonar_data);
  nh.subscribe(sub_cmd_vel);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  /* Debugging motor */
  main_loop_motor();

  main_loop_sensor(pub_sonar_data);
  main_led();

  nh.spinOnce();
}
