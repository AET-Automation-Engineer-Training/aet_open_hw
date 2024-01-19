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

// geometry_msgs::Twist cmd_vel_msg;
// ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel", &commandVelocityCallback);

ros::Publisher pub_debug("/debug", &debug);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  setup_sensor(nh);
  // setup_motor();
  
  nh.advertise(pub_sonar_data);
}

void loop() {
  // put your main code here, to run repeatedly:
  // threads.addThread(main_loop_sensor);
  // threads.yield();
  main_loop_sensor(pub_sonar_data);
  main_led();
  // main_loop_motor();

  nh.spinOnce();
}
