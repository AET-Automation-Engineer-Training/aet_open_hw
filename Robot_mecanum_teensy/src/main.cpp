#include <Arduino.h>
#include "sensor.h"
#include "TeensyThreads.h"
#include <sensor_msgs/Range.h>

// put function declarations here:
ros::NodeHandle nh;

std_msgs::String debug;

ros::Publisher pub_sonar_data(TOPIC_SONAR_DATA, &debug);

ros::Publisher pub_debug("/debug", &debug);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  setup_sensor(nh);
  nh.advertise(pub_sonar_data);
}

void loop() {
  // put your main code here, to run repeatedly:
  // threads.addThread(main_loop_sensor);
  // threads.yield();
  main_loop_sensor(pub_sonar_data);

  nh.spinOnce();
}
