#include <Arduino.h>
#include "sensor.h"
#include "TeensyThreads.h"
#include "ros.h"

// put function declarations here:
ros::NodeHandle nh;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  // threads.addThread(main_loop_sensor);
}

void loop() {
  // put your main code here, to run repeatedly:
  // threads.addThread(main_loop_sensor);
  nh.spinOnce();
}
