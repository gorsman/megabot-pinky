/*
 * Pinky entry point.
 */

#include "Arduino.h"

#include <ros.h>
#include <std_msgs/Empty.h>

#include "example/example.h"

uint8_t pin = 13;
long duration = 1000;

ros::NodeHandle nh;

void messageCb(const std_msgs::Empty& toggle_msg){
  blink_led(pin, duration);
}

ros::Subscriber<std_msgs::Empty> sub("blink_led", &messageCb );

void setup() {
  pinMode(pin, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(1);
}

