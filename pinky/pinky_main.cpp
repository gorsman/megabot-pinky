/*
 * Pinky entry point.
 */

#include "Arduino.h"

#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>

#include "example/example.h"
#include "motor.h"

uint8_t pin = 13;
long duration = 1000;

ros::NodeHandle nh;

void messageCb(const std_msgs::Empty& toggle_msg){
  blink_led(pin, duration);
}
ros::Subscriber<std_msgs::Empty> sub("blink_led", &messageCb );

std_msgs::Int32 log_msg;
ros::Publisher logger("logger", &log_msg);

void logMsg(int32_t x) {
  log_msg.data = x;
  logger.publish( &log_msg );
}

Motor& motor = Motor::LEFT;

void setup() {
  pinMode(pin, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(logger);

  motor.setPower(100);
}

void loop() {
  nh.spinOnce();

  static int power = 100;
  static int count = 0;
  count++;
  if (count % 20 == 0) {
    power = -power;
    motor.setPower(power);
  }
  logMsg(motor.getTicks());
  delay(200);
}

