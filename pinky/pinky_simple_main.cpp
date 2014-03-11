/*
 * Pinky entry point intended to be used with ROS differential_drive.
 */

#include "Arduino.h"

#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

#include "motor.h"

#define WHEEL_PUBLISH_PERIOD_MS 20
#define SHUTDOWN_TIMEOUT_MS 1000

ros::NodeHandle nh;

long lastMotorCommandTime = 0;
long lastWheelPublishTime = 0;

// Motor Subscribers.
void lmotorCb(const std_msgs::Float32& power){
  Motor::LEFT.setPower(int16_t(power.data));
  lastMotorCommandTime = millis();
}
void rmotorCb(const std_msgs::Float32& power){
  Motor::RIGHT.setPower(int16_t(power.data));
  lastMotorCommandTime = millis();
}
ros::Subscriber<std_msgs::Float32> lmotorSubscriber("lmotor", &lmotorCb);
ros::Subscriber<std_msgs::Float32> rmotorSubscriber("rmotor", &rmotorCb);

// Wheel Publishers.
std_msgs::Int16 lwheelMsg;
std_msgs::Int16 rwheelMsg;
ros::Publisher lwheelPublisher("lwheel", &lwheelMsg);
ros::Publisher rwheelPublisher("rwheel", &rwheelMsg);

void setup() {
  nh.getHardware()->setBaud(57600);  // 115200 9600
  nh.initNode();

  nh.subscribe(lmotorSubscriber);
  nh.subscribe(rmotorSubscriber);
  nh.advertise(lwheelPublisher);
  nh.advertise(rwheelPublisher);

  lastWheelPublishTime = millis();
}

void loop() {
  nh.spinOnce();
  long curTime = millis();
  if (curTime > lastMotorCommandTime + SHUTDOWN_TIMEOUT_MS) {
    Motor::LEFT.setPower(0);
    Motor::RIGHT.setPower(0);
  }
  if (curTime > lastWheelPublishTime + WHEEL_PUBLISH_PERIOD_MS) {
    lastWheelPublishTime = curTime;
    lwheelMsg.data = Motor::LEFT.getTicks();
    rwheelMsg.data = Motor::RIGHT.getTicks();
    lwheelPublisher.publish(&lwheelMsg);
    rwheelPublisher.publish(&rwheelMsg);
  }
}
