/*
 * Pinky entry point.
 */

#include "Arduino.h"

#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>

#include "example/example.h"
#include "motor.h"
#include "motor_controller.h"

ros::NodeHandle nh;

uint8_t pin = 13;
long duration = 1000;
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
MotorController motorController(motor);

long lastTrigger = 0;
long triggerPeriod = 200;

void setup() {
  pinMode(pin, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(logger);

  lastTrigger = millis();
  motorController.setTargetSpeed(2000);
}


void loop() {
  nh.spinOnce();
  motorController.update();

  long curTime = millis();
  if (curTime - lastTrigger > triggerPeriod) {
    lastTrigger = curTime;
    /*
    static int speed = 2000;
    static int count = 0;
    count++;
    if (count % 100 == 0) {
      speed = -speed;
      motorController.setTargetSpeed(speed);
    }
    */
    // logMsg(motorController.getSpeed());
    logMsg(motor.getTicks());
  }
}

