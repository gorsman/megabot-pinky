/*
 * Pinky entry point.
 */

#include "Arduino.h"

#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32MultiArray.h>

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

#define LOG_DATA_LEN 5
std_msgs::Int32MultiArray log_msg;
int32_t logData[LOG_DATA_LEN];
ros::Publisher logger("logger", &log_msg);

void log(int32_t v0, int32_t v1, int32_t v2, int32_t v3, int32_t v4) {
  logData[0] = v0;
  logData[1] = v1;
  logData[2] = v2;
  logData[3] = v3;
  logData[4] = v4;
  logger.publish( &log_msg );
}

Motor& motor = Motor::LEFT;
MotorController motorController(motor);

long startTime = 0;
long lastTrigger = 0;
long triggerPeriod = 20;
long lastUpdate = 0;

void setup() {
  log_msg.data = logData;
  log_msg.data_length = LOG_DATA_LEN;

  pinMode(pin, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(logger);

  lastTrigger = millis();
  startTime = lastTrigger + 5000;
}


void loop() {
  nh.spinOnce();
  motorController.update();

  long curTime = millis();
  if (lastUpdate < motorController.lastUpdate) {
    lastUpdate = motorController.lastUpdate;
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

    long t = curTime - startTime;
    // log(motor.getTicks(), t * 2, motorController.motorPower);
    log(t * 2, motor.getTicks(), motorController.lastTicksDelta, motorController.lastInstantSpeed, motorController.motorPower);
    // logMsg(motor.getTicks());
  }

  if (motorController.targetSpeed == 0 && curTime >= startTime) {
    motorController.setTargetSpeed(2000);
  }
}

