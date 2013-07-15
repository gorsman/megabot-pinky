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
LowPassMotorSpeedMeasurer speedMeasurer(motor);

long startTime = 0;
long lastTrigger = 0;
long triggerPeriod = 20;

void setup() {
  log_msg.data = logData;
  log_msg.data_length = LOG_DATA_LEN;

  pinMode(pin, OUTPUT);

  // nh.getHardware()->setBaud(115200);
  nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(logger);

  lastTrigger = millis();
  startTime = lastTrigger + 5000;
}

#define SPEED 1000


void loop() {
  long t0 = micros();
  nh.spinOnce();
  long spinTime = micros();
  motorController.update();
  long ctrlTime = micros();

  ctrlTime -= spinTime;
  spinTime -= t0;

  long curTime = millis();
  // int32_t curTicks = motor.getTicks();
  // speedMeasurer.update(curTime, curTicks);

  if (curTime - lastTrigger > 50) {
    lastTrigger = curTime;

    long t = curTime - startTime;

    static long logTime = 0;
    long t1 = micros();
    // log(t * SPEED / 1000, motor.getTicks(), motorController.lastTicksDelta, motorController.lastInstantSpeed, motorController.motorPower);
    // log(t * SPEED / 1000, motor.getTicks(), motorController.getSpeed(), motorController.lastInstantSpeed, motorController.motorPower);
    log(t * SPEED / 1000, motor.getTicks(), spinTime, ctrlTime, logTime);
    logTime = micros() - t1;
    // log(t * SPEED / 1000, motor.getTicks(), motorController.getSpeed(), speedMeasurer.getSpeed(), motorController.lastInstantSpeed);
  }

  if (motorController.targetSpeed == 0 && curTime >= startTime) {
    motorController.setTargetSpeed(SPEED);
  }
}

