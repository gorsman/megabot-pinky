/*
 * Pinky entry point.
 */

#include "Arduino.h"

#include <ros.h>
#include <megabot/DriveCommand.h>
#include <megabot/DriveTelemetry.h>

#include "motor.h"
#include "driver.h"

#define TELEMETRY_PUBLISH_PERIOD_MS 50

ros::NodeHandle nh;

Driver driver(Motor::LEFT, Motor::RIGHT);

void driveCommandCb(const megabot::DriveCommand& driveCmd){
  driver.executeCommand(driveCmd);
}
megabot::DriveTelemetry telemetryMsg;
ros::Subscriber<megabot::DriveCommand> driveCommandSubscriber("command", &driveCommandCb);
ros::Publisher driveTelemetryPublisher("telemetry", &telemetryMsg );

long lastTrigger = 0;

void setup() {
  nh.getHardware()->setBaud(115200);  // 9600
  nh.initNode();

  nh.subscribe(driveCommandSubscriber);
  nh.advertise(driveTelemetryPublisher);

  lastTrigger = millis();
}

void loop() {
  nh.spinOnce();
  driver.update();
  long curTime = millis();
  if (curTime > lastTrigger + TELEMETRY_PUBLISH_PERIOD_MS) {
    driver.getTelemetry(&telemetryMsg);
    driveTelemetryPublisher.publish(&telemetryMsg);
    lastTrigger = curTime;
  }
}
