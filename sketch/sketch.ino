#include "motor.h"
#include "motor_controller.h"

// Motor& motor = Motor::RIGHT;

MotorController leftController(Motor::LEFT);
MotorController rightController(Motor::RIGHT);

#define SPEED 100

void setup() {
  Serial.begin(115200);
  Serial.println("Hello world!!!");
  
  leftController.setTargetSpeed(SPEED);
  // rightController.setTargetSpeed(SPEED);
  // Motor::LEFT.setPower(63);
  // Motor::RIGHT.setPower(63);
}

int cnt = 0;

void loop() {
  leftController.update();
  rightController.update();
  
  if (cnt % 20 == 0) {
    Serial.print("left=");
    Serial.print(Motor::LEFT.getTicks());
    Serial.print(" right=");
    Serial.println(Motor::RIGHT.getTicks());
  }
  cnt++;
  
  delay(50);
}
