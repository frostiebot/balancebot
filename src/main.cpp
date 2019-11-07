#include "balancebot.h"

#include <Arduino.h>


void setup() {
  Serial.begin(115200);

  setupMotionSensor();
  // setupSerial();
  // setupStepper();
  setupArmServo();

  // s.attach(SERVO_PWM_PIN);
  // s.setAngle(0);
  // s.setAngle(180);
  // delay(5000);
  // s.setAngle(90);
  // delay(5000);
  // s.setAngle(45);
  // delay(5000);
  // s.setAngle(180);
}

void loop() {
  // put your main code here, to run repeatedly:
}
