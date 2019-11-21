#include "balancebot.h"

#include <Arduino.h>

void setup() {
  Serial.begin(115200);

  #ifdef ENABLE_MOTION_CALIBRATE_OFFSETS
  beginCalibrate();
  #else
  setupArmServo();
  setupMotionSensor();
  // setupSerial();
  // setupStepper();

  // s.attach(SERVO_PWM_PIN);
  // s.setAngle(0);
  // s.setAngle(180);
  // delay(5000);
  // s.setAngle(90);
  // delay(5000);
  // s.setAngle(45);
  // delay(5000);
  // s.setAngle(180);
  #endif
}

void loop() {
  // put your main code here, to run repeatedly:
}
