#include "cpu_map.h"
#include "balancebot.h"

#include <Arduino.h>

void setup() {
  Serial.begin(115200);

  #ifdef ENABLE_MOTION_CALIBRATE_OFFSETS
  beginCalibrate();
  #else
  setupStepperController();

  // updateStepperSpeed(100, 100);

  enableSteppers();

  // stepper_left.setStep(0);
  // stepper_right.setStep(0);

  // setupArmServo();
  setupMotionSensor();

  // Serial.println("CONNECTING TO STEPPER...");
  // s.attach(STEPPER_LEFT_STEP_PIN, STEPPER_LEFT_DIR_PIN);
  // s.go(); // THIS DOES NOT BLOCK! YAY!
  // Serial.println("I AM UNBLOCKED.");

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
  // s.go();
  // vTaskDelay(1000 / portTICK_PERIOD_MS);
  // put your main code here, to run repeatedly:

  // 200 appears to be max
  // (although I haven't fixed Vref on the drivers yet)
  // Need to be able to specify INVERT_DIR, since the physical placement
  // of the motors matters in this case.
  updateStepperSpeed(-230, 100);
  updateSteppers();
}
