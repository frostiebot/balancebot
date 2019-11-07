#include "arm_servo.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

Servo ArmServo;

void setupArmServo() {
  ArmServo.attach(SERVO_PWM_PIN);
  xTaskCreatePinnedToCore(&armServoTask, "armServoTask", 4096, NULL, 1, NULL, 0);
}

void armServoTask(void* pvParameters) {
  // uint32_t count;

  while (1) {
    // for (count = 0; count < SERVO_MAX_ANGLE; count++) {
    //   ArmServo.setAngle(count);
    //   vTaskDelay(20);
    // }
  }
}
