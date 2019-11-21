#include "arm_servo.h"
#include "cpu_map.h"
#include "servo.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

Servo ArmServo;

void setupArmServo() {
  ArmServo.attach(SERVO_PWM_PIN);
  xTaskCreatePinnedToCore(armServoTask, "armServoTask", 4096, NULL, 1, NULL, 0);
}

void armServoTask(void* pvParameters) {
  // uint32_t count;

  while (true) {
    // for (count = 0; count < SERVO_MAX_ANGLE; count++) {
    //   ArmServo.setAngle(count);
      // vTaskDelay(20);
      vTaskDelay(20000 / portTICK_PERIOD_MS);
      // }
  }

}
