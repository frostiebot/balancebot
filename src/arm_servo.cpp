#include "arm_servo.h"

static TaskHandle_t syncArmServoTaskHandle = 0;

Servo ArmServo;

void setupArmServo() {
  ArmServo.attach(SERVO_PWM_PIN);

  xTaskCreatePinnedToCore(syncArmServoTask, "syncArmServoTask", 4096, NULL, 1, &syncArmServoTaskHandle, 0);
}

void syncArmServoTask(void* args) {
  // TickType_t xLastWakeTime;
  // const TickType_t xArmServoTimerTaskFrequency = ARM_SERVO_TASK_FREQUENCY;

  uint32_t count;

  // xLastWakeTime = xTaskGetTickCount();

  while (1) {
    for (count = 0; count < SERVO_MAX_ANGLE; count++) {
      // printf("Angle of rotation: %d\n", count);
      // angle = servo_per_degree_init(count);
      // printf("pulse width: %dus\n", angle);
      // mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);
      // vTaskDelay(10); // Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
      // Serial.print("Duty: ");
      // Serial.println(ArmServo.getDuty());

      ArmServo.setAngle(count);
      // vTaskDelayUntil(&xLastWakeTime, xArmServoTimerTaskFrequency);
      vTaskDelay(20);
    }
  }
}
