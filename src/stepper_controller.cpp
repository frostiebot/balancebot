#include "stepper_controller.h"
#include "cpu_map.h"

#include <base_stepper.h>

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

BaseStepper stepper_left(STEPPER_LEFT_STEP_PIN, STEPPER_LEFT_DIR_PIN, 0, leftStepperTimerTask);
BaseStepper stepper_right(STEPPER_RIGHT_STEP_PIN, STEPPER_RIGHT_DIR_PIN, 1, rightStepperTimerTask);

float maxStepSpeed = 3000;

void setupStepperController() {
  gpio_set_direction(STEPPER_ENABLE_PIN, GPIO_MODE_OUTPUT);

  disableSteppers();

  stepper_left.init();
  stepper_right.init();
}

void IRAM_ATTR leftStepperTimerTask() {
  vTaskEnterCritical(&timerMux);
  stepper_left.timerFunction();
  vTaskExitCritical(&timerMux);
}

void IRAM_ATTR rightStepperTimerTask() {
  vTaskEnterCritical(&timerMux);
  stepper_right.timerFunction();
  vTaskExitCritical(&timerMux);
}

void enableSteppers() {
  gpio_set_level(STEPPER_ENABLE_PIN, 0);
}

void disableSteppers() {
  gpio_set_level(STEPPER_ENABLE_PIN, 1);
}

void updateStepperSpeed(float leftSpeed, float rightSpeed) {
  stepper_left.setSpeed(leftSpeed);
  stepper_right.setSpeed(rightSpeed);
}

void updateSteppers() {
  stepper_left.update();
  stepper_right.update();
}
