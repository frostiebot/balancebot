#include "servo.h"

Servo::Servo() {}

void Servo::attach(gpio_num_t pin) {
  mcpwm_gpio_init(pwmUnit, pwmSignal, pin);
  mcpwm_init(pwmUnit, pwmTimer, &config.mcpwm_config);
  disable();
}

void Servo::setAngle(uint32_t angle) {
  uint32_t pulseWidth = 0;
  pulseWidth =
      (config.min_duty_us + (((config.max_duty_us - config.min_duty_us) * (angle)) / (config.max_angle)));
  setDuty(pulseWidth);
}

float Servo::getDuty() {
  return mcpwm_get_duty(pwmUnit, pwmTimer, pwmOperator);
}

void Servo::setDuty(uint32_t duty) {
  // if (_readDuty() != duty) {
  mcpwm_set_duty_in_us(pwmUnit, pwmTimer, pwmOperator, duty);
  // }
}

void Servo::setDutyPercent(float duty) {
  mcpwm_set_duty(pwmUnit, pwmTimer, pwmOperator, duty);
}

void Servo::disable() {
  setDuty(0);
}
