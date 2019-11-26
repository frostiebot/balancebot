#include "stepper.h"

// #include <esp32-hal-ledc.h>
#include <driver/ledc.h>
#include <stdlib.h>

Stepper::Stepper() {};

Stepper::Stepper(gpio_num_t stepPin, gpio_num_t directionPin) {
  attach(stepPin, directionPin);
}

void Stepper::attach(gpio_num_t stepPin, gpio_num_t directionPin) {
  this->_stepPin = stepPin;
  this->_directionPin = directionPin;

  gpio_set_direction(this->_stepPin, GPIO_MODE_OUTPUT);
  gpio_set_direction(this->_directionPin, GPIO_MODE_OUTPUT);

  gpio_set_level(this->_stepPin, 0);
  // gpio_set_level(this->_directionPin, 0);
  // this->setDirection(FORWARD);
  this->setSpeed(0);

  // _ledc_timer.duty_resolution = LEDC_TIMER_13_BIT;
  // _ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;
  // _ledc_timer.timer_num = LEDC_TIMER_0;
  // _ledc_timer.freq_hz = 5000;

  // ledc_timer_config(&_ledc_timer);

  // _ledc_conf.gpio_num = this->_stepPin;
  // _ledc_conf.channel = LEDC_CHANNEL_0;
  // _ledc_conf.duty = 0;
  // _ledc_conf.hpoint = 0;
  // _ledc_conf.intr_type = LEDC_INTR_DISABLE;
  // _ledc_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
  // _ledc_conf.timer_sel = LEDC_TIMER_0;

  // ledc_channel_config(&_ledc_conf);
  this->configureLedcTimer();
  this->configureLedcChannel();
}

stepper_direction_t Stepper::getDirection() {
  return gpio_get_level(_directionPin) == 0 ? FORWARD : BACKWARD;
}

void Stepper::setDirection(stepper_direction_t direction) {
  gpio_set_level(_directionPin, direction);
}

int Stepper::getSpeed() {
  return int(abs(_speed));
}

/**
 * TODO: If the stepper is running, we need to stop sending STEP
 * pulses, set the direction via gpio then wait _at least_ 650ns
 * before sending STEP pulses again.
 *
 */
void Stepper::setSpeed(int speed) {
  this->_speed = speed;
  // stepper_direction_t currentDirection = getDirection();
  // if (_speed < 0 && currentDirection != BACKWARD) setDirection(BACKWARD);
  // if (_speed > 0 && currentDirection != FORWARD) setDirection(FORWARD);

  setDirection(this->_speed < 0 ? BACKWARD : FORWARD);
}

void Stepper::go() {
  // ledc_set_duty(_ledc_conf.speed_mode, _ledc_conf.channel, 250);
  // ledc_update_duty(_ledc_conf.speed_mode, _ledc_conf.channel);
  ledc_set_duty_and_update(this->_ledc_conf.speed_mode, this->_ledc_conf.channel, 250, 0);
}

void Stepper::stop() {
  ledc_stop(this->_ledc_conf.speed_mode, this->_ledc_conf.channel, 0);
}

void Stepper::configureLedcTimer() {
  // 13-bit value == 2**13 (meaning max 8192) - this allows us to use
  // high microstepping (eg. 32) where the math is
  // (360 / STEP_ANGLE) * MICROSTEPPING_LEVEL
  // eg. (360 / 1.8) * 32 = 6400
  this->_ledc_timer.duty_resolution = LEDC_TIMER_13_BIT;
  this->_ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;
  this->_ledc_timer.timer_num = LEDC_TIMER_0;
  // freq_hz is how fast/long(?) to generate the square wave.
  // DRV8825 needs at least 1.9uS for each HIGH and LOW step pulse
  // soooo... I need to learn math
  this->_ledc_timer.freq_hz = 500000;

  ledc_timer_config(&_ledc_timer);
}

void Stepper::configureLedcChannel() {
  this->_ledc_conf.gpio_num = this->_stepPin;
  this->_ledc_conf.channel = LEDC_CHANNEL_0;
  this->_ledc_conf.duty = 0;
  this->_ledc_conf.hpoint = 0;
  this->_ledc_conf.intr_type = LEDC_INTR_DISABLE;
  this->_ledc_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
  this->_ledc_conf.timer_sel = LEDC_TIMER_0;

  ledc_channel_config(&_ledc_conf);
  // Seems like we need to install a fade function (or use default
  // by passing 0) to be able to use the thread-safe
  // `ledc_set_duty_and_update` call in `Stepper::go`
  ledc_fade_func_install(0);
}
