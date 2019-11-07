#include "stepper.h"


Stepper::Stepper() {
  _speed = 0;
};

void Stepper::attach(gpio_num_t stepPin, gpio_num_t directionPin, gpio_num_t disablePin) {
  _stepPin = stepPin;
  _directionPin = directionPin;
  _disablePin = disablePin;

  gpio_set_direction(_stepPin, GPIO_MODE_OUTPUT);
  gpio_set_direction(_directionPin, GPIO_MODE_OUTPUT);

  gpio_set_direction(_disablePin, GPIO_MODE_OUTPUT);

  gpio_set_level(_stepPin, 0);
  gpio_set_level(_directionPin, 0);
}

void Stepper::setDirection(stepper_direction_t direction) {
  gpio_set_level(_directionPin, direction);
}

void Stepper::setDisabled(bool disabled) {
  gpio_set_level(_disablePin, disabled);
}

void Stepper::setSpeed(int16_t speed) {
  // long timer_period;
  // int16_t _speed, speed_M1, dir_M1;

  // speed_M1 = 0;
  // dir_M1 = 1;

  // // Limit max speed?

  // // WE LIMIT MAX ACCELERATION of the motors
  // if ((speed_M1 - speed) > MAX_ACCELERATION)
  //   speed_M1 -= MAX_ACCELERATION;
  // else if ((speed_M1 - speed) < -MAX_ACCELERATION)
  //   speed_M1 += MAX_ACCELERATION;
  // else
  //   speed_M1 = speed;

  // #if MICROSTEPPING == 32
  //   _speed = speed_M1 * 75;
  // #endif

  // #if MICROSTEPPING == 16
  //   _speed = speed_M1 * 50; // Adjust factor from control output speed to real motor speed in steps/second
  // #else
  //   _speed = speed_M1 * 25; // 1/8 Microstepping
  // #endif

  // if (_speed == 0) {
  //   timer_period = ZERO_SPEED;
  //   dir_M1 = 0;
  // } else if (_speed > 0) {
  //   timer_period = 2000000 / _speed; // 2Mhz timer
  //   dir_M1 = 1;
  //   // digitalWrite(PIN_MOTOR1_DIR, HIGH);
  //   setDirection(FORWARD);
  // } else {
  //   timer_period = 2000000 / -_speed;
  //   dir_M1 = -1;
  //   // digitalWrite(PIN_MOTOR1_DIR, LOW);
  //   setDirection(BACKWARD);
  // }
  // if (timer_period > ZERO_SPEED) // Check for minimun speed (maximun period without overflow)
  //   timer_period = ZERO_SPEED;

  // // timerAlarmWrite(timer1, timer_period, true);
}

int16_t Stepper::getSpeed() {
  return _speed;
}

    // void setupStepper() {
    //   gpio_set_direction(STEPPER_ENABLE_PIN, GPIO_MODE_OUTPUT);

    //   gpio_set_direction(STEPPER_LEFT_DIR_PIN, GPIO_MODE_OUTPUT);
    // 	gpio_set_direction(STEPPER_LEFT_STEP_PIN, GPIO_MODE_OUTPUT);

    // 	gpio_set_direction(STEPPER_RIGHT_DIR_PIN, GPIO_MODE_OUTPUT);
    // 	gpio_set_direction(STEPPER_RIGHT_STEP_PIN, GPIO_MODE_OUTPUT);
    // }

    // void enableSteppers() {
    //   gpio_set_level(STEPPER_ENABLE_PIN, 0);
    // }

    // void disableSteppers() {
    //   gpio_set_level(STEPPER_ENABLE_PIN, 1);
    // }
