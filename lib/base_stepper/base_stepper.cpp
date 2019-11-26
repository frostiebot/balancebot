#include "base_stepper.h"

BaseStepper::BaseStepper(gpio_num_t stepPin, gpio_num_t dirPin, uint8_t timerNo, void (*f)()) {
  _stepPin = stepPin;
  _dirPin = dirPin;

  _timerNo = timerNo;
  _timerFn = f;
}

void BaseStepper::init() {
  gpio_set_direction(_stepPin, GPIO_MODE_OUTPUT);
  gpio_set_direction(_dirPin, GPIO_MODE_OUTPUT);

  gpio_set_level(_stepPin, 0);

  _timer = timerBegin(_timerNo, 2, true);
  timerAttachInterrupt(_timer, _timerFn, true);
}

void BaseStepper::timerFunction() {
  if (!pinState) {
    _step += _direction; // Step is made on rising edge
    if (_stepPin < 32) {
      GPIO.out_w1ts = ((uint32_t)1 << _stepPin);
    } else if (_stepPin < 34) {
      GPIO.out1_w1ts.val = ((uint32_t)1 << (_stepPin - 32));
    }
    pinState = 1;
  } else {
    if (_stepPin < 32) {
      GPIO.out_w1tc = ((uint32_t)1 << _stepPin);
    } else if (_stepPin < 34) {
      GPIO.out1_w1tc.val = ((uint32_t)1 << (_stepPin - 32));
    }
    pinState = 0;
  }
}

// A step is triggered on the rising edge of the step pin of the stepper driver.
// Therefore, we use a 50% duty cycle, variable frequency PWM output.
// Maximum step rate is 250 kHz. We also want some resolution left (to prevent sudden steps in speed).
// Because of the pin toggling at 50% duty cycle, we need double the interrupt frequency.
// Also, we want plenty resolution, to prevent discrete steps in stepper speed.

// Use speed as input variable?
// Use static member for microstep pins?
//TODO: Add flag for direction change notification instead of testing
// for lastDir
void BaseStepper::update() {
  float absSpeed;
  uint32_t absSpeedInt;
  int8_t lastDir = _direction;

  if (_speed > 0) {
    _direction = 1;
    absSpeed = _speed * (microStep / 16.0);
  } else {
    _direction = -1;
    absSpeed = -_speed * (microStep / 16.0);
  }

  if (absSpeed > maxSpeed)
    absSpeed = maxSpeed; // Clip speed

  // Only update dir pin if changed (it doesn't change very often)
  // if (_direction != lastDir) {
  gpio_set_level(_dirPin, _direction == 1 ? 1 : 0);
  // }

  if (lastSpeed != _speed) {
    if (absSpeed != 0) {
      absSpeedInt = (uint32_t)(400000.0 / absSpeed);

      // Serial.println(absSpeedInt);
      timerAlarmWrite(_timer, absSpeedInt, true);

      if (!_timerEnable) {
        timerAlarmEnable(_timer); // Re-enable timer
        _timerEnable = 1;
      }
    } else {
      timerAlarmWrite(_timer, 100000, true);
      timerAlarmDisable(_timer);
      _timerEnable = 0;
    }
  }

  lastSpeed = _speed;
}

int32_t BaseStepper::getStep() {
  int32_t step;
  vTaskEnterCritical(&timerMux);
  step = _step;
  vTaskExitCritical(&timerMux);
  return step;
}

void BaseStepper::setStep(int32_t step) {
  vTaskEnterCritical(&timerMux);
  _step = step;
  vTaskExitCritical(&timerMux);
}

float BaseStepper::getSpeed() {
  float speed;
  vTaskEnterCritical(&timerMux);
  speed = _speed;
  vTaskExitCritical(&timerMux);
  return speed;
}

void BaseStepper::setSpeed(float speed) {
  vTaskEnterCritical(&timerMux);
  //TODO: Set _direction in here?
  _speed = speed;
  vTaskExitCritical(&timerMux);
}
