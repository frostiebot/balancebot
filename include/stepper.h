#ifndef STEPPER_H
#define STEPPER_H

#include "cpu_map.h"

#include <driver/gpio.h>

#define MAX_ACCELERATION    14 // Maximum motor acceleration (MAX RECOMMENDED VALUE: 20) (default:14)
#define MICROSTEPPING       32

#define ZERO_SPEED          0xffffff

// #define LEFT_RMT_CHANNEL  0
// #define RIGHT_RMT_CHANNEL 1

// void setupStepper();
// void enableSteppers();
// void disableSteppers();

// void setStepperSpeed(int16_t speed);


typedef enum {
  FORWARD = 0,
  BACKWARD = 1,
} stepper_direction_t;


class Stepper {
  public:
    Stepper();

    void attach(gpio_num_t stepPin, gpio_num_t directionPin, gpio_num_t enablePin);

    void setDirection(stepper_direction_t direction);
    void setDisabled(bool disabled);
    void setSpeed(int16_t speed);

    int16_t getSpeed();

  private:
    gpio_num_t _enablePin;
    gpio_num_t _directionPin;
    gpio_num_t _stepPin;

    int16_t _speed;
};

#endif
