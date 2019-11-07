#ifndef STEPPER_H
#define STEPPER_H

#include "cpu_map.h"

#include <driver/gpio.h>

#define MAX_ACCELERATION        14 // Maximum motor acceleration (MAX RECOMMENDED VALUE: 20) (default:14)
#define ZERO_SPEED              0xffffff

#define STEPS_PER_REVOLUTION    200
#define MICROSTEPPING           32


typedef enum {
  FORWARD = 0,    // GPIO Level Low
  BACKWARD = 1,   // GPIO Level High
} stepper_direction_t;


class Stepper {
  public:
    Stepper();

    void attach(gpio_num_t stepPin, gpio_num_t directionPin, gpio_num_t disablePin);

    void setDirection(stepper_direction_t direction);
    void setDisabled(bool disabled);
    void setSpeed(int16_t speed);

    int16_t getSpeed();

  private:
    gpio_num_t _disablePin;
    gpio_num_t _directionPin;
    gpio_num_t _stepPin;

    int16_t _speed;
};

#endif
