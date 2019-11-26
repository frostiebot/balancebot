#ifndef STEPPER_H
#define STEPPER_H

// #include "cpu_map.h"

#include <driver/gpio.h>
#include <driver/ledc.h>
// #include <driver/mcpwm.h>

// #define MAX_ACCELERATION        14 // Maximum motor acceleration (MAX RECOMMENDED VALUE: 20) (default:14)
// #define ZERO_SPEED              0xffffff

// #define STEPS_PER_REVOLUTION    200
// #define MICROSTEPPING           32


typedef enum {
  FORWARD = 0,    // GPIO Level Low
  BACKWARD = 1,   // GPIO Level High
} stepper_direction_t;


class Stepper {
  public:
    Stepper();
    Stepper(gpio_num_t stepPin, gpio_num_t directionPin);

    // void attach(gpio_num_t stepPin, gpio_num_t directionPin, gpio_num_t disablePin);
    void attach(gpio_num_t stepPin, gpio_num_t directionPin);

    int getSpeed();
    void setSpeed(int speed);

    stepper_direction_t getDirection();
    void setDirection(stepper_direction_t direction);

    void go();
    void stop();

  private:
    // gpio_num_t _disablePin;
    gpio_num_t _directionPin;
    gpio_num_t _stepPin;

    int _speed;

    ledc_channel_config_t _ledc_conf;
    ledc_timer_config_t _ledc_timer;

    void configureLedcChannel();
    void configureLedcTimer();
};

#endif
