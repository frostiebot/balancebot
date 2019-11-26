#ifndef BASE_STEPPER_H
#define BASE_STEPPER_H

#include <Arduino.h>

extern portMUX_TYPE timerMux;

class BaseStepper {
  public:
    BaseStepper(gpio_num_t stepPin, gpio_num_t dirPin, uint8_t timerNo, void (*f)());

    void timerFunction();
    void init();
    void update();

    int32_t getStep();
    void setStep(int32_t step);

    float getSpeed();
    void setSpeed(float speed);

    float maxSpeed = 3000;

  private:
    void (*_timerFn)();

    gpio_num_t _stepPin;
    gpio_num_t _dirPin;

    volatile boolean pinState = 0;
    volatile int32_t _step = 0; // Step counter
    volatile float _speed = 0;
    volatile int8_t _direction = 1;

    uint8_t microStep = 32;
    float lastSpeed = 0.0;

    hw_timer_t* _timer;
    uint8_t _timerNo;
    boolean _timerEnable = 0;

};

#endif
