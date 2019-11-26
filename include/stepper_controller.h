#ifndef STEPPER_CONTROLLER_H
#define STEPPER_CONTROLLER_H

#include "base_stepper.h"

extern BaseStepper stepper_left;
extern BaseStepper stepper_right;

void leftStepperTimerTask();
void rightStepperTimerTask();

void setupStepperController();

void enableSteppers();
void disableSteppers();

void updateStepperSpeed(float leftSpeed, float rightSpeed);
void updateSteppers();

#endif
