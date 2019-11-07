#ifndef ARM_SERVO_H
#define ARM_SERVO_H

#include "servo.h"

#define ARM_SERVO_TASK_FREQUENCY  20

void setupArmServo();
void armServoTask(void* pvParameters);

#endif
