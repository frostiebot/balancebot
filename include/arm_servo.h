#ifndef ARM_SERVO_H
#define ARM_SERVO_H

#include "servo.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
// #include <HardwareSerial.h>

#define ARM_SERVO_TASK_FREQUENCY  20

void setupArmServo();
void syncArmServoTask(void* params);

#endif
