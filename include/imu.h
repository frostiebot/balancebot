#ifndef IMU_H
#define IMU_H

#include "cpu_map.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define MPU6050_ADDRESS_AD0_LOW 0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH  0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS MPU6050_ADDRESS_AD0_LOW

#define MOTION_I2C_CLOCK 400000L

void setupIMU();
void dmpReadFIFOTask(void* pvParameters);

#endif
