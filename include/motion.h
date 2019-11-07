#ifndef MOTION_H
#define MOTION_H

#include "cpu_map.h"

#define __PGMSPACE_H_ 1 // stop compile errors of redefined typedefs and defines with ESP32-Arduino

// Set to 100000L (100kHz) I2C clock if having compilation difficulties
#define MOTION_I2C_CLOCK      400000L

#define MOTION_X_GYRO_OFFSET  220
#define MOTION_Y_GYRO_OFFSET  76
#define MOTION_Z_GYRO_OFFSET  -85
#define MOTION_Z_ACCEL_OFFSET 1788

// typedef struct {
//   uint32_t yaw;
//   uint32_t pitch;
//   uint32_t roll;
// } yaw_pitch_roll_t;

void setupMotionSensor();
void motionSensorTask(void* pvParameters);

#endif
