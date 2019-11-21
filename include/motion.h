#ifndef MOTION_H
#define MOTION_H

// #define DEBUG
#define __PGMSPACE_H_ 1 // stop compile errors of redefined typedefs and defines with ESP32-Arduino

// Set to 100000L (100kHz) I2C clock if having compilation difficulties
#define MOTION_I2C_CLOCK      (400000L)

//           X Accel  Y Accel  Z Accel   X Gyro   Y Gyro   Z Gyro
// OFFSETS      124,   -6657,     861,      95,     -68,     -47

#define MOTION_X_ACCEL_OFFSET (125)    // [87,88]       --> [0,19]         // NO PREVIOUS VALUE
#define MOTION_Y_ACCEL_OFFSET (-6661) // [-6633,-6632] --> [-4,14]        // NO PREVIOUS VALUE
#define MOTION_Z_ACCEL_OFFSET (859)   // [849,850]     --> [16378,16406]  // 1788
#define MOTION_X_GYRO_OFFSET  (94)    // [98,99]       --> [0,3]          // 220
#define MOTION_Y_GYRO_OFFSET  (-68)   // [-71,-70]     --> [0,3]          // 76
#define MOTION_Z_GYRO_OFFSET  (-47)   // [-46,-45]     --> [0,3]          // -85

// typedef struct {
//   uint32_t yaw;
//   uint32_t pitch;
//   uint32_t roll;
// } yaw_pitch_roll_t;

void setupMotionSensor();

void onMotionDataReadyISR(void* args);
void motionDataHandlerTask(void* pvParameters);

void computePIDTask(void* pvParameters);

#endif
