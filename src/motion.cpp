#include "motion.h"
#include "compat.h"

#include <Arduino.h>
#include <PID_v1.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

static const char* TAG = "motion";

MPU6050 mpu;

uint8_t mpuInterruptStatus;         // holds actual interrupt status byte from MPU
uint8_t mpuDeviceOperationStatus;   // return status after each device operation (0 = success, !0 = error)

uint16_t fifoPacketSize;            // expected DMP packet size (default is 42 bytes)
uint16_t fifoByteCount;             // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];             // FIFO storage buffer

Quaternion q;                       // [w, x, y, z]         quaternion container
VectorFloat gravity;                // [x, y, z]            gravity vector
float ypr[3];                       // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

// PID Junk
double originalSetpoint = 175.8; //175.8;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;
// int moveState = 0; // 0 = balance; 1 = back; 2 = forth
double Kp = 50;
double Kd = 1.4;
double Ki = 60;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);


void IRAM_ATTR dmpDataReady() {
  mpuInterrupt = true;
}

void setMpuOffsets(int16_t xGyroOffset, int16_t yGyroOffset, int16_t zGyroOffset, int16_t zAccelOffset) {
  mpu.setXGyroOffset(xGyroOffset);
  mpu.setYGyroOffset(yGyroOffset);
  mpu.setZGyroOffset(zGyroOffset);
  mpu.setZAccelOffset(zAccelOffset);
}

uint8_t initializeMpuDmp() {
  mpu.initialize();
  ESP_LOGD(TAG, "Testing MPU Connection: %d", mpu.testConnection());
  return mpu.dmpInitialize();
}

void initializePID() {
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-255, 255);
}

void setupMotionSensor() {
  // ESP_ERROR_CHECK(gpio_set_direction(MPU_INTERRUPT_PIN, GPIO_MODE_INPUT));
  gpio_set_direction(MPU_INTERRUPT_PIN, GPIO_MODE_INPUT);

  Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN, MOTION_I2C_CLOCK);
  // Wire.setClock(MOTION_I2C_CLOCK);

  mpuDeviceOperationStatus = initializeMpuDmp();

  mpu.CalibrateAccel();
  mpu.CalibrateGyro();

  if (mpuDeviceOperationStatus == 0) {
    mpu.setDMPEnabled(true);

    // setMpuOffsets(
    //   MOTION_X_GYRO_OFFSET,
    //   MOTION_Y_GYRO_OFFSET,
    //   MOTION_Z_GYRO_OFFSET,
    //   MOTION_Z_ACCEL_OFFSET
    // );

    //TODO Convert this into "native" esp-idf code

    mpuInterruptStatus = mpu.getIntStatus();
    fifoPacketSize = mpu.dmpGetFIFOPacketSize();

    initializePID();

    xTaskCreatePinnedToCore(&motionSensorTask, "motionSensorTask", 4096, NULL, 20, NULL, 0);

    attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), dmpDataReady, RISING);
  } else {
    // ERROR !
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    ESP_LOGE(TAG, "DMP Init failed (code: %d)", mpuDeviceOperationStatus);
  }
}

void motionSensorTask(void* pvParameters) {
  while (1) {
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoByteCount < fifoPacketSize) {
      // no mpu data - performing PID calculations and output to motors
      pid.Compute();
      ESP_LOGI(TAG, "PID Output: %d", output);
      // motorController.move(output, MIN_ABS_SPEED);
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuInterruptStatus = mpu.getIntStatus();
    fifoByteCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuInterruptStatus & 0x10) || fifoByteCount >= 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      ESP_LOGW(TAG, "FIFO overflow!");

      // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuInterruptStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoByteCount < fifoPacketSize) {
        fifoByteCount = mpu.getFIFOCount();
      }

      // bool MPU6050::dmpPacketAvailable() { return getFIFOCount() >= dmpGetFIFOPacketSize(); }
      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, fifoPacketSize);

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoByteCount -= fifoPacketSize;

      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      // mpu.dmpGetAccel()
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      ESP_LOGD(TAG, "[YPR]\tY: %f\tP: %f\tR: %f", (ypr[0] * 180 / M_PI), (ypr[1] * 180 / M_PI), (ypr[2] * 180 / M_PI));

      input = ypr[2] * 180 / M_PI + 180;
    }
  }

  vTaskDelete(NULL);
}
