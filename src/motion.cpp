#include "config.h"
#include "cpu_map.h"
#include "compat.h"

#include "motion.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps_V6_12.h>
#include <PID_v1.h>

static const char* TAG = "motion";

uint8_t interruptStatus;         // holds actual interrupt status byte from MPU
uint8_t deviceStatus;            // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;             // expected DMP packet size (default is 42 bytes)
uint16_t fifoByteCount;          // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];          // FIFO storage buffer

Quaternion quat;                 // [w, x, y, z]         quaternion container
VectorFloat gravity;             // [x, y, z]            gravity vector
float ypr[3];                    // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

#ifdef ENABLE_MOTION_OUTPUT_TEAPOT
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
#endif

// PID Junk
double setpoint = 180.8; // 175.8;
double Kp = 21;   // 50;
double Kd = 0.8; // 1.4;
double Ki = 140;   // 60;

double input, output;

MPU6050 mpu;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

SemaphoreHandle_t motionInterruptSemaphore;


void updateTeapotPacket(uint8_t* packet) {
  #ifdef ENABLE_MOTION_OUTPUT_TEAPOT
  teapotPacket[2] = packet[0];
  teapotPacket[3] = packet[1];
  teapotPacket[4] = packet[4];
  teapotPacket[5] = packet[5];
  teapotPacket[6] = packet[8];
  teapotPacket[7] = packet[9];
  teapotPacket[8] = packet[12];
  teapotPacket[9] = packet[13];
  Serial.write(teapotPacket, 14);
  teapotPacket[11]++;
  #endif
}

void setupMotionSensor() {
  Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN, MOTION_I2C_CLOCK);

  mpu.initialize();
  ESP_LOGD(TAG, "Testing MPU Connection: %d", mpu.testConnection());
  deviceStatus = mpu.dmpInitialize();

  // Calibrate
  mpu.setXGyroOffset(MOTION_X_GYRO_OFFSET);
  mpu.setYGyroOffset(MOTION_Y_GYRO_OFFSET);
  mpu.setZGyroOffset(MOTION_Z_GYRO_OFFSET);
  mpu.setXAccelOffset(MOTION_X_ACCEL_OFFSET);
  mpu.setYAccelOffset(MOTION_Y_ACCEL_OFFSET);
  mpu.setZAccelOffset(MOTION_Z_ACCEL_OFFSET);

  if (deviceStatus == 0) {
    ESP_LOGD(TAG, "Enabling DMP...");
    mpu.setDMPEnabled(true);

    constexpr gpio_config_t gpioConfig {
      .pin_bit_mask = (1 << MPU_INTERRUPT_PIN),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_ENABLE,
      .intr_type = GPIO_INTR_POSEDGE
    };

    gpio_config(&gpioConfig);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(MPU_INTERRUPT_PIN, onMotionDataReadyISR, NULL);

    motionInterruptSemaphore = xSemaphoreCreateBinary();

    ESP_LOGD(TAG, "DMP ready! Waiting for first interrupt...");
    interruptStatus = mpu.getIntStatus();
    packetSize = mpu.dmpGetFIFOPacketSize();

    xTaskCreate(motionDataHandlerTask, "motionDataHandlerTask", 4 * 2048, NULL, 6, NULL);
    xTaskCreate(computePIDTask, "computePIDTask", 2 * 1024, NULL, 5, NULL);
  } else {
    // ERROR !
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    ESP_LOGE(TAG, "DMP Init failed (code: %d)", deviceStatus);
  }
}

void onMotionDataReadyISR(void *args) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(motionInterruptSemaphore, &xHigherPriorityTaskWoken);
}

void motionDataHandlerTask(void* pvParameters) {
  mpu.resetFIFO();

  while (true) {
    xSemaphoreTake(motionInterruptSemaphore, portMAX_DELAY);

    // if (notificationValue > 1) {
    //   ESP_LOGW(TAG, "Task Notification higher than 1, value: %d", notificationValue);
    //   mpu.resetFIFO();
    //   continue;
    // }

    // wait for MPU interrupt or extra packet(s) available
    // while (!mpuInterrupt && fifoByteCount < packetSize) {
    // no mpu data - performing PID calculations and output to motors
    // pid.Compute();
    // ESP_LOGI(TAG, "PID Output: %d", output);
    // motorController.move(output, MIN_ABS_SPEED);
    // }

    // reset interrupt flag and get INT_STATUS byte
    // mpuInterrupt = false;
    interruptStatus = mpu.getIntStatus();
    fifoByteCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((interruptStatus & 0x10) || fifoByteCount >= 1024) {
    // if (notificationValue & 0x10 || fifoByteCount >= 1024) {
      // reset so we can continue cleanly
      ESP_LOGW(TAG, "FIFO overflow or something!");
      mpu.resetFIFO();
      continue;
    }

      // otherwise, check for DMP data ready interrupt (this should happen frequently)
    if (interruptStatus & 0x02) {
    // if (notificationValue & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoByteCount < packetSize) {
        fifoByteCount = mpu.getFIFOCount();
      }

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoByteCount -= packetSize;

      mpu.dmpGetQuaternion(&quat, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &quat);
      mpu.dmpGetYawPitchRoll(ypr, &quat, &gravity);

      updateTeapotPacket(fifoBuffer);

      input = ypr[2] * 180.0 / M_PI + 180.0;
    }
  }

  vTaskDelete(NULL);
}

void computePIDTask(void* pvParameters) {
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-255, 255);

  vTaskDelay(2000 / portTICK_PERIOD_MS);

  while (true) {
    // if (pid.Compute())
    //   ESP_LOGD(TAG, "PID COMPUTED - Input: %+6.2f\tOutput: %+6.2f", input, output);
    // ESP_LOGD(TAG, "PID - INPUT: %d\tOUTPUT: %d\t| MPU - Yaw: %+6.1f\tPitch: %+6.1f\tRoll: %+6.1f", input, output, ypr[0], ypr[1], ypr[2]);
    ESP_LOGD(TAG, "MPU - Yaw: %+6.1f\tPitch: %+6.1f\tRoll: %+6.1f\tRM: %+6.1f\t", ypr[0], ypr[1], ypr[2], (ypr[2] * 180.0 / M_PI + 180.0));
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }

  vTaskDelete(NULL);
}
