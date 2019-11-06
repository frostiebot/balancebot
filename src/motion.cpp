#include "motion.h"

#define __PGMSPACE_H_ 1 // stop compile errors of redefined typedefs and defines with ESP32-Arduino

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful

uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus;    // return status after each device operation (0 = success, !0 = error)

uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q; // [w, x, y, z]         quaternion container

VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector

float euler[3]; // [psi, theta, phi]    Euler angle container
float ypr[3];   // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuDataReady = false;
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high


void IRAM_ATTR dmpDataReady() {
  mpuInterrupt = true;
}

void setupMotionSensor() {
  xTaskCreatePinnedToCore(motionSensorTask, "motionSensorTask", 10000, NULL, 20, NULL, 1);
}

void motionSensorTask(void* pvParameters) {
  delay(100);

  Wire.begin();
  Wire.setClock(400000L); // 400kHz I2C clock. Comment this line if having compilation difficulties
  // Wire.setClock(100000L); // 100kHz I2C clock. Comment this line if having compilation difficulties

  delay(1000);

  mpu.initialize();
  delay(100);

  mpu.reset(); // help startup reliably - doesn't always work though.
  delay(100);

  mpu.resetI2CMaster();
  delay(100);

  mpu.initialize();

  gpio_set_direction(MPU_INTERRUPT_PIN, GPIO_MODE_INPUT);

  // while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for someone else's test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);
    // enable Arduino interrupt detection
    // gpio_ins
    attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
  }

  delay(100);

  // ================== LOOP ==================

  while (true) {
    // if programming failed, don't try to do anything
    if (!dmpReady) delay(100);

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
      // other program behavior stuff here
      //
      // if you are really paranoid you can frequently test in between other
      // stuff to see if mpuInterrupt is true, and if so, "break;" from the
      // while() loop to immediately process the MPU data
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount >= 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();

      // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize)
        fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      // display real acceleration, adjusted to remove gravity
      // mpu.dmpGetQuaternion(&q, fifoBuffer); // already got above
      mpu.dmpGetAccel(&aa, fifoBuffer);
      // mpu.dmpGetGravity(&gravity, &q); // already got above
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

      // display initial world-frame acceleration, adjusted to remove gravity
      // and rotated based on known orientation from quaternion
      // mpu.dmpGetQuaternion(&q, fifoBuffer); // already got above
      // mpu.dmpGetAccel(&aa, fifoBuffer); // already got above
      // mpu.dmpGetGravity(&gravity, &q); // already got above
      // mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity); // already got above
      mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    }
  } // end of loop
} // end motionsensorTask
