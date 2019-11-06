#ifndef MOTION_H
#define MOTION_H

#include "cpu_map.h"

void dmpDataReady();
void setupMotionSensor();
void motionSensorTask(void* pvParameters);

#endif
