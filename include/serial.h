#ifndef SERIAL_H
#define SERIAL_H

// #include <HardwareSerial.h>

#define SERIAL_BAUD_RATE 115200

void setupSerial();
void serialCheckTask(void* params);

#endif
