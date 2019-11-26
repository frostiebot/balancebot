#include "pid.h"

#include <esp32-hal.h>

// Declare variables
float _Kp = 7;       // (P)roportional Tuning Parameter
float _Ki = 6;       // (I)ntegral Tuning Parameter
float _Kd = 3;       // (D)erivative Tuning Parameter

float iTerm;        // Used to accumulate error (integral)
float lastTime = 0; // Records the last time function was called
float maxPID = 255; // The maximum value that can be output
float oldCurrent;

float computePID(float target, float current) {
  // Calculate the time since function was last called
  float thisTime = millis();
  float dT = thisTime - lastTime;
  lastTime = thisTime;

  // Calculate error between target and current values
  float error = target - current;
  float pid = 0;

  // Calculate the integral and derivative terms
  iTerm += error * dT;
  float dTerm = (current - oldCurrent) / dT;

  // Set old variable to equal new ones
  oldCurrent = current;

  // Multiply each term by its constant, and add it all up
  pid = (error * _Kp) + (iTerm * _Ki) - (dTerm * _Kd);

  // Limit PID value to maximum values
  if (maxPID > 0 && pid > maxPID)
    pid = maxPID;
  else if (maxPID > 0 && pid < -maxPID)
    pid = -maxPID;

  return pid;
}
