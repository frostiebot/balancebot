#ifndef COMPAT_H
#define COMPAT_H

#define M_PI 3.14159265358979323846

#define _CONSTRAIN(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))
#define _MIN(a, b) ((a) < (b) ? (a) : (b))
#define _MAX(a, b) ((a) > (b) ? (a) : (b))

#endif
