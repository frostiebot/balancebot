#ifndef COMPAT_H
#define COMPAT_H

#define _constrain(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))
#define _min(a, b) ((a) < (b) ? (a) : (b))
#define _max(a, b) ((a) > (b) ? (a) : (b))

#endif
