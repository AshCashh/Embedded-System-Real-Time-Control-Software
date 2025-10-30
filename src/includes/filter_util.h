#ifndef FILTER_UTIL_H
#define FILTER_UTIL_H

#include <stdint.h>

// Function prototypes for digital filters
float MovingAverageFilter(float *buffer, int *index, float *sum, int size, float newValue);
float LowPassFilter(float previousValue, float newValue, float alpha);
float BandPassFilter(float *buffer, int size, float newValue, float lowCutoff, float highCutoff);

#endif // FILTER_UTIL_H