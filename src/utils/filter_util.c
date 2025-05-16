#include "includes/filter_util.h"

float MovingAverageFilter(float *buffer, int *index, float *sum, int size, float newValue) {
    *sum -= buffer[*index];
    buffer[*index] = newValue;
    *sum += newValue;
    *index = (*index + 1) % size;
    return *sum / size;
}

float LowPassFilter(float previousValue, float newValue, float alpha) {
    return alpha * newValue + (1 - alpha) * previousValue;
}

float BandPassFilter(float *buffer, int size, float newValue, float lowCutoff, float highCutoff) {
    
    return newValue; 
}