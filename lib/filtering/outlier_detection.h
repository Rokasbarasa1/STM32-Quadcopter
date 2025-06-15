#pragma once
#include <stdio.h>
#include <math.h>

struct outlier_detection{
    float *buffer;
    uint8_t window_size;
    uint8_t index;
    uint8_t count;
    float outlier_threshold;
};

struct outlier_detection outlier_detection_init(uint8_t window_size, float outlier_threshold);
float outlier_detection_process(struct outlier_detection* filter, float input);