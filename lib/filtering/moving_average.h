#pragma once
#include <stdio.h>
#include <math.h>

struct moving_average{
    uint8_t window_size;
    float *input_history;
    uint8_t index;
    float sum;
    uint8_t count;
};

struct moving_average moving_average_init(uint8_t window_size);
float moving_average_process(struct moving_average* filter, float input);