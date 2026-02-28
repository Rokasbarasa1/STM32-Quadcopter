#pragma once
#include <stdio.h>
#include <math.h>

struct running_average{
    uint16_t count;    // Current sample count
    float mean;     // Running mean
    float delta;    // Temp
};

struct running_average running_average_init();

float running_average_update(struct running_average* avg, float new_val);

void running_average_reset(struct running_average* avg);

float running_average_get_average(struct running_average* avg);