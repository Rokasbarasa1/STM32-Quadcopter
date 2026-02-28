#include "averaging.h"

struct running_average running_average_init(){
    struct running_average running_average;

    running_average.count = 0;
    running_average.mean = 0.0f;
    running_average.delta = 0.0f;

    return running_average;
}

float running_average_update(struct running_average* avg, float new_val) {
    float delta = new_val - avg->mean;
    avg->count++;
    avg->mean += delta / avg->count;
    return avg->mean;
}

void running_average_reset(struct running_average* avg){
    avg->count = 0;
    avg->mean = 0.0f;
    avg->delta = 0.0f;
}

float running_average_get_average(struct running_average* avg){
    return avg->mean;
}