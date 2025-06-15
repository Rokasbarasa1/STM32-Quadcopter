#include "./moving_average.h"

struct moving_average moving_average_init(uint8_t window_size){
    struct moving_average new_filter;
        
    new_filter.window_size = window_size;
    new_filter.input_history = (float *)calloc(window_size, sizeof(float));
    new_filter.index = 0;
    new_filter.sum = 0.0f;
    new_filter.count = 0;

    return new_filter;
}

float moving_average_process(struct moving_average* filter, float input){
    // Remove the oldest value from the sum
    filter->sum -= filter->input_history[filter->index];
    // Add the new value
    filter->input_history[filter->index] = input;
    filter->sum += input;

    // Advance the index
    filter->index = (filter->index + 1) % filter->window_size;

    // Track the number of samples processed (for startup)
    if (filter->count < filter->window_size) {
        filter->count++;
    }

    // Compute the average
    return filter->sum / filter->count;
}
