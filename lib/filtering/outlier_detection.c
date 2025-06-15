#include "./outlier_detection.h"

struct outlier_detection outlier_detection_init(uint8_t window_size, float outlier_threshold){
    struct outlier_detection new_filter;
        
    new_filter.window_size = window_size;
    new_filter.index = 0;
    new_filter.count = 0;
    new_filter.outlier_threshold = outlier_threshold;
    new_filter.buffer = (float *)malloc(window_size * sizeof(float));

    return new_filter;
}

float outlier_detection_process(struct outlier_detection* filter, float input){
   // Update buffer with new input
    filter->buffer[filter->index] = input;
    filter->index = (filter->index + 1) % filter->window_size;
    
    // Update count until window is filled
    if(filter->count < filter->window_size) {
        filter->count++;
    }

    // Calculate mean and standard deviation
    float sum = 0.0f;
    float sum_sq = 0.0f;
    
    for(uint8_t i = 0; i < filter->count; i++) {
        sum += filter->buffer[i];
        sum_sq += filter->buffer[i] * filter->buffer[i];
    }
    
    float mean = sum / filter->count;
    float variance = (sum_sq / filter->count) - (mean * mean);
    float std_dev = sqrtf(fabsf(variance));

    // Check if input is an outlier
    if(fabsf(input - mean) > (filter->outlier_threshold * std_dev)) {
        // Replace outlier with mean value
        filter->buffer[(filter->index - 1 + filter->window_size) % filter->window_size] = mean;
        return mean;
    }
    
    return input;
}