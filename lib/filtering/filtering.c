#include "./filtering.h"

struct low_pass_filter filtering_init_low_pass_filter(float alpha){
    struct low_pass_filter new_filter;
    new_filter.alpha = alpha;
    new_filter.previous_filter_output = 0;

    return new_filter;
}

// Formula to calculate what frequency the filter attenuates:
// Frequency = <Sampling rate> / (2 * M_PI * ((1/alpha) - 1))

// Delay calculation based on frequency:
// Delay seconds = 1/(2 * M_PI * Frequency)
float low_pass_filter_read(struct low_pass_filter* filter, float current_value){
    float calculated_value = filter->alpha * filter->previous_filter_output + (1 - filter->alpha) * current_value;
    // printf("%.3f = %.3f * %.3f + (1 - %.3f) * %.3f;\n",calculated_value, filter->alpha, filter->previous_filter_output, filter->alpha, current_value);

    filter->previous_filter_output = calculated_value;
    return calculated_value;
}

struct high_pass_filter filtering_init_high_pass_filter(float alpha){
    struct high_pass_filter new_filter;
    new_filter.alpha = alpha;
    new_filter.previous_filter_output = 0;
    new_filter.previous_input_value = 0;

    return new_filter;
}

float high_pass_filter_read(struct high_pass_filter* filter, float current_value){
    float calculated_value = filter->alpha * (filter->previous_filter_output + current_value - filter->previous_input_value);

    filter->previous_filter_output = calculated_value;
    filter->previous_input_value = current_value;

    return calculated_value;
}
