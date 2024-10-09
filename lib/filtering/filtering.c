#include "./filtering.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct low_pass_filter filtering_init_low_pass_filter(float cutoff_frequency, float sample_rate){
    struct low_pass_filter new_filter;
    new_filter.alpha = 1.0 / (1.0 + (2.0 * M_PI * cutoff_frequency / sample_rate));
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

float calculate_low_pass_phase_delay_seconds(float cutoff_frequency, float sample_rate){
    float phase_shift = atan((2 * M_PI * cutoff_frequency) / sample_rate);
    return phase_shift / (2.0 * M_PI * cutoff_frequency);
}

struct high_pass_filter filtering_init_high_pass_filter(float cutoff_frequency, float sample_rate){
    struct high_pass_filter new_filter;
    new_filter.alpha = 1.0 / (1.0 + (sample_rate / (2.0 * M_PI * cutoff_frequency)));
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

float high_pass_calculate_phase_delay_seconds(float cutoff_frequency, float sample_rate){
    float phase_shift = atan((2 * M_PI * cutoff_frequency) / sample_rate);
    return phase_shift / (2.0 * M_PI * cutoff_frequency); 
}

