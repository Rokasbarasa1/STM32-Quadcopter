#pragma once
#include <stdio.h>

// PT1 type
struct low_pass_filter{
    float previous_filter_output;
    float alpha;
};

struct low_pass_filter filtering_init_low_pass_filter(float cutoff_frequency, float sample_rate);
float low_pass_filter_read(struct low_pass_filter* filter, float current_value);

struct high_pass_filter{
    float previous_filter_output;
    float alpha;
    float previous_input_value;
};

struct high_pass_filter filtering_init_high_pass_filter(float cutoff_frequency, float sample_rate);
float high_pass_filter_read(struct high_pass_filter* filter, float current_value);

struct notch_filter {
    float a1, a2;  // Denominator coefficients
    float b0, b1, b2;  // Numerator coefficients
    float previous_input1, previous_input2;  // Previous input values
    float previous_output1, previous_output2;  // Previous output values
};
