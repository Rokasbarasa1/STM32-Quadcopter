#pragma once
#include <stdio.h>
#include <math.h>

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
    float alpha;
    float beta;
    float input_array[3];
    float output_array[3];
};

struct notch_filter notch_filter_init(float center_frequency, float notch_width_hz, float sample_time_seconds);
float notch_filter_read(struct notch_filter* filter, float input);