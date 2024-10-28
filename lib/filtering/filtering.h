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
    float q; // Alternative to beta
    float input_array[3];
    float output_array[3];

    float center_frequency_radians;
    float notch_width_hz_radians;

    float sample_time_seconds;
    float sample_time_seconds_squared;
    float pre_wrap_calculation_1;
};

struct notch_filter notch_filter_init(float center_frequency, float notch_width_hz, float refresh_rate);
float notch_filter_read(struct notch_filter* filter, float input);
void notch_filter_set_Q(struct notch_filter* filter, float q_value);
void notch_filter_set_center_frequency(struct notch_filter* filter, float new_center_frequency);
void notch_filter_set_center_frequency_Q_constant(struct notch_filter* filter, float new_center_frequency);