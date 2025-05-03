#pragma once
#include <stdio.h>
#include <math.h>

// --------------------------------------------------------------------------------------------------------------------------------
struct low_pass_pt1_filter{
    float previous_filter_output;
    float alpha;
};

struct low_pass_pt1_filter filtering_init_low_pass_filter_pt1(float cutoff_frequency, float sample_rate);
void low_pass_filter_pt1_set_initial_values(struct low_pass_pt1_filter* filter, float initial_value);
float low_pass_filter_pt1_read(struct low_pass_pt1_filter* filter, float current_value);


// --------------------------------------------------------------------------------------------------------------------------------
struct low_pass_biquad_filter{
    float coefficient_b0;
    float coefficient_b1;
    float coefficient_b2;
    float coefficient_a1;// a0 is the alpha in init
    float coefficient_a2;
    float previous_input1;
    float previous_input2;
    float previous_output1; // Previous
    float previous_output2; // Previous previous
    float sample_rate_division; // Optimization of performance
};

struct low_pass_biquad_filter filtering_init_low_pass_filter_biquad(float cutoff_frequency, float sample_rate);
void low_pass_filter_biquad_set_initial_values(struct low_pass_biquad_filter* filter, float initial_value);
float low_pass_filter_biquad_read(struct low_pass_biquad_filter* filter, float current_value);
void low_pass_filter_biquad_set_cutoff_frequency(struct low_pass_biquad_filter* filter, float cutoff_frequency);
void low_pass_filter_biquad_set_cutoff_frequency_using_reference_filter(struct low_pass_biquad_filter* filter, struct low_pass_biquad_filter* filter_reference);

// --------------------------------------------------------------------------------------------------------------------------------
struct high_pass_filter{
    float previous_filter_output;
    float alpha;
    float previous_input_value;
};

struct high_pass_filter filtering_init_high_pass_filter(float cutoff_frequency, float sample_rate);
float high_pass_filter_read(struct high_pass_filter* filter, float current_value);

// --------------------------------------------------------------------------------------------------------------------------------
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
void notch_filter_set_center_frequency_Q_constant_using_reference_filter(struct notch_filter* filter, struct notch_filter* filter_reference);

struct notch_filter2 {
    float input_array[2];
    float output_array[2];

    float a1;
    float a2;
    float b0;
    float b1;
    float b2;

    float precalculated_refresh_rate_division;
};

struct notch_filter2 notch_filter2_init(float center_frequency, float notch_width_hz, float refresh_rate);

struct notch_filter_q {
    float q; // Alternative to beta
    float input_array[2];
    float output_array[2];

    float a0;
    float a1;
    float a2;
    float b0;
    float b1;
    float b2;

    float precalculated_refresh_rate_division;
    float precalculated_q_division;
};

struct notch_filter_q notch_filter_q_init(float center_frequency, float q, float refresh_rate);
void notch_filter_q_set_center_frequency(struct notch_filter_q* filter, float new_center_frequency);
void notch_filter_q_set_center_frequency_using_reference_filter(struct notch_filter_q* filter, struct notch_filter_q* filter_reference);
float notch_filter_q_read(struct notch_filter_q* filter, float input);