#pragma once
#include <stdio.h>

// PT1 type
struct low_pass_filter{
    float previous_filter_output;
    float alpha;
};

struct low_pass_filter filtering_init_low_pass_filter(float alpha);
float low_pass_filter_read(struct low_pass_filter* filter, float current_value);

struct high_pass_filter{
    float previous_filter_output;
    float alpha;
    float previous_input_value;
};

struct high_pass_filter filtering_init_high_pass_filter(float alpha);
float high_pass_filter_read(struct high_pass_filter* filter, float current_value);