#pragma once
#include <stdio.h>
#include <math.h>

struct iir_filter{
    uint8_t filter_order;
    float *feedback_coefficients;
    float *feedfoward_coefficients;
    float *input_history;
    float *output_history;
};

struct iir_filter iir_filter_init(float filter_order, float *feedback_coefficients, float *feedfoward_coefficients);
float iir_filter_process(struct iir_filter* filter, float input);