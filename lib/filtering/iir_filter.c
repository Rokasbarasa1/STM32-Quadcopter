#include "./iir_filter.h"
#include "stdlib.h"

struct iir_filter iir_filter_init(float filter_order, float *feedback_coefficients, float *feedfoward_coefficients){

    struct iir_filter new_filter;
        
    new_filter.filter_order = filter_order;
    new_filter.feedback_coefficients = (float *)malloc(sizeof(float) * (filter_order + 1));
    new_filter.feedfoward_coefficients = (float *)malloc(sizeof(float) * (filter_order + 1));

    new_filter.input_history = (float *)calloc(filter_order + 1, sizeof(float)); // zero-initialized
    new_filter.output_history = (float *)calloc(filter_order + 1, sizeof(float)); // zero-initialized


    // Copy coefficients
    for (int i = 0; i <= filter_order; i++) {
        new_filter.feedback_coefficients[i] = feedback_coefficients[i];
        new_filter.feedfoward_coefficients[i] = feedfoward_coefficients[i];
    }

    return new_filter;
}

float iir_filter_process(struct iir_filter* filter, float input){
    // Shift history
    for (int i = filter->filter_order; i > 0; i--) {
        filter->input_history[i] = filter->input_history[i - 1];
        filter->output_history[i] = filter->output_history[i - 1];
    }

    filter->input_history[0] = input;

    // Compute output
    float output = 0.0f;
    for (int i = 0; i <= filter->filter_order; i++) {
        output += filter->feedfoward_coefficients[i] * filter->input_history[i];
    }
    for (int i = 1; i <= filter->filter_order; i++) {
        output -= filter->feedback_coefficients[i] * filter->output_history[i];
    }
    filter->output_history[0] = output;
    return output;
}
