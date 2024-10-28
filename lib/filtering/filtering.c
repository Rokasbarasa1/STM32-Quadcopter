#include "./filtering.h"
#include <math.h>
#include "../utils/math_constants.h"

struct low_pass_filter filtering_init_low_pass_filter(float cutoff_frequency, float sample_rate){
    struct low_pass_filter new_filter;
    new_filter.alpha = 1.0 / (1.0 + (M_PI_2 * cutoff_frequency / sample_rate));
    new_filter.previous_filter_output = 0;

    return new_filter;
}

// Formula to calculate what frequency the filter attenuates:
// Frequency = <Sampling rate> / (2 * M_PI * ((1/alpha) - 1))

// Delay calculation based on frequency:
// Delay seconds = 1/(2 * M_PI * Frequency)
float low_pass_filter_read(struct low_pass_filter* filter, float current_value){
    filter->previous_filter_output = filter->alpha * filter->previous_filter_output + (1 - filter->alpha) * current_value;
    // printf("%.3f = %.3f * %.3f + (1 - %.3f) * %.3f;\n",calculated_value, filter->alpha, filter->previous_filter_output, filter->alpha, current_value);

    return filter->previous_filter_output;
}

float calculate_low_pass_phase_delay_seconds(float cutoff_frequency, float sample_rate){
    float phase_shift = atan((M_PI_2 * cutoff_frequency) / sample_rate);
    return phase_shift / (M_PI_2 * cutoff_frequency);
}

struct high_pass_filter filtering_init_high_pass_filter(float cutoff_frequency, float sample_rate){
    struct high_pass_filter new_filter;
    new_filter.alpha = 1.0 / (1.0 + (sample_rate / (M_PI_2 * cutoff_frequency)));
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
    float phase_shift = atan((M_PI_2 * cutoff_frequency) / sample_rate);
    return phase_shift / (M_PI_2 * cutoff_frequency); 
}

// The notch width is meant to help represent the Q value 
// Calculate Q:  Q = center_frequency / notch_width_hz
struct notch_filter notch_filter_init(float center_frequency, float notch_width_hz, float refresh_rate){
    struct notch_filter new_filter;

    new_filter.center_frequency_radians = M_PI_2 * center_frequency;
    new_filter.notch_width_hz_radians = M_PI_2 * notch_width_hz;


    new_filter.sample_time_seconds = 1.0/(float)refresh_rate;
    new_filter.sample_time_seconds_squared = new_filter.sample_time_seconds * new_filter.sample_time_seconds;


    new_filter.q = new_filter.center_frequency_radians / new_filter.notch_width_hz_radians;
    new_filter.pre_wrap_calculation_1 = 2.0f / new_filter.sample_time_seconds;

    // Pre-wrap center frequency
    float pre_wrap_radians = new_filter.pre_wrap_calculation_1 * tanf(0.5f * new_filter.center_frequency_radians * new_filter.sample_time_seconds);

    new_filter.alpha = 4.0f + pre_wrap_radians * pre_wrap_radians * new_filter.sample_time_seconds_squared;
    new_filter.beta = 2.0f * new_filter.notch_width_hz_radians * new_filter.sample_time_seconds;
    
    // Clear arrays
    memset(new_filter.input_array, 0, 3 * sizeof(float));
    memset(new_filter.output_array, 0, 3 * sizeof(float));

    return new_filter;
}

float notch_filter_read(struct notch_filter* filter, float input){
    // Shift input samples and add the new one
    filter->input_array[2] = filter->input_array[1];
    filter->input_array[1] = filter->input_array[0];
    filter->input_array[0] = input;
    
    // Shift the output samples and add the new computed one
    filter->output_array[2] = filter->output_array[1];
    filter->output_array[1] = filter->output_array[0];

    // This could be optimized if the frequency is not reset all the time
    filter->output_array[0] =  (filter->alpha * filter->input_array[0] + 2.0f * (filter->alpha - 8.0f) * filter->input_array[1] + filter->alpha * filter->input_array[2]
                             - (2.0f * (filter->alpha - 8.0f) * filter->output_array[1] + (filter->alpha - filter->beta) * filter->output_array[2]))
                             / (filter->alpha + filter->beta);

    return filter->output_array[0];
}

// Set the Q and as a result calculate the new notch width and beta
void notch_filter_set_Q(struct notch_filter* filter, float q_value){
    filter->q = q_value;

    //Recalculate beta based on Q and the notch width derived from it
    filter->notch_width_hz_radians = filter->center_frequency_radians / filter->q;
    filter->beta = 2.0f * filter->notch_width_hz_radians * filter->sample_time_seconds;
}

// Set the center frequency and do nothing for the beta
void notch_filter_set_center_frequency(struct notch_filter* filter, float new_center_frequency){

    // Same calculation as the init
    filter->center_frequency_radians = M_PI_2 * new_center_frequency;

    float pre_wrap_radians = filter->pre_wrap_calculation_1 * tanf(0.5f * filter->center_frequency_radians * filter->sample_time_seconds);

    filter->alpha = 4.0f + pre_wrap_radians * pre_wrap_radians * filter->sample_time_seconds_squared;

    // Beta does not change here
}

// Set the frequency but adjust the beta based on the frequency
void notch_filter_set_center_frequency_Q_constant(struct notch_filter* filter, float new_center_frequency){
    // This sets the frequency
    notch_filter_set_center_frequency(filter, new_center_frequency);

    // Now set the new beta again
    notch_filter_set_Q(filter, filter->q);

    // A bigger center frequency means the notch gets wider
}