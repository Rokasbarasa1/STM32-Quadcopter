#include "./filtering.h"
#include <math.h>
#include "../utils/math_constants.h"

struct low_pass_pt1_filter filtering_init_low_pass_filter_pt1(float cutoff_frequency, float sample_rate){
    struct low_pass_pt1_filter new_filter;
    new_filter.alpha = 1.0 / (1.0 + (M_PI_2 * cutoff_frequency / sample_rate));
    new_filter.previous_filter_output = 0;

    return new_filter;
}

// Formula to calculate what frequency the filter attenuates:
// Frequency = <Sampling rate> / (2 * M_PI * ((1/alpha) - 1))

// Delay calculation based on frequency:
// Delay seconds = 1/(2 * M_PI * Frequency)
float low_pass_filter_pt1_read(struct low_pass_pt1_filter* filter, float current_value){
    filter->previous_filter_output = filter->alpha * filter->previous_filter_output + (1 - filter->alpha) * current_value;
    // printf("%.3f = %.3f * %.3f + (1 - %.3f) * %.3f;\n",calculated_value, filter->alpha, filter->previous_filter_output, filter->alpha, current_value);

    return filter->previous_filter_output;
}

float calculate_low_pass_phase_delay_seconds(float cutoff_frequency, float sample_rate){
    float phase_shift = atan((M_PI_2 * cutoff_frequency) / sample_rate);
    return phase_shift / (M_PI_2 * cutoff_frequency);
}

#define OPTIMIZE_COEFFICIENT_A0_DIVISION (1.0f / (2.0f * 0.707f))
struct low_pass_biquad_filter filtering_init_low_pass_filter_biquad(float cutoff_frequency, float sample_rate){
    struct low_pass_biquad_filter new_filter;

    new_filter.sample_rate_division = 1.0f/sample_rate; // Optimize out the division operation

    float omega = M_PI_2 * cutoff_frequency * new_filter.sample_rate_division;
    float sin_omega = sinf(omega);
    float cos_omega = cosf(omega);
    float coefficient_a0 = sin_omega * OPTIMIZE_COEFFICIENT_A0_DIVISION;

    new_filter.coefficient_b0 = (1 - cos_omega) * 0.5f;
    new_filter.coefficient_b1 = 1 - cos_omega;
    new_filter.coefficient_b2 = (1 - cos_omega) * 0.5f;
    new_filter.coefficient_a1 = -2.0f * cos_omega;
    new_filter.coefficient_a2 = 1.0f - coefficient_a0;

    // Normalize to prepare for direct from 1 normalized formula
    float a0_normalization = 1.0f / (1.0f + coefficient_a0);

    // Use that normalized value and normalize all of them based on it
    new_filter.coefficient_b0 *= a0_normalization;
    new_filter.coefficient_b1 *= a0_normalization;
    new_filter.coefficient_b2 *= a0_normalization;
    new_filter.coefficient_a1 *= a0_normalization;
    new_filter.coefficient_a2 *= a0_normalization;

    new_filter.previous_input1 = 0;
    new_filter.previous_input2 = 0;
    new_filter.previous_output1 = 0;
    new_filter.previous_output2 = 0;

    return new_filter;
}

float low_pass_filter_biquad_read(struct low_pass_biquad_filter* filter, float current_value){
    // Direct form 1 normalized formula
    float output = filter->coefficient_b0 * current_value + 
        filter->coefficient_b1 * filter->previous_input1 + 
        filter->coefficient_b2 * filter->previous_input2 - 
        filter->coefficient_a1 * filter->previous_output1 - 
        filter->coefficient_a2 * filter->previous_output2;

    filter->previous_input2 = filter->previous_input1;
    filter->previous_input1 = current_value;

    filter->previous_output2 = filter->previous_output1;
    filter->previous_output1 = output;

    return output;
}

void low_pass_filter_biquad_set_cutoff_frequency(struct low_pass_biquad_filter* filter, float cutoff_frequency){
    float omega = M_PI_2 * cutoff_frequency * filter->sample_rate_division;
    float sin_omega = sinf(omega);
    float cos_omega = cosf(omega);
    float coefficient_a0 = sin_omega * OPTIMIZE_COEFFICIENT_A0_DIVISION;

    filter->coefficient_b0 = (1 - cos_omega) * 0.5f;
    filter->coefficient_b1 = 1 - cos_omega;
    filter->coefficient_b2 = (1 - cos_omega) * 0.5f;
    filter->coefficient_a1 = -2.0f * cos_omega;
    filter->coefficient_a2 = 1.0f - coefficient_a0;

    // Normalize to prepare for direct from 1 normalized formula
    float a0_normalization = 1.0f / (1.0f + coefficient_a0);

    // Use that normalized value and normalize all of them based on it
    filter->coefficient_b0 *= a0_normalization;
    filter->coefficient_b1 *= a0_normalization;
    filter->coefficient_b2 *= a0_normalization;
    filter->coefficient_a1 *= a0_normalization;
    filter->coefficient_a2 *= a0_normalization;
}

// To not repeat calculations just copy the filter coefficients from another filter
void low_pass_filter_biquad_set_cutoff_frequency_using_reference_filter(struct low_pass_biquad_filter* filter, struct low_pass_biquad_filter* filter_reference){
    filter->coefficient_b0 = filter_reference->coefficient_b0;
    filter->coefficient_b1 = filter_reference->coefficient_b1;
    filter->coefficient_b2 = filter_reference->coefficient_b2;
    filter->coefficient_a1 = filter_reference->coefficient_a1;
    filter->coefficient_a2 = filter_reference->coefficient_a2;
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


// Notch filter V!, not working and very slow
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

// To not do the q constant calculation again use a reference that already has been calculated
void notch_filter_set_center_frequency_Q_constant_using_reference_filter(struct notch_filter* filter, struct notch_filter* filter_reference){
    filter->center_frequency_radians = filter_reference->center_frequency_radians;
    filter->alpha = filter_reference->alpha;
    filter->q = filter_reference->q;
    filter->notch_width_hz_radians = filter_reference->notch_width_hz_radians;
    filter->beta = filter_reference->beta;
}





// Notch filter V2 doesn't use Q
struct notch_filter2 notch_filter2_init(float center_frequency, float notch_width_hz, float refresh_rate){

    float omega = 2 * M_PI * center_frequency / refresh_rate;
    float bandwidth = 2 * M_PI * notch_width_hz / refresh_rate;
    float r = 1 - bandwidth * 0.5;

    struct notch_filter2 new_filter;

    new_filter.a1 = -2 * r * cosf(omega);
    new_filter.a2 = r * r;
    new_filter.b0 = 1;
    new_filter.b1 = -2 * cosf(omega);
    new_filter.b1 = 1;

    new_filter.input_array[0] = 0;
    new_filter.input_array[1] = 0;

    new_filter.output_array[0] = 0;
    new_filter.output_array[1] = 0;

    new_filter.precalculated_refresh_rate_division = 1.0f / refresh_rate;

    return new_filter;
}

float notch_filter2_read(struct notch_filter2* filter, float input){
    float output_value = filter->b0 * input + 
        filter->b1 * filter->input_array[0] + 
        filter->b2 * filter->input_array[1] - 
        filter->a1 * filter->output_array[0] -
        filter->a2 * filter->output_array[1];

    filter->input_array[1] = filter->input_array[0];
    filter->input_array[0] = input;

    filter->output_array[1] = filter->output_array[0];
    filter->output_array[0] = output_value;

    return output_value;
}

void notch_filter2_set_center_frequency(struct notch_filter2* filter, float new_center_frequency){
    float omega = M_PI_2 * new_center_frequency * filter->precalculated_refresh_rate_division;
    float bandwidth = M_PI_2 * new_center_frequency * filter->precalculated_refresh_rate_division;
    float r = 1 - bandwidth * 0.5;

    filter->a1 = -2 * r * cosf(omega);
    filter->a2 = r * r;
    filter->b0 = 1;
    filter->b1 = -2 * cosf(omega);
    filter->b2 = 1;
}


// Notch filter V3 that uses Q
struct notch_filter_q notch_filter_q_init(float center_frequency, float q, float refresh_rate){
    float new_q = q / 100.0f;

    float omega = M_PI_2 * center_frequency / refresh_rate;
    float cos_omega = cosf(omega);
    float sin_omega = sinf(omega);
    float alpha = sin_omega / (2 * new_q);

    struct notch_filter_q new_filter;

    float b0 = 1.0f;
    float b1 = -2.0f * cos_omega;
    float b2 = 1.0f;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cos_omega;
    float a2 = 1.0f - alpha;

    float normalize_a0 = 1.0f / a0;

    new_filter.b0 = b0 * normalize_a0;
    new_filter.b1 = b1 * normalize_a0;
    new_filter.b2 = b2 * normalize_a0;
    new_filter.a1 = a1 * normalize_a0;
    new_filter.a2 = a2 * normalize_a0;

    new_filter.input_array[0] = 0;
    new_filter.input_array[1] = 0;

    new_filter.output_array[0] = 0;
    new_filter.output_array[1] = 0;

    new_filter.precalculated_refresh_rate_division = 1.0f / refresh_rate;
    new_filter.q = new_q;
    new_filter.precalculated_q_division = 1.0f / (2.0f * new_q);

    return new_filter;
}

void notch_filter_q_set_center_frequency(struct notch_filter_q* filter, float new_center_frequency){

    float omega = M_PI_2 * new_center_frequency * filter->precalculated_refresh_rate_division;
    float cos_omega = cosf(omega);
    float sin_omega = sinf(omega);
    float alpha = sin_omega * filter->precalculated_q_division;

    float b0 = 1.0f;
    float b1 = -2.0f * cos_omega;
    float b2 = 1.0f;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cos_omega;
    float a2 = 1.0f - alpha;

    float normalize_a0 = 1.0f / a0;
    filter->b0 = b0 * normalize_a0;
    filter->b1 = b1 * normalize_a0;
    filter->b2 = b2 * normalize_a0;
    filter->a1 = a1 * normalize_a0;
    filter->a2 = a2 * normalize_a0;
}

void notch_filter_q_set_center_frequency_using_reference_filter(struct notch_filter_q* filter, struct notch_filter_q* filter_reference){
    filter->b0 = filter_reference->b0;
    filter->b1 = filter_reference->b1;
    filter->b2 = filter_reference->b2;
    filter->a1 = filter_reference->a1;
    filter->a2 = filter_reference->a2;
}

float notch_filter_q_read(struct notch_filter_q* filter, float input){
    float output_value = filter->b0 * input + 
        filter->b1 * filter->input_array[0] + 
        filter->b2 * filter->input_array[1] - 
        filter->a1 * filter->output_array[0] -
        filter->a2 * filter->output_array[1];

    filter->input_array[1] = filter->input_array[0];
    filter->input_array[0] = input;

    filter->output_array[1] = filter->output_array[0];
    filter->output_array[0] = output_value;

    return output_value;
}