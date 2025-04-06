#pragma once
#include "stdint.h"

struct pid_double{
    double m_gain_proportional;
    double m_gain_integral;
    double m_gain_derivative;
    double m_integral_sum;
    double m_last_error_for_d_term;
    double m_desired_value;
    uint64_t m_previous_time;
    double m_max_value;
    double m_min_value;
    uint8_t m_stop_windup_integral;
    uint8_t m_stop_windup_derivative;
    double m_last_proportional_error;
    double m_last_integral_error;
    double m_last_derivative_error;
};

struct pid_double pid_double_init(
    double gain_proportional, 
    double gain_integral, 
    double gain_derivative, 
    double desired_value,
    uint64_t time,
    double max_value,
    double min_value,
    uint8_t stop_windup_integral,
    uint8_t stop_windup_derivative
);
double pid_double_get_error(struct pid_double* pid_instance, double value, uint64_t time);
double pid_double_get_error_own_error(struct pid_double* pid_instance, double error, uint64_t time);
void pid_double_calculate_error(struct pid_double* pid_instance, double value, uint64_t time);
void pid_double_set_desired_value(struct pid_double* pid_instance, double value);
void pid_double_set_proportional_gain(struct pid_double* pid_instance, double proportional_gain);
void pid_double_set_integral_gain(struct pid_double* pid_instance, double integral_gain);
void pid_double_set_derivative_gain(struct pid_double* pid_instance, double derivative_gain);
void pid_double_reset_integral_sum(struct pid_double* pid_instance);
void pid_double_set_previous_time(struct pid_double* pid_instance, uint64_t time);
double pid_double_get_last_proportional_error(struct pid_double* pid_instance);
double pid_double_get_last_integral_error(struct pid_double* pid_instance);
double pid_double_get_last_derivative_error(struct pid_double* pid_instance);

void pid_double_calculate_error_pi(struct pid_double* pid_instance, double value, uint64_t time);
void pid_double_calculate_error_d(struct pid_double* pid_instance, double value, uint64_t time);