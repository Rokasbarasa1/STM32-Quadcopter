#pragma once
#include "stdint.h"

struct pid{
    float m_gain_proportional;
    float m_gain_integral;
    float m_gain_derivative;
    float m_integral_sum;
    float m_last_error;
    float m_desired_value;
    uint64_t m_previous_time;
    float m_max_value;
    float m_min_value;
    uint8_t m_stop_windup;
    float m_last_proportional_error;
    float m_last_integral_error;
    float m_last_derivative_error;
};

struct pid pid_init(
    float gain_proportional, 
    float gain_integral, 
    float gain_derivative, 
    float desired_value,
    uint64_t time,
    float max_value,
    float min_value,
    uint8_t stop_windup
);
float pid_get_error(struct pid* pid_instance, float value, uint64_t time);
float pid_get_error_own_error(struct pid* pid_instance, float error, uint64_t time);
void pid_set_desired_value(struct pid* pid_instance, float value);
void pid_set_proportional_gain(struct pid* pid_instance, float proportional_gain);
void pid_set_integral_gain(struct pid* pid_instance, float integral_gain);
void pid_set_derivative_gain(struct pid* pid_instance, float derivative_gain);
void pid_reset_integral_sum(struct pid* pid_instance);
void pid_set_previous_time(struct pid* pid_instance, uint64_t time);
float pid_get_last_proportional_error(struct pid* pid_instance);
float pid_get_last_integral_error(struct pid* pid_instance);
float pid_get_last_derivative_error(struct pid* pid_instance);