#include "./pid_double.h"
#include "../utils/math_constants.h"
#include "stdio.h"
/**
 * @brief Initialize pid configuration and store it in struct
 * 
 * @param gain_proportional 
 * @param gain_integral 
 * @param gain_derivative 
 * @param desired_value value that you want to achieve
 * @param time time since system start in microseconds
 * @return struct pid 
 */
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
){
    struct pid_double new_pid;

    new_pid.m_gain_proportional = gain_proportional;
    new_pid.m_gain_integral = gain_integral;
    new_pid.m_gain_derivative = gain_derivative;
    new_pid.m_integral_sum = 0;
    new_pid.m_last_error_for_d_term = 0;
    new_pid.m_desired_value = desired_value;
    new_pid.m_previous_time = time;
    new_pid.m_max_value = max_value;
    new_pid.m_min_value = min_value;
    new_pid.m_stop_windup_integral = stop_windup_integral;
    new_pid.m_stop_windup_derivative = stop_windup_derivative;
    new_pid.m_last_proportional_error = 0;
    new_pid.m_last_integral_error = 0;
    new_pid.m_last_derivative_error = 0;

    return new_pid;
}


/**
 * @brief Calculate the error based on the configuration of the pid and the new value
 * 
 * @param pid_instance pid config
 * @param value current value that the error will be calculated for 
 * @param time time since system start in microseconds
 * @return double error result
 */
double pid_double_get_error(struct pid_double* pid_instance, double value, uint64_t time){

    double error_p = 0, error_i = 0, error_d = 0;

    double error = (pid_instance->m_desired_value - value);

    double elapsed_time_sec = (double)(time-pid_instance->m_previous_time)*CONVERT_MICROSECONDS_TO_SECONDS;

    // proportional
    {
        error_p = error;
    }

    // integral
    {
        pid_instance->m_integral_sum += (error * elapsed_time_sec);

        if(pid_instance->m_stop_windup_integral == 1){
            // clamp the integral if it is getting out of bounds
            if(pid_instance->m_integral_sum * pid_instance->m_gain_integral > pid_instance->m_max_value){
                pid_instance->m_integral_sum = pid_instance->m_max_value / pid_instance->m_gain_integral;
            }else if(pid_instance->m_integral_sum * pid_instance->m_gain_integral < pid_instance->m_min_value){
                pid_instance->m_integral_sum = pid_instance->m_min_value / pid_instance->m_gain_integral;
            }
        }
        error_i = pid_instance->m_integral_sum;
    }

    // derivative
    {
        // divide by the time passed, the smaller the time gap the larger the rate of change is, remember?
        error_d = (error_p - pid_instance->m_last_error_for_d_term) / elapsed_time_sec;

        if(pid_instance->m_stop_windup_derivative == 1){
            // Dont let it get out of bounds 
            if(error_d * pid_instance->m_gain_derivative > pid_instance->m_max_value){
                error_d = pid_instance->m_max_value;
            }else if(error_d * pid_instance->m_gain_derivative < pid_instance->m_min_value){
                error_d = pid_instance->m_min_value;
            }
        }
        
        // set the previous error for the next iteration
        pid_instance->m_last_error_for_d_term = error;
    }


    pid_instance->m_last_proportional_error = pid_instance->m_gain_proportional * error_p;
    pid_instance->m_last_integral_error = pid_instance->m_gain_integral * error_i;
    pid_instance->m_last_derivative_error = pid_instance->m_gain_derivative * error_d;

    // end result
    double total_error = pid_instance->m_last_proportional_error +
                pid_instance->m_last_integral_error +
                pid_instance->m_last_derivative_error;

    // save the time for next calculation
    pid_instance->m_previous_time = time;
    return total_error;
}


/**
 * @brief Calculate the error based on the configuration of the pid and the new value
 * 
 * @param pid_instance pid config
 * @param error your own calculated error that will be used to get pid error
 * @param time time since system start in microseconds
 * @return double error result
 */
double pid_double_get_error_own_error(struct pid_double* pid_instance, double error, uint64_t time){

    double error_p = 0, error_i = 0, error_d = 0;

    double elapsed_time_sec = (double)(time-pid_instance->m_previous_time)*CONVERT_MICROSECONDS_TO_SECONDS;

    // proportional
    {
        error_p = error;
    }

    // integral
    {
        pid_instance->m_integral_sum += (error * elapsed_time_sec);

        if(pid_instance->m_stop_windup_integral == 1){
            // clamp the integral if it is getting out of bounds
            if((pid_instance->m_integral_sum * pid_instance->m_gain_integral) > pid_instance->m_max_value){
                pid_instance->m_integral_sum = pid_instance->m_max_value / pid_instance->m_gain_integral;
            }else if(pid_instance->m_integral_sum * pid_instance->m_gain_integral < pid_instance->m_min_value){
                pid_instance->m_integral_sum = pid_instance->m_min_value / pid_instance->m_gain_integral;
            }
        }
        error_i = pid_instance->m_integral_sum;
    }

    // derivative
    {
        // divide by the time passed
        error_d = (error_p - pid_instance->m_last_error_for_d_term) / elapsed_time_sec;

        // Dont let it get out of bounds 
        if(error_d  * pid_instance->m_gain_derivative> pid_instance->m_max_value){
            error_d = pid_instance->m_max_value;
        }else if(error_d  * pid_instance->m_gain_derivative < pid_instance->m_min_value){
            error_d = pid_instance->m_min_value;
        }
        
        // set the previous error for the next iteration
        pid_instance->m_last_error_for_d_term = error;
    }

    pid_instance->m_last_proportional_error = pid_instance->m_gain_proportional * error_p;
    pid_instance->m_last_integral_error = pid_instance->m_gain_integral * error_i;
    pid_instance->m_last_derivative_error = pid_instance->m_gain_derivative * error_d;

    // end result
    double total_error = pid_instance->m_last_proportional_error +
                pid_instance->m_last_integral_error +
                pid_instance->m_last_derivative_error;

    // save the time for next calculation
    pid_instance->m_previous_time = time;
    return total_error;
}

/**
 * @brief Calculate the error based on the configuration of the pid and the new value. Does not return the sum of the pid
 * 
 * @param pid_instance pid config
 * @param value current value that the error will be calculated for 
 * @param time time since system start in microseconds
 */
void pid_double_calculate_error(struct pid_double* pid_instance, double value, uint64_t time){

    double error_p = 0, error_i = 0, error_d = 0;

    double error = (pid_instance->m_desired_value - value);

    double elapsed_time_sec = (double)(time-pid_instance->m_previous_time)*CONVERT_MICROSECONDS_TO_SECONDS;

    // proportional
    {
        error_p = error;
    }

    // integral
    {
        pid_instance->m_integral_sum += (error * elapsed_time_sec);

        if(pid_instance->m_stop_windup_integral == 1){
            // clamp the integral if it is getting out of bounds
            if((pid_instance->m_integral_sum * pid_instance->m_gain_integral) > pid_instance->m_max_value){
                pid_instance->m_integral_sum = pid_instance->m_max_value / pid_instance->m_gain_integral;
            }else if(pid_instance->m_integral_sum * pid_instance->m_gain_integral < pid_instance->m_min_value){
                pid_instance->m_integral_sum = pid_instance->m_min_value / pid_instance->m_gain_integral;
            }
        }
        error_i = pid_instance->m_integral_sum;
    }

    // derivative
    {
        // divide by the time passed, the smaller the time gap the larger the rate of change is, remember?
        error_d = (error_p - pid_instance->m_last_error_for_d_term) / elapsed_time_sec;

        if(pid_instance->m_stop_windup_derivative == 1){
            // Dont let it get out of bounds 
            if(error_d * pid_instance->m_gain_derivative > pid_instance->m_max_value){
                error_d = pid_instance->m_max_value;
            }else if(error_d * pid_instance->m_gain_derivative < pid_instance->m_min_value){
                error_d = pid_instance->m_min_value;
            }
        }        
        // set the previous error for the next iteration
        pid_instance->m_last_error_for_d_term = error;
    }


    pid_instance->m_last_proportional_error = pid_instance->m_gain_proportional * error_p;
    pid_instance->m_last_integral_error = pid_instance->m_gain_integral * error_i;
    pid_instance->m_last_derivative_error = pid_instance->m_gain_derivative * error_d;
}

/**
 * @brief Calculate the error based on the configuration of the pid and the new value. Does not return the sum of the pid
 * 
 * @param pid_instance pid config
 * @param value current value that the error will be calculated for 
 * @param time time since system start in microseconds
 */
void pid_double_calculate_error_pi(struct pid_double* pid_instance, double value, uint64_t time){

    double error_p = 0, error_i = 0;

    double error = (pid_instance->m_desired_value - value);

    double elapsed_time_sec = (double)(time-pid_instance->m_previous_time)*CONVERT_MICROSECONDS_TO_SECONDS;

    // proportional
    {
        error_p = error;
    }

    // integral
    {
        pid_instance->m_integral_sum += (error * elapsed_time_sec);

        if(pid_instance->m_stop_windup_integral == 1){
            // clamp the integral if it is getting out of bounds
            if((pid_instance->m_integral_sum * pid_instance->m_gain_integral) > pid_instance->m_max_value){
                pid_instance->m_integral_sum = pid_instance->m_max_value / pid_instance->m_gain_integral;
            }else if(pid_instance->m_integral_sum * pid_instance->m_gain_integral < pid_instance->m_min_value){
                pid_instance->m_integral_sum = pid_instance->m_min_value / pid_instance->m_gain_integral;
            }
        }
        error_i = pid_instance->m_integral_sum;
    }

    pid_instance->m_last_proportional_error = pid_instance->m_gain_proportional * error_p;
    pid_instance->m_last_integral_error = pid_instance->m_gain_integral * error_i;
}

/**
 * @brief Calculate the error based on the configuration of the pid and the new value. Does not return the sum of the pid
 * 
 * @param pid_instance pid config
 * @param value current value that the error will be calculated for 
 * @param time time since system start in microseconds
 */
void pid_double_calculate_error_d(struct pid_double* pid_instance, double value, uint64_t time){

    double error_d = 0;

    double error = (pid_instance->m_desired_value - value);

    double elapsed_time_sec = (double)(time-pid_instance->m_previous_time)*CONVERT_MICROSECONDS_TO_SECONDS;

    // derivative
    {
        // divide by the time passed, the smaller the time gap the larger the rate of change is, remember?
        error_d = (error - pid_instance->m_last_error_for_d_term) / elapsed_time_sec;

        if(pid_instance->m_stop_windup_derivative == 1){
            // Dont let it get out of bounds 
            if(error_d * pid_instance->m_gain_derivative > pid_instance->m_max_value){
                error_d = pid_instance->m_max_value;
            }else if(error_d * pid_instance->m_gain_derivative < pid_instance->m_min_value){
                error_d = pid_instance->m_min_value;
            }
        }
        
        // set the previous error for the next iteration
        pid_instance->m_last_error_for_d_term = error;
    }

    pid_instance->m_last_derivative_error = pid_instance->m_gain_derivative * error_d;
}

/**
 * @brief Change the desired value. Mainly for user inputs for pid
 * 
 * @param pid_instance pid config
 * @param value new desired value
 */
void pid_double_set_desired_value(struct pid_double* pid_instance, double value){
    pid_instance->m_desired_value = value;
}

void pid_double_set_proportional_gain(struct pid_double* pid_instance, double proportional_gain){
    pid_instance->m_gain_proportional = proportional_gain;
}

void pid_double_set_integral_gain(struct pid_double* pid_instance, double integral_gain){
    pid_instance->m_gain_integral = integral_gain;
}

void pid_double_set_derivative_gain(struct pid_double* pid_instance, double derivative_gain){
    pid_instance->m_gain_derivative = derivative_gain;
}

void pid_double_reset_integral_sum(struct pid_double* pid_instance){
    pid_instance->m_integral_sum = 0;
}

void pid_double_set_previous_time(struct pid_double* pid_instance, uint64_t time){
    pid_instance->m_previous_time = time;
}

double pid_double_get_last_proportional_error(struct pid_double* pid_instance){
    return pid_instance->m_last_proportional_error;
}

double pid_double_get_last_integral_error(struct pid_double* pid_instance){
    return pid_instance->m_last_integral_error;
}

double pid_double_get_last_derivative_error(struct pid_double* pid_instance){
    return pid_instance->m_last_derivative_error;
}