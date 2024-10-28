#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

// NOT WORKING GOOD

typedef void (*predict_jacobian)(float, float **, float **, float **);
typedef void (*update_jacobian)(float **, float **);

typedef void (*prediction_measurement_function)(float **, float **, float **);
typedef void (*update_measurement_function)(float **, float **);

struct extended_kalman_filter{
    float ** S_state;
    float ** P_covariance;
    float ** F_prediction_jacobian;
    float ** Q_proccess_noise_covariance;
    float ** H_update_jacobian;
    float ** R_observation_noise_covariance;
    float ** B_external_input_control_matrix;
    float ** u_external_input;
    float ** z_measurements;
    float ** u_measurements;
    uint8_t state_size;

    // Temporary states for reducing load on MCU
    float ** S_state_temp;
    float ** P_covariance_temp;
    float ** P_covariance_temp2;
    float ** Fk_transposed;
    float ** Yk;
    float ** residual_temp;
    float ** Sk;
    float ** Sk_temp;
    float ** Hk_transposed;
    float ** Kk;
    float ** Kk_temp;
    float ** Sk_inverse;
    float ** Xk_temp;
    float ** identity_matrix;
    float ** Pk_temp;
    float ** Pk_temp2;
    // Timing stuff

    predict_jacobian predict_jacobian;
    update_jacobian update_jacobian;
    prediction_measurement_function f_predict_measurement_function;
    float ** f_state_transition_matrix;
    update_measurement_function h_measurement_function;
    float ** h_measurement_matrix;


    // Init states
    uint8_t got_S_state;
    uint8_t got_P_covariance;
    uint8_t got_F_state_transition_model;
    uint8_t got_Q_proccess_noise_covariance;
    uint8_t got_H_observation_model;
    uint8_t got_R_observation_noise_covariance;
};

struct extended_kalman_filter extended_kalman_filter_init(prediction_measurement_function m_f_state_transition_function, update_measurement_function m_h_measurement_function, predict_jacobian m_predict_jacobian, update_jacobian m_update_jacobian);
uint8_t extended_kalman_filter_set_S_state(struct extended_kalman_filter* filter, float * state_vector, uint8_t number_of_states);
uint8_t extended_kalman_filter_set_P_covariance(struct extended_kalman_filter* filter, float * covariance_matrix, uint8_t row, uint8_t col);
uint8_t extended_kalman_filter_set_F_state_transition_model(struct extended_kalman_filter* filter, float * state_transition_model_matrix, uint8_t row, uint8_t col);
uint8_t extended_kalman_filter_set_Q_proccess_noise_covariance(struct extended_kalman_filter* filter, float * proccess_noise_covariance, uint8_t row, uint8_t col);
uint8_t extended_kalman_filter_set_H_observation_model(struct extended_kalman_filter* filter, float * observation_model, uint8_t row, uint8_t col);
uint8_t extended_kalman_filter_set_R_observation_noise_covariance(struct extended_kalman_filter* filter, float * observation_noise_covariance, uint8_t row, uint8_t col);
uint8_t extended_kalman_filter_set_B_external_input_control_matrix(struct extended_kalman_filter* filter, float * external_input_control_matrix, uint8_t row, uint8_t col);
uint8_t extended_kalman_filter_set_u_external_input(struct extended_kalman_filter* filter, float * external_input, uint8_t input_count);
uint8_t extended_kalman_filter_finish_initialization(struct extended_kalman_filter* filter);
void extended_kalman_filter_predict(struct extended_kalman_filter* filter, float * Uk_measurement_vector, float delta_time);
void extended_kalman_filter_update(struct extended_kalman_filter* filter, float * Zk_measurement_vector);
float ** extended_kalman_filter_get_state(struct extended_kalman_filter* filter);