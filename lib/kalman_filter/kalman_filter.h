#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

struct kalman_filter{
    float ** S_state;
    uint8_t S_rows;
    uint8_t S_cols;
    float ** P_covariance;
    uint8_t P_rows;
    uint8_t P_cols;
    float ** F_state_transition_model;
    uint8_t F_rows;
    uint8_t F_cols;
    float ** Q_proccess_noise_covariance;
    uint8_t Q_rows;
    uint8_t Q_cols;
    float ** H_observation_model;
    uint8_t H_rows;
    uint8_t H_cols;
    float ** R_observation_noise_covariance;
    uint8_t R_rows;
    uint8_t R_cols;
    float ** B_external_input_control_matrix;
    uint8_t B_rows;
    uint8_t B_cols;
    float ** u_measurements;
    uint8_t u_measurements_size;
    float ** z_measurements;
    uint8_t z_measurements_size;

    // Temporary states for reducing load on MCU
    float ** S_state_temp;
    float ** P_covariance_temp;
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

    uint8_t got_S_state;
    uint8_t got_P_covariance;
    uint8_t got_F_state_transition_model;
    uint8_t got_Q_proccess_noise_covariance;
    uint8_t got_H_observation_model;
    uint8_t got_R_observation_noise_covariance;
};

struct kalman_filter kalman_filter_init();
uint8_t kalman_filter_set_S_state(struct kalman_filter* filter, float * state_vector, uint8_t number_of_states);
uint8_t kalman_filter_set_P_covariance(struct kalman_filter* filter, float * covariance_matrix, uint8_t row, uint8_t col);
uint8_t kalman_filter_set_F_state_transition_model(struct kalman_filter* filter, float * state_transition_model_matrix, uint8_t row, uint8_t col);
uint8_t kalman_filter_set_Q_proccess_noise_covariance(struct kalman_filter* filter, float * proccess_noise_covariance, uint8_t row, uint8_t col);
uint8_t kalman_filter_set_H_observation_model(struct kalman_filter* filter, float * observation_model, uint8_t row, uint8_t col);
uint8_t kalman_filter_set_R_observation_noise_covariance(struct kalman_filter* filter, float * observation_noise_covariance, uint8_t row, uint8_t col);
uint8_t kalman_filter_set_B_external_input_control_matrix(struct kalman_filter* filter, float * external_input_control_matrix, uint8_t row, uint8_t col);
uint8_t kalman_filter_configure_u_measurement_input(struct kalman_filter* filter, uint8_t input_count);
uint8_t kalman_filter_configure_z_measurement_input(struct kalman_filter* filter, uint8_t input_count);
uint8_t kalman_filter_finish_initialization(struct kalman_filter* filter);
void kalman_filter_predict(struct kalman_filter* filter, float *Uk_external_influence);
void kalman_filter_update(struct kalman_filter* filter, float * Zk_measurement_vector);
float ** kalman_filter_get_state(struct kalman_filter* filter);

// void kalman_filter_predict();
// void kalman_filter_update(float * Zk_measurement_vector);