#include "./kalman_filter.h"
#include "../utils/matrix_operations/matrix_operations.h"

// Based on: https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/

#define KALMAN_FILTER_DEBUG 0

struct kalman_filter kalman_filter_init(){
    struct kalman_filter new_filter;
    new_filter.state_size = 0;
    return new_filter;
}

uint8_t kalman_filter_set_S_state(struct kalman_filter* filter, float *state_vector, uint8_t number_of_states){

    filter->S_state = allocate_matrix(number_of_states, 1);
    matrix_copy_flat_to_allocated(state_vector, number_of_states, 1, filter->S_state); // Special case

    filter->state_size = number_of_states;
    filter->got_S_state = 1;

    return 1;
}

uint8_t kalman_filter_set_P_covariance(struct kalman_filter* filter, float * covariance_matrix, uint8_t row, uint8_t col){
    if(filter->state_size == 0 || col != filter->state_size || row != filter->state_size){
#if(KALMAN_FILTER_DEBUG)
        printf("kalman_filter_set_P_covariance failed\n");
#endif
        return 0;
    }

    filter->P_covariance = allocate_matrix(filter->state_size, filter->state_size);
    matrix_copy_flat_to_allocated(covariance_matrix, filter->state_size, filter->state_size, filter->P_covariance);

    filter->got_P_covariance = 1;

    return 1;
}

uint8_t kalman_filter_set_F_state_transition_model(struct kalman_filter* filter, float * state_transition_model_matrix, uint8_t row, uint8_t col){
    if(filter->state_size == 0 || col != filter->state_size || row != filter->state_size){
#if(KALMAN_FILTER_DEBUG)
        printf("kalman_filter_set_F_state_transition_model failed\n");
#endif
        return 0;
    }

    
    filter->F_state_transition_model = allocate_matrix(filter->state_size, filter->state_size);
    matrix_copy_flat_to_allocated(state_transition_model_matrix, filter->state_size, filter->state_size, filter->F_state_transition_model);

    filter->got_F_state_transition_model = 1;
    
    return 1;
}

uint8_t kalman_filter_set_Q_proccess_noise_covariance(struct kalman_filter* filter, float * proccess_noise_covariance, uint8_t row, uint8_t col){
    if(filter->state_size == 0 || col != filter->state_size || row != filter->state_size){
#if(KALMAN_FILTER_DEBUG)
        printf("kalman_filter_set_Q_proccess_noise_covariance failed\n");
#endif
        return 0;
    }

    filter->Q_proccess_noise_covariance = allocate_matrix(filter->state_size, filter->state_size);
    matrix_copy_flat_to_allocated(proccess_noise_covariance, filter->state_size, filter->state_size, filter->Q_proccess_noise_covariance);

    filter->got_Q_proccess_noise_covariance = 1;
    
    return 1;
}

uint8_t kalman_filter_set_H_observation_model(struct kalman_filter* filter, float * observation_model, uint8_t row, uint8_t col){
    if(filter->state_size == 0 || col != filter->state_size || row != filter->state_size){
#if(KALMAN_FILTER_DEBUG)
        printf("kalman_filter_set_H_observation_model failed\n");
#endif
        return 0;
    }

    filter->H_observation_model = allocate_matrix(filter->state_size, filter->state_size);
    matrix_copy_flat_to_allocated(observation_model, filter->state_size, filter->state_size, filter->H_observation_model);

    print_matrix(filter->H_observation_model, 2, 2, "H_observation_model", 4);
    filter->got_H_observation_model = 1;
    
    return 1;
}

uint8_t kalman_filter_set_R_observation_noise_covariance(struct kalman_filter* filter, float * observation_noise_covariance, uint8_t row, uint8_t col){
    if(filter->state_size == 0 || col != filter->state_size || row != filter->state_size){
#if(KALMAN_FILTER_DEBUG)
        printf("kalman_filter_set_R_observation_noise_covariance failed\n");
#endif
        return 0;
    }

    filter->R_observation_noise_covariance = allocate_matrix(filter->state_size, filter->state_size);
    matrix_copy_flat_to_allocated(observation_noise_covariance, filter->state_size, filter->state_size, filter->R_observation_noise_covariance);

    filter->got_R_observation_noise_covariance = 1;
    
    return 1;
}

uint8_t kalman_filter_set_B_external_input_control_matrix(struct kalman_filter* filter, float * external_input_control_matrix, uint8_t row, uint8_t col){
    if(filter->state_size == 0 || col != filter->state_size || row != filter->state_size){
#if(KALMAN_FILTER_DEBUG)
        printf("kalman_filter_set_B_external_input_control_matrix failed\n");
#endif
        return 0;
    }

    filter->B_external_input_control_matrix = allocate_matrix(filter->state_size, filter->state_size);
    matrix_copy_flat_to_allocated(external_input_control_matrix, filter->state_size, filter->state_size, filter->B_external_input_control_matrix);

    return 1;
}

uint8_t kalman_filter_set_u_external_input(struct kalman_filter* filter, float * external_input, uint8_t input_count){
    if(filter->state_size == 0){
        return 0;
    }
    
    filter->u_external_input = allocate_matrix(filter->state_size, 1);
    matrix_copy_flat_to_allocated(external_input, filter->state_size, 1, filter->u_external_input);

    return 1;
}



uint8_t kalman_filter_finish_initialization(struct kalman_filter* filter){
    if(filter->got_S_state == 0 || filter->got_P_covariance != 1 || filter->got_F_state_transition_model != 1 || filter->got_Q_proccess_noise_covariance != 1 || filter->got_H_observation_model != 1 || filter->got_R_observation_noise_covariance != 1){
        return 0;
    }

    uint8_t size = filter->state_size;
    filter->S_state_temp = allocate_matrix(size, 1); // Col = 1 because it is a vector
    filter->P_covariance_temp = allocate_matrix(size, size);
    filter->Fk_transposed = allocate_matrix(size, size);
    filter->Yk = allocate_matrix(size, 1);
    filter->residual_temp = allocate_matrix(size, 1);
    filter->Sk = allocate_matrix(size, size);
    filter->Sk_temp = allocate_matrix(size, size);
    filter->Hk_transposed = allocate_matrix(size, size);
    filter->Kk = allocate_matrix(size, size);
    filter->Kk_temp = allocate_matrix(size, size);
    filter->Sk_inverse = allocate_matrix(size, size);
    filter->Xk_temp = allocate_matrix(size, size);
    filter->identity_matrix = allocate_matrix(size, size);
    set_identity_matrix(filter->identity_matrix, size);
    // saDASdasd IDENTITY MATRIX
    filter->Pk_temp = allocate_matrix(size, size);
    filter->Pk_temp2 = allocate_matrix(size, size);

    filter->z_measurements = allocate_matrix(filter->state_size, 1); // Col = 1 because it is a vector

    return 1;
}

void kalman_filter_predict(struct kalman_filter* filter){
    // Xk = Fk * Xk−1
    // S_state_predicted = F_state_transition_model * S_state_previous
    
#if(KALMAN_FILTER_DEBUG)
    printf("------------------------------------------------------------------------------------------- PREDICT\n");
    printf("(OK) Xk = Fk * Xk-1\n");
    
    print_matrix(filter->F_state_transition_model, 2, 2, "Fk", 4);
    printf("times\n");
    print_matrix(filter->S_state, 2, 1, "Xk-1", 4);
    printf("equals\n");
#endif

    matrix_matrix_multiply(
        filter->F_state_transition_model, 
        filter->state_size, 
        filter->state_size, 
        filter->S_state, 
        filter->state_size, 
        1, 
        filter->S_state_temp
    );

    matrix_copy(
        filter->S_state_temp, 
        filter->state_size, 
        1, 
        filter->S_state
    );


#if(KALMAN_FILTER_DEBUG)
    print_matrix(filter->S_state, 2, 1, "Xk", 4);
#endif
    // Pk = Fk * Pk-1 * Fk(Transpose) + Qk
    // P_covariance = F_state_transition_model * P_covariance * F_state_transition_model (Transposed) + Q_proccess_noise_covariance

#if(KALMAN_FILTER_DEBUG)
    printf("(OK) Pk = Fk * Pk-1 * Fk(Transpose) + Qk\n");
    
    print_matrix(filter->F_state_transition_model, 2, 2, "Fk", 4);
    printf("times\n");
    print_matrix(filter->P_covariance, 2, 2, "Pk-1", 4);
    printf("times\n");
    print_matrix(filter->F_state_transition_model, 2, 2, "Fk(transpose)", 4);
    printf("plus\n");
    print_matrix(filter->Q_proccess_noise_covariance, 2, 2, "Qk", 4);
    printf("equals\n");
#endif

    matrix_matrix_multiply(
        filter->F_state_transition_model, 
        filter->state_size, 
        filter->state_size, 
        filter->P_covariance, 
        filter->state_size, 
        filter->state_size, 
        filter->P_covariance_temp
    );
    matrix_transpose(
        filter->F_state_transition_model, 
        filter->state_size, 
        filter->state_size, 
        filter->Fk_transposed
    );

    matrix_matrix_multiply(
        filter->P_covariance_temp, 
        filter->state_size, 
        filter->state_size, 
        filter->Fk_transposed, 
        filter->state_size, 
        filter->state_size, 
        filter->P_covariance
    );
    matrix_matrix_add(
        filter->P_covariance, 
        filter->state_size, 
        filter->state_size, 
        filter->Q_proccess_noise_covariance, 
        filter->state_size, 
        filter->state_size, 
        filter->P_covariance
    );
#if(KALMAN_FILTER_DEBUG)
    print_matrix(filter->P_covariance, 2, 2, "Pk", 4);
#endif
}

void kalman_filter_update(struct kalman_filter* filter, float * Zk_measurement_vector){
    // Convert the measurements into the correct format
#if(KALMAN_FILTER_DEBUG)
    printf("------------------------------------------------------------------------------------------- UPDATE \n");
#endif
    for (uint8_t i = 0; i < filter->state_size; i++){
        filter->z_measurements[i][0] = Zk_measurement_vector[i];
    }

    
    // Measurement residual
    // Yk = Zk - Hk * Xk
#if(KALMAN_FILTER_DEBUG)
    printf("------------------------------------------- Yk = Zk - Hk * Xk\n");
    
    print_matrix(filter->z_measurements, 2, 1, "Zk", 4);
    printf("minus\n");
    print_matrix(filter->H_observation_model, 2, 2, "Hk", 4);
    printf("times\n");
    print_matrix(filter->S_state, 2, 1, "Xk", 4);
#endif
    matrix_matrix_multiply(
        filter->H_observation_model, 
        filter->state_size, 
        filter->state_size, 
        filter->S_state, 
        filter->state_size, 
        1, 
        filter->residual_temp
    );
    matrix_matrix_subtract(
        filter->z_measurements, 
        filter->state_size, 
        1, 
        filter->residual_temp, 
        filter->state_size, 
        1, 
        filter->Yk
    );

#if(KALMAN_FILTER_DEBUG)
    printf("equals\n");
    print_matrix(filter->Yk, 2, 1, "Yk", 4);
#endif

    // Residual covariance 
    // Sk = Hk * Pk * Hk(transposed) + Rk
#if(KALMAN_FILTER_DEBUG)
    printf("------------------------------------------- Sk = Hk * Pk * Hk(transposed) + Rk\n");

    print_matrix(filter->H_observation_model, 2, 2, "Hk", 4);
    printf("times\n");
    print_matrix(filter->P_covariance, 2, 2, "Pk", 4);
    printf("times\n");
    print_matrix(filter->Hk_transposed, 2, 2, "Hk(transpose)", 4);
    printf("PLUS\n");
    print_matrix(filter->R_observation_noise_covariance, 2, 2, "Rk", 4);
#endif

    matrix_matrix_multiply(
        filter->H_observation_model, 
        filter->state_size, 
        filter->state_size, 
        filter->P_covariance, 
        filter->state_size, 
        filter->state_size, 
        filter->Sk_temp
    );
    matrix_transpose(
        filter->H_observation_model, 
        filter->state_size, 
        filter->state_size, 
        filter->Hk_transposed
    );
    matrix_matrix_multiply(
        filter->Sk_temp, 
        filter->state_size, 
        filter->state_size, 
        filter->Hk_transposed, 
        filter->state_size, 
        filter->state_size, 
        filter->Sk
    );
    matrix_matrix_add(
        filter->Sk, 
        filter->state_size, 
        filter->state_size, 
        filter->R_observation_noise_covariance, 
        filter->state_size, 
        filter->state_size, 
        filter->Sk
    );

#if(KALMAN_FILTER_DEBUG)
    printf("equals\n");
    print_matrix(filter->Sk, 2, 2, "Sk", 4);
#endif

    // Kalman gain
    // Kk = Pk * Hk (transposed) * Sk (inverse)
#if(KALMAN_FILTER_DEBUG)
    printf("------------------------------------------- Kk = Pk * Hk (transposed) * Sk (inverse)\n");
    print_matrix(filter->P_covariance, 2, 2, "Hk", 4);
    printf("times\n");
    print_matrix(filter->H_observation_model, 2, 2, "Hk(transposed)", 4);
    printf("times\n");
    print_matrix(filter->Sk, 2, 2, "Sk(Inverse)", 4);
#endif

    matrix_matrix_multiply(
        filter->P_covariance, 
        filter->state_size, 
        filter->state_size, 
        filter->Hk_transposed, 
        filter->state_size, 
        filter->state_size, 
        filter->Kk_temp
    );
    matrix_inverse(
        filter->Sk, 
        filter->state_size, 
        filter->Sk_inverse
    );
    matrix_matrix_multiply(
        filter->Kk_temp, 
        filter->state_size, 
        filter->state_size, 
        filter->Sk_inverse, 
        filter->state_size, 
        filter->state_size, 
        filter->Kk
    );

#if(KALMAN_FILTER_DEBUG)
    printf("equals\n");
    print_matrix(filter->Kk, 2, 2, "Kk", 4);
#endif
    // State update
    // Xk = Xk + Kk * Yk
#if(KALMAN_FILTER_DEBUG)
    printf("------------------------------------------- Xk = Xk + Kk * Yk\n");

    print_matrix(filter->S_state, 2, 1, "Xk", 4);
    printf("PLUS\n");
    print_matrix(filter->Kk, 2, 2, "Kk", 4);
    printf("times\n");
    print_matrix(filter->Yk, 2, 1, "Yk", 4);
#endif

    matrix_matrix_multiply(
        filter->Kk, 
        filter->state_size, 
        filter->state_size, 
        filter->Yk, 
        filter->state_size, 
        1, 
        filter->Xk_temp
    );
    matrix_matrix_add(
        filter->S_state, 
        filter->state_size, 
        1, 
        filter->Xk_temp, 
        filter->state_size, 
        1, 
        filter->S_state
    );

#if(KALMAN_FILTER_DEBUG)
    printf("equals\n");
    print_matrix(filter->S_state, 2, 1, "Xk", 4);
#endif

    // Covariance update
    // Pk = (I(identity matrix) - Kk * Hk) * Pk
#if(KALMAN_FILTER_DEBUG)
    printf("------------------------------------------- Pk = (I(identity matrix) - Kk * Hk) * Pk\n");

    print_matrix(filter->identity_matrix, 2, 2, "I(identity matrix)", 4);
    printf("minus\n");
    print_matrix(filter->Kk, 2, 2, "Kk", 4);
    printf("times\n");
    print_matrix(filter->H_observation_model, 2, 2, "Hk", 4);
    printf("all of this before added in parentheses times\n");
    print_matrix(filter->P_covariance, 2, 2, "Pk", 4);
#endif

    matrix_matrix_multiply(
        filter->Kk, 
        filter->state_size, 
        filter->state_size, 
        filter->H_observation_model, 
        filter->state_size, 
        filter->state_size, 
        filter->Pk_temp
    );
    matrix_matrix_subtract(
        filter->identity_matrix, 
        filter->state_size, 
        filter->state_size, 
        filter->Pk_temp, 
        filter->state_size, 
        filter->state_size, 
        filter->Pk_temp
    );
    matrix_matrix_multiply(
        filter->Pk_temp, 
        filter->state_size, 
        filter->state_size, 
        filter->P_covariance, 
        filter->state_size, 
        filter->state_size, 
        filter->Pk_temp2
    );
    matrix_copy(
        filter->Pk_temp2, 
        filter->state_size, 
        filter->state_size, 
        filter->P_covariance
    );

#if(KALMAN_FILTER_DEBUG)
    printf("equals\n");
    print_matrix(filter->P_covariance, 2, 2, "Pk", 4);
#endif
    // Done
}

float ** kalman_filter_get_state(struct kalman_filter* filter){
    return filter->S_state;
}





// float S_state[1][STATE_SIZE] = {
//     {0, 0},
// };

// float P_covariance[STATE_SIZE][STATE_SIZE] = {
//     {1, 0},
//     {0, 1}
// }; 

// float F_state_transition_model[STATE_SIZE][STATE_SIZE] = {
//     {1, 0},
//     {0, 1} // All of previous degrees are used in the next state
// };

// float Q_proccess_noise_covariance[STATE_SIZE][STATE_SIZE] = {
//     {4, 0}, // Expected disturbance from environment by +-2 degrees
//     {0, 4}
// };

// float H_observation_model[STATE_SIZE][STATE_SIZE] = {
//     {1/200, 0},
//     {0, 1/200} // 1/200 is a second divided by 200Hz
// };

// float R_observation_noise_covariance[STATE_SIZE][STATE_SIZE] = {
//     {0.005, 0},
//     {0, 0.005} // based on the mpu6050 gyro datasheet
// };

// float Q_environment_uncertainty[STATE_SIZE][STATE_SIZE] = {
//     {0, 0},
//     {0, 0}
// };

// void kalman_filter_predict(){
//     // Predict the current state. Calculate Xk = Fk * Xk−1. With known extenal influences
    
//     // Optionally add + Bk * uk
    
//     // Xk = Fk * Xk−1
//     // S_state_predicted = F_state_transition_model * S_state_previous
//     float temp_S_state[2][1];
//     matrix_matrix_multiply(F_state_transition_model, 2, 2, S_state, 1, 2, temp_S_state);
//     matrix_copy(temp_S_state, 1, 2, S_state);
//     // matrix_matrix_add(F_state_transition_model, 4, 4, S_state_previous, 4, 4, S_state_predicted);
    
//     // Calculate uncertainty. Calculate Pk = Fk * Pk-1 * Fk(Transpose) + Qk
//     // P_covariance = F_state_transition_model * P_covariance * F_state_transition_model (Transposed) + Q_proccess_noise_covariance
//     float P_covariance_temp[2][2];
//     float Fk_transposed[2][2];

//     matrix_matrix_multiply(F_state_transition_model, 2, 2, P_covariance, 2, 2, P_covariance_temp);
//     matrix_transpose(F_state_transition_model, 2, 2, Fk_transposed);
//     matrix_matrix_multiply(P_covariance_temp, 2, 2, Fk_transposed, 2, 2, P_covariance);
//     matrix_matrix_add(P_covariance, 2, 2, Q_proccess_noise_covariance, 2, 2, P_covariance);

// }

// void kalman_filter_update(float * Zk_measurement_vector){ // Vector has to be the size of the state

//     // Get Zk (the measurements that come in)
//     // Convert the measurements into the correct format
//     float Zk[1][2] = {{Zk_measurement_vector[0], Zk_measurement_vector[1]}};

//     // Measurement residual
//     // Yk = Zk - Hk * Xk
//     float Yk[1][2];
//     float residual_temp[2][1];
//     matrix_matrix_multiply(H_observation_model, 2, 2, S_state, 1, 2, residual_temp);
//     matrix_matrix_subtract(Zk, 2, 1, residual_temp, 2, 2, Yk);

//     // Residual covariance 
//     // Sk = Hk * Pk * Hk(transposed) + Rk
//     float Sk[2][2];
//     float Sk_temp[2][2];
//     float Hk_transposed[2][2];
//     matrix_matrix_multiply(H_observation_model, 2, 2, P_covariance, 2, 2, Sk_temp);
//     matrix_transpose(H_observation_model, 2, 2, Hk_transposed);
//     matrix_matrix_multiply(Sk_temp, 2, 2, Hk_transposed, 2, 2, Sk);
//     matrix_matrix_add(Sk, 2, 2, R_observation_noise_covariance, 2, 2, Sk);

//     // Kalman gain
//     // Kk = Pk * Hk (transposed) * Sk (inverse)
//     float Kk[2][2];
//     float Kk_temp[2][2];
//     float Sk_inverse[2][2];

//     matrix_matrix_multiply(P_covariance, 2, 2, Hk_transposed, 2, 2, Kk_temp);
//     matrix_inverse(Sk, 2, Sk_inverse);
//     matrix_matrix_multiply(Kk_temp, 2, 2, Sk_inverse, 2, 2, Kk);

//     // State update
//     // Xk (S_state)= Xk + Kk * Yk
//     float Xk_temp[1][2];
//     matrix_matrix_multiply(Kk, 2, 2, Yk, 1, 2, Xk_temp);
//     matrix_matrix_add(S_state,1, 2, Xk_temp, 1, 2, S_state);


//     // Covariance update
//     // Pk = (I(identity matrix) - Kk * Hk) * Pk
//     float identity_matrix[2][2]={
//         {1,0},
//         {0,1}
//     };

//     float Pk_temp[2][2];
//     float Pk_temp2[2][2];

//     matrix_matrix_multiply(Kk, 2, 2, H_observation_model, 2, 2, Pk_temp);
//     matrix_matrix_subtract(identity_matrix, 2, 2, Pk_temp, 2, 2, Pk_temp);
//     matrix_matrix_multiply(Pk_temp, 2, 2, P_covariance, 2, 2, Pk_temp2);
//     matrix_copy(Pk_temp2, 2, 2, P_covariance);

//     // Done
// }