#include "./extended_kalman_filter.h"
#include "../utils/matrix_operations/matrix_operations.h"
#include "../utils/math_constants.h"

#define EXTENDED_KALMAN_FILTER_DEBUG 0


struct extended_kalman_filter extended_kalman_filter_init(prediction_measurement_function m_f_state_transition_function, update_measurement_function m_h_measurement_function, predict_jacobian m_predict_jacobian, update_jacobian m_update_jacobian){
    struct extended_kalman_filter new_filter;
    new_filter.state_size = 0;
    new_filter.predict_jacobian = m_predict_jacobian;
    new_filter.update_jacobian = m_update_jacobian;
    new_filter.f_predict_measurement_function = m_f_state_transition_function;
    new_filter.h_measurement_function = m_h_measurement_function;

    return new_filter;
}

uint8_t extended_kalman_filter_set_S_state(struct extended_kalman_filter* filter, float *state_vector, uint8_t number_of_states){

    filter->S_state = allocate_matrix(number_of_states, 1);
    matrix_copy_flat_to_allocated(state_vector, number_of_states, 1, filter->S_state); // Special case

    filter->state_size = number_of_states;
    filter->got_S_state = 1;

    return 1;
}

uint8_t extended_kalman_filter_set_P_covariance(struct extended_kalman_filter* filter, float * covariance_matrix, uint8_t row, uint8_t col){
    if(filter->state_size == 0 || col != filter->state_size || row != filter->state_size){
#if(EXTENDED_KALMAN_FILTER_DEBUG)
        printf("extended_kalman_filter_set_P_covariance failed\n");
#endif
        return 0;
    }

    filter->P_covariance = allocate_matrix(filter->state_size, filter->state_size);
    matrix_copy_flat_to_allocated(covariance_matrix, filter->state_size, filter->state_size, filter->P_covariance);

    filter->got_P_covariance = 1;

    return 1;
}

uint8_t extended_kalman_filter_set_Q_proccess_noise_covariance(struct extended_kalman_filter* filter, float * proccess_noise_covariance, uint8_t row, uint8_t col){
    if(filter->state_size == 0 || col != filter->state_size || row != filter->state_size){
#if(EXTENDED_KALMAN_FILTER_DEBUG)
        printf("extended_kalman_filter_set_Q_proccess_noise_covariance failed\n");
#endif
        return 0;
    }

    filter->Q_proccess_noise_covariance = allocate_matrix(filter->state_size, filter->state_size);
    matrix_copy_flat_to_allocated(proccess_noise_covariance, filter->state_size, filter->state_size, filter->Q_proccess_noise_covariance);

    filter->got_Q_proccess_noise_covariance = 1;
    
    return 1;
}

uint8_t extended_kalman_filter_set_R_observation_noise_covariance(struct extended_kalman_filter* filter, float * observation_noise_covariance, uint8_t row, uint8_t col){
    if(filter->state_size == 0 || col != filter->state_size || row != filter->state_size){
#if(EXTENDED_KALMAN_FILTER_DEBUG)
        printf("extended_kalman_filter_set_R_observation_noise_covariance failed\n");
#endif
        return 0;
    }

    filter->R_observation_noise_covariance = allocate_matrix(filter->state_size, filter->state_size);
    matrix_copy_flat_to_allocated(observation_noise_covariance, filter->state_size, filter->state_size, filter->R_observation_noise_covariance);

    filter->got_R_observation_noise_covariance = 1;
    
    return 1;
}

uint8_t extended_kalman_filter_set_B_external_input_control_matrix(struct extended_kalman_filter* filter, float * external_input_control_matrix, uint8_t row, uint8_t col){
    if(filter->state_size == 0 || col != filter->state_size || row != filter->state_size){
#if(EXTENDED_KALMAN_FILTER_DEBUG)
        printf("extended_kalman_filter_set_B_external_input_control_matrix failed\n");
#endif
        return 0;
    }

    filter->B_external_input_control_matrix = allocate_matrix(filter->state_size, filter->state_size);
    matrix_copy_flat_to_allocated(external_input_control_matrix, filter->state_size, filter->state_size, filter->B_external_input_control_matrix);

    return 1;
}

uint8_t extended_kalman_filter_set_u_external_input(struct extended_kalman_filter* filter, float * external_input, uint8_t input_count){
    if(filter->state_size == 0){
        return 0;
    }
    
    filter->u_external_input = allocate_matrix(filter->state_size, 1);
    matrix_copy_flat_to_allocated(external_input, filter->state_size, 1, filter->u_external_input);

    return 1;
}



uint8_t extended_kalman_filter_finish_initialization(struct extended_kalman_filter* filter){
    if(filter->got_S_state == 0 || filter->got_P_covariance != 1 || filter->got_Q_proccess_noise_covariance != 1 || filter->got_R_observation_noise_covariance != 1){
        return 0;
    }

    uint8_t size = filter->state_size;

    filter->F_prediction_jacobian = allocate_matrix(size, size);
    filter->H_update_jacobian = allocate_matrix(size, size);
    filter->S_state_temp = allocate_matrix(size, 1); // Col = 1 because it is a vector
    filter->P_covariance_temp = allocate_matrix(size, size);
    filter->P_covariance_temp2 = allocate_matrix(size, size);
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

    filter->z_measurements = allocate_matrix(filter->state_size, 1);
    filter->u_measurements = allocate_matrix(filter->state_size, 1);
    filter->f_state_transition_matrix = allocate_matrix(filter->state_size, 1);
    filter->h_measurement_matrix = allocate_matrix(filter->state_size, 1);

    return 1;
}

void extended_kalman_filter_predict(struct extended_kalman_filter* filter, float * Uk_measurement_vector, float delta_time){

    for (uint8_t i = 0; i < 2; i++){
        filter->u_measurements[i][0] = Uk_measurement_vector[i];
    }

    // Convert the data according to the transition function
    filter->f_predict_measurement_function(filter->S_state, filter->u_measurements, filter->f_state_transition_matrix);

    // Xk = Xk-1 + delta_time * f(xk-1, uk)
    {
    #if(EXTENDED_KALMAN_FILTER_DEBUG)
        printf("------------------------------------------------------------------------------------------- PREDICT\n");
        printf("------------------------------------------- Xk = Xk-1 + delta_time * f(xk-1, uk)\n");

        print_matrix(filter->S_state, 2, 1, "Xk-1", 4);
        printf("plus\n");
        printf("%.4f\n", delta_time);
        printf("times\n");
        print_matrix(filter->f_state_transition_matrix, 2, 1, "f(xk-1, uk)", 4);
    #endif

        matrix_scalar_multiply(
            filter->f_state_transition_matrix, 
            filter->state_size, 
            1, 
            delta_time, 
            filter->S_state_temp
        );

        matrix_matrix_add(
            filter->S_state,
            filter->state_size,
            1, 
            filter->S_state_temp,
            filter->state_size,
            1, 
            filter->S_state
        );


    #if(EXTENDED_KALMAN_FILTER_DEBUG)
        print_matrix(filter->S_state, 2, 1, "Xk", 4);
    #endif
    }


    // Recalculate the Jacobian matrix (Fk)
    filter->predict_jacobian(delta_time, filter->S_state, filter->u_measurements, filter->F_prediction_jacobian);


    // Pk = Fk * Pk-1 * Fk(Transpose) + Qk
    // {

    //     matrix_transpose(
    //         filter->F_prediction_jacobian, 
    //         filter->state_size, 
    //         filter->state_size, 
    //         filter->Fk_transposed
    //     );
    // #if(EXTENDED_KALMAN_FILTER_DEBUG)
    //     printf("------------------------------------------- Pk = Fk * Pk-1 * Fk(Transpose) + Qk\n");
        
    //     print_matrix(filter->F_prediction_jacobian, 2, 2, "Fk", 4);
    //     printf("times\n");
    //     print_matrix(filter->P_covariance, 2, 2, "Pk-1", 4);
    //     printf("times\n");
    //     print_matrix(filter->Fk_transposed, 2, 2, "Fk(transposed)", 4);
    //     printf("plus\n");
    //     print_matrix(filter->Q_proccess_noise_covariance, 2, 2, "Qk", 4);
    //     printf("equals\n");
    // #endif

    //     matrix_matrix_multiply(
    //         filter->F_prediction_jacobian, 
    //         filter->state_size, 
    //         filter->state_size, 
    //         filter->P_covariance, 
    //         filter->state_size, 
    //         filter->state_size, 
    //         filter->P_covariance_temp
    //     );

    //     matrix_matrix_multiply(
    //         filter->P_covariance_temp, 
    //         filter->state_size, 
    //         filter->state_size, 
    //         filter->Fk_transposed, 
    //         filter->state_size, 
    //         filter->state_size, 
    //         filter->P_covariance
    //     );
    //     matrix_matrix_add(
    //         filter->P_covariance, 
    //         filter->state_size, 
    //         filter->state_size, 
    //         filter->Q_proccess_noise_covariance, 
    //         filter->state_size, 
    //         filter->state_size, 
    //         filter->P_covariance
    //     );
    // #if(EXTENDED_KALMAN_FILTER_DEBUG)
    //     print_matrix(filter->P_covariance, 2, 2, "Pk", 4);
    // #endif
    // }
    
    // Pk = Pk-1 + T * (Fk * Pk-1 + Pk-1 * Fk(Transpose) + Qk)
    {
        matrix_transpose(
            filter->F_prediction_jacobian, 
            filter->state_size, 
            filter->state_size, 
            filter->Fk_transposed
        );
    #if(EXTENDED_KALMAN_FILTER_DEBUG)
        printf("------------------------------------------- Pk = Pk-1 + T * (Fk * Pk-1 + Pk-1 * Fk(Transpose) + Qk)\n");
        
        print_matrix(filter->P_covariance, 4, 4, "Pk-1", 4);
        printf("plus\n");
        printf("%.4f\n", delta_time);
        printf("times\n");
        printf("(\n");
        print_matrix(filter->F_prediction_jacobian, 4, 4, "Fk", 4);
        printf("times\n");
        print_matrix(filter->P_covariance, 4, 4, "Pk-1", 4);
        printf("plus\n");
        print_matrix(filter->P_covariance, 4, 4, "Pk-1", 4);
        printf("times\n");
        print_matrix(filter->Fk_transposed, 4, 4, "Fk(transposed)", 4);
        printf("plus\n");
        print_matrix(filter->Q_proccess_noise_covariance, 2, 2, "Qk", 4);
        printf(")\n");
        printf("equals\n");
    #endif

        // Fk * Pk-1
        matrix_matrix_multiply(
            filter->F_prediction_jacobian, 
            filter->state_size, 
            filter->state_size, 
            filter->P_covariance, 
            filter->state_size, 
            filter->state_size, 
            filter->P_covariance_temp
        );
        // Pk-1 * Fk(transposed)
        matrix_matrix_multiply(
            filter->P_covariance, 
            filter->state_size, 
            filter->state_size, 
            filter->Fk_transposed, 
            filter->state_size, 
            filter->state_size, 
            filter->P_covariance_temp2
        );

        // Fk * Pk-1 + Pk-1 * Fk(transposed)
        matrix_matrix_add(
            filter->P_covariance_temp, 
            filter->state_size, 
            filter->state_size, 
            filter->P_covariance_temp2, 
            filter->state_size, 
            filter->state_size, 
            filter->P_covariance_temp
        );

        // Fk * Pk-1 + Pk-1 * Fk(transposed) + Qk
        matrix_matrix_add(
            filter->P_covariance_temp, 
            filter->state_size, 
            filter->state_size, 
            filter->Q_proccess_noise_covariance, 
            filter->state_size, 
            filter->state_size, 
            filter->P_covariance_temp
        );

        // T * (Fk * Pk-1 + Pk-1 * Fk(transposed) + Qk)
        matrix_scalar_multiply(
            filter->P_covariance_temp, 
            filter->state_size, 
            filter->state_size, 
            delta_time, 
            filter->P_covariance_temp2
        );

        // Pk = Pk-1 + T * (Fk * Pk-1 + Pk-1 * Fk(Transpose) + Qk)
        matrix_matrix_add(
            filter->P_covariance, 
            filter->state_size, 
            filter->state_size, 
            filter->P_covariance_temp2, 
            filter->state_size, 
            filter->state_size, 
            filter->P_covariance
        );
    #if(EXTENDED_KALMAN_FILTER_DEBUG)
        print_matrix(filter->P_covariance, 4, 4, "Pk", 4);
    #endif

    }
}

void extended_kalman_filter_update(struct extended_kalman_filter* filter, float * Zk_measurement_vector){
    // Convert the measurements into the correct format
#if(EXTENDED_KALMAN_FILTER_DEBUG)
    printf("------------------------------------------------------------------------------------------- UPDATE \n");
#endif
    for (uint8_t i = 0; i < filter->state_size; i++){
        filter->z_measurements[i][0] = Zk_measurement_vector[i];
    }

    // Get the measurement function h(Xk) or Z hat
    filter->h_measurement_function(filter->S_state, filter->h_measurement_matrix);

    // Get the jacobian matrix for update Hk
    // Uses S_state, h(Xk)
    filter->update_jacobian(filter->S_state, filter->H_update_jacobian);

    // Measurement residual h(Xk) = Z hat
    // Yk = Zk - h(Xk)
    {
    #if(EXTENDED_KALMAN_FILTER_DEBUG)
        printf("------------------------------------------- Yk = Zk - (h(Xk))\n");
        
        print_matrix(filter->z_measurements, 2, 1, "Zk", 4);
        printf("minus\n");
        print_matrix(filter->h_measurement_matrix, 2, 1, "h(Xk)", 4);
        printf("equals\n");
    #endif

        matrix_matrix_subtract(
            filter->z_measurements,
            filter->state_size,
            1,
            filter->h_measurement_matrix,
            filter->state_size,
            1,
            filter->Yk
        );

    #if(EXTENDED_KALMAN_FILTER_DEBUG)
        print_matrix(filter->Yk, 2, 1, "Yk", 4);
    #endif
    }
    
    // Residual covariance 
    // Sk = Hk * Pk * Hk(transposed) + Rk
    {
        matrix_transpose(
            filter->H_update_jacobian, 
            filter->state_size, 
            filter->state_size, 
            filter->Hk_transposed
        );
    #if(EXTENDED_KALMAN_FILTER_DEBUG)
        printf("------------------------------------------- Sk = Hk * Pk * Hk(transposed) + Rk\n");

        print_matrix(filter->H_update_jacobian, 2, 2, "Hk", 4);
        printf("times\n");
        print_matrix(filter->P_covariance, 2, 2, "Pk", 4);
        printf("times\n");
        print_matrix(filter->Hk_transposed, 2, 2, "Hk(transposed)", 4);
        printf("PLUS\n");
        print_matrix(filter->R_observation_noise_covariance, 2, 2, "Rk", 4);
    #endif

        matrix_matrix_multiply(
            filter->H_update_jacobian, 
            filter->state_size, 
            filter->state_size, 
            filter->P_covariance, 
            filter->state_size, 
            filter->state_size, 
            filter->Sk_temp
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
        // R is 3x3
        matrix_matrix_add(
            filter->Sk, 
            filter->state_size, 
            filter->state_size, 
            filter->R_observation_noise_covariance, 
            filter->state_size, 
            filter->state_size, 
            filter->Sk
        );

    #if(EXTENDED_KALMAN_FILTER_DEBUG)
        printf("equals\n");
        print_matrix(filter->Sk, 2, 2, "Sk", 4);
    #endif
    }

    // Kalman gain
    // Kk = Pk * Hk (transposed) * Sk (inverse)
    {
    #if(EXTENDED_KALMAN_FILTER_DEBUG)
        printf("------------------------------------------- Kk = Pk * Hk (transposed) * Sk (inverse)\n");
        print_matrix(filter->P_covariance, 2, 2, "Hk", 4);
        printf("times\n");
        print_matrix(filter->Hk_transposed, 2, 2, "Hk(transposed)", 4);
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

    #if(EXTENDED_KALMAN_FILTER_DEBUG)
        printf("equals\n");
        print_matrix(filter->Kk, 2, 2, "Kk", 4);
    #endif
    }

    // State update
    // Xk = Xk + Kk * Yk
    {
    #if(EXTENDED_KALMAN_FILTER_DEBUG)
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

    #if(EXTENDED_KALMAN_FILTER_DEBUG)
        printf("equals\n");
        print_matrix(filter->S_state, 2, 1, "Xk", 4);
    #endif
    }

    // Covariance update
    // Pk = (I(identity matrix) - Kk * Hk) * Pk
    {
    #if(EXTENDED_KALMAN_FILTER_DEBUG)
        printf("------------------------------------------- Pk = (I(identity matrix) - Kk * Hk) * Pk\n");

        print_matrix(filter->identity_matrix, 2, 2, "I(identity matrix)", 4);
        printf("minus\n");
        print_matrix(filter->Kk, 2, 2, "Kk", 4);
        printf("times\n");
        print_matrix(filter->H_update_jacobian, 2, 2, "Hk", 4);
        printf("all of this before added in parentheses times\n");
        print_matrix(filter->P_covariance, 2, 2, "Pk", 4);
    #endif

        matrix_matrix_multiply(
            filter->Kk, 
            filter->state_size, 
            filter->state_size, 
            filter->H_update_jacobian, 
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

    #if(EXTENDED_KALMAN_FILTER_DEBUG)
        printf("equals\n");
        print_matrix(filter->P_covariance, 2, 2, "Pk", 4);
    #endif
    }

    // Done
}

float ** extended_kalman_filter_get_state(struct extended_kalman_filter* filter){
    return filter->S_state;
}