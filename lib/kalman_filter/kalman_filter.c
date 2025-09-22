#include "./kalman_filter.h"
#include "../utils/matrix_operations/matrix_operations.h"

// Formulas based on: https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/

#define KALMAN_FILTER_DEBUG 0

struct kalman_filter kalman_filter_init(){
    struct kalman_filter new_filter;
    new_filter.S_rows = 0;
    new_filter.S_cols = 0;
    return new_filter;
}

uint8_t kalman_filter_set_S_state(struct kalman_filter *filter, float *state_vector, uint8_t number_of_states){

    filter->S_state = allocate_matrix(number_of_states, 1);
    matrix_copy_flat_to_allocated(state_vector, number_of_states, 1, filter->S_state); // Special case

    filter->S_rows = number_of_states;
    filter->S_cols = 1;

    filter->got_S_state = 1;

    return 1;
}

uint8_t kalman_filter_set_P_covariance(struct kalman_filter *filter, float *covariance_matrix, uint8_t row, uint8_t col){
    //     if(filter->state_size == 0 || col != filter->state_size || row != filter->state_size){
    // #if(KALMAN_FILTER_DEBUG)
    //         printf("kalman_filter_set_P_covariance failed\n");
    // #endif
    //         return 0;
    //     }

    filter->P_covariance = allocate_matrix(row, col);
    matrix_copy_flat_to_allocated(covariance_matrix, row, col, filter->P_covariance);

    filter->got_P_covariance = 1;
    filter->P_rows = row;
    filter->P_cols = col;

    return 1;
}

uint8_t kalman_filter_set_F_state_transition_model(struct kalman_filter *filter, float *state_transition_model_matrix, uint8_t row, uint8_t col){
    //     if(filter->state_size == 0 || col != filter->state_size || row != filter->state_size){
    // #if(KALMAN_FILTER_DEBUG)
    //         printf("kalman_filter_set_F_state_transition_model failed\n");
    // #endif
    //         return 0;
    //     }

    filter->F_state_transition_model = allocate_matrix(row, col);
    matrix_copy_flat_to_allocated(state_transition_model_matrix, row, col, filter->F_state_transition_model);

    filter->got_F_state_transition_model = 1;
    filter->F_rows = row;
    filter->F_cols = col;

    return 1;
}

uint8_t kalman_filter_set_Q_proccess_noise_covariance(struct kalman_filter *filter, float *proccess_noise_covariance, uint8_t row, uint8_t col){
    //     if(filter->state_size == 0 || col != filter->state_size || row != filter->state_size){
    // #if(KALMAN_FILTER_DEBUG)
    //         printf("kalman_filter_set_Q_proccess_noise_covariance failed\n");
    // #endif
    //         return 0;
    //     }

    filter->Q_proccess_noise_covariance = allocate_matrix(row, col);
    matrix_copy_flat_to_allocated(proccess_noise_covariance, row, col, filter->Q_proccess_noise_covariance);

    filter->got_Q_proccess_noise_covariance = 1;
    filter->Q_rows = row;
    filter->Q_cols = col;

    return 1;
}

uint8_t kalman_filter_set_H_observation_model(struct kalman_filter *filter, float *observation_model, uint8_t row, uint8_t col){
    //     if(filter->state_size == 0 || col != filter->state_size || row != filter->state_size){
    // #if(KALMAN_FILTER_DEBUG)
    //         printf("kalman_filter_set_H_observation_model failed\n");
    // #endif
    //         return 0;
    //     }

    filter->H_observation_model = allocate_matrix(row, col);
    matrix_copy_flat_to_allocated(observation_model, row, col, filter->H_observation_model);

    filter->got_H_observation_model = 1;
    filter->H_rows = row;
    filter->H_cols = col;

    return 1;
}

uint8_t kalman_filter_set_R_observation_noise_covariance(struct kalman_filter *filter, float *observation_noise_covariance, uint8_t row, uint8_t col){
    //     if(filter->state_size == 0 || col != filter->state_size || row != filter->state_size){
    // #if(KALMAN_FILTER_DEBUG)
    //         printf("kalman_filter_set_R_observation_noise_covariance failed\n");
    // #endif
    //         return 0;
    //     }

    filter->R_observation_noise_covariance = allocate_matrix(row, col);
    matrix_copy_flat_to_allocated(observation_noise_covariance, row, col, filter->R_observation_noise_covariance);

    filter->got_R_observation_noise_covariance = 1;
    filter->R_rows = row;
    filter->R_cols = col;

    return 1;
}

uint8_t kalman_filter_set_B_external_input_control_matrix(struct kalman_filter *filter, float *external_input_control_matrix, uint8_t row, uint8_t col){
    //     if(filter->state_size == 0 || col != filter->state_size || row != filter->state_size){
    // #if(KALMAN_FILTER_DEBUG)
    //         printf("kalman_filter_set_B_external_input_control_matrix failed\n");
    // #endif
    //         return 0;
    //     }

    filter->B_external_input_control_matrix = allocate_matrix(row, col);
    matrix_copy_flat_to_allocated(external_input_control_matrix, row, col, filter->B_external_input_control_matrix);

    filter->B_rows = row;
    filter->B_cols = col;
    return 1;
}

uint8_t kalman_filter_configure_u_measurement_input(struct kalman_filter *filter, uint8_t input_count){
    // if(filter->state_size == 0){
    //     return 0;
    // }

    filter->u_measurements = allocate_matrix(input_count, 1);
    filter->u_measurements_size = input_count;

    return 1;
}

uint8_t kalman_filter_configure_z_measurement_input(struct kalman_filter *filter, uint8_t input_count){
    // if(filter->state_size == 0){
    //     return 0;
    // }

    filter->z_measurements = allocate_matrix(input_count, 1);
    filter->z_measurements_size = input_count;

    return 1;
}

uint8_t kalman_filter_finish_initialization(struct kalman_filter *filter){
    //    if(filter->got_S_state == 0 || filter->got_P_covariance != 1 || filter->got_F_state_transition_model != 1 || filter->got_Q_proccess_noise_covariance != 1 || filter->got_H_observation_model != 1 || filter->got_R_observation_noise_covariance != 1){
    //         return 0;
    //     }

    filter->S_state_temp = allocate_matrix(filter->F_rows, filter->S_cols); // F_rows X S_cols
    filter->P_covariance_temp = allocate_matrix(filter->P_rows, filter->P_cols); // F_rows X P_cols
    filter->Fk_transposed = allocate_matrix(filter->F_cols, filter->F_rows); // F_cols X F_rows
    filter->Yk = allocate_matrix(filter->H_rows, filter->S_cols); // H_rows X S_cols
    filter->residual_temp = allocate_matrix(filter->H_rows, filter->S_cols); // H_rows X S_cols
    filter->Sk_temp = allocate_matrix(filter->H_rows, filter->P_cols); // H_rows X P_cols
    filter->Sk = allocate_matrix(filter->H_rows, filter->H_rows); // H_rows X H_rows
    filter->Hk_transposed = allocate_matrix(filter->H_cols, filter->H_rows); // H_cols X H_rows 
    filter->Kk = allocate_matrix(filter->P_rows, filter->H_rows); // P_rows X H_rows
    filter->Kk_temp = allocate_matrix(filter->P_rows, filter->H_rows); // P_rows X H_rows
    filter->Sk_inverse = allocate_matrix(filter->H_rows, filter->H_rows); // H_rows X H_rows
    filter->Xk_temp = allocate_matrix(filter->P_rows, filter->S_cols); // P_rows X S_cols
    filter->identity_matrix = allocate_matrix(filter->P_rows, filter->P_cols); // P_rows X P_cols
    set_identity_matrix(filter->identity_matrix, filter->P_rows);
    // saDASdasd IDENTITY MATRIX
    filter->Pk_temp = allocate_matrix(filter->P_rows, filter->H_cols); // P_rows X H_cols
    filter->Pk_temp2 = allocate_matrix(filter->P_rows, filter->P_cols); // P_rows X P_cols

    return 1;
}

void kalman_filter_predict(struct kalman_filter *filter, float *Uk_external_influence){

    for (uint8_t i = 0; i < filter->u_measurements_size; i++)
    {
        filter->u_measurements[i][0] = Uk_external_influence[i];
    }

    // Xk = Fk * Xk−1
    {
#if (KALMAN_FILTER_DEBUG)
        printf("------------------------------------------------------------------------------------------- PREDICT\n");
        printf("------------------------------------------- Xk = Fk * Xk-1 + Bk * Uk\n");

        print_matrix(filter->F_state_transition_model, filter->F_rows, filter->F_cols, "Fk", 4);
        printf("times\n");
        print_matrix(filter->S_state, filter->S_rows, filter->S_cols, "Xk-1", 4);
        printf("plus\n");
        print_matrix(filter->B_external_input_control_matrix, filter->B_rows, filter->B_cols, "Bk", 4);
        printf("times\n");
        print_matrix(filter->u_measurements, filter->u_measurements_size, 1, "Uk", 4);

        printf("equals\n");
#endif

        matrix_matrix_multiply(
            filter->F_state_transition_model,
            filter->F_rows,
            filter->F_cols,
            filter->S_state,
            filter->S_rows,
            filter->S_cols,
            filter->S_state_temp); // F_rows X S_cols

        matrix_copy(
            filter->S_state_temp,
            filter->S_rows,
            filter->S_cols,
            filter->S_state
        );

        if (filter->u_measurements_size != 0){
            matrix_matrix_multiply(
                filter->B_external_input_control_matrix,
                filter->B_rows,
                filter->B_cols,
                filter->u_measurements,
                filter->u_measurements_size,
                1,
                filter->S_state_temp // B_rows X u_cols
            );

            matrix_matrix_add(
                filter->S_state,
                filter->S_rows,
                filter->S_cols,
                filter->S_state_temp,
                filter->S_rows,
                filter->S_cols,
                filter->S_state // B_rows X u_cols OR F_rows X S_cols
            );
        }

#if (KALMAN_FILTER_DEBUG)
        print_matrix(filter->S_state, filter->S_rows, filter->S_cols, "Xk", 4);
#endif
    }

    // Pk = Fk * Pk-1 * Fk(Transpose) + Qk
    {
#if (KALMAN_FILTER_DEBUG)
        printf("------------------------------------------- Pk = Fk * Pk-1 * Fk(Transpose) + Qk\n");

        print_matrix(filter->F_state_transition_model, filter->F_rows, filter->F_cols, "Fk", 4);
        printf("times\n");
        print_matrix(filter->P_covariance, filter->P_rows, filter->P_cols, "Pk-1", 4);
        printf("times\n");
        print_matrix(filter->F_state_transition_model, filter->F_rows, filter->F_cols, "Fk(transpose)", 4);
        printf("plus\n");
        print_matrix(filter->Q_proccess_noise_covariance, filter->Q_rows, filter->Q_cols, "Qk", 4);
        printf("equals\n");
#endif

        matrix_matrix_multiply(
            filter->F_state_transition_model,
            filter->F_rows,
            filter->F_cols,
            filter->P_covariance,
            filter->P_rows,
            filter->P_cols,
            filter->P_covariance_temp // F_rows X P_cols
        );
        
        matrix_transpose(
            filter->F_state_transition_model,
            filter->F_rows,
            filter->F_cols,
            filter->Fk_transposed // F_cols X F_rows
        );

        matrix_matrix_multiply(
            filter->P_covariance_temp,
            filter->P_rows,
            filter->P_cols,
            filter->Fk_transposed,
            filter->F_cols,
            filter->F_rows,
            filter->P_covariance // F_rows X F_rows
        );
        
        matrix_matrix_add(
            filter->P_covariance,
            filter->P_rows,
            filter->P_cols,
            filter->Q_proccess_noise_covariance,
            filter->Q_rows,
            filter->Q_cols,
            filter->P_covariance  // F_rows X F_rows
        );
#if (KALMAN_FILTER_DEBUG)
        print_matrix(filter->P_covariance, filter->P_rows, filter->P_cols, "Pk", 4);
#endif
    }
}

void kalman_filter_update(struct kalman_filter *filter, float *Zk_measurement_vector){
    // Convert the measurements into the correct format
#if (KALMAN_FILTER_DEBUG)
    printf("------------------------------------------------------------------------------------------- UPDATE \n");
#endif
    for (uint8_t i = 0; i < filter->z_measurements_size; i++)
    {
        filter->z_measurements[i][0] = Zk_measurement_vector[i];
    }

    // Measurement residual
    // Yk = Zk - Hk * Xk
    {
#if (KALMAN_FILTER_DEBUG)
        printf("------------------------------------------- Yk = Zk - Hk * Xk\n");

        print_matrix(filter->z_measurements, filter->z_measurements_size, 1, "Zk", 4);
        printf("minus\n");
        print_matrix(filter->H_observation_model, filter->H_rows, filter->H_cols, "Hk", 4);
        printf("times\n");
        print_matrix(filter->S_state, filter->S_rows, filter->S_cols, "Xk", 4);
#endif
        matrix_matrix_multiply(
            filter->H_observation_model,
            filter->H_rows,
            filter->H_cols,
            filter->S_state,
            filter->S_rows,
            filter->S_cols,
            filter->residual_temp // H_rows X S_cols
        );
        matrix_matrix_subtract(
            filter->z_measurements,
            filter->z_measurements_size,
            1,
            filter->residual_temp,
            filter->z_measurements_size,
            1,
            filter->Yk // H_rows X S_cols
        );

#if (KALMAN_FILTER_DEBUG)
        printf("equals\n");
        print_matrix(filter->Yk, filter->H_rows, filter->S_cols, "Yk", 4);
#endif
    }

    // Residual covariance
    // Sk = Hk * Pk * Hk(transposed) + Rk
    {
#if (KALMAN_FILTER_DEBUG)
        printf("------------------------------------------- Sk = Hk * Pk * Hk(transposed) + Rk\n");

        print_matrix(filter->H_observation_model, filter->H_rows, filter->H_cols, "Hk", 4);
        printf("times\n");
        print_matrix(filter->P_covariance, filter->P_rows, filter->P_cols, "Pk", 4);
        printf("times\n");
        print_matrix(filter->H_observation_model, filter->H_rows, filter->H_cols, "Hk(transpose)", 4);
        printf("PLUS\n");
        print_matrix(filter->R_observation_noise_covariance, filter->R_rows, filter->R_cols, "Rk", 4);
#endif

        matrix_matrix_multiply(
            filter->H_observation_model,
            filter->H_rows,
            filter->H_cols,
            filter->P_covariance,
            filter->P_rows,
            filter->P_cols,
            filter->Sk_temp // H_rows X P_cols
        );

        matrix_transpose(
            filter->H_observation_model,
            filter->H_rows,
            filter->H_cols,
            filter->Hk_transposed // H_cols X H_rows
        );

        matrix_matrix_multiply(
            filter->Sk_temp,
            filter->H_rows,
            filter->P_cols,
            filter->Hk_transposed,
            filter->H_cols,
            filter->H_rows,
            filter->Sk // H_rows X H_rows
        );

        matrix_matrix_add(
            filter->Sk,
            filter->H_rows,
            filter->H_rows,
            filter->R_observation_noise_covariance,
            filter->R_rows,
            filter->R_cols,
            filter->Sk // H_rows X H_rows
        );

#if (KALMAN_FILTER_DEBUG)
        printf("equals\n");
        print_matrix(filter->Sk, filter->H_rows, filter->H_rows, "Sk", 4);
#endif
    }
    // Kalman gain
    // Kk = Pk * Hk (transposed) * Sk (inverse)
    {
#if (KALMAN_FILTER_DEBUG)
        printf("------------------------------------------- Kk = Pk * Hk (transposed) * Sk (inverse)\n");
        print_matrix(filter->P_covariance, filter->P_rows, filter->P_cols, "Pk", 4);
        printf("times\n");
        print_matrix(filter->H_observation_model, filter->H_rows, filter->H_cols, "Hk(transposed)", 4);
        printf("times\n");
        print_matrix(filter->Sk, filter->H_rows, filter->H_rows, "Sk(Inverse)", 4);
#endif

        matrix_matrix_multiply(
            filter->P_covariance,
            filter->P_rows,
            filter->P_cols,
            filter->Hk_transposed,
            filter->H_cols,
            filter->H_rows,
            filter->Kk_temp // P_rows X H_rows
        );

        matrix_inverse(
            filter->Sk,
            filter->H_rows,    // H_rows X H_rows
            filter->Sk_inverse // Same
        );
        
        matrix_matrix_multiply(
            filter->Kk_temp,
            filter->P_rows,
            filter->H_rows,
            filter->Sk_inverse,
            filter->H_rows,
            filter->H_rows,
            filter->Kk // P_rows X H_rows
        );

#if (KALMAN_FILTER_DEBUG)
        printf("equals\n");
        print_matrix(filter->Kk, filter->P_rows, filter->H_rows, "Kk", 7);
#endif
    }

    // State update
    // Xk = Xk + Kk * Yk
    {
#if (KALMAN_FILTER_DEBUG)
        printf("------------------------------------------- Xk = Xk + Kk * Yk\n");

        print_matrix(filter->S_state, filter->S_rows, filter->S_cols, "Xk", 4);
        printf("PLUS\n");
        print_matrix(filter->Kk, filter->P_rows, filter->H_rows, "Kk", 4);
        printf("times\n");
        print_matrix(filter->Yk, filter->S_rows, filter->S_cols, "Yk", 4);
#endif

        matrix_matrix_multiply(
            filter->Kk,
            filter->P_rows,
            filter->H_rows,
            filter->Yk,
            filter->H_rows,
            filter->S_cols,
            filter->Xk_temp // P_rows X S_cols
        );
        matrix_matrix_add(
            filter->S_state,
            filter->S_rows,
            filter->S_cols,
            filter->Xk_temp,
            filter->S_rows,
            filter->S_cols,
            filter->S_state // S_rows X S_cols
        );

#if (KALMAN_FILTER_DEBUG)
        printf("equals\n");
        print_matrix(filter->S_state, filter->S_rows, filter->S_cols, "Xk", 4);
#endif
    }

    // Covariance update
    // Pk = (I(identity matrix) - Kk * Hk) * Pk
    {
#if (KALMAN_FILTER_DEBUG)
        printf("------------------------------------------- Pk = (I(identity matrix) - Kk * Hk) * Pk\n");

        print_matrix(filter->identity_matrix, filter->P_rows, filter->P_cols, "I(identity matrix)", 4);
        printf("minus\n");
        print_matrix(filter->Kk, filter->P_rows, filter->H_rows, "Kk", 4);
        printf("times\n");
        print_matrix(filter->H_observation_model, filter->H_rows, filter->H_cols, "Hk", 4);
        printf("all of this before added in parentheses times\n");
        print_matrix(filter->P_covariance, filter->P_rows, filter->P_cols, "Pk", 4);
#endif

        matrix_matrix_multiply(
            filter->Kk,
            filter->P_rows,
            filter->H_rows,
            filter->H_observation_model,
            filter->H_rows,
            filter->H_cols,
            filter->Pk_temp // P_rows X H_cols
        );

        matrix_matrix_subtract(
            filter->identity_matrix,
            filter->P_rows,
            filter->P_cols,
            filter->Pk_temp,
            filter->P_rows,
            filter->P_cols,
            filter->Pk_temp // // P_rows, P_cols
        );
            
        matrix_matrix_multiply(
            filter->Pk_temp,
            filter->P_rows,
            filter->P_cols,
            filter->P_covariance,
            filter->P_rows,
            filter->P_cols,
            filter->Pk_temp2 // P_rows X P_cols
        );

        matrix_copy(
            filter->Pk_temp2,
            filter->P_rows,
            filter->P_cols,
            filter->P_covariance
        );

#if (KALMAN_FILTER_DEBUG)
        printf("equals\n");
        print_matrix(filter->P_covariance, 2, 2, "Pk", 4);
#endif
    }

    // Done
}

float **kalman_filter_get_state(struct kalman_filter *filter){
    return filter->S_state;
}