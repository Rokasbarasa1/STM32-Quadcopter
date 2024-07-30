#include "./matrix_operations.h"

#define EXTENDED_KALMAN_FILTER_DEBUG 1

float** allocate_matrix(uint8_t row, uint8_t col) {
    // Allocate an array of pointers for each row
    float** matrix = (float**)malloc(row * sizeof(float*));

    for (int i = 0; i < row; i++) {
        matrix[i] = (float*)calloc(col, sizeof(float));  // Use calloc here
    }

    // for (int i = 0; i < row; i++) {
    //     for (int j = 0; j < col; j++) {
    //         printf("matrix[%d][%d] = %f\n", i, j, matrix[i][j]);
    //     }
    // }
    
    return matrix;
}

void free_matrix(float** matrix, int size) {
    for (int i = 0; i < size; i++) {
        free(matrix[i]);
    }
    free(matrix);
}

void matrix_scalar_multiply(float**  matrix, uint8_t row, uint8_t col, float scalar, float** output_matrix) {
    for (int i = 0; i < row; i++) {
        for (int j = 0; j < col; j++) {
            output_matrix[i][j] = matrix[i][j] * scalar;
        }
    }
}

void matrix_matrix_multiply(float**  matrix1, uint8_t matrix1_row, uint8_t matrix1_col, float** matrix2, uint8_t matrix2_row, uint8_t matrix2_col, float** output_matrix){

    // Check constraints
    if (matrix1_col != matrix2_row) {
#if(EXTENDED_KALMAN_FILTER_DEBUG)
        printf("matrix_matrix_multiply: Dimensions not compatible\n");
#endif
        if(matrix2_row == 1 && matrix2_col == 1){
            printf("Switching to scalar multi\n");
            return matrix_scalar_multiply(matrix1, matrix1_row, matrix1_col, matrix2[0][0], output_matrix);
        }else{
            return;  // The number of columns in the first matrix must equal the number of rows in the second.
        }
    }

    // Initialize the output matrix elements to zero
    for (uint8_t i = 0; i < matrix1_row; i++) {
        for (uint8_t j = 0; j < matrix2_col; j++) {
            // printf("1$ %d %d\n", i, j);
            output_matrix[i][j] = 0;  // Reset to zero before summing
        }
    }

    // Perform matrix multiplication
    for (uint8_t i = 0; i < matrix1_row; i++) {
        for (uint8_t j = 0; j < matrix2_col; j++) {
            for (uint8_t k = 0; k < matrix1_col; k++) {
                // printf("2$ %d %d %d\n", i, j, k);
                output_matrix[i][j] += matrix1[i][k] * matrix2[k][j];
            }
        }
    }
}



void matrix_matrix_add(float**  matrix1, uint8_t matrix1_row, uint8_t matrix1_col, float** matrix2, uint8_t matrix2_row, uint8_t matrix2_col, float** output_matrix){
    if(matrix1_col != matrix2_col || matrix1_row != matrix2_row){
#if(EXTENDED_KALMAN_FILTER_DEBUG)
        printf("matrix_matrix_add: Dimensions not equal\n");
#endif
        return;
    }

    for (uint8_t i = 0; i < matrix1_row; i++){
        for (uint8_t j = 0; j < matrix1_col; j++){
            // printf("matrix_matrix_add %f = %f * %f\n", output_matrix[i][j], matrix1[i][j], matrix2[i][j]);
            output_matrix[i][j] = matrix1[i][j] + matrix2[i][j];
        }
    }
}

void matrix_matrix_subtract(float**  matrix1, uint8_t matrix1_row, uint8_t matrix1_col, float** matrix2, uint8_t matrix2_row, uint8_t matrix2_col, float** output_matrix){
    if(matrix1_col != matrix2_col || matrix1_row != matrix2_row){
#if(EXTENDED_KALMAN_FILTER_DEBUG)
        printf("matrix_matrix_subtract: Dimensions not equal\n");
#endif
        return;
    }

    for (uint8_t i = 0; i < matrix1_row; i++){
        for (uint8_t j = 0; j < matrix1_col; j++){
            // printf("matrix_matrix_subtract %f = %f - %f\n", output_matrix[i][j], matrix1[i][j], matrix2[i][j]);
            output_matrix[i][j] = matrix1[i][j] - matrix2[i][j];
        }
    }
}

void matrix_transpose(float** matrix1, uint8_t matrix1_row, uint8_t matrix1_col, float** output_matrix){

    
    for (int i = 0; i < matrix1_row; i++) {
        for (int j = 0; j < matrix1_col; j++) {
            output_matrix[j][i] = matrix1[i][j];
        }
    }
}

void matrix_copy(float** matrix1, uint8_t matrix1_row, uint8_t matrix1_col, float** output_matrix) {
    for (int i = 0; i < matrix1_row; i++) {
        // printf("matrix_copy col %d\n", i);
        for (int j = 0; j < matrix1_col; j++) {
            // printf("matrix_copy %f -> %f\n", matrix1[i][j], output_matrix[i][j]);
            output_matrix[i][j] = matrix1[i][j];
        }
    }
    // printf("matrix_copy done\n");
}

void matrix_copy_flat_to_allocated(float* flat_array, uint8_t row, uint8_t col, float** dynamic_array) {
    for (int i = 0; i < row; i++) {
        // printf("matrix_copy_flat_to_allocated col %d\n", i);
        for (int j = 0; j < col; j++) {
            // printf("matrix_copy_flat_to_allocated %f -> %f\n", flat_array[i * col + j], dynamic_array[i][j]);
            dynamic_array[i][j] = flat_array[i * col + j];
        }
    }
    // printf("matrix_copy_flat_to_allocated done\n");
}

void matrix_swap_rows(float** matrix, int row1, int row2, int size) {
    for (int i = 0; i < size; i++) {
        float temp = matrix[row1][i];
        matrix[row1][i] = matrix[row2][i];
        matrix[row2][i] = temp;
    }
}

int matrix_inverse(float** matrix1, int size, float** matrix2) {
    float** temp = allocate_matrix(size, size);

    // Initialize matrix2 as an identity matrix
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            matrix2[i][j] = (i == j) ? 1.0f : 0.0f;
            temp[i][j] = matrix1[i][j];
        }
    }

    // Perform Gaussian elimination
    for (int i = 0; i < size; i++) {
        // Partial pivoting
        float maxEl = fabs(temp[i][i]);
        int maxRow = i;
        for (int k = i + 1; k < size; k++) {
            if (fabs(temp[k][i]) > maxEl) {
                maxEl = fabs(temp[k][i]);
                maxRow = k;
            }
        }

        // Swap maximum row with current row
        if (maxRow != i) {
            matrix_swap_rows(temp, i, maxRow, size);
            matrix_swap_rows(matrix2, i, maxRow, size);
        }

        // Set to zero all other elements in the current column
        for (int k = 0; k < size; k++) {
            if (k != i) {
                float c = -temp[k][i] / temp[i][i];
                for (int j = i; j < size; j++) {
                    if (i == j) {
                        temp[k][j] = 0;
                    } else {
                        temp[k][j] += c * temp[i][j];
                    }
                }
                for (int j = 0; j < size; j++) {
                    matrix2[k][j] += c * matrix2[i][j];
                }
            }
        }

        // Scale row to make the diagonal element 1
        float c = 1.0f / temp[i][i];
        for (int j = 0; j < size; j++) {
            temp[i][j] *= c;
            matrix2[i][j] *= c;
        }
    }

    free_matrix(temp, size);
    return 1; // Return 1 to indicate success
}

void print_matrix(float** matrix, uint8_t rows, uint8_t cols, const char* title, int precision) {
    // Print the title above the matrix
    printf("%s\n", title);
    
    // Calculate the maximum width of printed numbers for alignment
    int max_width = 0;
    for (uint8_t i = 0; i < rows; i++) {
        for (uint8_t j = 0; j < cols; j++) {
            int current_width = snprintf(NULL, 0, "%.*f", precision, matrix[i][j]);
            if (current_width > max_width) {
                max_width = current_width;
            }
        }
    }

    // Add extra space for padding
    max_width += 2;

    // Print the matrix with each element aligned properly
    for (uint8_t i = 0; i < rows; i++) {
        printf("| ");
        for (uint8_t j = 0; j < cols; j++) {
            printf("%*.*f ", max_width - 2, precision, matrix[i][j]);
        }
        printf("|\n");
    }
    printf("\n");  // Print a newline for better separation after the matrix
}

void set_identity_matrix(float** matrix, uint8_t size) {
    for (uint8_t i = 0; i < size; i++) {
        for (uint8_t j = 0; j < size; j++) {
            matrix[i][j] = (i == j) ? 1.0f : 0.0f;  // Set diagonal elements to 1, others to 0
        }
    }
}