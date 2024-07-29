#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

float** allocate_matrix(uint8_t row, uint8_t col);
void free_matrix(float** matrix, int size);
void matrix_matrix_multiply(float**  matrix1, uint8_t matrix1_row, uint8_t matrix1_col, float** matrix2, uint8_t matrix2_row, uint8_t matrix2_col, float** output_matrix);
void matrix_scalar_multiply(float**  matrix, uint8_t row, uint8_t col, float scalar, float** output_matrix);
void matrix_matrix_add(float**  matrix1, uint8_t matrix1_row, uint8_t matrix1_col, float** matrix2, uint8_t matrix2_row, uint8_t matrix2_col, float** output_matrix);
void matrix_matrix_subtract(float**  matrix1, uint8_t matrix1_row, uint8_t matrix1_col, float** matrix2, uint8_t matrix2_row, uint8_t matrix2_col, float** output_matrix);
void matrix_transpose(float**  matrix1, uint8_t matrix1_row, uint8_t matrix1_col, float** output_matrix);
void matrix_copy(float** matrix1, uint8_t matrix1_row, uint8_t matrix1_col, float** output_matrix);
void matrix_copy_flat_to_allocated(float* flat_array, uint8_t row, uint8_t col, float** dynamic_array);
void matrix_swap_rows(float** matrix, int row1, int row2, int size);
int matrix_inverse(float** matrix1, int size, float** matrix2);
void print_matrix(float** matrix, uint8_t rows, uint8_t cols, const char* title, int precision);
void set_identity_matrix(float** matrix, uint8_t size);