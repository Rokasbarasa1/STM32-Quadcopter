#include "./ist8310.h"
#include "../utils/string_utils/string_utils.h"

#include <inttypes.h>
#include <stdio.h>

#define IST8310_I2C_ID  (0x0E << 1)
#define IST8310_WHO_AM_I_REG 0x00
#define IST8310_STATUS_REGISTER_1_REG 0x02
#define IST8310_OUTPUT_VALUE_X_L_REG 0x03
#define IST8310_OUTPUT_VALUE_X_H_REG 0x04
#define IST8310_OUTPUT_VALUE_Y_L_REG 0x05
#define IST8310_OUTPUT_VALUE_Y_H_REG 0x06
#define IST8310_OUTPUT_VALUE_Z_L_REG 0x07
#define IST8310_OUTPUT_VALUE_Z_H_REG 0x08
#define IST8310_STATUS_REGISTER_2_REG 0x09
#define IST8310_CONTROL_REGISTER_1_REG 0x0A
#define IST8310_CONTROL_REGISTER_2_REG 0x0B
#define IST8310_SELF_TEST_REG 0x0C
#define IST8310_OUTPUT_VALUE_T_L_REG 0x1C
#define IST8310_OUTPUT_VALUE_T_H_REG 0x1D
#define IST8310_AVERAGE_CONTROL_REG 0x41
#define IST8310_PULSE_DURATION_CONTROL_REG 0x42

#define IST8310_INT_TO_MICRO_TESLAS 0.3f

#define IST8310_WHO_AM_I_VALUE 0x10
#define IST8310_STATUS_REGISTER_1_GET_DRDY(value) ((value) & 0b00000001)
#define IST8310_STATUS_REGISTER_1_GET_DOR(value) (((value) & 0b00000010) >> 1)

#define IST8310_STATUS_REGISTER_2_GET_INT(value) (((value) & 0b00001000) >> 3)

enum ist8310_CONTROL_REGISTER_1 {
    IST8310_CONTROL_REGISTER_1_MODE_STAND_BY                                = 0b00000000,
    IST8310_CONTROL_REGISTER_1_MODE_SINGLE_MEASUREMENT                      = 0b00000001
};

enum ist8310_CONTROL_REGISTER_2 {
    IST8310_CONTROL_REGISTER_2_ENABLE_DATA_READY_ENABLE_CONTROL             = 0b00001000,
    IST8310_CONTROL_REGISTER_2_DISABLE_DATA_READY_ENABLE_CONTROL            = 0b00000000,
    IST8310_CONTROL_REGISTER_2_PIN_POLARITY_CONTROL_ACTIVE_LOW              = 0b00000000,
    IST8310_CONTROL_REGISTER_2_PIN_POLARITY_CONTROL_ACTIVE_HIGH             = 0b00000100,
    IST8310_CONTROL_REGISTER_2_DO_SOFT_RESET                                = 0b00000001,
};

enum ist8310_SELF_TEST {
    IST8310_SELF_TEST_START_SELF_TEST                                       = 0b01000000,
    IST8310_SELF_TEST_STOP_SELF_TEST                                        = 0b00000000,
};

enum ist8310_AVERAGE_CONTROL {
    IST8310_AVERAGE_CONTROL_AVERAGE_FOR_X_Z_NO_AVERAGE                      = 0b00000000,
    IST8310_AVERAGE_CONTROL_AVERAGE_FOR_X_Z_AVERAGE_2                       = 0b00000001,
    IST8310_AVERAGE_CONTROL_AVERAGE_FOR_X_Z_AVERAGE_4                       = 0b00000010,
    IST8310_AVERAGE_CONTROL_AVERAGE_FOR_X_Z_AVERAGE_8                       = 0b00000011,
    IST8310_AVERAGE_CONTROL_AVERAGE_FOR_X_Z_AVERAGE_16                      = 0b00000100,
    IST8310_AVERAGE_CONTROL_AVERAGE_FOR_Y_NO_AVERAGE                        = 0b00000000,
    IST8310_AVERAGE_CONTROL_AVERAGE_FOR_Y_AVERAGE_2                         = 0b00001000,
    IST8310_AVERAGE_CONTROL_AVERAGE_FOR_Y_AVERAGE_4                         = 0b00010000,
    IST8310_AVERAGE_CONTROL_AVERAGE_FOR_Y_AVERAGE_8                         = 0b00011000,
    IST8310_AVERAGE_CONTROL_AVERAGE_FOR_Y_AVERAGE_16                        = 0b00100100,
};

enum ist8310_PULSE_DURATION_CONTROL {
    IST8310_PULSE_DURATION_CONTROL_PULSE_DURATION_LONG                      = 0b01000000,
    IST8310_PULSE_DURATION_CONTROL_PULSE_DURATION_NORMAL                    = 0b11000000,
};


I2C_HandleTypeDef *ist8310_i2c_handle;

// Storage of hard iron correction, values should be replaced by what is passed
volatile float ist8310_hard_iron[3] = {
    0, 0, 0
};

// Storage of soft iron correction, values should be replaced by what is passed
volatile float ist8310_soft_iron[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
};

float ist8310_old_magnetometer_readings[3] = {0, 0, 0};


HAL_StatusTypeDef ist8310_i2c_read(uint16_t register_address, uint8_t *pointer_to_data, uint16_t size){
    return HAL_I2C_Mem_Read(
        ist8310_i2c_handle,
        IST8310_I2C_ID + 1,
        register_address,
        1,
        pointer_to_data,
        size,
        5
    );
}

HAL_StatusTypeDef ist8310_i2c_write(uint16_t register_address, uint8_t *pointer_to_data, uint16_t size){
    return HAL_I2C_Mem_Write(
        ist8310_i2c_handle,
        IST8310_I2C_ID,
        register_address,
        1,
        pointer_to_data,
        size,
        100
    );
}

uint8_t ist8310_init(
    I2C_HandleTypeDef *i2c_handle_temp, 
    uint8_t apply_calibration, 
    const float hard_iron[3], 
    const float soft_iron[3][3]
){
    ist8310_i2c_handle = i2c_handle_temp;

    // assign the correction for irons
    if (apply_calibration){
        for (uint8_t i = 0; i < 3; i++){
            ist8310_hard_iron[i] = hard_iron[i];
        }

        for (uint8_t i = 0; i < 3; i++){
            for (uint8_t k = 0; k < 3; k++){
                ist8310_soft_iron[i][k] = soft_iron[i][k];
            }
        }
    }

    HAL_StatusTypeDef result;

    // Soft reset
    uint8_t data = 0b00000000;
    data |= IST8310_CONTROL_REGISTER_2_DO_SOFT_RESET;
    
    result = ist8310_i2c_write(IST8310_CONTROL_REGISTER_2_REG, &data, 1);

    if(result != HAL_OK){
        printf("IST8310 init failed: soft reset i2c %d\n", result);
        return 0;
    }

    // Wait a bit for it to reset
    HAL_Delay(200);


    // Read the ID
    uint8_t retrieved_data[] = {0};

    result = ist8310_i2c_read(IST8310_WHO_AM_I_REG, retrieved_data, 1);

    if(retrieved_data[0] != IST8310_WHO_AM_I_VALUE ){
        printf("IST8310 init failed: who am i id not correct - ");
        print_binary(retrieved_data[0]);
        printf("\n");
        return 0;
    }

    // Set Pulse duration to normal
    data = 0b00000000;
    data |= IST8310_PULSE_DURATION_CONTROL_PULSE_DURATION_NORMAL;

    result = ist8310_i2c_write(IST8310_PULSE_DURATION_CONTROL_REG, &data, 1);


    // Set averaging
    data = 0b00000000;
    data |= IST8310_AVERAGE_CONTROL_AVERAGE_FOR_X_Z_NO_AVERAGE;
    data |= IST8310_AVERAGE_CONTROL_AVERAGE_FOR_Y_NO_AVERAGE;

    result = ist8310_i2c_write(IST8310_AVERAGE_CONTROL_REG, &data, 1);

    printf("starting self test\n");
    // Self test
    float initial_values[] = {0, 0, 0};
    float after_values[] = {0, 0, 0};

    ist8310_magnetometer_readings_micro_teslas_poll(initial_values, 1);
    
    // Enable self test
    data = IST8310_SELF_TEST_START_SELF_TEST;
    result = ist8310_i2c_write(IST8310_SELF_TEST_REG, &data, 1);


    ist8310_magnetometer_readings_micro_teslas_poll(after_values, 1);

    // Disable self test as value already read
    data = IST8310_SELF_TEST_STOP_SELF_TEST;
    result = ist8310_i2c_write(IST8310_SELF_TEST_REG, &data, 1);

    // Apply absolute and compare
    printf("X %f == %f\n", initial_values[0], after_values[0]);
    printf("Y %f == %f\n", initial_values[1], after_values[1]);
    printf("Z %f == %f\n", initial_values[2], after_values[2]);
    for (uint8_t i = 0; i < 3; i++){
        initial_values[i] = fabs(initial_values[i]);
        after_values[i] = fabs(after_values[i]);
    }



    if(initial_values[0] != after_values[0] || initial_values[1] != after_values[1] || initial_values[2] != after_values[2]){
        printf("Self test BAD\n");
    }else{
        printf("Self test OK\n");
    }

    retrieved_data[0] = 0;

    result = ist8310_i2c_read(0x40, retrieved_data, 1);


    return 1;
}

void ist8310_magnetometer_readings_micro_teslas(float *data, uint8_t perfrom_temperature_correction){
    uint8_t retrieved_data[] = {0, 0, 0, 0, 0, 0};

    ist8310_i2c_read(IST8310_OUTPUT_VALUE_X_L_REG, retrieved_data, 6);

    // First is least significant and second is most significant
    int16_t X = ((int16_t)retrieved_data[1] << 8) | (int16_t)retrieved_data[0];
    int16_t Y = ((int16_t)retrieved_data[3] << 8) | (int16_t)retrieved_data[2];
    int16_t Z = ((int16_t)retrieved_data[5] << 8) | (int16_t)retrieved_data[4];

    data[0] = (float)X * IST8310_INT_TO_MICRO_TESLAS;
    data[1] = (float)Y * IST8310_INT_TO_MICRO_TESLAS;
    data[2] = (float)Z * IST8310_INT_TO_MICRO_TESLAS;

    ist8310_old_magnetometer_readings[0] = data[0];
    ist8310_old_magnetometer_readings[1] = data[1];
    ist8310_old_magnetometer_readings[2] = data[2];

    // Use the soft and hard iron calibrations
    for (uint8_t i = 0; i < 3; i++){
        data[i] = data[i] - ist8310_hard_iron[i];
    }
    
    float temp[3];
    for (uint8_t i = 0; i < 3; i++){
        temp[i] = (ist8310_soft_iron[i][0] * data[0]) +
                  (ist8310_soft_iron[i][1] * data[1]) +
                  (ist8310_soft_iron[i][2] * data[2]);
    }
    for (uint8_t i = 0; i < 3; i++) data[i] = temp[i];

}

void ist8310_magnetometer_initiate_reading(){
    uint8_t data = IST8310_CONTROL_REGISTER_1_MODE_SINGLE_MEASUREMENT;
    ist8310_i2c_write(IST8310_CONTROL_REGISTER_1_REG, &data, 1);
}

void ist8310_magnetometer_readings_micro_teslas_poll(float *data, uint8_t perfrom_temperature_correction){
    ist8310_magnetometer_initiate_reading();
    // Wait 5 ms minimum
    HAL_Delay(5);

    uint8_t retrieved_data[] = {0};
    do{
        ist8310_i2c_read(IST8310_STATUS_REGISTER_1_REG, retrieved_data, 1);
    } while (IST8310_STATUS_REGISTER_1_GET_DRDY(retrieved_data[0]) == 0); // loop while data ready bit not set

    ist8310_magnetometer_readings_micro_teslas(data, perfrom_temperature_correction);
}

void ist8310_previous_raw_magetometer_readings(float *data){
    data[0] = ist8310_old_magnetometer_readings[0];
    data[1] = ist8310_old_magnetometer_readings[1];
    data[2] = ist8310_old_magnetometer_readings[2];
}


// Init guide
// Soft reset
// Wait 50 ms or smth
// Set Pulse duration to normal
// Set averaging


// Selft test guide
// Read all mag values
// Turn on self test
// Read all mag values again
// If the absolute valuesa re the same. (turn negatives in to positive values) then its good.
// Turn off self test


// Measuring guide
// Enter single measurement mode
// Wait some time. Min is 5 ms
// Check status register 1 DRDY
// DRDY turns 0 when any measurement is read
// Min waiting 