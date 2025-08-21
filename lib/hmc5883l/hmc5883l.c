#include "./hmc5883l.h"
#include "../utils/math_constants.h"

#define HMC5883L_I2C_ID (0x3C << 1)
// #define HMC5883L_I2C_ID (0x1E << 1)
// #define HMC5883L_I2C_ID  (0x2C << 1)

#define HMC5883L_CONTROL_A_REG 0x00
#define HMC5883L_CONTROL_B_REG 0x01
#define HMC5883L_MODE_REG 0x02
#define HMC5883L_DATA_OUTPUT_X_MSB_REG 0x03
#define HMC5883L_DATA_OUTPUT_X_LSB_REG 0x04
#define HMC5883L_DATA_OUTPUT_Y_MSB_REG 0x05
#define HMC5883L_DATA_OUTPUT_Y_LSB_REG 0x06
#define HMC5883L_DATA_OUTPUT_Z_MSB_REG 0x07
#define HMC5883L_DATA_OUTPUT_Z_LSB_REG 0x08
#define HMC5883L_STATUS_REG 0x09
#define HMC5883L_ID_A_REG 0x0A
#define HMC5883L_ID_B_REG 0x0B
#define HMC5883L_ID_C_REG 0x0C

enum hmc5883l_control_A {
    HMC5883L_CONTROL_A_SAMPLES_AVERAGED_1                  = 0b00000000,
    HMC5883L_CONTROL_A_SAMPLES_AVERAGED_2                  = 0b00100000,
    HMC5883L_CONTROL_A_SAMPLES_AVERAGED_4                  = 0b01000000,
    HMC5883L_CONTROL_A_SAMPLES_AVERAGED_8                  = 0b01100000,

    HMC5883L_CONTROL_A_DATA_OUTPUT_RATE_0_75_HZ            = 0b00000000,
    HMC5883L_CONTROL_A_DATA_OUTPUT_RATE_1_50_HZ            = 0b00000100,
    HMC5883L_CONTROL_A_DATA_OUTPUT_RATE_3_HZ               = 0b00001000,
    HMC5883L_CONTROL_A_DATA_OUTPUT_RATE_7_50_HZ            = 0b00001100,
    HMC5883L_CONTROL_A_DATA_OUTPUT_RATE_15_HZ              = 0b00010000,
    HMC5883L_CONTROL_A_DATA_OUTPUT_RATE_30_HZ              = 0b00010100,
    HMC5883L_CONTROL_A_DATA_OUTPUT_RATE_75_HZ              = 0b00011000,

    HMC5883L_CONTROL_A_MEASUREMENT_MODE_NORMAL             = 0b00000000,
    HMC5883L_CONTROL_A_MEASUREMENT_MODE_POSITIVE_BIAS      = 0b00000001,
    HMC5883L_CONTROL_A_MEASUREMENT_MODE_NEGATIVE_BIAS      = 0b00000010,
};

#define HMC5883L_MAG_OUTPUT_TO_SENSOR_RESOLUTION_0_88_GA (1.0f / 1370.0f)
#define HMC5883L_MAG_OUTPUT_TO_SENSOR_RESOLUTION_1_30_GA (1.0f / 1090.0f)
#define HMC5883L_MAG_OUTPUT_TO_SENSOR_RESOLUTION_1_90_GA (1.0f / 820.0f)
#define HMC5883L_MAG_OUTPUT_TO_SENSOR_RESOLUTION_2_50_GA (1.0f / 660.0f)
#define HMC5883L_MAG_OUTPUT_TO_SENSOR_RESOLUTION_4_00_GA (1.0f / 440.0f)
#define HMC5883L_MAG_OUTPUT_TO_SENSOR_RESOLUTION_4_70_GA (1.0f / 390.0f)
#define HMC5883L_MAG_OUTPUT_TO_SENSOR_RESOLUTION_5_60_GA (1.0f / 330.0f)
#define HMC5883L_MAG_OUTPUT_TO_SENSOR_RESOLUTION_8_10_GA (1.0f / 230.0f)

float hmc5883l_convert_magnetometer_output = 0.0f;

enum hmc5883l_control_B {
    HMC5883L_CONTROL_B_SENSOR_RESOLUTION_0_88_GA           = 0b00000000,
    HMC5883L_CONTROL_B_SENSOR_RESOLUTION_1_30_GA           = 0b00100000,
    HMC5883L_CONTROL_B_SENSOR_RESOLUTION_1_90_GA           = 0b01000000,
    HMC5883L_CONTROL_B_SENSOR_RESOLUTION_2_50_GA           = 0b01100000,
    HMC5883L_CONTROL_B_SENSOR_RESOLUTION_4_00_GA           = 0b10000000,
    HMC5883L_CONTROL_B_SENSOR_RESOLUTION_4_70_GA           = 0b10100000,
    HMC5883L_CONTROL_B_SENSOR_RESOLUTION_5_60_GA           = 0b11000000,
    HMC5883L_CONTROL_B_SENSOR_RESOLUTION_8_10_GA           = 0b11100000,
};


enum hmc5883l_mode {
    HMC5883L_MODE_CONTINUOUS                               = 0b00000000,
    HMC5883L_MODE_SINGLE_MEASUREMENT                       = 0b01000000,
    HMC5883L_MODE_IDLE                                     = 0b10000000,
};

I2C_HandleTypeDef *hmc5883l_i2c_handle;

// Storage of hard iron correction, values should be replaced by what is passed
volatile float hmc5883l_hard_iron[3] = {
    0, 0, 0
};

// Storage of soft iron correction, values should be replaced by what is passed
volatile float hmc5883l_soft_iron[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
};

float hmc5883l_old_magenetometer_readings[3] = {0, 0, 0};

// max value output is at 200 Hz
uint8_t init_hmc5883l(I2C_HandleTypeDef *i2c_handle_temp, uint8_t apply_calibration, const float hard_iron[3], const float soft_iron[3][3]){
    hmc5883l_i2c_handle = i2c_handle_temp;

    // assign the correction for irons
    if (apply_calibration){
        for (uint8_t i = 0; i < 3; i++){
            hmc5883l_hard_iron[i] = hard_iron[i];
        }

        for (uint8_t i = 0; i < 3; i++){
            for (uint8_t k = 0; k < 3; k++){
                hmc5883l_soft_iron[i][k] = soft_iron[i][k];
            }
        }
    }



    HAL_StatusTypeDef result;

    uint8_t retrieved_data_all[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    result = HAL_I2C_Mem_Read(
        hmc5883l_i2c_handle,
        HMC5883L_I2C_ID + 1,
        0x00,
        1,
        retrieved_data_all,
        18,
        5
    );

    for (size_t i = 0; i < 18; i++){
        printf("Reg %d -> %d\n", i, retrieved_data_all[i]);
    }
    



    uint8_t retrieved_data[] = {0, 0, 0, 0, 0, 0};

    result = HAL_I2C_Mem_Read(
        hmc5883l_i2c_handle,
        HMC5883L_I2C_ID + 1,
        HMC5883L_ID_A_REG,
        1,
        retrieved_data,
        6,
        5
    );

    if(result != HAL_OK){
        printf("HMC5883L initialization failed: Failed reading id of sensor %d\n", result);
        return 0;
    }

    if(retrieved_data[0] != 0x48 || retrieved_data[1] != 0x34 || retrieved_data[2] != 0x33){
        printf("HMC5883L initialization failed: Got invalid id %d;%d;%d;%d;%d;%d;\n",retrieved_data[0], retrieved_data[1], retrieved_data[2], retrieved_data[3], retrieved_data[4], retrieved_data[5]);
        return 0;
    }



    uint8_t data = 0b00000000;
    data |= HMC5883L_CONTROL_A_SAMPLES_AVERAGED_1;
    data |= HMC5883L_CONTROL_A_DATA_OUTPUT_RATE_75_HZ;
    data |= HMC5883L_CONTROL_A_MEASUREMENT_MODE_NORMAL;
    result = HAL_I2C_Mem_Write(
        hmc5883l_i2c_handle,
        HMC5883L_I2C_ID,
        HMC5883L_CONTROL_A_REG,
        1,
        &data,
        1,
        2000
    );

    if(result != HAL_OK){
        
        printf("HMC5883L initialization failed: Failed to set control reg A settings %d\n", result);
        return 0;
    }


    data = 0b00000000;
    data |= HMC5883L_CONTROL_B_SENSOR_RESOLUTION_1_90_GA;

    switch (HMC5883L_CONTROL_B_SENSOR_RESOLUTION_1_90_GA){
        case HMC5883L_CONTROL_B_SENSOR_RESOLUTION_0_88_GA:
            hmc5883l_convert_magnetometer_output = HMC5883L_MAG_OUTPUT_TO_SENSOR_RESOLUTION_0_88_GA;
            break;
        case HMC5883L_CONTROL_B_SENSOR_RESOLUTION_1_30_GA:
            hmc5883l_convert_magnetometer_output = HMC5883L_MAG_OUTPUT_TO_SENSOR_RESOLUTION_1_30_GA;
            break;
        case HMC5883L_CONTROL_B_SENSOR_RESOLUTION_1_90_GA:
            hmc5883l_convert_magnetometer_output = HMC5883L_MAG_OUTPUT_TO_SENSOR_RESOLUTION_1_90_GA;
            break;
        case HMC5883L_CONTROL_B_SENSOR_RESOLUTION_2_50_GA:
            hmc5883l_convert_magnetometer_output = HMC5883L_MAG_OUTPUT_TO_SENSOR_RESOLUTION_2_50_GA;
            break;
        case HMC5883L_CONTROL_B_SENSOR_RESOLUTION_4_00_GA:
            hmc5883l_convert_magnetometer_output = HMC5883L_MAG_OUTPUT_TO_SENSOR_RESOLUTION_4_00_GA;
            break;
        case HMC5883L_CONTROL_B_SENSOR_RESOLUTION_4_70_GA:
            hmc5883l_convert_magnetometer_output = HMC5883L_MAG_OUTPUT_TO_SENSOR_RESOLUTION_4_70_GA;
            break;
        case HMC5883L_CONTROL_B_SENSOR_RESOLUTION_5_60_GA:
            hmc5883l_convert_magnetometer_output = HMC5883L_MAG_OUTPUT_TO_SENSOR_RESOLUTION_5_60_GA;
            break;
        case HMC5883L_CONTROL_B_SENSOR_RESOLUTION_8_10_GA:
            hmc5883l_convert_magnetometer_output = HMC5883L_MAG_OUTPUT_TO_SENSOR_RESOLUTION_8_10_GA;
            break;
        default:
            hmc5883l_convert_magnetometer_output = HMC5883L_MAG_OUTPUT_TO_SENSOR_RESOLUTION_1_90_GA;
            break;
    }


    result = HAL_I2C_Mem_Write(
        hmc5883l_i2c_handle,
        HMC5883L_I2C_ID,
        HMC5883L_CONTROL_B_REG,
        1,
        &data,
        1,
        5
    );

    if(result != HAL_OK){
        printf("HMC5883L initialization failed: Failed to set control reg B settings %d\n", result);
        return 0;
    }

    data = 0b00000000;
    data |= HMC5883L_MODE_CONTINUOUS;
    result = HAL_I2C_Mem_Write(
        hmc5883l_i2c_handle,
        HMC5883L_I2C_ID,
        HMC5883L_MODE_REG,
        1,
        &data,
        1,
        5
    );

    if(result != HAL_OK){
        printf("HMC5883L initialization failed: Failed to set mode register settings %d\n", result);
        return 0;
    }

    printf("HMC5883L initialized\n");

    return 1;
}

void hmc5883l_magnetometer_readings_micro_teslas(float *data){
    uint8_t retrieved_data[] = {0, 0, 0, 0, 0, 0};

    HAL_I2C_Mem_Read(
        hmc5883l_i2c_handle,
        HMC5883L_I2C_ID + 1,
        HMC5883L_DATA_OUTPUT_X_MSB_REG,
        1,
        retrieved_data,
        6, // read six registers in total so from 
        5
    );

    // First is least significant and second is most significant
    int16_t X = ((int16_t)retrieved_data[1] << 8) | (int16_t)retrieved_data[0];
    int16_t Y = ((int16_t)retrieved_data[3] << 8) | (int16_t)retrieved_data[2];
    int16_t Z = ((int16_t)retrieved_data[5] << 8) | (int16_t)retrieved_data[4];
    
    // Convert the mag's adc value to gauss
    // ADC accuracy is 2 MilliGauss per 1 step
    // Divide by 20 to do micro teslas
    data[0] = (float)X * hmc5883l_convert_magnetometer_output;
    data[1] = (float)Y * hmc5883l_convert_magnetometer_output;
    data[2] = (float)Z * hmc5883l_convert_magnetometer_output;

    hmc5883l_old_magenetometer_readings[0] = data[0];
    hmc5883l_old_magenetometer_readings[1] = data[1];
    hmc5883l_old_magenetometer_readings[2] = data[2];

    // Use the soft and hard iron calibrations
    for (uint8_t i = 0; i < 3; i++){
        data[i] = data[i] - hmc5883l_hard_iron[i];
    }
    
    float temp[3];
    for (uint8_t i = 0; i < 3; i++){
        temp[i] = (hmc5883l_soft_iron[i][0] * data[0]) +
                  (hmc5883l_soft_iron[i][1] * data[1]) +
                  (hmc5883l_soft_iron[i][2] * data[2]);
    }
    for (uint8_t i = 0; i < 3; i++) data[i] = temp[i];
}

void hmc5883l_previous_raw_magetometer_readings(float *data){
    data[0] = hmc5883l_old_magenetometer_readings[0];
    data[1] = hmc5883l_old_magenetometer_readings[1];
    data[2] = hmc5883l_old_magenetometer_readings[2];
}