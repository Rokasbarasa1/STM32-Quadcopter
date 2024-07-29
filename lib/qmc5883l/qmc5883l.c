#include "./qmc5883l.h"

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

#define QMC5883L_I2C_ID (0x0D << 1)
#define ID_REG 0x0D
#define ID_VALUE 0b11111111
#define CONTROL1_REG 0x09
#define CONTROL2_REG 0x0A

#define OUTPUT_DATA1_REG 0x00

I2C_HandleTypeDef *i2c_handle;

// Storage of hard iron correction, values should be replaced by what is passed
volatile float m_hard_iron[3] = {
    0, 0, 0
};

// Storage of soft iron correction, values should be replaced by what is passed
volatile float m_soft_iron[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
};

// max value output is at 200 Hz
uint8_t init_qmc5883l(I2C_HandleTypeDef *i2c_handle_temp, uint8_t apply_calibration, const float hard_iron[3], const float soft_iron[3][3])
{
    i2c_handle = i2c_handle_temp;

    // assign the correction for irons
    if (apply_calibration){
        for (uint8_t i = 0; i < 3; i++){
            m_hard_iron[i] = hard_iron[i];
        }

        for (uint8_t i = 0; i < 3; i++){
            for (uint8_t k = 0; k < 3; k++){
                m_soft_iron[i][k] = soft_iron[i][k];
            }
        }
    }

    // Test the sensor by reading it's id register
    uint8_t check;
    HAL_I2C_Mem_Read(
        i2c_handle,
        QMC5883L_I2C_ID + 1,
        ID_REG,
        1,
        &check,
        1,
        100
    );

    // Check if the id value is as it should be
    if (check != ID_VALUE){
        printf("QMC5883L initialization failed\n");
        return 0;
    }

    // Disable interrupts
    uint8_t settings2 = 0b00000000;
    settings2 |= INTERRUPT_PIN_DISABLED;

    HAL_I2C_Mem_Write(
        i2c_handle,
        QMC5883L_I2C_ID,
        CONTROL2_REG,
        1,
        &settings2,
        1,
        100
    );

    // Set some essential settings that control the data being outputted
    uint8_t settings1 = 0b00000000;
    settings1 |= OS_RATIO_512;
    settings1 |= MEASURE_SCALE_2G;
    settings1 |= ODR_50HZ;
    settings1 |= MODE_CONTINUOUS;

    HAL_I2C_Mem_Write(
        i2c_handle,
        QMC5883L_I2C_ID,
        CONTROL1_REG,
        1,
        &settings1,
        1,
        100
    );

    printf("QMC5883L initialized\n");

    return 1;
}

void qmc5883l_magnetometer_readings_micro_teslas(float *data){
    uint8_t retrieved_data[] = {0, 0, 0, 0, 0, 0};

    HAL_I2C_Mem_Read(
        i2c_handle,
        QMC5883L_I2C_ID + 1,
        OUTPUT_DATA1_REG,
        1,
        retrieved_data,
        6, // read six registers in total so from 
        100
    );

    // First is least significant and second is most significant
    int16_t X = ((int16_t)retrieved_data[1] << 8) | (int16_t)retrieved_data[0];
    int16_t Y = ((int16_t)retrieved_data[3] << 8) | (int16_t)retrieved_data[2];
    int16_t Z = ((int16_t)retrieved_data[5] << 8) | (int16_t)retrieved_data[4];
    
    // Convert the mag's adc value to gauss
    // ADC accuracy is 2 MilliGauss per 1 step
    // Divide by 20 to do micro teslas
    data[0] = (float)X / 20;
    data[1] = (float)Y / 20;
    data[2] = (float)Z / 20;

    // Use the soft and hard iron calibrations
    for (uint8_t i = 0; i < 3; i++){
        data[i] = data[i] - m_hard_iron[i];
    }

    for (uint8_t i = 0; i < 3; i++){
        data[i] = (m_soft_iron[i][0] * data[0]) +
                  (m_soft_iron[i][1] * data[1]) +
                  (m_soft_iron[i][2] * data[2]);
    }
}

void calculate_yaw(float *magnetometer_data, float *yaw){
    float x = magnetometer_data[0];
    float y = magnetometer_data[1];
    float z = magnetometer_data[2];

    // rotation around the z axis
    *yaw = atan2f(y, x) * (180 / M_PI);

    // Convert yaw to [0, 360] range
    if (*yaw > 180) {
        *yaw -= 360;
    }
}


void calculate_yaw_tilt_compensated(float *magnetometer_data, float *yaw, float gyro_x_axis_rotation_degrees, float gyro_y_axis_rotation_degrees){
    float roll = gyro_x_axis_rotation_degrees * (M_PI / 180);  //  Convert roll from degrees to radians
    float pitch = gyro_y_axis_rotation_degrees * (M_PI / 180);  // Convert pitch from degrees to radians

    float mx = magnetometer_data[0];
    float my = magnetometer_data[1];
    float mz = magnetometer_data[2];

    float Xc = mx * cos(pitch) + mz * sin(pitch);
    float Yc = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch);

    *yaw = atan2(Yc, Xc) * (180 / M_PI);
    if (*yaw > 180) {
        *yaw -= 360;
    }
}