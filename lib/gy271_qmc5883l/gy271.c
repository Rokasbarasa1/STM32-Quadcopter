#include "./gy271.h"

#define GY271 (0x0D << 1)

#define I2C_MASTER_NUM 0            /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)       \
    (byte & 0x80 ? '1' : '0'),     \
        (byte & 0x40 ? '1' : '0'), \
        (byte & 0x20 ? '1' : '0'), \
        (byte & 0x10 ? '1' : '0'), \
        (byte & 0x08 ? '1' : '0'), \
        (byte & 0x04 ? '1' : '0'), \
        (byte & 0x02 ? '1' : '0'), \
        (byte & 0x01 ? '1' : '0')

#ifndef M_PI // i guess not the right compiler ...
#define M_PI (3.14159265358979323846)
#endif

volatile float hard_iron_loc[3] = {
    0, 0, 0};

volatile float soft_iron_loc[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}};

I2C_HandleTypeDef *i2c_address;

// max value output is at 200 Hz
uint8_t init_gy271(I2C_HandleTypeDef *i2c_address_temp, uint8_t apply_calibration, float hard_iron[3], float soft_iron[3][3])
{
    HAL_StatusTypeDef ret;

    i2c_address = i2c_address_temp;

    if (apply_calibration)
    {
        // assign the correction for irons
        for (uint8_t i = 0; i < 3; i++)
        {
            hard_iron_loc[i] = hard_iron[i];
        }

        for (uint8_t i = 0; i < 3; i++)
        {
            for (uint8_t k = 0; k < 3; k++)
            {
                soft_iron_loc[i][k] = soft_iron[i][k];
            }
        }
    }

    // Test the sensor by reading it's address register
    uint8_t check;
    HAL_I2C_Mem_Read(i2c_address, GY271 + 1, 0x0D, 1, &check, 1, 100);

    if (check != 0b11111111)
    {
        printf("GY271 initialization failed\n");
        return 0;
    }
    // reset it
    uint8_t reset_device1[] = {0x0A, 0b00000001};
    HAL_I2C_Mem_Write(i2c_address, GY271, reset_device1[0], 1, &reset_device1[1], 1, 100);

    uint8_t oversampling = 0b00000000; // 512 over sample ratio
    uint8_t range = 0b00000000;        // 2 gauss
    uint8_t rate = 0b00000100;         // 50Hz sample rate
    uint8_t mode = 0b00000001;         // continuous mode

    uint8_t reset_device2[] = {0x09, oversampling | range | rate | mode};
    HAL_I2C_Mem_Write(i2c_address, GY271, reset_device2[0], 1, &reset_device2[1], 1, 100);

    printf("GY271 initialized\n");

    return 1;
}

void gy271_magnetometer_readings_micro_teslas(float *data)
{
    uint8_t data_register[] = {0x00};
    uint8_t retrieved_data[] = {0, 0, 0, 0, 0, 0};

    HAL_I2C_Mem_Read(
        i2c_address,
        GY271 + 1,
        0x00,
        1,
        retrieved_data,
        6,
        100);
    // i2c_master_write_read_device(
    //     I2C_MASTER_NUM,
    //     GY271,
    //     data_register,
    //     sizeof(data_register),
    //     retrieved_data,
    //     sizeof(retrieved_data),
    //     I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS
    // );

    int16_t X, Y, Z;

    // First is least significant and second is most significant
    X = ((int16_t)retrieved_data[1] << 8) | (int16_t)retrieved_data[0];
    Y = ((int16_t)retrieved_data[3] << 8) | (int16_t)retrieved_data[2];
    Z = ((int16_t)retrieved_data[5] << 8) | (int16_t)retrieved_data[4];

    // Convert the mag's adc value to gauss
    // ADC accuracy is 2 MilliGauss per 1 step
    // Divide by 20 to do micro teslas
    data[0] = (float)X / 20;
    data[1] = (float)Y / 20;
    data[2] = (float)Z / 20;

    // Use the soft and hard iron calibrations
    for (uint8_t i = 0; i < 3; i++)
    {
        data[i] = data[i] - hard_iron_loc[i];
    }

    for (uint8_t i = 0; i < 3; i++)
    {
        data[i] = (soft_iron_loc[i][0] * data[0]) +
                  (soft_iron_loc[i][1] * data[1]) +
                  (soft_iron_loc[i][2] * data[2]);
    }
}

void calculate_yaw(float *magnetometer_data, float *yaw)
{
    float x = magnetometer_data[0];
    float y = magnetometer_data[1];
    float z = magnetometer_data[2];

    float acc_vector_length = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
    x = x / acc_vector_length;
    y = y / acc_vector_length;
    z = z / acc_vector_length;

    // rotation around the x axis
    *yaw = atan2f(y, x) * (180 / M_PI);

    // rotation around the y axis
    // *pitch = asinf(x) * (180 / M_PI);
}