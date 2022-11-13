#include "./mpu6050.h"

#define MPU6050 (0b1101000 << 1)

// #define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
// #define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
// #define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
// #define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#ifndef M_PI // i guess not the right compiler ...
#define M_PI (3.14159265358979323846)
#endif


volatile float accelerometer_correction_loc[3] = {
    0,0,1
};

volatile float gyro_correction_loc[3] = {
    0,0,0
};

I2C_HandleTypeDef * i2c_address;

uint8_t init_mpu6050(I2C_HandleTypeDef *i2c_address_temp, uint8_t apply_calibration, float accelerometer_correction[3], float gyro_correction[3]){
    HAL_StatusTypeDef ret;

    i2c_address = i2c_address_temp;

    if(apply_calibration){
        // assign the correction for gyro
        for (uint8_t i = 0; i < 3; i++){
            gyro_correction_loc[i] = gyro_correction[i];
        }

        // assign the correction for accelerometer
        for (uint8_t i = 0; i < 3; i++){
            accelerometer_correction_loc[i] = accelerometer_correction[i];
        }
    }

    uint8_t check;
    HAL_I2C_Mem_Read(i2c_address, MPU6050+1, 0x75, 1, &check, 1, 100);

    if(check != 104){
        printf("MPU6050 initialization failed\n");
        return 0;
    }

    // reset it 
    uint8_t reset_device1[] = {0x6B, 0b10001000};
    HAL_I2C_Mem_Write(i2c_address, MPU6050, reset_device1[0], 1, &reset_device1[1], 1, 100);
    HAL_Delay(100);

    // uint8_t reset_device2[] = {0x6B, 0b00000111};
    uint8_t reset_device2[] = {0x6B, 0b00000111};
    HAL_I2C_Mem_Write(i2c_address, MPU6050, reset_device2[0], 1, &reset_device2[1], 1, 100);
    HAL_Delay(100);

    uint8_t reset_device3[] = {0x6B, 0b00001000};
    HAL_I2C_Mem_Write(i2c_address, MPU6050, reset_device3[0], 1, &reset_device3[1], 1, 100);


    printf("MPU6050 initialized\n");
    return 1;
}

void mpu6050_accelerometer_readings_float(float* data){
    HAL_StatusTypeDef ret;
    uint8_t data_register[] = {0x3B};
    uint8_t retrieved_data[] = {0,0,0,0,0,0};

    HAL_I2C_Mem_Read(
        i2c_address, 
        MPU6050+1, 
        0x3B, 
        1, 
        retrieved_data, 
        6, 
        100
    );

    int16_t X, Y, Z;
    float X_out, Y_out, Z_out;

    X = ((int16_t)retrieved_data[0] << 8) | (int16_t)retrieved_data[1];
    Y = ((int16_t)retrieved_data[2] << 8) | (int16_t)retrieved_data[3];
    Z = ((int16_t)retrieved_data[4] << 8) | (int16_t)retrieved_data[5];
    
    X_out = ((float) X) / 16384.0;
    Y_out = ((float) Y) / 16384.0;
    Z_out = ((float) Z) / 16384.0;

    data[0] = X_out - (accelerometer_correction_loc[0]);
    data[1] = Y_out - (accelerometer_correction_loc[1]);
    data[2] = Z_out - (accelerometer_correction_loc[2] - 1);
}

void mpu6050_gyro_readings_float(float* data){
    uint8_t data_register[] = {0x43};
    uint8_t retrieved_data[] = {0,0,0,0,0,0};

    HAL_I2C_Mem_Read(
        i2c_address, 
        MPU6050+1, 
        0x43, 
        1, 
        retrieved_data, 
        6, 
        100
    );

    int16_t X, Y, Z;
    float X_out, Y_out, Z_out;

    X = ((int16_t)retrieved_data[0] << 8) | (int16_t)retrieved_data[1];
    Y = ((int16_t)retrieved_data[2] << 8) | (int16_t)retrieved_data[3];
    Z = ((int16_t)retrieved_data[4] << 8) | (int16_t)retrieved_data[5];
    
    X_out = ((float) X) / 131.0;
    Y_out = ((float) Y) / 131.0;
    Z_out = ((float) Z) / 131.0;

    data[0] = X_out - (gyro_correction_loc[0]);// + 0.096947;
    data[1] = Y_out - (gyro_correction_loc[1]);// + 4.177492;
    data[2] = Z_out - (gyro_correction_loc[2]);// - 0.440870;

    // data[0] = X_out - (0.400450);// + 0.096947;
    // data[1] = Y_out - (-4.267702);// + 4.177492;
    // data[2] = Z_out - (0.505000);// - 0.440870;
}


void calculate_pitch_and_roll(float* data, float *roll, float *pitch){

    float x = data[0];
    float y = data[1];
    float z = data[2];

    float acc_vector_length = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
    x = x /acc_vector_length;
    y = y /acc_vector_length;
    z = z /acc_vector_length;

    // rotation around the x axis
    *roll = atan2f(y, z) * (180 / M_PI);

    // rotation around the y axis
    *pitch = asinf(x) * (180 / M_PI);
}

void find_accelerometer_error(uint64_t sample_size){

    float x_sum = 0, y_sum = 0, z_sum = 0;
    float data[] = {0,0,0};
    
    for(uint64_t i = 0; i < sample_size ; i++){
        mpu6050_accelerometer_readings_float(data);
        x_sum += data[0];
        y_sum += data[1];
        z_sum += data[2];
        // It can sample stuff at 1KHz
        // but 0.5Khz is just to be safe
        HAL_Delay(2);
    }

    printf(
        "ACCELEROMETER errors:  X,Y,Z  %f, %f, %f\n", 
        x_sum/sample_size, 
        y_sum/sample_size, 
        z_sum/sample_size
    );
}

void find_gyro_error(uint64_t sample_size){

    float x_sum = 0, y_sum = 0, z_sum = 0;
    float data[] = {0,0,0};
    
    for(uint64_t i = 0; i < sample_size ; i++){
        mpu6050_gyro_readings_float(data);

        x_sum += data[0];
        y_sum += data[1];
        z_sum += data[2];
        // It can sample stuff at 1KHz
        // but 0.5Khz is just to be safe
        HAL_Delay(2);
    }

    printf(
        "GYRO errors:  X,Y,Z  %f, %f, %f\n", 
        x_sum/sample_size, 
        y_sum/sample_size, 
        z_sum/sample_size
    );
}