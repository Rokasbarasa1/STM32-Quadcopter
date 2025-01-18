#include "./mpu6050.h"

#include "../complementary_filter/complementary_filter.h"
#include "../utils/math_constants.h"

#define MPU6050 (0b1101000 << 1)
#define I2C_MASTER_TIMEOUT_MS 1000

#define ID_REG 0x75
#define ID_VALUE 104

#define PWR_MGMT_REG 0x6B
#define ACCEL_XOUT_H_REG 0x3B
#define GYRO_XOUT_H_REG 0x43

#define REG_ACCEL_CONFIG 0x1C
#define REG_GYRO_CONFIG 0x1B
#define REG_CONFIG 0x1A


#define GYRO_VALUE_ACCURACY (1.0f / 131.0f)
#define ACCELEROMETER_VALUE_ACCURACY (1.0f / 16384.0f)

enum t_mpu6050_power_management {
    PWR_RESET    = 0b10000000,
    PWR_SLEEP    = 0b01000000,
    PWR_CYCLE    = 0b00100000,
    PWR_TEMP_DIS = 0b00001000,
    PWR_CLOCK_INTERNAL_8MHZ = 0b00000000,
    PWR_CLOCK_INTERNAL_STOP = 0b00000111,
};

volatile float m_accelerometer_correction[3] = {
    0, 0, 0
};

volatile float m_gyro_correction[3] = {
    0, 0, 0
};

volatile float m_accelerometer_scale_factor_correction[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
};


volatile int64_t m_previous_time_roll_pitch = 0;
volatile int64_t m_previous_time_yaw = 0;
volatile float m_complementary_ratio = 0.0;

I2C_HandleTypeDef *i2c_handle;

// enum t_mpu6050_accel_config accelerometer_range, enum t_mpu6050_gyro_config gyro_range, enum t_mpu6050_low_pass_filter low_pass_setting
uint8_t init_mpu6050(I2C_HandleTypeDef *i2c_handle_temp, enum t_mpu6050_accel_config accelerometer_range, enum t_mpu6050_gyro_config gyro_range, enum t_mpu6050_low_pass_filter low_pass_setting, uint8_t apply_calibration, float accelerometer_scale_factor_correction[3][3], float accelerometer_correction[3], float gyro_correction[3], float refresh_rate_hz, float complementary_ratio, float complementary_beta){
    i2c_handle = i2c_handle_temp;

    if (apply_calibration){
        // assign the correction for gyro
        for (uint8_t i = 0; i < 3; i++){
            m_gyro_correction[i] = gyro_correction[i];
        }

        // assign the correction for accelerometer
        for (uint8_t i = 0; i < 3; i++){
            m_accelerometer_correction[i] = accelerometer_correction[i];
        }

        for (uint8_t i = 0; i < 3; i++){
            for (uint8_t k = 0; k < 3; k++){
                m_accelerometer_scale_factor_correction[i][k] = accelerometer_scale_factor_correction[i][k];
            }
        }
    }


    uint8_t check;
    HAL_StatusTypeDef response = HAL_I2C_Mem_Read(
        i2c_handle, 
        MPU6050 + 1, 
        ID_REG, 1, 
        &check, 
        1, 
        5);
    if (check != ID_VALUE){
        printf("MPU6050 initialization failed %d\n", response);
        return 0;
    }

    uint8_t data = 0b00000000;
    data |= PWR_RESET;
    data |= PWR_TEMP_DIS;
    data |= PWR_CLOCK_INTERNAL_8MHZ;
    HAL_I2C_Mem_Write(
        i2c_handle, 
        MPU6050, 
        PWR_MGMT_REG, 
        1, 
        &data, 
        1, 
        5
    );
    HAL_Delay(100);

    data = 0b00000000;
    data |= PWR_CLOCK_INTERNAL_STOP;
    HAL_I2C_Mem_Write(
        i2c_handle, 
        MPU6050, 
        PWR_MGMT_REG, 
        1, 
        &data, 
        1, 
        5
    );
    HAL_Delay(100);

    data = 0b00000000;
    data |= PWR_TEMP_DIS;
    data |= PWR_CLOCK_INTERNAL_8MHZ;
    HAL_I2C_Mem_Write(
        i2c_handle, 
        MPU6050, 
        PWR_MGMT_REG, 
        1, 
        &data, 
        1, 
        5
    );

    data = 0b00000000;
    data |= accelerometer_range;
    HAL_I2C_Mem_Write(
        i2c_handle, 
        MPU6050, 
        REG_ACCEL_CONFIG, 
        1, 
        &data, 
        1, 
        5
    );

    data = 0b00000000;
    data |= gyro_range;
    HAL_I2C_Mem_Write(
        i2c_handle, 
        MPU6050, 
        REG_GYRO_CONFIG, 
        1, 
        &data, 
        1, 
        5
    );

    data = 0b00000000;
    data |= low_pass_setting;
    HAL_I2C_Mem_Write(
        i2c_handle, 
        MPU6050, 
        REG_CONFIG, 
        1, 
        &data, 
        1, 
        5
    );

    m_complementary_ratio = complementary_ratio;
    // pitch_2_complementary = init_second_order_coplementary_filter(complementary_ratio, complementary_beta);
    // roll_2_complementary = init_second_order_coplementary_filter(complementary_ratio, complementary_beta);
    // yaw_2_complementary = init_second_order_coplementary_filter(0.05, complementary_beta);

    printf("MPU6050 initialized\n");
    return 1;
}

// Read accelerometer in gravity units
void mpu6050_get_accelerometer_readings_gravity(float *data){
    uint8_t retrieved_data[] = {0, 0, 0, 0, 0, 0};

    HAL_I2C_Mem_Read(
        i2c_handle,
        MPU6050 + 1,
        ACCEL_XOUT_H_REG,
        1,
        retrieved_data,
        6,  // Read all of the accelerometer registers
        5);

    int16_t X = ((int16_t)retrieved_data[0] << 8) | (int16_t)retrieved_data[1];
    int16_t Y = ((int16_t)retrieved_data[2] << 8) | (int16_t)retrieved_data[3];
    int16_t Z = ((int16_t)retrieved_data[4] << 8) | (int16_t)retrieved_data[5];

    float X_out = (float)X * ACCELEROMETER_VALUE_ACCURACY;
    float Y_out = (float)Y * ACCELEROMETER_VALUE_ACCURACY;
    float Z_out = (float)Z * ACCELEROMETER_VALUE_ACCURACY;

    data[0] = X_out - (m_accelerometer_correction[0]);
    data[1] = Y_out - (m_accelerometer_correction[1]);
    data[2] = Z_out - (m_accelerometer_correction[2] - 1);

    for (uint8_t i = 0; i < 3; i++){
        data[i] = (m_accelerometer_scale_factor_correction[i][0] * data[0]) +
                  (m_accelerometer_scale_factor_correction[i][1] * data[1]) +
                  (m_accelerometer_scale_factor_correction[i][2] * data[2]);
    }
}

// Read gyro in degrees per second units 
void mpu6050_get_gyro_readings_dps(float *data){
    uint8_t retrieved_data[] = {0, 0, 0, 0, 0, 0};

    volatile HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        i2c_handle,
        MPU6050 + 1,
        GYRO_XOUT_H_REG,
        1,
        retrieved_data,
        6,  // Read all of the gyroscope registers
        5);

    int16_t X = ((int16_t)retrieved_data[0] << 8) | (int16_t)retrieved_data[1];
    int16_t Y = ((int16_t)retrieved_data[2] << 8) | (int16_t)retrieved_data[3];
    int16_t Z = ((int16_t)retrieved_data[4] << 8) | (int16_t)retrieved_data[5];

    float X_out = (float)X * GYRO_VALUE_ACCURACY;
    float Y_out = (float)Y * GYRO_VALUE_ACCURACY;
    float Z_out = (float)Z * GYRO_VALUE_ACCURACY;

    data[0] = X_out - (m_gyro_correction[0]);
    data[1] = Y_out - (m_gyro_correction[1]);
    data[2] = Z_out - (m_gyro_correction[2]);
}

void calculate_roll_pitch_from_accelerometer_data(float *data, float *accelerometer_roll, float *accelerometer_pitch, float roll_offset, float pitch_offset){
    float x = data[0];
    float y = data[1];
    float z = data[2];

    *accelerometer_roll = atan2f(y, sqrtf(x * x + z * z)) * M_180_DIV_BY_PI + roll_offset;
    *accelerometer_pitch = -(atan2f(x, sqrtf(y * y + z * z)) * M_180_DIV_BY_PI) + pitch_offset;
}

// Get many values of the accelerometer error and average them together. Then print out the result
void find_accelerometer_error(uint64_t sample_size){
    float x_sum = 0, y_sum = 0, z_sum = 0;
    float data[] = {0, 0, 0};

    float accelerometer_correction_temp[3];
    float gyro_correction_temp[3];

    // Copy and delete the original corrections
    for (uint8_t i = 0; i < 3; i++){
        gyro_correction_temp[i] = m_gyro_correction[i];
        m_gyro_correction[i] = 0;

        accelerometer_correction_temp[i] = m_accelerometer_correction[i];
        m_accelerometer_correction[i] = 0;
    }
    m_accelerometer_correction[2] = 1;


    for (uint64_t i = 0; i < sample_size; i++){
        __disable_irq();
        mpu6050_get_accelerometer_readings_gravity(data);
        __enable_irq();

        x_sum += data[0];
        y_sum += data[1];
        z_sum += data[2];
        // It can sample stuff at 1KHz
        // but 0.5Khz is just to be safe
        HAL_Delay(2);
    }

    // Reassign the corrections 
    for (uint8_t i = 0; i < 3; i++){
        m_gyro_correction[i] = gyro_correction_temp[i];
        m_accelerometer_correction[i] = accelerometer_correction_temp[i];
    }

    printf(
        "ACCELEROMETER errors:  X,Y,Z  %f, %f, %f\n",
        x_sum / sample_size,
        y_sum / sample_size,
        z_sum / sample_size );
}

void find_accelerometer_error_with_corrections(uint64_t sample_size){
    float x_sum = 0, y_sum = 0, z_sum = 0;
    float data[] = {0, 0, 0};

    for (uint64_t i = 0; i < sample_size; i++){
        __disable_irq();
        mpu6050_get_accelerometer_readings_gravity(data);
        __enable_irq();

        x_sum += data[0];
        y_sum += data[1];
        z_sum += data[2];
        // It can sample stuff at 1KHz
        // but 0.5Khz is just to be safe
        HAL_Delay(2);
    }

    printf(
        "ACCELEROMETER errors:  X,Y,Z  %f, %f, %f\n",
        x_sum / sample_size,
        y_sum / sample_size,
        z_sum / sample_size);
}

// Get many values of the gyro error and average them together. Then print out the result
void find_gyro_error(uint64_t sample_size){

    float x_sum = 0, y_sum = 0, z_sum = 0;
    float data[] = {0, 0, 0};

    float accelerometer_correction_temp[3];
    float gyro_correction_temp[3];

    // Copy and delete the original corrections
    for (uint8_t i = 0; i < 3; i++){
        gyro_correction_temp[i] = m_gyro_correction[i];
        m_gyro_correction[i] = 0;

        accelerometer_correction_temp[i] = m_accelerometer_correction[i];
        m_accelerometer_correction[i] = 0;
    }

    for (uint64_t i = 0; i < sample_size; i++){
        __disable_irq();
        mpu6050_get_gyro_readings_dps(data);
        __enable_irq();

        x_sum += data[0];
        y_sum += data[1];
        z_sum += data[2];
        // It can sample stuff at 1KHz
        // but 0.5Khz is just to be safe
        HAL_Delay(2);
    }

    // Reassign the corrections 
    for (uint8_t i = 0; i < 3; i++){
        m_gyro_correction[i] = gyro_correction_temp[i];
        m_accelerometer_correction[i] = accelerometer_correction_temp[i];
    }

    printf(
        "GYRO errors:  X,Y,Z  %f, %f, %f\n",
        x_sum / sample_size,
        y_sum / sample_size,
        z_sum / sample_size);
}

void find_and_return_gyro_error(uint64_t sample_size, float *return_array){

    float x_sum = 0, y_sum = 0, z_sum = 0;
    float data[] = {0, 0, 0};

    float gyro_correction_temp[3];

    // Copy and delete the original corrections
    for (uint8_t i = 0; i < 3; i++){
        gyro_correction_temp[i] = m_gyro_correction[i];
        m_gyro_correction[i] = 0;
    }

    for (uint64_t i = 0; i < sample_size; i++)    {
        __disable_irq();
        mpu6050_get_gyro_readings_dps(data);
        __enable_irq();

        x_sum += data[0];
        y_sum += data[1];
        z_sum += data[2];
        // It can sample stuff at 1KHz
        // but 0.5Khz is just to be safe
        HAL_Delay(1);
    }

    // Reassign the corrections 
    for (uint8_t i = 0; i < 3; i++){
        m_gyro_correction[i] = gyro_correction_temp[i];
    }

    printf(
        "GYRO errors:  X,Y,Z  %f, %f, %f\n",
        x_sum / sample_size,
        y_sum / sample_size,
        z_sum / sample_size);

    return_array[0] = x_sum / sample_size;
    return_array[1] = y_sum / sample_size;
    return_array[2] = z_sum / sample_size;
}

void mpu6050_apply_calibrations(float accelerometer_correction[3], float gyro_correction[3]){
    // assign the correction for gyro
    for (uint8_t i = 0; i < 3; i++){
        m_gyro_correction[i] = gyro_correction[i];
    }

    // assign the correction for accelerometer
    for (uint8_t i = 0; i < 3; i++){
        m_accelerometer_correction[i] = accelerometer_correction[i];
    }
}

void mpu6050_apply_calibrations_gyro(float gyro_correction[3]){
    // assign the correction for gyro
    for (uint8_t i = 0; i < 3; i++){
        m_gyro_correction[i] = gyro_correction[i];
    }
}

void mpu6050_apply_calibration_accelerometers(float accelerometer_correction[3]){
    // assign the correction for accelerometer
    for (uint8_t i = 0; i < 3; i++){
        m_accelerometer_correction[i] = accelerometer_correction[i];
    }
}


// Do complementary filter for x(pitch) and y(roll). Combine accelerometer and gyro to get a more usable gyro value. Please make sure the coefficient is scaled by refresh rate. It helps a lot.
void sensor_fusion_roll_pitch(float* gyro_angular, float accelerometer_roll, float accelerometer_pitch, int64_t time, uint8_t set_timestamp, float* imu_orientation){

    if(m_previous_time_roll_pitch == 0){
        m_previous_time_roll_pitch = time;
        return;
    }

    float elapsed_time_sec= (((float)time*CONVERT_MICROSECONDS_TO_SECONDS)-((float)m_previous_time_roll_pitch*CONVERT_MICROSECONDS_TO_SECONDS));
    if(set_timestamp == 1) m_previous_time_roll_pitch = time;

    // Convert degrees per second and add the complementary filter with accelerometer degrees
    imu_orientation[0] = complementary_filter_calculate(
        m_complementary_ratio,
        imu_orientation[0] + gyro_angular[0] * elapsed_time_sec,
        accelerometer_roll
    );
    imu_orientation[1] = complementary_filter_calculate(
        m_complementary_ratio, 
        imu_orientation[1] + gyro_angular[1] * elapsed_time_sec,
        accelerometer_pitch
    );

    // I dont want to track how many times the degrees went over the 360 degree mark, no point.
    while (imu_orientation[0] > 180.0) imu_orientation[0] -= 360.0;
    while (imu_orientation[0] < -180.0) imu_orientation[0] += 360.0;
    while (imu_orientation[1] > 180.0) imu_orientation[1] -= 360.0;
    while (imu_orientation[1] < -180.0) imu_orientation[1] += 360.0;
}

void sensor_fusion_yaw(float* gyro_angular, float magnetometer_yaw, int64_t time, uint8_t set_timestamp, float* imu_orientation){

    if(m_previous_time_yaw == 0){
        m_previous_time_yaw = time;
        return;
    }

    float elapsed_time_sec= (((float)time*CONVERT_MICROSECONDS_TO_SECONDS)-((float)m_previous_time_yaw*CONVERT_MICROSECONDS_TO_SECONDS));
    if(set_timestamp == 1) m_previous_time_yaw = time;

    imu_orientation[2] = complementary_filter_calculate(
        m_complementary_ratio, 
        imu_orientation[2] + gyro_angular[2] * elapsed_time_sec, 
        magnetometer_yaw
    );

    while (imu_orientation[2] >= 360.0) imu_orientation[2] -= 360.0;
    while (imu_orientation[2] < 0.0) imu_orientation[2] += 360.0;
}

// Find the shortest value between two angles. Range -180 to 180
float angle_difference(float a, float b) {

    /// How to understand this
    //  So -177 % 179 = 2
    //  2 - 180 = -178
    // -178 < -180 False
    // -178 returned

    // If it moves in the same direction over time it will be -178, -179, -180, 180, 181

    //  So 177 % -179 = -2
    //  - 2 - 180 = -182
    // -182 < -180 True
    // -182+360 = 178 returned

    // fmodf is modulus operator (%) for floats
    float diff = fmodf(b - a + 180, 360) - 180;
    return diff < -180 ? diff + 360 : diff;
}

uint64_t last_vertical_sample_time = 0;
#define G 9.81

float mpu6050_calculate_vertical_speed(float last_vertical_speed, float acceleration_data[3], float gyro_degrees[3], int64_t time) {
    if (last_vertical_sample_time == 0) {
        last_vertical_sample_time = time;
        return 0;
    }

    float elapsed_time_sec = ((float)time / 1000.0f) - ((float)last_vertical_sample_time / 1000.0f);
    last_vertical_sample_time = time;

    // printf("acceleration_data: %f, %f, %f\n", acceleration_data[0], acceleration_data[1], acceleration_data[2]);
    // printf("degrees: %f, %f, %f\n", gyro_degrees[0], gyro_degrees[1], gyro_degrees[2]);

    // Convert pitch and roll angles to radians
    float pitch_rad = gyro_degrees[0] * M_PI_DIV_BY_180;
    float roll_rad = gyro_degrees[1] * M_PI_DIV_BY_180;

    // Calculate the vertical component of the acceleration
    float a_x = acceleration_data[0] * G;
    float a_y = acceleration_data[1] * G;
    float a_z = acceleration_data[2] * G;

    // Correct vertical acceleration calculation considering quadcopter tilt
    float vertical_acceleration = (a_z * cos(pitch_rad) * cos(roll_rad)) + 
                                  (a_x * sin(pitch_rad)) -
                                  (a_y * sin(roll_rad));
    // printf("vertical_acceleration: %f\n", vertical_acceleration);

    // Remove the effect of gravity
    vertical_acceleration -= G * cos(pitch_rad) * cos(roll_rad);
    // printf("vertical_acceleration - G: %f\n", vertical_acceleration);

    // Calculate the vertical speed
    float vertical_speed = last_vertical_speed + (vertical_acceleration * elapsed_time_sec);

    return vertical_speed;
}

float mpu6050_calculate_vertical_acceleration_cm_per_second(float acceleration_data[3], float gyro_degrees[3]){

    // Convert pitch and roll angles to radians
    float pitch_rad = gyro_degrees[0] * M_PI_DIV_BY_180;
    float roll_rad = gyro_degrees[1] * M_PI_DIV_BY_180;

    // Calculate the vertical component of the acceleration
    float a_x = acceleration_data[0] * G;
    float a_y = acceleration_data[1] * G;
    float a_z = acceleration_data[2] * G;

    float vertical_acceleration = -a_x * sin(pitch_rad) + 
                                   a_y * sin(roll_rad) * cos(pitch_rad) +
                                   a_z * cos(roll_rad) * cos(pitch_rad);

    vertical_acceleration = (vertical_acceleration - G) * 100;

    return vertical_acceleration;
}

void mpu6050_set_complementary_ratio(float new_complementary_ratio){
    m_complementary_ratio = new_complementary_ratio;
}