#include "./qmc5883l.h"
#include "../utils/math_constants.h"
#include "math.h"

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

float qmc5883l_old_magenetometer_readings[3] = {0, 0, 0};

enum t_interrupts {
    INTERRUPT_PIN_ENABLED  = 0b00000000,
    INTERRUPT_PIN_DISABLED = 0b00000001,
};

enum t_oversampling_ratio {
    OS_RATIO_64   = 0b11000000,
    OS_RATIO_128  = 0b10000000,
    OS_RATIO_256  = 0b01000000,
    OS_RATIO_512  = 0b00000000
};

enum t_mode_control {
    MODE_STANDBY    = 0b00000000,
    MODE_CONTINUOUS = 0b00000001,
};

enum t_output_data_rate {
    ODR_10HZ  = 0b00000000,
    ODR_50HZ  = 0b00000100,
    ODR_100HZ = 0b00001000,
    ODR_200HZ = 0b00001100,
};

enum t_measure_scale {
    MEASURE_SCALE_2G = 0b00000000,
    MEASURE_SCALE_8G = 0b00010000,
};

// max value output is at 200 Hz
uint8_t init_qmc5883l(I2C_HandleTypeDef *i2c_handle_temp, uint8_t apply_calibration, const float hard_iron[3], const float soft_iron[3][3]){
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
        5
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
        5
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
        5
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
        5
    );

    // First is least significant and second is most significant
    int16_t X = ((int16_t)retrieved_data[1] << 8) | (int16_t)retrieved_data[0];
    int16_t Y = ((int16_t)retrieved_data[3] << 8) | (int16_t)retrieved_data[2];
    int16_t Z = ((int16_t)retrieved_data[5] << 8) | (int16_t)retrieved_data[4];
    
    // Convert the mag's adc value to gauss
    // ADC accuracy is 2 MilliGauss per 1 step
    // Divide by 20 to do micro teslas
    data[0] = (float)X * 0.2f;
    data[1] = (float)Y * 0.2f;
    data[2] = (float)Z * 0.2f;

    qmc5883l_old_magenetometer_readings[0] = data[0];
    qmc5883l_old_magenetometer_readings[1] = data[1];
    qmc5883l_old_magenetometer_readings[2] = data[2];

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

void calculate_yaw_using_magnetometer_data(float *magnetometer_data, float *yaw_output, float yaw_offset) {
    float x = magnetometer_data[0];
    float y = magnetometer_data[1];

    // Rotation around the z-axis (simple yaw calculation)
    *yaw_output = atan2f(y, x) * M_RAD_TO_DEG;

    *yaw_output += yaw_offset;

    // Convert yaw to [0, 360] range
    if (*yaw_output < 0) {
        *yaw_output += 360;
    }
}


void calculate_yaw_tilt_compensated_using_magnetometer_data(float *magnetometer_data, float *yaw_output, float roll, float pitch, float yaw_offset){

    // You better make sure yur roll and pitch are in good shape as well as the magnetometer being aligned with device front when north facing
    // Otherwise the data you will get will be trash.

    float roll_radians = roll * M_DEG_TO_RAD;  // Convert roll from degrees to radians
    float pitch_radians = pitch * M_DEG_TO_RAD;  // Convert pitch from degrees to radians

    float mx = magnetometer_data[0];
    float my = magnetometer_data[1];
    float mz = magnetometer_data[2];

    // Corrected tilt compensation
    // float Mx = mx * cos(pitch_radians) - my * sin(roll_radians) * sin(pitch_radians) + mz * cos(roll_radians) * sin(pitch_radians);
    // float My = my * cos(roll_radians) + mz * sin(roll_radians);
    // *yaw_output = atan2(My, Mx) * M_RAD_TO_DEG; // 'My' cannot be inverted here

    float Xh = mx * cos(pitch_radians) + mz * sin(pitch_radians);
    float Yh = mx * sin(roll_radians) * sin(pitch_radians) + my * cos(roll_radians) - mz * sin(roll_radians) * cos(pitch_radians);
    *yaw_output = atan2(Yh, Xh) * M_RAD_TO_DEG;

    *yaw_output += yaw_offset;

    // Convert yaw to [0, 360] range
    while (*yaw_output >= 360) *yaw_output -= 360;
    while (*yaw_output < 0) *yaw_output += 360;
}

void calculate_yaw_tilt_compensated_using_magnetometer_data_virtual(
    float *magnetometer_data, 
    float *yaw_output, 
    float roll, 
    float pitch, 
    float yaw_offset_degrees,      // final yaw offset to apply
    float virtual_yaw_rotation_degrees // virtual yaw rotation applied before calculation
) {
    // Convert degrees to radians
    float roll_radians = roll * M_DEG_TO_RAD;
    float pitch_radians = pitch * M_DEG_TO_RAD;
    float virtual_yaw_rotation = virtual_yaw_rotation_degrees * M_DEG_TO_RAD;

    float mx = magnetometer_data[0];
    float my = magnetometer_data[1];
    float mz = magnetometer_data[2];

    // Apply virtual yaw rotation (+virtual_yaw_rotation) to magnetometer data
    float cos_yaw = cosf(virtual_yaw_rotation);
    float sin_yaw = sinf(virtual_yaw_rotation);
    float mx_rot = mx * cos_yaw - my * sin_yaw;
    float my_rot = mx * sin_yaw + my * cos_yaw;
    float mz_rot = mz;

    // Tilt compensation calculation
    float Xh = mx_rot * cosf(pitch_radians) + mz_rot * sinf(pitch_radians);
    float Yh = mx_rot * sinf(roll_radians) * sinf(pitch_radians)
             + my_rot * cosf(roll_radians)
             - mz_rot * sinf(roll_radians) * cosf(pitch_radians);

    // Calculate yaw in degrees
    float yaw = atan2f(Yh, Xh) * M_RAD_TO_DEG;

    // Subtract virtual yaw rotation to "rotate back"
    yaw -= virtual_yaw_rotation_degrees;

    // Add the final yaw offset calibration
    yaw += yaw_offset_degrees;

    // Normalize yaw to [0, 360)
    while (yaw < 0) yaw += 360;
    while (yaw >= 360) yaw -= 360;

    *yaw_output = yaw;
}

void rotate_magnetometer_data_yaw(float *mag_data, float yaw_degrees) {
    float yaw_radians = yaw_degrees * (M_PI / 180.0f);
    float cos_yaw = cosf(yaw_radians);
    float sin_yaw = sinf(yaw_radians);

    float mx = mag_data[0];
    float my = mag_data[1];
    // mz remains unchanged when rotating about Z axis
    float mz = mag_data[2];

    float mx_rot = mx * cos_yaw - my * sin_yaw;
    float my_rot = mx * sin_yaw + my * cos_yaw;

    mag_data[0] = mx_rot;
    mag_data[1] = my_rot;
    mag_data[2] = mz;
}


void rotate_magnetometer_output_90_degrees_anti_clockwise(float *magnetometer_data){
    float Mx = magnetometer_data[0];  // Original x-axis reading
    float My = magnetometer_data[1];  // Original y-axis reading
    float Mz = magnetometer_data[2];  // Original z-axis reading

    // Rotate axes to align x with the forward direction (pitch)
    magnetometer_data[0] = My;   // New x-axis (forward) is the original y-axis
    magnetometer_data[1] = -Mx;  // New y-axis (right side) is the negative of the original x-axis
    magnetometer_data[2] = Mz;   // z-axis remains the same
}

void rotate_magnetometer_output(float *mag_data, float angle_deg) {
    float angle_rad = angle_deg * (M_PI / 180.0f);

    float mx = mag_data[0];
    float my = mag_data[1];
    float mz = mag_data[2]; // unchanged

    // Rotate around Z (yaw axis)
    mag_data[0] = mx * cosf(angle_rad) - my * sinf(angle_rad); // new x
    mag_data[1] = mx * sinf(angle_rad) + my * cosf(angle_rad); // new y
    mag_data[2] = mz; // unchanged
}

// Build rotation matrix from roll, pitch, yaw (degrees)
void build_rotation_matrix(float roll_deg, float pitch_deg, float yaw_deg, float R[3][3]) {
    float roll = roll_deg * M_PI / 180.0f;
    float pitch = pitch_deg * M_PI / 180.0f;
    float yaw = yaw_deg * M_PI / 180.0f;

    float cr = cosf(roll), sr = sinf(roll);
    float cp = cosf(pitch), sp = sinf(pitch);
    float cy = cosf(yaw), sy = sinf(yaw);

    R[0][0] = cp * cy;
    R[0][1] = cp * sy;
    R[0][2] = -sp;

    R[1][0] = sr * sp * cy - cr * sy;
    R[1][1] = sr * sp * sy + cr * cy;
    R[1][2] = sr * cp;

    R[2][0] = cr * sp * cy + sr * sy;
    R[2][1] = cr * sp * sy - sr * cy;
    R[2][2] = cr * cp;
}

// Rotate a magnetometer vector by rotation matrix R
void rotate_magnetometer_inplace(const float R[3][3], float mag[3]) {
    float temp[3] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 3; i++) {
        temp[i] = 0.0f;
        for (int j = 0; j < 3; j++) {
            temp[i] += R[i][j] * mag[j];
        }
    }
    // Copy back rotated values to original array
    for (int i = 0; i < 3; i++) {
        mag[i] = temp[i];
    }
}


void rotate_magnetometer_vector(float *mag, float roll_deg, float pitch_deg, float yaw_deg) {
    // Convert degrees to radians
    float roll = roll_deg * (float)M_PI / 180.0f;
    float pitch = pitch_deg * (float)M_PI / 180.0f;
    float yaw = yaw_deg * (float)M_PI / 180.0f;

    // Precompute sines and cosines
    float sr = sinf(roll);
    float cr = cosf(roll);
    float sp = sinf(pitch);
    float cp = cosf(pitch);
    float sy = sinf(yaw);
    float cy = cosf(yaw);

    // Original magnetometer vector
    float mx = mag[0];
    float my = mag[1];
    float mz = mag[2];

    // Apply rotation: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    float x1 = cy * cp * mx + (cy * sp * sr - sy * cr) * my + (cy * sp * cr + sy * sr) * mz;
    float y1 = sy * cp * mx + (sy * sp * sr + cy * cr) * my + (sy * sp * cr - cy * sr) * mz;
    float z1 = -sp * mx     + cp * sr * my                   + cp * cr * mz;

    // Overwrite the original vector with rotated version
    mag[0] = x1;
    mag[1] = y1;
    mag[2] = z1;
}

void qmc5883l_previous_raw_magetometer_readings(float *data){
    data[0] = qmc5883l_old_magenetometer_readings[0];
    data[1] = qmc5883l_old_magenetometer_readings[1];
    data[2] = qmc5883l_old_magenetometer_readings[2];
}

// Function to rotate magnetometer data by 90 degrees anti-clockwise
void rotate_magnetometer_data_90(float data[3]) {
    float Mx = data[0]; // Original x-axis reading
    float My = data[1]; // Original y-axis reading
    float Mz = data[2]; // Original z-axis reading

    // Rotate axes to align x with the forward direction (pitch)
    data[0] = My;  // New x-axis (forward) is the original y-axis
    data[1] = -Mx; // New y-axis (right side) is the negative of the original x-axis
    data[2] = Mz;  // z-axis remains the same
}

// 180° rotation (about Z axis)
// x' = -x; y' = -y; z' = z
void rotate_magnetometer_data_180(float data[3]) {
    float Mx = data[0];
    float My = data[1];
    float Mz = data[2];

    data[0] = -Mx;
    data[1] = -My;
    data[2] = Mz;
}

// 270° counter-clockwise rotation (about Z axis)
// x' = -y; y' = x; z' = z
void rotate_magnetometer_data_270(float data[3]) {
    float Mx = data[0];
    float My = data[1];
    float Mz = data[2];

    data[0] = -My;
    data[1] = Mx;
    data[2] = Mz;
}

float calculate_mag_offset_using_compass_rpm(float average_rpm, const float* rpm_values, const float* offset_values, uint8_t length) {
    if (length == 0 || rpm_values == NULL || offset_values == NULL) {
        return 0.0f; // safety check
    }

    // Below lowest RPM sample threshold: return zero offset
    if (average_rpm <= rpm_values[0]) {
        return 0.0f;
    }

    // Above highest RPM sample: return max offset
    if (average_rpm >= rpm_values[length - 1]) {
        return offset_values[length - 1];
    }

    // Linear search for interval where average_rpm fits
    for (size_t i = 1; i < length; i++) {
        if (average_rpm < rpm_values[i]) {
            // Interpolate between i-1 and i
            float x0 = rpm_values[i - 1];
            float x1 = rpm_values[i];
            float y0 = offset_values[i - 1];
            float y1 = offset_values[i];

            float t = (average_rpm - x0) / (x1 - x0);
            return y0 + t * (y1 - y0);
        }
    }

    // Should not reach here, but return last offset just in case
    return offset_values[length - 1];
}