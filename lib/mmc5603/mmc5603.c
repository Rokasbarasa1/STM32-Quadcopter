#include "./mmc5603.h"

#define MMC5603_I2C_ID 0b01100000
// #define MMC5603_I2C_ID (0x30 < 1)


#define ID_VALUE 0b00010000

#define REG_OUTPUT_XOUT0 0x00
#define REG_STATUS 0x18
#define REG_ODR 0x1A
#define REG_CONTROL0 0x1B
#define REG_CONTROL1 0x1C
#define REG_CONTROL2 0x1D

#define REG_SELF_TEST_X_THRESHOLD 0x1E
#define REG_SELF_TEST_Y_THRESHOLD 0x1F
#define REG_SELF_TEST_Z_THRESHOLD 0x20

#define REG_SELF_TEST_X 0x27
#define REG_SELF_TEST_Y 0x28
#define REG_SELF_TEST_Z 0x29
#define REG_ID 0x39

#define DECIMAL_TO_MILLI_GAUSS_RATIO 0.0625
#define DECIMAL_TO_MICRO_TESLA_RATIO 0.00625

I2C_HandleTypeDef *i2c_handle;


float mmc5603_old_magnetometer_readings[3] = {0, 0, 0};
float mmc5603_magnetometer_readings_without_rotation[3] = {0, 0, 0};

// Storage of hard iron correction, values should be replaced by what is passed
volatile float m_mmc5603_hard_iron[3] = {
    0, 0, 0
};

// Storage of soft iron correction, values should be replaced by what is passed
volatile float m_mmc5603_soft_iron[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
};

volatile float mmc5603_rotation_matrix[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
};


enum t_mmc6503_control_0 {
    MMC5603_CONTROL0_TAKE_MAGNETIC_MEASUREMENT                  = 0b00000001,
    MMC5603_CONTROL0_TAKE_TEMPERATURE_MEASUREMENT               = 0b00000010,
    MMC5603_CONTROL0_DO_SET                                     = 0b00001000,
    MMC5603_CONTROL0_DO_RESET                                   = 0b00010000,
    MMC5603_CONTROL0_AUTO_SET_RESET                             = 0b00100000,
    MMC5603_CONTROL0_AUTO_SELF_TEST                             = 0b01000000,
    MMC5603_CONTROL0_START_CALCULATE_MEASUREMENT_PERIOD_ODR     = 0b10000000
};

enum t_mmc6503_control_1 {
    MMC5603_CONTROL1_BANDWIDTH_6_6ms                = 0b00000000,
    MMC5603_CONTROL1_BANDWIDTH_3_5ms                = 0b00000001,
    MMC5603_CONTROL1_BANDWIDTH_2_0ms                = 0b00000010,
    MMC5603_CONTROL1_BANDWIDTH_1_2ms                = 0b00000011,
    MMC5603_CONTROL1_DISABLE_X                      = 0b00000100,
    MMC5603_CONTROL1_DISABLE_Y                      = 0b00001000,
    MMC5603_CONTROL1_DISABLE_Z                      = 0b00010000,
    MMC5603_CONTROL1_SELF_TEST_CURRENT_ON           = 0b00100000,
    MMC5603_CONTROL1_SELF_TEST_CURRENT_ON_OPPOSITE  = 0b01000000,
    MMC5603_CONTROL1_SOFTWARE_RESET                 = 0b10000000
};

enum t_mmc6503_control_2 {
    MMC5603_CONTROL2_EXECUTE_SET_AFTER_1_SAMPLE                 = 0b00000000,
    MMC5603_CONTROL2_EXECUTE_SET_AFTER_25_SAMPLE                = 0b00000001,
    MMC5603_CONTROL2_EXECUTE_SET_AFTER_75_SAMPLE                = 0b00000010,
    MMC5603_CONTROL2_EXECUTE_SET_AFTER_100_SAMPLE               = 0b00000011,
    MMC5603_CONTROL2_EXECUTE_SET_AFTER_250_SAMPLE               = 0b00000100,
    MMC5603_CONTROL2_EXECUTE_SET_AFTER_500_SAMPLE               = 0b00000101,
    MMC5603_CONTROL2_EXECUTE_SET_AFTER_1000_SAMPLE              = 0b00000110,
    MMC5603_CONTROL2_EXECUTE_SET_AFTER_2000_SAMPLE              = 0b00000111,
    MMC5603_CONTROL2_ENTER_ENABLE_PERIODICAL_SET                = 0b00001000,
    MMC5603_CONTROL2_ENTER_CONTINUOUS_MODE                      = 0b00010000,
    MMC5603_CONTROL2_ENABLE_HIGH_ODR_POWER                      = 0b10000000
};

enum t_mmc6503_status_register_bits {
    MMC5603_STATUS_REGISTER_OTP_READ_DONE                  = 0b00010000,
    MMC5603_STATUS_REGISTER_SELF_TEST_SIGNAL               = 0b00100000,
    MMC5603_STATUS_REGISTER_MEASUREMENT_MAGNETIC_DONE      = 0b01000000,
    MMC5603_STATUS_REGISTER_MEASUREMENT_TEMPERATURE_DONE   = 0b10000000
};

volatile uint8_t m_use_continuos_mode;
volatile uint8_t m_use_automatic_set_reset;

// Ths function helps to not overwrite old register bits
uint8_t mmc5603_get_register_value(uint8_t register_address){
    uint8_t register_value;
    HAL_I2C_Mem_Read(
        i2c_handle,
        MMC5603_I2C_ID,
        register_address,
        1,
        &register_value,
        1,
        5
    );

    return register_value;
}

uint8_t mmc5603_init(
    I2C_HandleTypeDef *i2c_handle_temp, 
    uint8_t apply_calibration, 
    const float hard_iron[3], 
    const float soft_iron[3][3],
    uint8_t use_continuos_mode,
    uint16_t refresh_rate,
    uint8_t use_automatic_set_reset
){
    m_use_automatic_set_reset = use_automatic_set_reset;
    m_use_continuos_mode = use_continuos_mode;
    i2c_handle = i2c_handle_temp;

    uint16_t enable_high_performance = 0;
    uint8_t desired_refresh_rate;
    if(refresh_rate > 255 && refresh_rate != 1000){
        desired_refresh_rate = 255;
    }else if(refresh_rate == 1000){
        desired_refresh_rate = 255;

        // It is only able to reach 1000Hz if auto set/reset is disabled
        if(!use_automatic_set_reset){
            enable_high_performance = 1;
        }
    }else{
        desired_refresh_rate = refresh_rate;
    }


    // assign the correction for irons
    if (apply_calibration){
        for (uint8_t i = 0; i < 3; i++){
            m_mmc5603_hard_iron[i] = hard_iron[i];
        }

        for (uint8_t i = 0; i < 3; i++){
            for (uint8_t k = 0; k < 3; k++){
                m_mmc5603_soft_iron[i][k] = soft_iron[i][k];
            }
        }
    }

    // Test the sensor by reading it's id register
    uint8_t check;
    HAL_I2C_Mem_Read(
        i2c_handle,
        MMC5603_I2C_ID,
        REG_ID,
        1,
        &check,
        1,
        5
    );

    // Check if the id value is as it should be
    if (check != ID_VALUE){
        printf("MMC5603 initialization failed: failed to get device id\n");
        return 0;
    }

    if(!mmc5603_perform_self_test()){
        printf("MMC5603 initialization failed: self test failed\n");
        return 0;
    }

    uint8_t data = 0b00000000;
    data |= MMC5603_CONTROL1_SOFTWARE_RESET;
    HAL_I2C_Mem_Write(
        i2c_handle,
        MMC5603_I2C_ID,
        REG_CONTROL1,
        1,
        &data,
        1,
        5
    );

    // Set bandwidth to the fastest possible
    data =  mmc5603_get_register_value(REG_CONTROL1);;
    data |= MMC5603_CONTROL1_BANDWIDTH_1_2ms;
    HAL_I2C_Mem_Write(
        i2c_handle,
        MMC5603_I2C_ID,
        REG_CONTROL1,
        1,
        &data,
        1,
        5
    );


    if(m_use_continuos_mode){
        // Initialize the ODR register
        data = 0b00000000;
        if(m_use_automatic_set_reset){
            data |= desired_refresh_rate;
        }else{
            // If set/reset is not done then it can be faster easily.
            // That is why divide by 2 to get actual frequency that it will perform at
            data |= desired_refresh_rate / 2; 
        }
        HAL_I2C_Mem_Write(
            i2c_handle,
            MMC5603_I2C_ID,
            REG_ODR,
            1,
            &data,
            1,
            5
        );

        // If the desired frequency is 1000Hz then have to enable high performance
        if(enable_high_performance){
            uint8_t data2 = mmc5603_get_register_value(REG_CONTROL2);
            data2 |= MMC5603_CONTROL2_ENABLE_HIGH_ODR_POWER;
            HAL_I2C_Mem_Write(
                i2c_handle,
                MMC5603_I2C_ID,
                REG_CONTROL2,
                1,
                &data2,
                1,
                5
            );

        }

        // Enable the continuos mode calculation of period
        data = mmc5603_get_register_value(REG_CONTROL0);
        data |= MMC5603_CONTROL0_START_CALCULATE_MEASUREMENT_PERIOD_ODR;
        HAL_I2C_Mem_Write(
            i2c_handle,
            MMC5603_I2C_ID,
            REG_CONTROL0,
            1,
            &data,
            1,
            5
        );

        // Enable the automatic set/reset
        if(m_use_automatic_set_reset){
            uint8_t data2 = mmc5603_get_register_value(REG_CONTROL0);;
            data2 |= MMC5603_CONTROL0_AUTO_SET_RESET;
            HAL_I2C_Mem_Write(
                i2c_handle,
                MMC5603_I2C_ID,
                REG_CONTROL0,
                1,
                &data2,
                1,
                5
            );
        }

        // Enable the actual continuos mode
        data = mmc5603_get_register_value(REG_CONTROL2);
        data |= MMC5603_CONTROL2_ENTER_CONTINUOUS_MODE;
        HAL_I2C_Mem_Write(
            i2c_handle,
            MMC5603_I2C_ID,
            REG_CONTROL2,
            1,
            &data,
            1,
            5
        );
    }

    printf("MMC5603 initialized\n");

    return 1;
}


void mmc5603_magnetometer_readings_micro_teslas(float *data, uint8_t apply_rotation_into_accelerometer_position){

    if(!m_use_continuos_mode){
        // Need to ask for the measurement to be made
        uint8_t data2 = mmc5603_get_register_value(REG_CONTROL0);
        data2 |= MMC5603_CONTROL0_TAKE_MAGNETIC_MEASUREMENT;
        HAL_I2C_Mem_Write(
            i2c_handle,
            MMC5603_I2C_ID,
            REG_CONTROL0,
            1,
            &data2,
            1,
            5
        );

        // Check if magnetic measurement done
        uint8_t status;
        do{
            status = mmc5603_read_status_register();
        } while (!(status & MMC5603_STATUS_REGISTER_MEASUREMENT_MAGNETIC_DONE));
    }


    // Get the magnetometer data
    uint8_t retrieved_data[9];
    HAL_I2C_Mem_Read(
        i2c_handle,
        MMC5603_I2C_ID,
        REG_OUTPUT_XOUT0,
        1,
        retrieved_data,
        9, // read nine registers in total
        5
    );
 
    // Combine the bits into a integer value
    int32_t X = ((int32_t)retrieved_data[0] << 12) | ((int32_t)retrieved_data[1] << 4) | ((int32_t)retrieved_data[6] >> 4);
    int32_t Y = ((int32_t)retrieved_data[2] << 12) | ((int32_t)retrieved_data[3] << 4) | ((int32_t)retrieved_data[7] >> 4);
    int32_t Z = ((int32_t)retrieved_data[4] << 12) | ((int32_t)retrieved_data[5] << 4) | ((int32_t)retrieved_data[8] >> 4);

    // Turn the integer value into a micro tesla value
    data[0] = (float)X * DECIMAL_TO_MICRO_TESLA_RATIO;
    data[1] = (float)Y * DECIMAL_TO_MICRO_TESLA_RATIO;
    data[2] = (float)Z * DECIMAL_TO_MICRO_TESLA_RATIO;

    mmc5603_old_magnetometer_readings[0] = data[0];
    mmc5603_old_magnetometer_readings[1] = data[1];
    mmc5603_old_magnetometer_readings[2] = data[2];

    // Fix the micro tesla value with calibrations
    for (uint8_t i = 0; i < 3; i++){
        data[i] = data[i] - m_mmc5603_hard_iron[i];
    }

    float temp[3];
    for (uint8_t i = 0; i < 3; i++){
        temp[i] = (m_mmc5603_soft_iron[i][0] * data[0]) +
                  (m_mmc5603_soft_iron[i][1] * data[1]) +
                  (m_mmc5603_soft_iron[i][2] * data[2]);
    }
    for (uint8_t i = 0; i < 3; i++) data[i] = temp[i];

    mmc5603_magnetometer_readings_without_rotation[0] = temp[0];
    mmc5603_magnetometer_readings_without_rotation[1] = temp[1];
    mmc5603_magnetometer_readings_without_rotation[2] = temp[2];

    if(apply_rotation_into_accelerometer_position){
        float temp2[3];
        for (uint8_t i = 0; i < 3; i++){
            temp2[i] = (mmc5603_rotation_matrix[i][0] * data[0]) +
                    (mmc5603_rotation_matrix[i][1] * data[1]) +
                    (mmc5603_rotation_matrix[i][2] * data[2]);
        }
        for (uint8_t i = 0; i < 3; i++) data[i] = temp2[i];
    }
}

void mmc5603_set(){
    // -------------------------------------------------------------------------------- Example of set 
    uint8_t data = mmc5603_get_register_value(REG_CONTROL0);
    data |= MMC5603_CONTROL0_DO_SET;
    data |= ~(MMC5603_CONTROL0_DO_RESET); // Unset the set bit just in case
    HAL_I2C_Mem_Write(
        i2c_handle,
        MMC5603_I2C_ID,
        REG_CONTROL0,
        1,
        &data,
        1,
        5
    );
}

void mmc5603_reset(){
    // Read what is in the register before 

    // -------------------------------------------------------------------------------- Example of reset
    uint8_t data = mmc5603_get_register_value(REG_CONTROL0);
    data |= MMC5603_CONTROL0_DO_RESET;
    data |= ~(MMC5603_CONTROL0_DO_SET); // Unset the set bit just in case

    HAL_I2C_Mem_Write(
        i2c_handle,
        MMC5603_I2C_ID,
        REG_CONTROL0,
        1,
        &data,
        1,
        5
    );
}

uint8_t mmc5603_read_status_register(){
    uint8_t data;

    HAL_I2C_Mem_Read(
        i2c_handle,
        MMC5603_I2C_ID,
        REG_STATUS,
        1,
        &data,
        1,
        5
    );

    return data;
}

uint8_t mmc5603_perform_self_test(){
    // -------------------------------------------------------------------------------- Example of self test

    // 1 Get the self test values
    uint8_t retrieved_data[] = {0, 0, 0};
    HAL_I2C_Mem_Read(
        i2c_handle,
        MMC5603_I2C_ID,
        REG_SELF_TEST_X,
        1,
        retrieved_data,
        3, // read 3 in total x, y and z
        5
    );

    // 2 Calculate the new value
    retrieved_data[0] *= 0.8;
    retrieved_data[1] *= 0.8;
    retrieved_data[2] *= 0.8;

    // 3 Set the new values to the threshold registers
    uint8_t data = retrieved_data[0];
    HAL_I2C_Mem_Write(
        i2c_handle,
        MMC5603_I2C_ID,
        REG_SELF_TEST_X_THRESHOLD,
        1,
        &data,
        1,
        5
    );

    data = retrieved_data[1];
    HAL_I2C_Mem_Write(
        i2c_handle,
        MMC5603_I2C_ID,
        REG_SELF_TEST_Y_THRESHOLD,
        1,
        &data,
        1,
        5
    );

    data = retrieved_data[2];
    HAL_I2C_Mem_Write(
        i2c_handle,
        MMC5603_I2C_ID,
        REG_SELF_TEST_Z_THRESHOLD,
        1,
        &data,
        1,
        5
    );

    // 4 Initiate self test

    data = mmc5603_get_register_value(REG_CONTROL0);
    data |= MMC5603_CONTROL0_AUTO_SELF_TEST;
    data |= MMC5603_CONTROL0_TAKE_MAGNETIC_MEASUREMENT;

    HAL_I2C_Mem_Write(
        i2c_handle,
        MMC5603_I2C_ID,
        REG_CONTROL0,
        1,
        &data,
        1,
        5
    );

    // 5 Read self test status bit

    uint8_t status_register = mmc5603_read_status_register();

    // If there is a 1 in the bit then it did not pass
    if(status_register & MMC5603_STATUS_REGISTER_SELF_TEST_SIGNAL){
        return 0;
    }else{
        return 1;
    }
}


void mmc5603_get_bridge_offset(float *data){
    // -------------------------------------------------------------------------------- Example remove bridge offset
    mmc5603_set();

    float set_magnetometer_data[3];
    mmc5603_magnetometer_readings_micro_teslas(set_magnetometer_data, 0);

    mmc5603_reset();

    float reset_magnetometer_data[3];
    mmc5603_magnetometer_readings_micro_teslas(reset_magnetometer_data, 0);


    // To get the offset the datasheet tells to add the measurements and divide by 2.
    data[0] = (set_magnetometer_data[0] + reset_magnetometer_data[0])/2;
    data[1] = (set_magnetometer_data[1] + reset_magnetometer_data[1])/2;
    data[2] = (set_magnetometer_data[2] + reset_magnetometer_data[2])/2;
}

void mmc5603_magnetometer_readings_micro_teslas_bridge_offset_removed(float *data){
    // -------------------------------------------------------------------------------- Example remove bridge offset
    mmc5603_set();

    float set_magnetometer_data[3];
    mmc5603_magnetometer_readings_micro_teslas(set_magnetometer_data, 0);

    mmc5603_reset();

    float reset_magnetometer_data[3];
    mmc5603_magnetometer_readings_micro_teslas(reset_magnetometer_data, 0);


    // To get the output without offset the datasheet tells to subtract the measurements and divide by 2.
    data[0] = (set_magnetometer_data[0] - reset_magnetometer_data[0])/2;
    data[1] = (set_magnetometer_data[1] - reset_magnetometer_data[1])/2;
    data[2] = (set_magnetometer_data[2] - reset_magnetometer_data[2])/2;
}

void mmc5603_previous_raw_magetometer_readings(float *data){
    data[0] = mmc5603_old_magnetometer_readings[0];
    data[1] = mmc5603_old_magnetometer_readings[1];
    data[2] = mmc5603_old_magnetometer_readings[2];
}

void mmc5603_set_rotation_matrix(const float rotation_matrix[3][3]){
    for (uint8_t i = 0; i < 3; i++){
        for (uint8_t k = 0; k < 3; k++){
            mmc5603_rotation_matrix[i][k] = rotation_matrix[i][k];
        }
    }
}

void mmc5603_magnetometer_readings_micro_teslas_unrotated(float *data){
    data[0] = mmc5603_magnetometer_readings_without_rotation[0];
    data[1] = mmc5603_magnetometer_readings_without_rotation[1];
    data[2] = mmc5603_magnetometer_readings_without_rotation[2];
}
