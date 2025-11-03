#include "./bmm350.h"
#include "../utils/string_utils/string_utils.h"

#include <inttypes.h>
#include <stdio.h>

#define BMM350_I2C_ID  (0x14 << 1)
// #define BMM350_I2C_ID  (0x7E << 1)

#define BMM350_CMD_REG 0x7E
#define BMM350_CTRL_USER_REG 0x61
#define BMM350_TMR_SELFTEST_USER_REG 0x60
#define BMM350_OTP_STATUS_REG 0x55
#define BMM350_OTP_DATA_LSB_REG 0x53
#define BMM350_OTP_DATA_MSB_REG 0x52
#define BMM350_OTP_CMD_REG 0x50
#define BMM350_SENSORTIME_MSB_REG 0x3F
#define BMM350_SENSORTIME_LSB_REG 0x3E
#define BMM350_SENSORTIME_XLSB_REG 0x3D
#define BMM350_TEMP_MSB_REG 0x3C
#define BMM350_TEMP_LSB_REG 0x3B
#define BMM350_TEMP_XLSB_REG 0x3A
#define BMM350_MAG_Z_MSB_REG 0x39
#define BMM350_MAG_Z_LSB_REG 0x38
#define BMM350_MAG_Z_XLSB_REG 0x37
#define BMM350_MAG_Y_MSB_REG 0x36
#define BMM350_MAG_Y_LSB_REG 0x35
#define BMM350_MAG_Y_XLSB_REG 0x34
#define BMM350_MAG_X_MSB_REG 0x33
#define BMM350_MAG_X_LSB_REG 0x32
#define BMM350_MAG_X_XLSB_REG 0x31
#define BMM350_INT_STATUS_REG 0x30
#define BMM350_INT_CTRL_IBI_REG 0x2F
#define BMM350_INT_CTRL_REG 0x2E
#define BMM350_TRANSDUCER_REV_ID_REG 0x0D
#define BMM350_I2C_WMT_SET_REG 0x0A
#define BMM350_I3C_ERR_REG 0x09
#define BMM350_PMU_CMD_STATUS_1_REG 0x08
#define BMM350_PMU_CMD_STATUS_0_REG 0x07
#define BMM350_PMU_CMD_REG 0x06
#define BMM350_PMU_CMD_AXIS_EN_REG 0x05
#define BMM350_PMU_CMD_AGGR_SET_REG 0x04
#define BMM350_PAD_CTRL_REG 0x03
#define BMM350_ERR_REG 0x02
#define BMM350_CHIP_ID_REG 0x00





#define BMM350_CHIP_ID_REG_GET_CHIP_ID(value) (((value) & 0b11110000) >> 4)
#define BMM350_CHIP_ID_REG_GET_OTP_ID(value) (((value) & 0b00001111) >> 4)

#define BMM350_ERR_REG_GET_PMU_CMD_ERROR(value) ((value) & 0b00000001)
#define BMM350_PAD_CTRL_GET_DRV(value) ((value) & 0b00000111)

#define BMM350_PMU_CMD_STATUS_0_GET_CMD_IS_ILLEGAL(value) (((value) & 0b00010000) >> 4)
#define BMM350_PMU_CMD_STATUS_0_GET_POWER_MODE_IS_NORMAL(value) (((value) & 0b00001000) >> 3)
#define BMM350_PMU_CMD_STATUS_0_GET_AVG_OVWR(value) (((value) & 0b00000100) >> 2)
#define BMM350_PMU_CMD_STATUS_0_GET_ODR_OVWR(value) (((value) & 0b00000010) >> 1)
#define BMM350_PMU_CMD_STATUS_0_GET_PMU_CMD_BUSY(value) ((value) & 0b00000001)

#define BMM350_PMU_CMD_STATUS_1_GET_PMU_AVG_S(value) (((value) & 0b00110000) >> 4)
#define BMM350_PMU_CMD_STATUS_1_GET_PMU_ODR_S(value) ((value) & 0b00001111)

#define BMM350_i3C_ERR_GET_I3C_ERROR_3(value) (((value) & 0b00001000) >> 3)
#define BMM350_i3C_ERR_GET_I3C_ERROR_0(value) ((value) & 0b00000001)

#define BMM350_I2C_WDT_SET_GET_I2C_WDT_SET(value) (((value) & 0b00000001) >> 1)
#define BMM350_I2C_WDT_SET_GET_I2C_WDT_EN(value) ((value) & 0b00000001)

#define BMM350_INT_STATUS_GET_DRDY_DATA_REG(value) (((value) & 0b00000100) >> 2)

#define BMM350_OTP_CMD_REG_GET_WORD_ADDR(value) ((value) & 0b00011111)
#define BMM350_OTP_CMD_REG_GET_OTP_CMD(value) (((value) & 0b11100000) >> 5)



#define BMM350_OTP_STATUS_REG_GET_OTP_CMD_DONE(value) ((value) & 0b00000001)
#define BMM350_OTP_STATUS_REG_GET_CUR_PAGE_ADDR(value) (((value) & 0b00011110) >> 1)
#define BMM350_OTP_STATUS_REG_GET_ERROR(value) (((value) & 0b11100000) >> 5)


#define BMM350_RESET_VALUE_0 0xB6
#define BMM350_RESET_VALUE_1 0x00

#define BMM350_AMOUNT_OF_OTP_DATA 32




// CHIP_ID value 0x33
enum bmm350_PMU_CMD_AXIS_EN {
    BMM350_PMU_CMD_AXIS_EN_Z                                     = 0b00000100,
    BMM350_PMU_CMD_AXIS_EN_Y                                     = 0b00000010,
    BMM350_PMU_CMD_AXIS_EN_X                                     = 0b00000001,
};

enum bmm350_PMU_CMD {
    BMM350_PMU_CMD_MODE_SUSPEND                              = 0b00000000,
    BMM350_PMU_CMD_MODE_NORMAL                               = 0b00000001,
    BMM350_PMU_CMD_DO_UPDATE_ODR_AND_AVG                     = 0b00000010,
    BMM350_PMU_CMD_MODE_FORCED                               = 0b00000011,
    BMM350_PMU_CMD_MODE_FORCED_FAST                          = 0b00000100,
    BMM350_PMU_CMD_DO_FLUX_GUIDE                             = 0b00000101,
    BMM350_PMU_CMD_DO_FLUX_GUIDE_FAST                        = 0b00000110,
    BMM350_PMU_CMD_DO_RESET                                  = 0b00000111,
    BMM350_PMU_CMD_DO_RESET_FAST                             = 0b00001000,
};

enum bmm350_I2C_WDT_SET {
    BMM350_I2C_WDT_SET_ENABLE_WATCHDOG                       = 0b00000001,
    BMM350_I2C_WDT_SET_DISABLE_WATCHDOG                      = 0b00000000,
    BMM350_I2C_WDT_SET_WATCHDOG_TIMEOUT_1_28_MS              = 0b00000000,
    BMM350_I2C_WDT_SET_WATCHDOG_TIMEOUT_40_96_MS             = 0b00000010,
};

enum bmm350_INT_CTRL {
    BMM350_INT_CTRL_MODE_PULSED                              = 0b00000000,
    BMM350_INT_CTRL_MODE_LATCHED                             = 0b00000001,
    BMM350_INT_CTRL_POLARITY_ACTIVE_LOW                      = 0b00000000,
    BMM350_INT_CTRL_POLARITY_ACTIVE_HIGH                     = 0b00000010,
    BMM350_INT_CTRL_OPEN_DRAIN                               = 0b00000000,
    BMM350_INT_CTRL_PUSH_PULL                                = 0b00000100,
    BMM350_INT_CTRL_OUTPUT_DISABLED                          = 0b00000000,
    BMM350_INT_CTRL_OUTPUT_ENABLED                           = 0b00001000,
    BMM350_INT_CTRL_DRDY_DATA_REGEN_DISABLED                 = 0b00000000,
    BMM350_INT_CTRL_DRDY_DATA_REGEN_ENABLED                  = 0b10000000,
};

enum bmm350_INT_CTRL_IBI {
    BMM350_INT_CTRL_IBI_INT_MAP_TO_IBI_DISABLED              = 0b00000000,
    BMM350_INT_CTRL_IBI_INT_MAP_TO_IBI_ENABLED               = 0b00000001,

    BMM350_INT_CTRL_IBI_CLEAR_DRDY_INT_IBI_STATUS_DISABLED   = 0b00000000,
    BMM350_INT_CTRL_IBI_CLEAR_DRDY_INT_IBI_STATUS_EABLED     = 0b00010000,
};


enum bmm350_OTP_CMD {
    BMM350_OTP_CMD_DIR_READ                                  = 0b00100000,
    BMM350_OTP_CMD_DIR_PRGM_1B                               = 0b01000000,
    BMM350_OTP_CMD_DIR_PRGM                                  = 0b01100000,
    BMM350_OTP_CMD_PWR_OFF_OTP                               = 0b10000000,
    BMM350_OTP_CMD_EXT_READ                                  = 0b10100000,
    BMM350_OTP_CMD_EXT_PRGM                                  = 0b11100000,



};

I2C_HandleTypeDef *bmm350_i2c_handle;

// Storage of hard iron correction, values should be replaced by what is passed
volatile float bmm350_hard_iron[3] = {
    0, 0, 0
};

// Storage of soft iron correction, values should be replaced by what is passed
volatile float bmm350_soft_iron[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
};

volatile float bmm350_rotation_matrix[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
};

float bmm350_old_magnetometer_readings[3] = {0, 0, 0};
float bmm350_magnetometer_readings_without_rotation[3] = {0, 0, 0};
float bmm350_old_temperature_reading = 0;

float bmm350_offset_mag_x = 0.0f;
float bmm350_offset_mag_y = 0.0f;
float bmm350_offset_mag_z = 0.0f;
float bmm350_offset_temp = 0.0f;
float bmm350_sensitivity_temp = 0.0f;
float bmm350_sensitivity_mag_x = 0.0f;
float bmm350_sensitivity_mag_y = 0.0f;
float bmm350_sensitivity_mag_z = 0.0f;
float bmm350_temp_coefficient_mag_x = 0.0f;
float bmm350_temp_coefficient_mag_y = 0.0f;
float bmm350_temp_coefficient_mag_z = 0.0f;
float bmm350_temp_coefficient_sensitivity_mag_x = 0.0f;
float bmm350_temp_coefficient_sensitivity_mag_y = 0.0f;
float bmm350_temp_coefficient_sensitivity_mag_z = 0.0f;
float bmm350_cross_axis_mag_x_y = 0.0f;
float bmm350_cross_axis_mag_y_x = 0.0f;
float bmm350_cross_axis_mag_z_x = 0.0f;
float bmm350_cross_axis_mag_z_y = 0.0f;
float bmm350_reference_temperature = 0.0f;

uint8_t bmm350_variant_id = 0;


const double bxy_sens = 14.55;
const double bz_sens = 9.0;
const double temp_sens = 0.00204;
const double ina_xy_gain_trgt = 19.46;
const double ina_z_gain_trgt = 31.0;
const double adc_gain = 1.0f / 1.5;
const double lut_gain = 0.714607238769531;
const double power = 1000000.0 / 1048576.0;

float lsb_to_ut_degc[] = { 0.0, 0.0, 0.0, 0.0};

void bmm350_initialize_conversion_values(){
    // This is the calculation to convert the raw mag readings to actual micro teslas
    lsb_to_ut_degc[0] = (float)((power / (bxy_sens * ina_xy_gain_trgt * adc_gain * lut_gain)));
    lsb_to_ut_degc[1] = (float)((power / (bxy_sens * ina_xy_gain_trgt * adc_gain * lut_gain)));
    lsb_to_ut_degc[2] = (float)((power / (bz_sens * ina_z_gain_trgt * adc_gain * lut_gain)));
    lsb_to_ut_degc[3] = (float)(1.0 / (temp_sens * adc_gain * lut_gain * 1048576.0));
}

uint8_t bmm350_init(
    I2C_HandleTypeDef *i2c_handle_temp, 
    uint8_t apply_calibration, 
    const float hard_iron[3], 
    const float soft_iron[3][3],
    enum bmm350_PAD_CTRL_odr odr_setting,
    enum bmm350_PAD_CTRL_avg average_setting
){
    bmm350_i2c_handle = i2c_handle_temp;

    uint8_t data = 0b00000000;
    // assign the correction for irons
    if (apply_calibration){
        for (uint8_t i = 0; i < 3; i++){
            bmm350_hard_iron[i] = hard_iron[i];
        }

        for (uint8_t i = 0; i < 3; i++){
            for (uint8_t k = 0; k < 3; k++){
                bmm350_soft_iron[i][k] = soft_iron[i][k];
            }
        }
    }

    // ========================= How it should initialize
    // Reset the chip
    // Wait 24 ms
    // Test read id
    // READ OTP STUFF
    // POWER OFF OTP
    // Read 1 byte                              0x07 PMU_CMD_STATUS_0_REG            // Got 0x00
    // Write 00000111                           0x06 BMM350_PMU_CMD_REG              // basically BMM350_PMU_CMD_DO_RESET
    // Wait 14 ms
    // Read 1 byte                              0x07 PMU_CMD_STATUS_0_REG            // Got 0xE0
    // Write 00000101                           0x06 BMM350_PMU_CMD_REG              // do BMM350_PMU_CMD_DO_FLUX_GUIDE_FAST
    // Wait 18 ms
    // Read 1 byte                              0x07 PMU_CMD_STATUS_0_REG            // Got 0xA0
    // Read 1 byte                              0x06 BMM350_PMU_CMD_REG              // Got 0x05
    // Write 00000001                           0x06 BMM350_PMU_CMD_REG              // Enter normal mode
    // Read 1 byte                              0x04 BMM350_PMU_CMD_AGGR_SET_REG     // Got 0x14
    // Wait 38 ms
    // Write 00110110                           0x04 BMM350_PMU_CMD_AGGR_SET_REG     // Config avg 8 and odr 25Hz
    // Write 00000010                           0x06 BMM350_PMU_CMD_REG              // Tell it to update odr and average parameter
    // Wait 1 ms
    // Start reading mag data i guess
    // Write 00000111                           0x05 BMM350_PMU_CMD_AXIS_EN_REG      // Enable the axies
    // Starts reading data registers

    bmm350_initialize_conversion_values();


    HAL_StatusTypeDef result;

    // Reset the chip completely
    data = BMM350_RESET_VALUE_0;
    result = HAL_I2C_Mem_Write(
        bmm350_i2c_handle,
        BMM350_I2C_ID,
        BMM350_CMD_REG,
        1,
        &data,
        1,
        5
    );
    if(result != HAL_OK){
        printf("BMM350 init failed: i2c read res %d for chip reset \n", result);
        return 0;
    }

    HAL_Delay(24);

    
    // Test the sensor by reading it's id register
    // The id is in the third read for some reason...
    uint8_t id_check[] = {0, 0, 0};
    result = HAL_I2C_Mem_Read(
        bmm350_i2c_handle,
        BMM350_I2C_ID + 1,
        BMM350_CHIP_ID_REG,
        1,
        id_check,
        3,
        5
    );
    
    if(result != HAL_OK){
        printf("BMM350 init failed: Failed i2c read %d for id check\n", result);
        return 0;
    }

    // Check if the id value is as it should be
    if (id_check[2] != 0x33){
        printf("BMM350 init failed at self id test: ");
        print_binary(id_check[2]);
        printf("\n");
        return 0;
    }


    // Read the OTP data
    bmm350_read_all_otp_data();

    // Check PMU_CMD_STATUS_0_REG
    uint8_t read_data[] = {0, 0, 0};
    result = HAL_I2C_Mem_Read(
        bmm350_i2c_handle,
        BMM350_I2C_ID + 1,
        BMM350_PMU_CMD_STATUS_0_REG,
        1,
        read_data,
        3,
        5
    );

    if(result != HAL_OK){
        printf("BMM350 init failed: i2c read res %d Check PMU_CMD_STATUS_0_REG \n", result);
        return 0;
    }

    // Soft reset
    data = BMM350_PMU_CMD_DO_RESET;
    result = HAL_I2C_Mem_Write(
        bmm350_i2c_handle,
        BMM350_I2C_ID,
        BMM350_PMU_CMD_REG,
        1,
        &data,
        1,
        5
    );

    if(result != HAL_OK){
        printf("BMM350 init failed: Failed i2c read %d failed soft reset\n", result);
        return 0;
    }
    HAL_Delay(14);

    // Check PMU_CMD_STATUS_0_REG
    read_data[0] = 0;
    read_data[1] = 0;
    read_data[2] = 0;
    result = HAL_I2C_Mem_Read(
        bmm350_i2c_handle,
        BMM350_I2C_ID + 1,
        BMM350_PMU_CMD_STATUS_0_REG,
        1,
        read_data,
        3,
        5
    );

    if(result != HAL_OK){
        printf("BMM350 init failed: i2c read res %d Check PMU_CMD_STATUS_0_REG \n", result);
        return 0;
    }

    // Dont care about result

    // Flux guide 
    data = BMM350_PMU_CMD_DO_FLUX_GUIDE_FAST;
    result = HAL_I2C_Mem_Write(
        bmm350_i2c_handle,
        BMM350_I2C_ID,
        BMM350_PMU_CMD_REG,
        1,
        &data,
        1,
        5
    );

    if(result != HAL_OK){
        printf("BMM350 init failed: Failed i2c read %d failed soft reset\n", result);
        return 0;
    }
    HAL_Delay(18);

    // Check PMU_CMD_STATUS_0_REG
    read_data[0] = 0;
    read_data[1] = 0;
    read_data[2] = 0;
    result = HAL_I2C_Mem_Read(
        bmm350_i2c_handle,
        BMM350_I2C_ID + 1,
        BMM350_PMU_CMD_STATUS_0_REG,
        1,
        read_data,
        3,
        5
    );
    if(result != HAL_OK){
        printf("BMM350 init failed: i2c read res %d Check PMU_CMD_STATUS_0_REG \n", result);
        return 0;
    }
    // Dont care about result


    // Check BMM350_PMU_CMD_REG 
    read_data[0] = 0;
    read_data[1] = 0;
    read_data[2] = 0;
    result = HAL_I2C_Mem_Read(
        bmm350_i2c_handle,
        BMM350_I2C_ID + 1,
        BMM350_PMU_CMD_REG,
        1,
        read_data,
        3,
        5
    );

    if(result != HAL_OK){
        printf("BMM350 init failed: i2c read res %d Check BMM350_PMU_CMD_REG \n", result);
        return 0;
    }
    // Dont care about result
    
    // Enter normal mode
    data = BMM350_PMU_CMD_MODE_NORMAL;
    result = HAL_I2C_Mem_Write(
        bmm350_i2c_handle,
        BMM350_I2C_ID,
        BMM350_PMU_CMD_REG,
        1,
        &data,
        1,
        5
    );

    if(result != HAL_OK){
        printf("BMM350 init failed: i2c read res %d for enter normal mode\n", result);
        return 0;
    }


    // Check BMM350_PMU_CMD_AGGR_SET_REG 
    read_data[0] = 0;
    read_data[1] = 0;
    read_data[2] = 0;
    result = HAL_I2C_Mem_Read(
        bmm350_i2c_handle,
        BMM350_I2C_ID + 1,
        BMM350_PMU_CMD_AGGR_SET_REG,
        1,
        read_data,
        3,
        5
    );
    if(result != HAL_OK){
        printf("BMM350 init failed: i2c read res %d  Check BMM350_PMU_CMD_AGGR_SET_REG \n", result);
        return 0;
    }
    // Dont care about result

    HAL_Delay(38);


    // Set the ODR/Refresh rate and the averaging
    data = 0b00000000;
    data |= odr_setting;
    data |= average_setting;

    result = HAL_I2C_Mem_Write(
        bmm350_i2c_handle,
        BMM350_I2C_ID,
        BMM350_PMU_CMD_AGGR_SET_REG,
        1,
        &data,
        1,
        5
    );

    if(result != HAL_OK){
        printf("BMM350 init failed: i2c read res %d for refresh rate and averaging config\n", result);
        return 0;
    }

    // Update odr and avg values
    data = BMM350_PMU_CMD_DO_UPDATE_ODR_AND_AVG;
    result = HAL_I2C_Mem_Write(
        bmm350_i2c_handle,
        BMM350_I2C_ID,
        BMM350_PMU_CMD_REG,
        1,
        &data,
        1,
        5
    );

    if(result != HAL_OK){
        printf("BMM350 init failed: i2c read res %d update odr and avg values\n", result);
        return 0;
    }

    HAL_Delay(1);

    // Enable all axies
    data = 0b00000000;
    data |= BMM350_PMU_CMD_AXIS_EN_Z;
    data |= BMM350_PMU_CMD_AXIS_EN_Y;
    data |= BMM350_PMU_CMD_AXIS_EN_X;

    result = HAL_I2C_Mem_Write(
        bmm350_i2c_handle,
        BMM350_I2C_ID,
        BMM350_PMU_CMD_AXIS_EN_REG,
        1,
        &data,
        1,
        5
    );
    
    if(result != HAL_OK){
        printf("BMM350 init failed: i2c read res %d for axis enabling\n", result);
        return 0;
    }

    printf("BMM350 initialized\n");

    return 1;
}

#define BMM350_SIGNED_8_BIT                         128
#define BMM350_SIGNED_12_BIT                        2048
#define BMM350_SIGNED_16_BIT                        32768
#define BMM350_SIGNED_21_BIT                        1048576
#define BMM350_SIGNED_24_BIT                        8388608

int32_t fix_sign(uint32_t value, int32_t power){
    int32_t return_value = (int32_t)value;

    if (return_value >= power) return_value = return_value - (power * 2);

    return return_value;
}

void bmm350_read_all_otp_data(){
    uint16_t otp_data[32] = {0};

    // Iterate over the possible otp data registers and read all of them
    for (uint8_t i = 0; i < BMM350_AMOUNT_OF_OTP_DATA; i++){

        // Say what OTP word to read
        uint8_t data = 0b00000000;
        data |= BMM350_OTP_CMD_DIR_READ;
        data |= i;

        HAL_I2C_Mem_Write(
            bmm350_i2c_handle,
            BMM350_I2C_ID,
            BMM350_OTP_CMD_REG,
            1,
            &data,
            1,
            5
        );

        // Poll for the otp data to be loaded into the otp register
        uint8_t status_register_value[] = {0, 0, 0};
        do{
            HAL_I2C_Mem_Read(
                bmm350_i2c_handle,
                BMM350_I2C_ID + 1,
                BMM350_OTP_STATUS_REG,
                1,
                status_register_value,
                3,
                5
            );
        }while (BMM350_OTP_STATUS_REG_GET_OTP_CMD_DONE(status_register_value[2]) == 0);// Iterate while status is not 1
        

        // Read the msb otp value
        uint8_t otp_value_msb[] = {0, 0, 0};
        HAL_I2C_Mem_Read(
            bmm350_i2c_handle,
            BMM350_I2C_ID + 1,
            BMM350_OTP_DATA_MSB_REG,
            1,
            otp_value_msb,
            3,
            5
        );

        // Read the lsb otp value
        uint8_t otp_value_lsb[] = {0, 0, 0};
        HAL_I2C_Mem_Read(
            bmm350_i2c_handle,
            BMM350_I2C_ID + 1,
            BMM350_OTP_DATA_LSB_REG,
            1,
            otp_value_lsb,
            3,
            5
        );

        otp_data[i] = (((uint16_t)otp_value_msb[2]) << 8) | ((uint16_t)otp_value_lsb[2]);
    }

    // Power off OTP
    uint8_t data = 0b00000000;
    data |= BMM350_OTP_CMD_PWR_OFF_OTP;
    HAL_I2C_Mem_Write(
        bmm350_i2c_handle,
        BMM350_I2C_ID,
        BMM350_OTP_CMD_REG,
        1,
        &data,
        1,
        5
    );
    

    // Convert the OPT data to real data

    /*
        OTP data meanings

        0  - Unknown
        1  - Unknown
        2  - Unknown
        3  - Unknown
        4  - Unknown
        5  - Unknown
        6  - Unknown
        7  - Unknown
        8  - Unknown
        9  - Unknown
        10 - Unknown
        11 - Unknown
        12 - Unknown
        13 - TEMP_OFF_SENS:
            bits[0..7]   = Temperature offset (signed 8-bit, scale 1/5 °C)
            bits[8..15]  = Temperature sensitivity (signed 8-bit, scale 1/512)
        14 - MAG_OFFSET_X / MAG_OFFSET_Y nibble:
            bits[0..11]  = X-axis offset (signed 12-bit)
            bits[12..15] = upper nibble of Y-axis offset
        15 - MAG_OFFSET_Y / MAG_OFFSET_Z nibble:
            bits[0..7]   = lower byte of Y-axis offset
            bits[8..11]  = upper nibble of Z-axis offset
            bits[12..15] unused/reserved?
        16 - MAG_OFFSET_Z LSB + SENS_X:
            bits[0..7]   = lower byte of Z-axis offset
            bits[8..15]  = X-axis sensitivity (signed 8-bit, /256)
        17 - SENS_Y / SENS_Z:
            bits[0..7]   = Y-axis sensitivity (/256 + corr_y)
            bits[8..15]  = Z-axis sensitivity (/256)
        18 - TCO_X / TCS_X:
            bits[0..7]   = Temp coeff of offset X (/32 µT/°C)
            bits[8..15]  = Temp coeff of sensitivity X (/16384 per °C)
        19 - TCO_Y / TCS_Y:
            bits[0..7]   = Temp coeff of offset Y (/32 µT/°C)
            bits[8..15]  = Temp coeff of sensitivity Y (/16384 per °C)
        20 - TCO_Z / TCS_Z:
            bits[0..7]   = Temp coeff of offset Z (/32 µT/°C)
            bits[8..15]  = Temp coeff of sensitivity Z (/16384 per °C, minus corr_z)
        21 - CROSS_X_Y / CROSS_Y_X:
            bits[0..7]   = Cross-axis term: X→Y (/800)
            bits[8..15]  = Cross-axis term: Y→X (/800)
        22 - CROSS_Z_X / CROSS_Z_Y:
            bits[0..7]   = Cross-axis term: Z→X (/800)
            bits[8..15]  = Cross-axis term: Z→Y (/800)
        23 - Unknown / reserved
        24 - MAG_DUT_T_0:
            full 16-bit signed = Reference calibration temperature (°C) (/512 + 23°C offset)
        25 - Unknown
        26 - Unknown
        27 - Unknown
        28 - Unknown
        29 - Unknown
        30 - Contains variant ID in bits [8..14] ((value & 0x7F00) >> 9)
        31 - Unknown
    */

    /* Offsets: signed 12-bit each */
    bmm350_offset_mag_x = fix_sign(
        otp_data[14] & 0b0000111111111111,
        BMM350_SIGNED_12_BIT);

    bmm350_offset_mag_y = fix_sign(
        ((otp_data[14] & 0b1111000000000000) >> 4) | (otp_data[15] & 0b0000000011111111),
        BMM350_SIGNED_12_BIT);
    
    bmm350_offset_mag_z = fix_sign(
        ((otp_data[15] & 0b0000111100000000) >> 8) | (otp_data[16] & 0b0000000011111111),
        BMM350_SIGNED_12_BIT);

    bmm350_offset_temp = fix_sign(
        otp_data[13] & 0b0000000011111111,
        BMM350_SIGNED_8_BIT) / 5.0f;



    bmm350_sensitivity_mag_x = fix_sign(
        (otp_data[16] & 0b1111111100000000) >> 8,
        BMM350_SIGNED_8_BIT) / 256.0f;

    bmm350_sensitivity_mag_y = (fix_sign(
        otp_data[17] & 0b0000000011111111,
        BMM350_SIGNED_8_BIT) / 256.0f
    ) + 0.01f;

    bmm350_sensitivity_mag_z = fix_sign(
        (otp_data[17] & 0b1111111100000000) >> 8,
        BMM350_SIGNED_8_BIT) / 256.0f;

    bmm350_sensitivity_temp = fix_sign(
        (otp_data[13] & 0b1111111100000000) >> 8,
        BMM350_SIGNED_8_BIT) / 512.0f;



    bmm350_temp_coefficient_mag_x = fix_sign(
        otp_data[18] & 0b0000000011111111, 
        BMM350_SIGNED_8_BIT) / 32.0f;

    bmm350_temp_coefficient_mag_y = fix_sign(
        otp_data[19] & 0b0000000011111111, 
        BMM350_SIGNED_8_BIT) / 32.0f;

    bmm350_temp_coefficient_mag_z = fix_sign(
        otp_data[20] & 0b0000000011111111, 
        BMM350_SIGNED_8_BIT) / 32.0f;


        
    bmm350_temp_coefficient_sensitivity_mag_x = fix_sign(
        (otp_data[18] & 0b1111111100000000) >> 8, 
        BMM350_SIGNED_8_BIT) / 16384.0f;

    bmm350_temp_coefficient_sensitivity_mag_y = fix_sign(
        (otp_data[19] & 0b1111111100000000) >> 8, 
        BMM350_SIGNED_8_BIT) / 16384.0f;

    bmm350_temp_coefficient_sensitivity_mag_z = (fix_sign(
        (otp_data[20] & 0b1111111100000000) >> 8, 
        BMM350_SIGNED_8_BIT) / 16384.0f
    ) - 0.0001f;



    bmm350_cross_axis_mag_x_y = fix_sign(
        otp_data[21] & 0b0000000011111111,
        BMM350_SIGNED_8_BIT) / 800.0f;
        
    bmm350_cross_axis_mag_y_x = fix_sign(
        (otp_data[21] & 0b1111111100000000) >> 8,
        BMM350_SIGNED_8_BIT) / 800.0f;
        
    bmm350_cross_axis_mag_z_x = fix_sign(
        otp_data[22] & 0b0000000011111111,
        BMM350_SIGNED_8_BIT) / 800.0f;
        
    bmm350_cross_axis_mag_z_y = fix_sign(
        (otp_data[22] & 0b1111111100000000) >> 8,
        BMM350_SIGNED_8_BIT) / 800.0f;



    bmm350_reference_temperature = (fix_sign(
        otp_data[24], 
        BMM350_SIGNED_16_BIT) / 512.0f
    ) + 23.0f;

    

    bmm350_variant_id = (otp_data[30] & 0b0111111100000000) >> 9;
}

void bmm350_apply_temperature_compensation(float *magnetometer_and_temperature_data){
    
    float cr_ax_comp_x, cr_ax_comp_y, cr_ax_comp_z;
    float dut_offset_coef[3], dut_sensit_coef[3], dut_tco[3], dut_tcs[3];

    /* Apply compensation to temperature reading */
    magnetometer_and_temperature_data[3] = (1.0f + bmm350_sensitivity_temp) * magnetometer_and_temperature_data[3] + bmm350_offset_temp;

    /* Store magnetic compensation structure to an array */
    dut_offset_coef[0] = bmm350_offset_mag_x;
    dut_offset_coef[1] = bmm350_offset_mag_y;
    dut_offset_coef[2] = bmm350_offset_mag_z;

    dut_sensit_coef[0] = bmm350_sensitivity_mag_x;
    dut_sensit_coef[1] = bmm350_sensitivity_mag_y;
    dut_sensit_coef[2] = bmm350_sensitivity_mag_z;

    dut_tco[0] = bmm350_temp_coefficient_mag_x;
    dut_tco[1] = bmm350_temp_coefficient_mag_y;
    dut_tco[2] = bmm350_temp_coefficient_mag_z;

    dut_tcs[0] = bmm350_temp_coefficient_sensitivity_mag_x;
    dut_tcs[1] = bmm350_temp_coefficient_sensitivity_mag_y;
    dut_tcs[2] = bmm350_temp_coefficient_sensitivity_mag_z;

    /* Compensate raw magnetic data */
    for (uint8_t i = 0; i < 3; i++)
    {
        magnetometer_and_temperature_data[i] *= 1 + dut_sensit_coef[i];
        magnetometer_and_temperature_data[i] += dut_offset_coef[i];
        magnetometer_and_temperature_data[i] += dut_tco[i] * (magnetometer_and_temperature_data[3] - bmm350_reference_temperature);
        magnetometer_and_temperature_data[i] /= 1 + dut_tcs[i] * (magnetometer_and_temperature_data[3] - bmm350_reference_temperature);
    }

    cr_ax_comp_x = (magnetometer_and_temperature_data[0] - bmm350_cross_axis_mag_x_y * magnetometer_and_temperature_data[1]) /
                    (1 - bmm350_cross_axis_mag_y_x * bmm350_cross_axis_mag_x_y);
    cr_ax_comp_y = (magnetometer_and_temperature_data[1] - bmm350_cross_axis_mag_y_x * magnetometer_and_temperature_data[0]) /
                    (1 - bmm350_cross_axis_mag_y_x * bmm350_cross_axis_mag_x_y);
    cr_ax_comp_z =
        (magnetometer_and_temperature_data[2] +
            (magnetometer_and_temperature_data[0] *
            (bmm350_cross_axis_mag_y_x * bmm350_cross_axis_mag_z_y -
            bmm350_cross_axis_mag_z_x) - magnetometer_and_temperature_data[1] *
            (bmm350_cross_axis_mag_z_y - bmm350_cross_axis_mag_x_y *
            bmm350_cross_axis_mag_z_x)) /
            (1 - bmm350_cross_axis_mag_y_x * bmm350_cross_axis_mag_x_y));

    magnetometer_and_temperature_data[0] = cr_ax_comp_x;
    magnetometer_and_temperature_data[1] = cr_ax_comp_y;
    magnetometer_and_temperature_data[2] = cr_ax_comp_z;
}

void bmm350_magnetometer_readings_micro_teslas(float *data, uint8_t perform_temperature_correction, uint8_t apply_rotation_into_accelerometer_position){
    // Read magnetometer data and temperature data

    uint8_t retrieved_data[14]; // First two bytes are dummy bytes
    HAL_I2C_Mem_Read(
        bmm350_i2c_handle,
        BMM350_I2C_ID,
        BMM350_MAG_X_XLSB_REG,
        1,
        retrieved_data,
        14, // read nine registers in total
        100
    );

    // Combine the bytes into 24 bit values and convert them to 24 bit ints 
    int32_t mag_x_raw = fix_sign(
        ((uint32_t)retrieved_data[4] << 16) |  ((uint32_t)retrieved_data[3] << 8)  | (uint32_t)retrieved_data[2],
        BMM350_SIGNED_24_BIT);

    int32_t mag_y_raw = fix_sign(
        ((uint32_t)retrieved_data[7] << 16) |  ((uint32_t)retrieved_data[6] << 8)  | (uint32_t)retrieved_data[5],
        BMM350_SIGNED_24_BIT);

    int32_t mag_z_raw = fix_sign(
        ((uint32_t)retrieved_data[10] << 16) | ((uint32_t)retrieved_data[9] << 8)  | (uint32_t)retrieved_data[8],
        BMM350_SIGNED_24_BIT);

    int32_t temp_raw = fix_sign(
        ((int32_t)retrieved_data[13] << 16) | ((int32_t)retrieved_data[12] << 8)  | (int32_t)retrieved_data[11],
        BMM350_SIGNED_24_BIT);

    // Convert the magnetometer values to micro teslas and celsius for temperature
    float data_output[] = {0.0f, 0.0f, 0.0f, 0.0f};
    data_output[0] = (float)mag_x_raw * lsb_to_ut_degc[0];
    data_output[1] = (float)mag_y_raw * lsb_to_ut_degc[1];
    data_output[2] = (float)mag_z_raw * lsb_to_ut_degc[2];
    data_output[3] = (float)temp_raw * lsb_to_ut_degc[3];

    // Some additional step for temperature value
    if(data_output[3] > 0.0){
        data_output[3] = data_output[3] - (1.0f * 25.49f);
    }else if(data_output[3] < 0.0){
        data_output[3] = data_output[3] - (-1.0f * 25.49f);
    }

    // Perform the temperature compensation of the data
    if(perform_temperature_correction){
        bmm350_apply_temperature_compensation(data_output);
    }

    // Keep the value before any soft or hard iron calibrations are applied
    bmm350_old_magnetometer_readings[0] = data_output[0];
    bmm350_old_magnetometer_readings[1] = data_output[1];
    bmm350_old_magnetometer_readings[2] = data_output[2];

    bmm350_old_temperature_reading = data_output[3];

    // Apply soft and hard iron calibrations
    for (uint8_t i = 0; i < 3; i++){
        data_output[i] = data_output[i] - bmm350_hard_iron[i];
    }
    
    float temp[3];
    for (uint8_t i = 0; i < 3; i++){
        temp[i] = (bmm350_soft_iron[i][0] * data_output[0]) +
                  (bmm350_soft_iron[i][1] * data_output[1]) +
                  (bmm350_soft_iron[i][2] * data_output[2]);
    }
    for (uint8_t i = 0; i < 3; i++) data_output[i] = temp[i];
    
    
    bmm350_magnetometer_readings_without_rotation[0] = data_output[0];
    bmm350_magnetometer_readings_without_rotation[1] = data_output[1];
    bmm350_magnetometer_readings_without_rotation[2] = data_output[2];

    if(apply_rotation_into_accelerometer_position){
        float temp2[3];
        for (uint8_t i = 0; i < 3; i++){
            temp2[i] = (bmm350_rotation_matrix[i][0] * data_output[0]) +
                    (bmm350_rotation_matrix[i][1] * data_output[1]) +
                    (bmm350_rotation_matrix[i][2] * data_output[2]);
        }
        for (uint8_t i = 0; i < 3; i++) data_output[i] = temp2[i];
    }

    data[0] = data_output[0];
    data[1] = data_output[1];
    data[2] = data_output[2];
    // data[3] = data_output[3];
}

void bmm350_previous_raw_magetometer_readings(float *data){
    data[0] = bmm350_old_magnetometer_readings[0];
    data[1] = bmm350_old_magnetometer_readings[1];
    data[2] = bmm350_old_magnetometer_readings[2];
}

float bmm350_previous_temperature_reading(){
    return bmm350_old_temperature_reading;
}

void bmm350_set_rotation_matrix(const float rotation_matrix[3][3]){
    for (uint8_t i = 0; i < 3; i++){
        for (uint8_t k = 0; k < 3; k++){
            bmm350_rotation_matrix[i][k] = rotation_matrix[i][k];
        }
    }
}

void bmm350_magnetometer_readings_micro_teslas_unrotated(float *data){
    data[0] = bmm350_magnetometer_readings_without_rotation[0];
    data[1] = bmm350_magnetometer_readings_without_rotation[1];
    data[2] = bmm350_magnetometer_readings_without_rotation[2];
}
