#include "ms5611.h"

// REMEMBER CSB pin is connected to VCC
// #define MS5611_I2C_ID 0b11101110
#define MS5611_I2C_ADDR_7BIT   0x76
#define MS5611_I2C_ADDR_WRITE  (MS5611_I2C_ADDR_7BIT << 1)       // 0xEC
#define MS5611_I2C_ADDR_READ   ((MS5611_I2C_ADDR_7BIT << 1) | 1) // 0xED

#define MS5611_RESET 0x1E
// The bits 1-4 are the bits used to specify what part of the PROM to read. EX: xxxx011x reads the 3rd PROM value
#define MS5611_READ_RPOM_BASE 0b10100000

#define MS5611_READ_ADC_RESULT 0x00

#define MS5611_ID_VALUE 0x58
#define MS5611_ID_REG 0x58

uint16_t factory_data_and_setup = 0;
uint16_t preasure_sensitivity = 0;
uint16_t preasure_offset = 0;
uint16_t temperature_coeficient_of_preasure_sensitivity = 0;
uint16_t temperature_coeficient_of_preasure_offset = 0;
uint16_t reference_temperature = 0;
uint16_t temperature_coeficient_of_the_temperature = 0;
uint16_t serial_code_and_crc = 0;

uint8_t m_prefered_conversion_temperature = 0;
uint8_t m_prefered_conversion_preasure = 0;

float m_temperature_celsius = 0.0f;
int32_t  m_temperature_celsius_times_100 = 0; // TEMP

uint32_t digital_temperature_value = 0; // D2
int32_t m_dT = 0; // dT

uint32_t digital_preasure_value = 0; // D1
int64_t m_off = 0; // OFF
int64_t m_sens = 0; // SENS
int32_t m_preasure_mbar_times_100 = 0; // P
float m_preasure_mbar = 0;

int32_t t2;
int64_t off2;
int64_t sens2;


float reference_pressure = 0.0;



I2C_HandleTypeDef *m_i2c_handle;

HAL_StatusTypeDef ms5611_reset();
HAL_StatusTypeDef ms5611_read_prom_index(uint8_t index, uint16_t *prom_data);

uint8_t write_bit(){
    return 0;
}

uint8_t read_bit(){
    return 1;
}

uint8_t init_ms5611(I2C_HandleTypeDef *i2c_handle){
    m_i2c_handle = i2c_handle;


    // Reset MS5611
    HAL_StatusTypeDef reset = ms5611_reset();

    if(reset != HAL_OK){

        printf("MS5611 reset failed %d\n", reset);
        return 0;
    }

    // WAIT 3 ms after reset
    HAL_Delay(3);

    // Load the PROM data
    uint16_t prom_data[8];
    for (uint8_t i = 0; i < 8; i++){
        HAL_StatusTypeDef prom_read = ms5611_read_prom_index(i, &prom_data[i]);
        if(prom_read != HAL_OK){
            printf("MS5611 PROM read failed\n");
            return 0;
        }
    }

    factory_data_and_setup = prom_data[0];
    preasure_sensitivity = prom_data[1];
    preasure_offset = prom_data[2];
    temperature_coeficient_of_preasure_sensitivity = prom_data[3];
    temperature_coeficient_of_preasure_offset = prom_data[4];
    reference_temperature = prom_data[5];
    temperature_coeficient_of_the_temperature = prom_data[6];
    serial_code_and_crc = prom_data[7];

    // AFTER conversion wait ~9ms for the conversion to finish

    printf("MS5611 initialized\n");
    return 1;
}

// Initiate reset sequence. REMEMBER TO WAIT 3ms after reset
HAL_StatusTypeDef ms5611_reset(){
    uint8_t ms5611_reset = 0b00000000;
    ms5611_reset |= MS5611_RESET;
    HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(
        m_i2c_handle,
        MS5611_I2C_ADDR_WRITE,
        &ms5611_reset,
        1,
        5
    );

    return ret;
}

// Read the prome value at index and return 16 bit value to reference variable provided
HAL_StatusTypeDef ms5611_read_prom_index(uint8_t index, uint16_t *prom_data){
    // There are only 8 PROM values
    if(index > 7){
        return HAL_ERROR;
    }

    // Initiate the prom read
    uint8_t ms5611_read_prom_index = 0b00000000;
    ms5611_read_prom_index |= MS5611_READ_RPOM_BASE;
    ms5611_read_prom_index |= (index << 1);
    HAL_StatusTypeDef initiate_prom_read = HAL_I2C_Master_Transmit(
        m_i2c_handle,
        MS5611_I2C_ADDR_WRITE,
        &ms5611_read_prom_index,
        1,
        5
    );

    if(initiate_prom_read != HAL_OK){
        return initiate_prom_read;
    }

    // Read the 16 bit response
    uint8_t one_prom_data_read[2];
    HAL_StatusTypeDef read_response = HAL_I2C_Master_Receive(
        m_i2c_handle,
        MS5611_I2C_ADDR_READ,
        one_prom_data_read,
        2,
        5
    );

    // Combine into 16 bit value and store in prom_data
    // WHAT DO THE first 4 bits of MSB mean?
    *prom_data = (one_prom_data_read[0] << 8) | one_prom_data_read[1];

    return read_response;
}


// Start conversion of specified type. Remember to wait 9ms (8.22ms) for conversion to finish
HAL_StatusTypeDef ms5611_start_conversion(uint8_t conversion_type){
    uint8_t ms5611_start_conversion = 0b00000000;
    ms5611_start_conversion |= conversion_type;
    HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(
        m_i2c_handle,
        MS5611_I2C_ADDR_WRITE,
        &ms5611_start_conversion,
        1,
        5
    );

    return ret;
}

// Send a command to initiate a read of the conversion result
HAL_StatusTypeDef ms5611_initiate_conversion_read(){
    uint8_t ms5611_initiate_conversion_read = 0b00000000;
    ms5611_initiate_conversion_read |= MS5611_READ_ADC_RESULT;
    HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(
        m_i2c_handle,
        MS5611_I2C_ADDR_WRITE,
        &ms5611_initiate_conversion_read + write_bit(),
        1,
        5
    );

    return ret;
}

// Read the conversion result and store in conversion_result array. Array has to be of at least size of 3 uint8_t
HAL_StatusTypeDef ms5611_read_conversion_result(uint8_t *conversion_result){
    HAL_StatusTypeDef ret = HAL_I2C_Master_Receive(
        m_i2c_handle,
        MS5611_I2C_ADDR_READ,
        conversion_result,
        3,
        5
    );

    return ret;
}

void ms5611_set_prefered_data_conversion_temperature(uint8_t prefered_conversion_temperature){
    m_prefered_conversion_temperature = prefered_conversion_temperature;
}

void ms5611_set_prefered_data_conversion_preasure(uint8_t prefered_conversion_preasure){
    m_prefered_conversion_preasure = prefered_conversion_preasure;
}

void ms5611_initiate_prefered_temperature_conversion(){
    ms5611_start_conversion(m_prefered_conversion_temperature);
}

void ms5611_initiate_prefered_preasure_conversion(){
    ms5611_start_conversion(m_prefered_conversion_preasure);
}


float ms5611_read_conversion_temperature_celsius(){
    ms5611_initiate_conversion_read();

    uint8_t conversion_results[3];
    ms5611_read_conversion_result(conversion_results);

    digital_temperature_value = (conversion_results[0] << 16) | (conversion_results[1] << 8) | conversion_results[2];

    // First order temperature compensation
    m_dT = digital_temperature_value - reference_temperature * 256; //D2 - C5 * 2^8


    // SOMETHING WRONG HERE
    m_off = (int64_t)preasure_offset * 65536 + ((int64_t)temperature_coeficient_of_preasure_offset * (int64_t)m_dT) / 128; // C2 * 2^16 + (C4 * dT) / 2^7

    m_sens = (int64_t)preasure_sensitivity * 32768 + ((int64_t)temperature_coeficient_of_preasure_sensitivity * (int64_t)m_dT) / 256; // C1 * 2^15 + (C3 * dT) / 2^8

    // Finally calculating the temperature
    m_temperature_celsius_times_100 = 2000 + (int64_t)m_dT * (int64_t)temperature_coeficient_of_the_temperature / 8388608; // 2000 + dT * C6 / 2^23
    m_temperature_celsius = ((float)m_temperature_celsius_times_100) / 100.0f;

    // Second order temperature compensation
    if(m_temperature_celsius_times_100 < 20.0f * 100.0f){
        t2 = (int64_t)m_dT * (int64_t)m_dT / 2147483648; // dT^2 / 2^31
        off2 = 5 * ((int64_t)m_temperature_celsius_times_100 - 2000) * ((int64_t)m_temperature_celsius_times_100 - 2000) / 2; // 5 * (TEMP - 2000)^2 / 2^1
        sens2 = off2 / 2; // 5 * (TEMP - 2000)^2 / 2^2

        if(m_temperature_celsius_times_100 < -15.0f * 100.0f){
            off2 = off2 + 7  * ((int64_t)m_temperature_celsius_times_100 + 1500) * ((int64_t)m_temperature_celsius_times_100 + 1500); // OFF2 + 7 * (TEMP + 1500)^2
            sens2 = sens2 + 11 * ((int64_t)m_temperature_celsius_times_100 + 1500) * ((int64_t)m_temperature_celsius_times_100 + 1500) / 2; // SENS2 + 11 * (TEMP + 1500)^2 / 2^1
        }
    }else{
        t2 = 0;
        off2 = 0;
        sens2 = 0;
    }

    m_temperature_celsius_times_100 = m_temperature_celsius_times_100 - t2;
    m_off = m_off - off2;
    m_sens = m_sens - sens2;


    return m_temperature_celsius;
}

float ms5611_read_conversion_preasure_hPa(){
    ms5611_initiate_conversion_read();

    uint8_t conversion_results[3];
    ms5611_read_conversion_result(conversion_results);

    digital_preasure_value = (conversion_results[0] << 16) | (conversion_results[1] << 8) | conversion_results[2];
    m_preasure_mbar_times_100 = ((int64_t)digital_preasure_value * m_sens / 2097152 - m_off) / 32768; // (D1 * SENS / 2^21 - OFF) / 2^15
    m_preasure_mbar = ((float)m_preasure_mbar_times_100) / 100.0f;
    return m_preasure_mbar; // 1 mbar = 1 hPa
}

float ms5611_read_existing_conversion_preasure_hPa(){
    return m_preasure_mbar;
}

uint32_t ms5611_conversion_wait_time_microseconds(){
    // The conversion time is 8.22ms
    return 8220;
}

float ms5611_get_height_meters_above_sea_level(uint8_t use_existing_pressure, uint8_t call_for_conversion, float pressure_sea_level_hpa, float temperature_sea_level){

    float pressure_hpa = 0.0f;
    if(use_existing_pressure){
        pressure_hpa = m_preasure_mbar;
    }else{
        if(call_for_conversion){
            ms5611_initiate_prefered_preasure_conversion();
            HAL_Delay(10);
        }

        pressure_hpa = ms5611_read_conversion_preasure_hPa();
    }

    return 44330.0f * (1.0f - pow(pressure_hpa / pressure_sea_level_hpa, 0.1903f));
}

float ms5611_get_height_meters_from_reference(uint8_t use_existing_pressure, uint8_t call_for_conversion, uint8_t reset_reference){

    float pressure_hpa = 0.0f;
    if(use_existing_pressure){
        pressure_hpa = m_preasure_mbar;
    }else{
        if(call_for_conversion){
            ms5611_initiate_prefered_preasure_conversion();
            HAL_Delay(10);
        }
        pressure_hpa = ms5611_read_conversion_preasure_hPa();
    }

    if(reference_pressure == 0.0 || reset_reference == 1){
        // printf("data: %.2f %.2f\n",reference_pressure, pressure);
        reference_pressure = pressure_hpa;
        // pressure_hpa = 1.0f / pressure_hpa; // Pre divided to improve perofrmance
        printf("MS5611 reference pressure: %f\n", reference_pressure);
    }
    
    // printf("44330 * (1.0 - pow(%.2f / %.2f, 0.1903))\n", pressure_hpa, reference_pressure);
    // printf("BMP280 pressure %f / ref %f\n", pressure, reference_pressure);
    return 44330.0f * (1.0f - pow(pressure_hpa / reference_pressure, 0.1903f));
}

float ms5611_get_height_centimeters_from_reference(uint8_t use_existing_pressure, uint8_t call_for_conversion, uint8_t reset_reference){
    return ms5611_get_height_meters_from_reference(use_existing_pressure, call_for_conversion, reset_reference) * 100.0f;
}

void ms5611_set_reference_pressure_from_number_of_samples(uint16_t sample_size){
    // If temperature has not been read yet
    if(m_temperature_celsius == 0.0f){
        ms5611_initiate_prefered_temperature_conversion();
        HAL_Delay(10);
        ms5611_read_conversion_temperature_celsius();
    }

    float total_pressure = 0;
    for (uint16_t i = 0; i < sample_size; i++){
        ms5611_initiate_prefered_preasure_conversion();
        HAL_Delay(10);
        total_pressure += ms5611_read_conversion_preasure_hPa();
    }

    reference_pressure = total_pressure / sample_size;
    printf("MS5611 reference pressure: %f\n", reference_pressure);
}
