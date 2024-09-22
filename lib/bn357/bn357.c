#include "./bn357.h"

#include "math.h"

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

UART_HandleTypeDef *uart;
DMA_HandleTypeDef *hdma_uart_rx;

volatile float m_latitude = 0.0;
volatile char m_latitude_direction = ' ';
volatile float m_longitude = 0.0;
volatile char m_longitude_direction = ' ';
volatile float m_altitude = 0.0;
volatile float m_altitude_geoid = 0.0;
volatile float m_accuracy = 0.0;
volatile uint8_t m_satellites_quantity = 0.0;
volatile uint8_t m_fix_quality = 0;
volatile uint8_t m_time_utc_hours = 0;
volatile uint8_t m_time_utc_minutes = 0;
volatile uint8_t m_time_utc_seconds = 0;
volatile uint8_t m_up_to_date_date = 0;
volatile uint32_t m_time_raw = 0;
volatile uint8_t m_fix_type = 0;

#define BN357_DEBUG 0
#define BN357_TRACK_TIMING 0

volatile uint8_t continue_with_gps_data = 0;

#define GPS_OUTPUT_BUFFER_SIZE 550
#define GPS_RECEIVE_BUFFER_SIZE 550

uint8_t gps_output_buffer[GPS_OUTPUT_BUFFER_SIZE];
uint8_t receive_buffer[GPS_RECEIVE_BUFFER_SIZE];

// Interrupt for uart 2 data received
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
    // make sure the dma is initialized before uart for this to work
    // and that dma is initialized after GPIO.

    // If the stm32 is restarted  while it is initializing dma the
    // data arrives from the uart and it fucks up. So you have to restart it a few times

#if(BN357_TRACK_TIMING)
    uint32_t start_time = HAL_GetTick();
#endif

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);

    if (huart->Instance == USART2){   
        // char gps_data[] = "$GNGGA,164257.60,5551.76402,N,00950.62685,E,1,10,0.58,1.2,M,43.4,M,,*4B\n$GNGSA,A,3,30,07,09,11,18,14,,,,,,,1.05,0.58,0.87*1B\n$GNGSA,A,3,80,87,69,73,,,,,,,,,1.05,0.58,0.87*16\n";
        // uint16_t gps_data_length = strlen(gps_data)+1;
        
        memcpy((uint8_t *)gps_output_buffer, receive_buffer, Size);

        if(continue_with_gps_data){
            bn357_parse_and_store((unsigned char*)gps_output_buffer, Size);
            // if(bn357_parse_and_store((unsigned char*)gps_output_buffer, Size)){
            //     // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
            //     // printf("Working UART2, %f, %f\n", bn357_get_latitude_decimal_format(), bn357_get_longitude_decimal_format());
            // }else{
            //     // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
            //     // printf("Failed UART2, %f, %f\n", bn357_get_latitude_decimal_format(), bn357_get_longitude_decimal_format());
            // }
        }else{
            bn357_parse_and_store((unsigned char*)gps_output_buffer, Size);
        }

        /* start the DMA again */
        HAL_UARTEx_ReceiveToIdle_DMA(uart, (uint8_t *)receive_buffer, GPS_RECEIVE_BUFFER_SIZE);
        __HAL_DMA_DISABLE_IT(hdma_uart_rx, DMA_IT_HT);

#if(BN357_TRACK_TIMING)
        uint32_t end_time = HAL_GetTick();
        printf("Gpt %ld\n", end_time-start_time);
#endif
    }
}

// Interrupt for uart 2 when it crashes to restart it
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
    if (huart->Instance == USART2){
        // printf("Error UART 2\n");
        HAL_UART_DeInit(uart);
        HAL_UART_Init(uart);

        HAL_UARTEx_ReceiveToIdle_DMA(uart, receive_buffer, GPS_RECEIVE_BUFFER_SIZE);
        __HAL_DMA_DISABLE_IT(hdma_uart_rx, DMA_IT_HT);
    }
}

volatile uint8_t m_minimal_gps_parse_enabled;

uint8_t init_bn357(UART_HandleTypeDef *uart_temp, DMA_HandleTypeDef *hdma_uart_rx_temp, uint8_t minimal_gps_parse_enabled){
    uart = uart_temp;
    hdma_uart_rx = hdma_uart_rx_temp;
    m_minimal_gps_parse_enabled = minimal_gps_parse_enabled;
    return 1;
}

// Check the status of the gps reading. If they are up to date or not
uint8_t bn357_get_status_up_to_date(uint8_t reset_afterwards){

    // To initialize the settings of the gps use UBlox desktop app the bn-357 is just a UBlox M8030 chip
    uint8_t temp_status = m_up_to_date_date;
    if(reset_afterwards){
        m_up_to_date_date = 0;
    }
    return temp_status;
}

void bn357_get_clear_status(){
    m_up_to_date_date = 0;
}

void bn357_start_uart_interrupt(){
    HAL_UARTEx_ReceiveToIdle_DMA(uart, receive_buffer, GPS_RECEIVE_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(hdma_uart_rx, DMA_IT_HT);
}

void bn357_toggle_gps_logging(uint8_t status){
    continue_with_gps_data = status;
}

uint8_t bn357_parse_and_store(unsigned char *gps_output_buffer, uint16_t size_of_buffer){
#if(BN357_TRACK_TIMING)
    uint32_t start_time = HAL_GetTick();
#endif

    // reset the state of successful gps parse
    m_up_to_date_date = 0;
    // find the starting position of the substring
    char* start = strstr(gps_output_buffer, "$GNGGA");  
    // Check if the start exists
    if(start == NULL){
#if(BN357_DEBUG)
        printf("GNGGA: Failed at Check if the start exists\n");
#endif
        return 0;
    }
    char* end = strchr(start, '\n');  // find the position of the first new line character after the substring
    // check if the end of it exists
    if (end == NULL){
#if(BN357_DEBUG)
        printf("GNGGA: Failed at check if the end of it exists\n");
#endif
        return 0;
    }
    int length = end - start;  // calculate the length of the substring
    char sub_string_gngga[length + 1];  // create a new string to store the substring, plus one for null terminator
    strncpy(sub_string_gngga, start, length);  // copy the substring to the new string
    sub_string_gngga[length] = '\0';  // add a null terminator to the new string
// #if(BN357_DEBUG)
//     printf("GNGGA Substring: %s\n", sub_string_gngga);  // print the substring
// #endif



    // find utc time
    start = strchr(sub_string_gngga, ',') + 1;
    if(start[0] == ','){
#if(BN357_DEBUG)
        printf("GNGGA: Failed at time\n");
#endif
        return 0;
    }
    end = strchr(start, ',');

    if(!m_minimal_gps_parse_enabled){
        length = end - start;
        char utc_time_string[length + 1];
        strncpy(utc_time_string, start, length);
        utc_time_string[length] = '\0';
        uint32_t time = atoi(utc_time_string);
        m_time_utc_seconds = time % 100;
        m_time_utc_minutes = ((time % 10000) - m_time_utc_seconds) / 100;
        m_time_utc_hours = ((time % 1000000) - (m_time_utc_minutes * 100) - m_time_utc_seconds) / 10000;
    #if(BN357_DEBUG)
        printf("GNGGA UTC time: %d%d%d%\n", m_time_utc_hours, m_time_utc_minutes, m_time_utc_seconds);
    #endif
    }




    // find latitude
    start = end+1;
    if(start[0] == ','){
#if(BN357_DEBUG)
        printf("GNGGA: Failed at latitude\n");
#endif
        return 0;
    }
    end = strchr(start, ',');
    length = end - start;
    char latitude_string[length + 1];
    strncpy(latitude_string, start, length);
    latitude_string[length] = '\0';
    uint16_t latitude_temp = atoi(latitude_string);
    uint16_t latitude_degrees = ((latitude_temp % 10000) - (latitude_temp % 100)) / 100;
    m_latitude = (float)latitude_degrees + ((atof(latitude_string) - (float)latitude_degrees * 100)/60);
#if(BN357_DEBUG)
    printf("GNGGA Latitude: %f\n", m_latitude);
#endif


    // find latitude direction
    m_latitude_direction = end+2;

    // find longitude
    start = end+3; // skip the N character
    if(start[0] == ','){
#if(BN357_DEBUG)
        printf("GNGGA: Failed at longitude\n");
#endif
        return 0;
    }
    end = strchr(start, ',');
    length = end - start;
    char longitude_string[length + 1];
    strncpy(longitude_string, start, length);
    longitude_string[length] = '\0';
    uint16_t longitude_temp = atoi(longitude_string);
    uint16_t longitude_degrees = ((longitude_temp % 10000) - (longitude_temp % 100)) / 100;
    m_longitude = (float)longitude_degrees + ((atof(longitude_string) - (float)longitude_degrees * 100)/60);
#if(BN357_DEBUG)
    printf("GNGGA Longitude: %f\n", m_longitude);
#endif

    // find longitude direction
    m_longitude_direction = end+2;

    // find fix quality
    start = end+3; // skip the E character
    if(start[0] == ','){
#if(BN357_DEBUG)
        printf("GNGGA: Failed at fix quality\n");
#endif
        return 0;
    }
    if(!m_minimal_gps_parse_enabled){
        end = strchr(start, ',');
        length = end - start;
        char fix_quality_string[length + 1];
        strncpy(fix_quality_string, start, length);
        fix_quality_string[length] = '\0';
        m_fix_quality = atoi(fix_quality_string);
    #if(BN357_DEBUG)
        printf("GNGGA Fix quality: %d\n", m_fix_quality);
    #endif
    }



    // find satellites quantity
    start = end+1;
    if(start[0] == ','){
#if(BN357_DEBUG)
        printf("GNGGA: Failed at satellites quantity\n");
#endif
        return 0;
    }
    if(!m_minimal_gps_parse_enabled){
        end = strchr(start, ',');
        length = end - start;
        char satellites_quantity_string[length + 1];
        strncpy(satellites_quantity_string, start, length);
        satellites_quantity_string[length] = '\0';
        m_satellites_quantity = atoi(satellites_quantity_string);
    #if(BN357_DEBUG)
        printf("GNGGA Satellites quantity: %d\n", m_satellites_quantity);
    #endif
    }


    // find accuracy
    start = end+1;
    if(start[0] == ','){
#if(BN357_DEBUG)
        printf("GNGGA: Failed at accuracy\n");
#endif
        return 0;
    } 

    if(!m_minimal_gps_parse_enabled){
        end = strchr(start, ',');
        length = end - start;
        char accuracy_quantity_string[length + 1];
        strncpy(accuracy_quantity_string, start, length);
        accuracy_quantity_string[length] = '\0';
        m_accuracy = atof(accuracy_quantity_string);
    #if(BN357_DEBUG)
        printf("GNGGA Accuracy: %f\n", m_accuracy);
    #endif
    }


    // find altitude above sea levels
    start = end+1;
    if(start[0] == ','){
#if(BN357_DEBUG)
        printf("GNGGA: Failed at altitude\n");
#endif
        return 0;
    } 

    if(!m_minimal_gps_parse_enabled){
        end = strchr(start, ',');
        length = end - start;
        char altitude_string[length + 1];
        strncpy(altitude_string, start, length);
        altitude_string[length] = '\0';
        m_altitude = atof(altitude_string);
    #if(BN357_DEBUG)
        printf("GNGGA Altitude: %f\n", m_altitude);
    #endif
    }


    // find geoid altitude deviation
    start = end+3;
    if(start[0] == ','){
#if(BN357_DEBUG)
        printf("GNGGA: Failed at Geoid altitude\n");
#endif
        return 0;
    } 

    if(!m_minimal_gps_parse_enabled){
        end = strchr(start, ',');
        length = end - start;
        char geoid_altitude_string[length + 1];
        strncpy(geoid_altitude_string, start, length);
        geoid_altitude_string[length] = '\0';
        m_altitude_geoid = atof(geoid_altitude_string);
    #if(BN357_DEBUG)
        printf("GNGGA Geoid altitude: %f\n", m_altitude_geoid);
    #endif
    }

    m_up_to_date_date = 1;



















    start = strstr(gps_output_buffer, "$GNGSA");  
    // Check if the start exists
    if(start == NULL){
#if(BN357_DEBUG)
        printf("GNGSA: Failed at Check if the start exists\n");
#endif
        return 0;
    }
    end = strchr(start, '\n');  // find the position of the first new line character after the substring
    // check if the end of it exists
    if (end == NULL){
#if(BN357_DEBUG)
        printf("GNGSA: Failed at check if the end of it exists\n");
#endif
        return 0;
    }
    length = end - start;  // calculate the length of the substring
    char sub_string_gngsa[length + 1];  // create a new string to store the substring, plus one for null terminator
    strncpy(sub_string_gngsa, start, length);  // copy the substring to the new string
    sub_string_gngsa[length] = '\0';  // add a null terminator to the new string
// #if(BN357_DEBUG)
//     printf("GNGSA Substring: %s\n", sub_string_gngsa);  // print the substring
// #endif



    start = strchr(sub_string_gngsa, ',') + 1;
    if(start[0] == ','){
#if(BN357_DEBUG)
        printf("GNGSA: Failed at skip\n");
#endif
        return 0;
    }
    end = strchr(start, ',');
    // Skip whatever is here



    start = end+1;
    if(start[0] == ','){
#if(BN357_DEBUG)
        printf("GNGSA: Failed at fix type\n");
#endif
        return 0;
    }
    end = strchr(start, ',');
    end = strchr(start, ',');
    length = end - start;
    char fix_type_string[length + 1];
    strncpy(fix_type_string, start, length);
    fix_type_string[length] = '\0';
    m_fix_type = atoi(fix_type_string);
#if(BN357_DEBUG)
    printf("GNGSA: Fix type: %d\n", m_fix_type);
#endif




#if(BN357_DEBUG)
    printf("GPS data read successful\n");
#endif
    return 1;
}


float bn357_get_latitude_decimal_format(){
    return m_latitude;
}

char bn357_get_latitude_direction(){
    return m_latitude_direction;
}

// For logging
float bn357_get_longitude_decimal_format(){
    return m_longitude;
}

// For calculations
float bn357_get_linear_longitude_decimal_format(){
    // The longitude increases in precision as we go up in latitude
    // That is not good for pid which measures proportional error
    // The pid will think the longitude is more responsive
    // Scale it to be linear
    return m_longitude*cos(m_latitude * (M_PI / 180.0));
}

char bn357_get_longitude_direction(){
    return m_longitude_direction;
}

float bn357_get_altitude_meters(){
    return m_altitude;
}

float bn357_get_geoid_altitude_meters(){
    return m_altitude_geoid;
}

float bn357_get_accuracy(){
    return m_accuracy;
}

uint8_t bn357_get_satellites_quantity(){
    return m_satellites_quantity;
}

uint8_t bn357_get_fix_quality(){
    return m_fix_quality;
}

uint8_t bn357_get_utc_time_hours(){
    return m_time_utc_hours;
}

uint8_t bn357_get_utc_time_minutes(){
    return m_time_utc_minutes;
}

uint8_t bn357_get_utc_time_seconds(){
    return m_time_utc_seconds;
}

uint8_t bn357_get_utc_time_raw(){
    return m_time_raw;
}

uint8_t bn357_get_fix_type(){
    return m_fix_type;
}
