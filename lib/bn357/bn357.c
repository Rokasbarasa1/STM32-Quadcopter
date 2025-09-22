#include "./bn357.h"

#include "math.h"
#include "../utils/math_constants.h"

UART_HandleTypeDef *uart;
DMA_HandleTypeDef *hdma_uart_rx;

volatile float m_latitude = 0.0;
volatile char m_latitude_direction = ' ';
volatile float m_longitude = 0.0;
volatile char m_longitude_direction = ' ';
volatile float m_altitude = 0.0;
volatile float m_altitude_geoid = 0.0;
volatile float m_accuracy = 0.0;
volatile uint8_t m_satellites_quantity = 0;
volatile uint8_t m_fix_quality = 0;
volatile uint8_t m_time_utc_hours = 0;
volatile uint8_t m_time_utc_minutes = 0;
volatile uint8_t m_time_utc_seconds = 0;
volatile uint8_t m_up_to_date_date = 0;
volatile uint32_t m_time_raw = 0;
volatile uint8_t m_fix_type = 0;
volatile float m_course_over_ground = 0;
volatile uint8_t m_date_day = 0;
volatile uint8_t m_date_month = 0;
volatile uint8_t m_date_year = 0;


#define BN357_DEBUG 0
#define BN357_TRACK_TIMING 0

volatile uint8_t continue_with_gps_data = 0;

#define GPS_RECEIVE_BUFFER_SIZE 700

uint8_t receive_buffer_1[GPS_RECEIVE_BUFFER_SIZE];
volatile uint16_t receive_buffer_1_size;
uint8_t receive_buffer_2[GPS_RECEIVE_BUFFER_SIZE];
volatile uint16_t receive_buffer_2_size;
volatile uint8_t buffer_for_dma = 0; // 0 is buf 1, 1 is buf 2

volatile uint8_t got_data = 0;

// Interrupt for uart 2 data received
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
    // make sure the dma is initialized before uart for this to work
    // and that dma is initialized after GPIO.

    // If the stm32 is restarted  while it is initializing dma the
    // data arrives from the uart and it fucks up. So you have to restart it a few times

    if (huart->Instance == USART2){

        // Pick which buffer to use next
        if(buffer_for_dma == 0){
            // If it was buffer 1 before then not it will be buf 2
            buffer_for_dma = 1;
            HAL_UARTEx_ReceiveToIdle_DMA(uart, receive_buffer_2, GPS_RECEIVE_BUFFER_SIZE);
            receive_buffer_1_size = Size;
        }else if(buffer_for_dma == 1){
            // If it was buffer 2 before then not it will be buf 1
            buffer_for_dma = 0;
            HAL_UARTEx_ReceiveToIdle_DMA(uart, receive_buffer_1, GPS_RECEIVE_BUFFER_SIZE);
            receive_buffer_2_size = Size;
        }
        
        got_data = 1;

        __HAL_DMA_DISABLE_IT(hdma_uart_rx, DMA_IT_HT);
    }
}

// Interrupt for uart 2 when it crashes to restart it
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
    if (huart->Instance == USART2){
        // printf("Error UART 2\n");
        HAL_UART_DeInit(uart);
        HAL_UART_Init(uart);

        if(buffer_for_dma == 0){
            // If it was bugger 1 then it will be buffer 1 again because it failed
            HAL_UARTEx_ReceiveToIdle_DMA(uart, receive_buffer_1, GPS_RECEIVE_BUFFER_SIZE);
        }else if(buffer_for_dma == 1){
            // If it was bugger 2 then it will be buffer 2 again because it failed
            HAL_UARTEx_ReceiveToIdle_DMA(uart, receive_buffer_2, GPS_RECEIVE_BUFFER_SIZE);
        }

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
    // Always start on buffer 1
    HAL_UARTEx_ReceiveToIdle_DMA(uart, receive_buffer_1, GPS_RECEIVE_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(hdma_uart_rx, DMA_IT_HT);
}

void bn357_toggle_gps_logging(uint8_t status){
    continue_with_gps_data = status;
}
uint8_t bn357_parse_data(){
    if(!got_data){
        return 0;
    }

// #if(BN357_DEBUG)
//     printf("1s %d '", receive_buffer_1_size);

//     for (uint16_t i = 0; i < receive_buffer_1_size; i++){
//         printf("%c", receive_buffer_1[i]);
//     }
//     printf("'\n");
// #endif

// #if(BN357_DEBUG)
//     printf("2s %d '", receive_buffer_2_size);

//     for (uint16_t i = 0; i < receive_buffer_2_size; i++){
//         printf("%c", receive_buffer_2[i]);
//     }
//     printf("'\n");
// #endif

    
    uint8_t response;
    if(buffer_for_dma == 0){

        // If buffer 1 is used currently by dma then use the other buffer for data.
        response = bn357_parse_and_store((char *)receive_buffer_2, receive_buffer_2_size);
    }else if(buffer_for_dma == 1){
        // If buffer 2 is used currently by dma then use the other buffer for data.
        response = bn357_parse_and_store((char *)receive_buffer_1, receive_buffer_1_size);
    }else{
        response = 0;
    }

    got_data = 0;

    return response;
}

uint8_t bn357_parse_and_store(char *gps_output_buffer, uint16_t size_of_buffer){
#if(BN357_TRACK_TIMING)
    uint32_t start_time = HAL_GetTick();
#endif

    // reset the state of successful gps parse
    m_up_to_date_date = 0;

    // ===============================================================================================Skip UBX binary protocol bytes
    // Messages should come in this order: $GNRMC -> $GNGGA -> $GNGSA

    // $GNRMC goes first then find it first
    uint8_t string_match[] = {'$','G','N','R','M','C'};
    uint8_t string_match_length = 6;
    uint8_t string_match_index = 0;
    for (uint16_t i = 0; i < size_of_buffer; i++){
        if(gps_output_buffer[i] == string_match[string_match_index]){
            string_match_index++;
            if(string_match_index == string_match_length){
                gps_output_buffer = gps_output_buffer + i - string_match_length + 1;
                size_of_buffer = size_of_buffer - i;
#if(BN357_DEBUG)
                printf("Skipped %d bytes\n", i);
#endif
                break;
            }
        }
    }
    
#if(BN357_DEBUG)
    printf("s %d '", size_of_buffer);

    for (uint16_t i = 0; i < size_of_buffer; i++){
        printf("%c", gps_output_buffer[i]);
    }
    printf("'\n");
#endif



    // =============================================================================================== Start parsing the string to find data
    
    // ------------------------------------------------------------- $GNRMC
    // $GNRMC,164334.70,A,5539.10716,N,01234.50271,E,0.025,,080925,,,A,V*1B

    char* start = strstr(gps_output_buffer, "$GNRMC");  
    if(start == NULL){ // Check if the start exists
#if(BN357_DEBUG)
        printf("GNRMC: Failed at Check if the start exists\n");
#endif
        return 0;
    }
    char* end = strchr(start, '\n');  // find the position of the first new line character after the substring
    if (end == NULL){ // check if the end of it exists
#if(BN357_DEBUG)
        printf("GNRMC: Failed at check if the end of it exists\n");
#endif
        return 0;
    }
    uint8_t length = end - start;  // calculate the length of the substring
    if(length !=0){
        char sub_string_gnrmc[length + 1];  // create a new string to store the substring, plus one for null terminator
        strncpy(sub_string_gnrmc, start, length);  // copy the substring to the new string
        sub_string_gnrmc[length] = '\0';  // add a null terminator to the new string
    #if(BN357_DEBUG)
        printf("GNRMC Substring: %s\n", sub_string_gnrmc);  // print the substring
    #endif
    }else{
        return 0;
    }



    // Skip the first 7 commas after $GNRMC
    start = strchr(start, ','); // Skip first one
    if (!start) return 0;
    start++; // move past the comma

    // Skip 6 more commas to get to the 8th field (course over ground)
    for (int i = 0; i < 7; i++) {
        start = strchr(start, ',');
        if (!start) return 0;
        start++;
    }

    end = strchr(start, ','); 
    length = end - start;
    if (length != 0){
        char course_over_ground_str[length + 1];
        strncpy(course_over_ground_str, start, length);
        course_over_ground_str[length] = '\0';
        m_course_over_ground = atof(course_over_ground_str);

        // #if(BN357_DEBUG)
            // printf("GNRMC course over ground: %s\n", course_over_ground_str);
        // #endif
    }else{
        // printf("len 0\n");
    }

    start = strchr(end, ',');
    length = end - start;
    if (length != 0 && length == 6){
        char date_day[2 + 1];
        char date_month[2 + 1];
        char date_year[2 + 1];
        strncpy(date_day, start, 2);
        strncpy(date_month, start+2, 2);
        strncpy(date_year, start+4, 2);
        date_day[2] = '\0';
        date_month[2] = '\0';
        date_year[2] = '\0';

        m_date_day = atoi(date_day);
        m_date_month = atoi(date_month);
        m_date_year = atoi(date_year);

        
        // #if(BN357_DEBUG)
            printf("GNRMC date: %d/%d/%d\n", m_date_day, m_date_month, m_date_year + 2000);
        // #endif
    }else{
        // printf("Date not found\n");
    }



    // ------------------------------------------------------------- $GNGGA
    // $GNGGA,165022.50,5539.10780,N,01234.50218,E,2,12,0.48,5.0,M,39.5,M,,*4D

    start = strstr(gps_output_buffer, "$GNGGA"); // find the starting position of the substring
    if(start == NULL){ // Check if the start exists
#if(BN357_DEBUG)
        printf("GNGGA: Failed at Check if the start exists\n");
#endif
        return 0;
    }
    end = strchr(start, '\n');  // find the position of the first new line character after the substring
    if (end == NULL){ // check if the end of it exists
#if(BN357_DEBUG)
        printf("GNGGA: Failed at check if the end of it exists\n");
#endif
        return 0;
    }
    length = end - start;  // calculate the length of the substring
    char sub_string_gngga[length + 1];  // create a new string to store the substring, plus one for null terminator
    strncpy(sub_string_gngga, start, length);  // copy the substring to the new string
    sub_string_gngga[length] = '\0';  // add a null terminator to the new string
#if(BN357_DEBUG)
    printf("GNGGA Substring: %s\n", sub_string_gngga);  // print the substring
#endif



    // ----------------------------------------------------------------------------------------- find utc time
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
        printf("GNGGA UTC time: %d%d%d\n", m_time_utc_hours, m_time_utc_minutes, m_time_utc_seconds);
    #endif
    }


    // ----------------------------------------------------------------------------------------- find latitude
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


    // ----------------------------------------------------------------------------------------- find latitude direction
    m_latitude_direction = (end+1)[0];
#if(BN357_DEBUG)
    printf("GNGGA Latitude direction: %c\n", m_latitude_direction);
#endif

    // ----------------------------------------------------------------------------------------- find longitude
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

    // ----------------------------------------------------------------------------------------- find longitude direction
    m_longitude_direction = (end+1)[0];
#if(BN357_DEBUG)
    printf("GNGGA Longitude direction: %c\n", m_longitude_direction);
#endif

    // ----------------------------------------------------------------------------------------- find fix quality
    start = end+3; // skip the E character
    if(start[0] == ','){
#if(BN357_DEBUG)
        printf("GNGGA: Failed at fix quality\n");
#endif
        return 0;
    }
    end = strchr(start, ',');
    if(!m_minimal_gps_parse_enabled){
        length = end - start;
        char fix_quality_string[length + 1];
        strncpy(fix_quality_string, start, length);
        fix_quality_string[length] = '\0';
        m_fix_quality = atoi(fix_quality_string);
    #if(BN357_DEBUG)
        printf("GNGGA Fix quality: %d\n", m_fix_quality);
    #endif
    }

    // ----------------------------------------------------------------------------------------- find satellites quantity
    start = end+1;
    if(start[0] == ','){
#if(BN357_DEBUG)
        printf("GNGGA: Failed at satellites quantity\n");
#endif
        return 0;
    }

    if(start[0] == '0'){ // To help parsing of the values like "08"
        start = start + 1;
    }

    end = strchr(start, ',');

    length = end - start;

    char satellites_quantity_string[length + 1];
    strncpy(satellites_quantity_string, start, length);
    satellites_quantity_string[length] = '\0';
    m_satellites_quantity = atoi(satellites_quantity_string);
#if(BN357_DEBUG)
    printf("GNGGA Satellites quantity: %d\n", m_satellites_quantity);
#endif

    // ----------------------------------------------------------------------------------------- find accuracy
    start = end+1;
    if(start[0] == ','){
#if(BN357_DEBUG)
        printf("GNGGA: Failed at accuracy\n");
#endif
        return 0;
    } 

    end = strchr(start, ',');
    if(!m_minimal_gps_parse_enabled){
        length = end - start;
        char accuracy_quantity_string[length + 1];
        strncpy(accuracy_quantity_string, start, length);
        accuracy_quantity_string[length] = '\0';
        m_accuracy = atof(accuracy_quantity_string);
    #if(BN357_DEBUG)
        printf("GNGGA Accuracy: %f\n", m_accuracy);
    #endif
    }

    // ----------------------------------------------------------------------------------------- find altitude above sea levels
    start = end+1;
    if(start[0] == ','){
#if(BN357_DEBUG)
        printf("GNGGA: Failed at altitude\n");
#endif
        return 0;
    } 

    end = strchr(start, ',');
    if(!m_minimal_gps_parse_enabled){
        length = end - start;
        char altitude_string[length + 1];
        strncpy(altitude_string, start, length);
        altitude_string[length] = '\0';
        m_altitude = atof(altitude_string);
    #if(BN357_DEBUG)
        printf("GNGGA Altitude: %f\n", m_altitude);
    #endif
    }


    // ----------------------------------------------------------------------------------------- find geoid altitude deviation
    start = end+3;
    if(start[0] == ','){
#if(BN357_DEBUG)
        printf("GNGGA: Failed at Geoid altitude\n");
#endif
        return 0;
    } 

    end = strchr(start, ',');
    if(!m_minimal_gps_parse_enabled){
        length = end - start;
        char geoid_altitude_string[length + 1];
        strncpy(geoid_altitude_string, start, length);
        geoid_altitude_string[length] = '\0';
        m_altitude_geoid = atof(geoid_altitude_string);
    #if(BN357_DEBUG)
        printf("GNGGA Geoid altitude: %f\n", m_altitude_geoid);
    #endif
    }

















    // ------------------------------------------------------------- $GNGSA
    // $GNGSA,A,3,02,10,13,14,15,17,19,22,23,24,30,,0.90,0.48,0.76,1*0F THIS ONE ONLY
    // $GNGSA,A,3,31,24,13,04,05,06,09,15,03,23,,,0.90,0.48,0.76,3*09
    // $GNGSA,A,3,07,10,14,24,26,29,33,40,41,42,35,,0.90,0.48,0.76,4*0B
    // $GNGSA,A,3,,,,,,,,,,,,,0.90,0.48,0.76,5*01

    start = strstr(gps_output_buffer, "$GNGSA");  
    if(start == NULL){ // Check if the start exists
#if(BN357_DEBUG)
        printf("GNGSA: Failed at Check if the start exists\n");
#endif
        return 0;
    }
    end = strchr(start, '\n');  // find the position of the first new line character after the substring
    if (end == NULL){ // check if the end of it exists
#if(BN357_DEBUG)
        printf("GNGSA: Failed at check if the end of it exists\n");
#endif
        return 0;
    }
    length = end - start;  // calculate the length of the substring
    char sub_string_gngsa[length + 1];  // create a new string to store the substring, plus one for null terminator
    strncpy(sub_string_gngsa, start, length);  // copy the substring to the new string
    sub_string_gngsa[length] = '\0';  // add a null terminator to the new string
#if(BN357_DEBUG)
    printf("GNGSA Substring: %s\n", sub_string_gngsa);  // print the substring
#endif


    //--------------------------------- skip one piece of data
    start = strchr(sub_string_gngsa, ',') + 1;
    if(start[0] == ','){
#if(BN357_DEBUG)
        printf("GNGSA: Failed at skip\n");
#endif
        return 0;
    }
    end = strchr(start, ',');
    // Skip whatever is here



    // ----------------------------------------------------------------------------------------- find fix type
    start = end+1;
    if(start[0] == ','){
#if(BN357_DEBUG)
        printf("GNGSA: Failed at fix type\n");
#endif
        return 0;
    }
    end = strchr(start, ',');
    length = end - start;
    char fix_type_string[length + 1];
    strncpy(fix_type_string, start, length);
    fix_type_string[length] = '\0';
    m_fix_type = atoi(fix_type_string);
#if(BN357_DEBUG)
    printf("GNGSA: Fix type: %d\n", m_fix_type);
#endif













    // ====================================== END
    m_up_to_date_date = 1;

#if(BN357_DEBUG)
    printf("GPS data read successful\n");
#endif
    return 1;
}


float bn357_get_latitude_decimal_format(){
    return m_latitude * 1000000.0;
}

float bn357_get_latitude(){
    return m_latitude;
}

char bn357_get_latitude_direction(){
    return m_latitude_direction;
}

// For logging
float bn357_get_longitude_decimal_format(){
    return m_longitude * 1000000.0;
}


float bn357_get_longitude(){
    return m_longitude;
}

// For calculations
float bn357_get_linear_longitude_decimal_format(){
    // The longitude increases in precision as we go up in latitude
    // That is not good for pid which measures proportional error
    // The pid will think the longitude is more responsive
    // Scale it to be linear
    return (m_longitude*cos(m_latitude * M_DEG_TO_RAD)) * 1000000.0;
}

float bn357_get_linear_longitude(){
    // The longitude increases in precision as we go up in latitude
    // That is not good for pid which measures proportional error
    // The pid will think the longitude is more responsive
    // Scale it to be linear
    return m_longitude*cos(m_latitude * M_DEG_TO_RAD);
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

float bn357_get_course_over_ground(){
    return m_course_over_ground;
}

uint8_t bn357_get_date_day(){
    return m_date_day;
}

uint8_t bn357_get_date_month(){
    return m_date_month;
}

uint8_t bn357_get_date_year(){
    return m_date_year;
}
