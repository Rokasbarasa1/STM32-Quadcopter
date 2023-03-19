#include "./bn357.h"

UART_HandleTypeDef *uart;

volatile double m_latitude = 0.0;
volatile double m_longitude = 0.0;
volatile double m_altitude = 0.0;
volatile double m_altitude_geoid = 0.0;
volatile double m_accuracy = 0.0;
volatile uint8_t m_satellites_quantity = 0.0;
volatile uint8_t m_fix_quality = 0;
volatile uint8_t m_time_utc_hours = 0;
volatile uint8_t m_time_utc_minutes = 0;
volatile uint8_t m_time_utc_seconds = 0;
volatile uint8_t m_up_to_date_date = 0;

volatile uint8_t m_logging = 0;

// consider adding mutexes even though interrupt always finished everything before letting the cpu continue reading

uint8_t init_bn357(UART_HandleTypeDef *uart_temp, uint8_t logging){
    uart = uart_temp;
    m_logging = logging;
    return 1;
}

// Check the status of the gps reading. If they are up to date or not
uint8_t bn357_get_status_up_to_date(){
    return m_up_to_date_date;
}

void bn357_get_clear_status(){
    m_up_to_date_date = 0;
}

void bn357_parse_and_store(unsigned char *gps_output_buffer, uint16_t size_of_buffer){
    // reset the state of successful gps parse
    m_up_to_date_date = 0;
    // find the starting position of the substring
    char* start = strstr(gps_output_buffer, "$GNGGA");  
    // Check if the start exists
    if(start == NULL) return;
    char* end = strchr(start, '\n');  // find the position of the first new line character after the substring
    // check if the end of it exists
    if (end == NULL) return;
    int length = end - start;  // calculate the length of the substring
    char sub_string[length + 1];  // create a new string to store the substring, plus one for null terminator
    strncpy(sub_string, start, length);  // copy the substring to the new string
    sub_string[length] = '\0';  // add a null terminator to the new string
    if(m_logging == 1){
        printf("Substring: %s\n", sub_string);  // print the substring
    }

    // find utc time
    start = strchr(sub_string, ',') + 1;
    if(start[0] == ',') return;
    end = strchr(start, ',');
    length = end - start;
    char utc_time_string[length + 1];
    strncpy(utc_time_string, start, length);
    utc_time_string[length] = '\0';
    uint32_t time = atoi(utc_time_string);
    m_time_utc_seconds = time % 100;
    m_time_utc_minutes = ((time % 10000) - m_time_utc_seconds) / 100;
    m_time_utc_hours = ((time % 1000000) - (m_time_utc_minutes * 100) - m_time_utc_seconds) / 10000;
    if(m_logging == 1){
        printf("UTC time: %d%d%d%\n", m_time_utc_hours, m_time_utc_minutes, m_time_utc_seconds);
    }

    // find latitude
    start = end+1;
    if(start[0] == ',') return;
    end = strchr(start, ',');
    length = end - start;
    char latitude_string[length + 1];
    strncpy(latitude_string, start, length);
    latitude_string[length] = '\0';
    uint16_t latitude_temp = atoi(latitude_string);
    uint16_t latitude_degrees = ((latitude_temp % 10000) - (latitude_temp % 100)) / 100;
    m_latitude = (double)latitude_degrees + ((atof(latitude_string) - (double)latitude_degrees * 100)/60);
    if(m_logging == 1){
        printf("Latitude: %f\n", m_latitude);
    }

    // find longitude
    start = end+3; // skip the N character
    if(start[0] == ',') return;
    end = strchr(start, ',');
    length = end - start;
    char longitude_string[length + 1];
    strncpy(longitude_string, start, length);
    longitude_string[length] = '\0';
    uint16_t longitude_temp = atoi(longitude_string);
    uint16_t longitude_degrees = ((longitude_temp % 10000) - (longitude_temp % 100)) / 100;
    m_longitude = (double)longitude_degrees + ((atof(longitude_string) - (double)longitude_degrees * 100)/60);
    if(m_logging == 1){
        printf("Longitude: %f\n", m_longitude);
    }

    // find fix quality
    start = end+3; // skip the E character
    if(start[0] == ',') return;
    end = strchr(start, ',');
    length = end - start;
    char fix_qualtiy_string[length + 1];
    strncpy(fix_qualtiy_string, start, length);
    fix_qualtiy_string[length] = '\0';
    m_fix_quality = atoi(fix_qualtiy_string);
    if(m_logging == 1){
        printf("Fix quality: %d\n", m_fix_quality);
    }

    // find satellites quantity
    start = end+1;
    if(start[0] == ',') return;
    end = strchr(start, ',');
    length = end - start;
    char satellites_quantity_string[length + 1];
    strncpy(satellites_quantity_string, start, length);
    satellites_quantity_string[length] = '\0';
    m_satellites_quantity = atoi(satellites_quantity_string);
    if(m_logging == 1){
        printf("Satellites quantity: %d\n", m_satellites_quantity);
    }

    // find accuracy
    start = end+1;
    if(start[0] == ',') return;
    end = strchr(start, ',');
    length = end - start;
    char accuracy_quantity_string[length + 1];
    strncpy(accuracy_quantity_string, start, length);
    accuracy_quantity_string[length] = '\0';
    m_accuracy = atof(accuracy_quantity_string);
    if(m_logging == 1){
        printf("Accuracy: %f\n", m_accuracy);
    }

    // find altitude above sea levels
    start = end+1;
    if(start[0] == ',') return;
    end = strchr(start, ',');
    length = end - start;
    char altitude_string[length + 1];
    strncpy(altitude_string, start, length);
    altitude_string[length] = '\0';
    m_altitude = atof(altitude_string);
    if(m_logging == 1){
        printf("Altitude: %f\n", m_altitude);
    }

    // find geoid altitude deviation
    start = end+3;
    if(start[0] == ',') return;
    end = strchr(start, ',');
    length = end - start;
    char geoid_altitude_string[length + 1];
    strncpy(geoid_altitude_string, start, length);
    geoid_altitude_string[length] = '\0';
    m_altitude_geoid = atof(geoid_altitude_string);
    if(m_logging == 1){
        printf("Geoid altitude: %f\n", m_altitude_geoid);
    }

    m_up_to_date_date = 1;
    if(m_logging == 1){
        printf("GPS data read successful\n");
    }
}

double bn357_get_latitude_decimal_format(){
    return m_latitude;
}

double bn357_get_longitude_decimal_format(){
    return m_longitude;
}

double bn357_get_altitude_meters(){
    return m_altitude;
}

double bn357_get_geoid_altitude_meters(){
    return m_altitude_geoid;
}

double bn357_get_accuracy(){
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