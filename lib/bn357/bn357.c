#include "./bn357.h"

UART_HandleTypeDef *uart;

uint8_t uart2_buffer[500] = {0};

float current_longitude = 0.0;
char current_longitude_direction = 'f';

float current_latitude = 0.0;
char current_latitude_direction = 'f';

uint8_t init_bn357(UART_HandleTypeDef *uart_temp)
{
    uart = uart_temp;

    // HAL_UART_Receive_IT(uart_temp, uart2_buffer, 500);
    // HAL_UART_Receive_IT(&uart_temp, uart2_buffer, 500);
    return 1;
}

// void wait_until(){
//     while (/* condition */)
//     {
//         /* code */
//     }

// }

void bn357_get_coordinates()
{
    // HAL_StatusTypeDef ret;

    // // Wait for GGA
    // uint8_t data[1000];

    // ret = HAL_UART_Receive(uart, data, 1000, 5000);
    // // char *string = data;

    // if (ret == HAL_OK)
    // {
    //     printf("WOrked\n");
    // }
    // else
    // {
    //     printf("DID NOT WORK\n");
    // }

    // // printf("%s")
    // for (uint32_t i = 1; i < 1000; i++)
    // {
    //     printf("%c", data[i]);
    // }
}

void bn357_interrupt()
{
    // if(interrupt_init == 0){
    //     HAL_UART_Receive_IT(&uart, uart2_buffer, 500);
    //     return;
    // }

    char *gps = "GNGGA";
    // printf("IN THE INTERRUPT");
    // uint8_t gps_length = strlen(gps)-1;
    // uint8_t gps_index = gps_length;

    // for(uint16_t i = 500; i > 0; i--){
    //     // Check if ack reached
    //     if(uart2_buffer[i] == gps[gps_index]){
    //         gps_index--;
    //         if(gps_index == 0){
    //             break;
    //         }
    //     }else{
    //         gps_index = gps_length;
    //     }
    // }

    // take the data
    // count backwards from the end of it
    // till you find the first mention of GNGGA, remember the index of it
    //
    // Keep counting backwards till you find the second mention of GNGGA. Remember the idex of it
    // Use the string of

    // HAL_UART_Receive_IT(&uart, uart2_buffer, 500);
}

float bn357_get_longitude(){
    return current_longitude;
}
float bn357_get_latitude(){
    return current_latitude;
}
char bn357_get_longitude_direction(){
    return current_longitude_direction;
}
char bn357_get_latutude_direction(){
    return current_longitude_direction;
}

uint8_t find_substring_from_end(uint8_t * buffer, uint16_t start_index, uint16_t buffer_size, char *string, uint16_t *found_index);
uint8_t find_substring(uint8_t * buffer, uint16_t start_index, uint16_t buffer_size, char *string, uint16_t *found_index);
char* get_substring(uint8_t* buffer, uint16_t buffer_size, uint16_t start_index, uint16_t end_index);

void bn357_parse_and_store(uint8_t * buffer, uint16_t size_of_buf){

    uint16_t gps_index = 9999;
    uint8_t gps_string_exists = find_substring(buffer, 0, size_of_buf, "GNGGA", &gps_index);
    
    // Quit if you could not find the correct gps text
    if(!gps_string_exists){
        printf("Bad data\n");
        return;
    }

    // $GNGGA,182459.00,5551.75853,N,00950.61588,E,1,05,5.31,-14.8,M,43.4,M,,*56</>
    //  ^

    // printf("READ GPS\n");

    // get longitude index starting counting offset from gps text that was found
    uint16_t longitude_start_index = 9999;
    find_substring(buffer, gps_index+6, size_of_buf, ",", &longitude_start_index);
    longitude_start_index++;
    // $GNGGA,182459.00,5551.75853,N,00950.61588,E,1,05,5.31,-14.8,M,43.4,M,,*56</p>
    //                  ^

    // get the end of longitude text
    uint16_t longitude_end_index = 9999;
    find_substring(buffer, longitude_start_index, size_of_buf, ",", &longitude_end_index);
    longitude_end_index--;
    // $GNGGA,182459.00,5551.75853,N,00950.61588,E,1,05,5.31,-14.8,M,43.4,M,,*56</p>
    //                           ^

    char longitude_direction = buffer[longitude_end_index+2];
    // $GNGGA,182459.00,5551.75853,N,00950.61588,E,1,05,5.31,-14.8,M,43.4,M,,*56</p>
    //                             ^

    uint16_t latitude_start_index = longitude_end_index+4;
    // $GNGGA,182459.00,5551.75853,N,00950.61588,E,1,05,5.31,-14.8,M,43.4,M,,*56</p>
    //                               ^

    uint16_t latitude_end_index = 9999;
    find_substring(buffer, latitude_start_index, size_of_buf, ",", &latitude_end_index);
    latitude_end_index--;
    // $GNGGA,182459.00,5551.75853,N,00950.61588,E,1,05,5.31,-14.8,M,43.4,M,,*56</p>
    //                                         ^

    char latitude_direction = buffer[longitude_end_index+4];
    // $GNGGA,182459.00,5551.75853,N,00950.61588,E,1,05,5.31,-14.8,M,43.4,M,,*56</p>
    //                                           ^

    char * longitude_number = get_substring(buffer, size_of_buf, longitude_start_index, longitude_end_index);
    for(uint16_t i = 0; i < 100; i++){
        if(longitude_number[i] == '\0'){
            break;
        }else{
            printf("%c", (char *)longitude_number[i]);
        }
    }
    printf("\n");

    current_longitude = strtof(longitude_number, NULL);
    free(longitude_number);
    printf("%c\n",longitude_direction);
    current_longitude_direction = longitude_direction;


    char * latitude_number = get_substring(buffer, size_of_buf, latitude_start_index, latitude_end_index);
    for(uint16_t i = 0; i < 100; i++){
        if(latitude_number[i] == '\0'){
            break;
        }else{
            printf("%c", (char *)latitude_number[i]);
        }
    }
    printf("\n");
    current_latitude = strtof(longitude_number, NULL);
    free(latitude_number);
    printf("%c\n",latitude_direction);
    current_latitude_direction = longitude_direction;
}

// Find if string exists in char array, starting from the end of the string
uint8_t find_substring_from_end(uint8_t * buffer, uint16_t start_index, uint16_t buffer_size, char *string, uint16_t *found_index){
    // Cant start after buffer is done
    if(start_index >= buffer_size-1){
        *found_index = 9999;
        return 0;
    }

    uint8_t string_length = strlen(string)-1;
    int8_t string_index = string_length;
    uint16_t string_found_index = 9999;
    uint8_t found_string = 0;
    for(uint16_t i = buffer_size-1; i > start_index; i--){
        // Check if ack reached
        // printf("Compare: %c - %c", (char *)buffer[i], string[string_index] );
        if(buffer[i] == string[string_index]){
            // printf(" OK\n");
            string_index--;
            if(string_index == -1){
                string_found_index = i;
                found_string = 1;
                break;
            }
        }else{
            // printf(" NO\n");
            string_index = string_length;
        }
    }

    *found_index = string_found_index;
    return found_string;
}

// Find if string exists in char array
uint8_t find_substring(uint8_t * buffer, uint16_t start_index, uint16_t buffer_size, char *string, uint16_t *found_index){
    // Cant start after buffer is done
    if(start_index >= buffer_size-1){
        *found_index = 9999;
        return 0;
    }
    
    uint8_t string_length = strlen(string);
    int8_t string_index = 0;
    uint16_t string_found_index = 9999;
    uint8_t found_string = 0;

    for(uint16_t i = start_index; i < buffer_size-1; i++){
        // Check if ack reached
        // printf("Compare: %c - %c", (char *)buffer[i], string[string_index] );
        if(buffer[i] == string[string_index]){
            // printf(" OK\n");
            string_index++;
            if(string_index == string_length){
                string_found_index = i - string_length + 1;
                found_string = 1;
                break;
            }
        }else{
            // printf(" NO\n");
            string_index = 0;
        }
    }

    *found_index = string_found_index;
    return found_string;
}


char* get_substring(uint8_t* buffer, uint16_t buffer_size, uint16_t start_index, uint16_t end_index){

    if(end_index > buffer_size-1){
        return NULL;
    }

    uint16_t length = end_index-start_index+1+1; // +1 for index calculation and +1 for the \0 char
    char *string = malloc(length);
    uint16_t string_index = 0;

    for(uint16_t i = start_index; i < buffer_size + length; i++){
        string[string_index] = buffer[i];
        string_index++;
    }
    string[length-1] = '\0';

    return string;
}

void print_char(char* data){
    for(uint16_t i = 0; i < 1000; i++){
        if(data[i] == '\0'){
            break;
        }else{
            printf("%c", (char *)data[i]);
        }
    }
    printf("\n");
}