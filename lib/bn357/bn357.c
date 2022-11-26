#include "./bn357.h"

UART_HandleTypeDef *uart;

float current_longitude = 0.0;
char current_longitude_direction = 'f';
float current_latitude = 0.0;
char current_latitude_direction = 'f';

uint8_t init_bn357(UART_HandleTypeDef *uart_temp)
{
    uart = uart_temp;
    return 1;
}

float bn357_get_longitude()
{
    return current_longitude;
}
float bn357_get_latitude()
{
    return current_latitude;
}
char bn357_get_longitude_direction()
{
    return current_longitude_direction;
}
char bn357_get_latitude_direction()
{
    return current_latitude_direction;
}

void bn357_parse_and_store(uint8_t *buffer, uint16_t size_of_buf)
{

    uint16_t gps_index = 9999;
    uint8_t gps_string_exists = find_substring(buffer, 0, size_of_buf, "GNGGA", &gps_index);

    // Quit if you could not find the correct gps text
    if (!gps_string_exists)
    {
        printf("Bad data\n");
        return;
    }
    if (buffer[gps_index + 6] == ',')
    {
        printf("Bad gps connection!\n");
        return;
    }

    // printf("COntinuing\n");
    // $GNGGA,182459.00,5551.75853,N,00950.61588,E,1,05,5.31,-14.8,M,43.4,M,,*56</>
    //  ^

    // printf("%d\n", gps_index);

    for (uint16_t i = 0 + gps_index; i < gps_index + 100; i++)
    {
        if (i >= size_of_buf - 1)
        {
            break;
        }
        printf("%c", (char *)buffer[i]);
    }

    printf("\n");

    // printf("READ GPS\n");

    // get longitude index starting counting offset from gps text that was found
    uint16_t longitude_start_index = 9999;
    find_substring(buffer, gps_index + 6, size_of_buf, ",", &longitude_start_index);
    longitude_start_index++;
    // $GNGGA,182459.00,5551.75853,N,00950.61588,E,1,05,5.31,-14.8,M,43.4,M,,*56</p>
    //                  ^

    if (buffer[longitude_start_index] == ',')
    {
        printf("Bad gps connection!\n");
        return;
    }


    // get the end of longitude text
    uint16_t longitude_end_index = 9999;
    find_substring(buffer, longitude_start_index, size_of_buf, ",", &longitude_end_index);
    longitude_end_index--;
    // $GNGGA,182459.00,5551.75853,N,00950.61588,E,1,05,5.31,-14.8,M,43.4,M,,*56</p>
    //                           ^

    char longitude_direction = buffer[longitude_end_index + 2];
    // $GNGGA,182459.00,5551.75853,N,00950.61588,E,1,05,5.31,-14.8,M,43.4,M,,*56</p>
    //                             ^

    uint16_t latitude_start_index = longitude_end_index + 4;
    // $GNGGA,182459.00,5551.75853,N,00950.61588,E,1,05,5.31,-14.8,M,43.4,M,,*56</p>
    //                               ^

    uint16_t latitude_end_index = 9999;
    find_substring(buffer, latitude_start_index, size_of_buf, ",", &latitude_end_index);
    latitude_end_index--;
    // $GNGGA,182459.00,5551.75853,N,00950.61588,E,1,05,5.31,-14.8,M,43.4,M,,*56</p>
    //                                         ^

    char latitude_direction = buffer[latitude_end_index + 2];
    // $GNGGA,182459.00,5551.75853,N,00950.61588,E,1,05,5.31,-14.8,M,43.4,M,,*56</p>
    //                                           ^

    char *longitude_number = get_substring(buffer, size_of_buf, longitude_start_index, longitude_end_index);
    // for (uint16_t i = 0; i < 100; i++)
    // {
    //     if (longitude_number[i] == '\0')
    //     {
    //         break;
    //     }
    //     else
    //     {
    //         printf("%c", (char *)longitude_number[i]);
    //     }
    // }
    // printf("\n");


    current_longitude = strtof(longitude_number, NULL);
    free(longitude_number);
    // printf("%c\n", longitude_direction);
    current_longitude_direction = longitude_direction;

    char *latitude_number = get_substring(buffer, size_of_buf, latitude_start_index, latitude_end_index);
    // for (uint16_t i = 0; i < 100; i++)
    // {
    //     if (latitude_number[i] == '\0')
    //     {
    //         break;
    //     }
    //     else
    //     {
    //         printf("%c", (char *)latitude_number[i]);
    //     }
    // }
    // printf("\n");
    current_latitude = strtof(longitude_number, NULL);
    free(latitude_number);
    // printf("%c\n", latitude_direction);
    current_latitude_direction = longitude_direction;
}