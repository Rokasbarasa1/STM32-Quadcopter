#include "./string_utils.h"

// Find if string exists in char array, starting from the end of the string
uint8_t find_substring_from_end(uint8_t *buffer, uint16_t start_index, uint16_t buffer_size, char *string, uint16_t *found_index){
    // Cant start after buffer is done
    if (start_index >= buffer_size - 1)
    {
        *found_index = 9999;
        return 0;
    }

    uint8_t string_length = strlen(string) - 1;
    int8_t string_index = string_length;
    uint16_t string_found_index = 9999;
    uint8_t found_string = 0;
    for (uint16_t i = buffer_size - 1; i > start_index; i--)
    {
        // Check if ack reached
        // printf("Compare: %c - %c", (char *)buffer[i], string[string_index] );
        if (buffer[i] == string[string_index])
        {
            // printf(" OK\n");
            string_index--;
            if (string_index == -1)
            {
                string_found_index = i;
                found_string = 1;
                break;
            }
        }
        else
        {
            // printf(" NO\n");
            string_index = string_length;
        }
    }

    *found_index = string_found_index;
    return found_string;
}

// Find if string exists in char array
uint8_t find_substring(uint8_t *buffer, uint16_t start_index, uint16_t buffer_size, char *string, uint16_t *found_index){
    // Cant start after buffer is done
    if (start_index >= buffer_size - 1)
    {
        *found_index = 9999;
        return 0;
    }

    uint8_t string_length = strlen(string);
    int8_t string_index = 0;
    uint16_t string_found_index = 9999;
    uint8_t found_string = 0;

    for (uint16_t i = start_index; i < buffer_size - 1; i++)
    {
        // Check if ack reached
        // printf("Compare: %c - %c", (char *)buffer[i], string[string_index] );
        if (buffer[i] == string[string_index])
        {
            // printf(" OK\n");
            string_index++;
            if (string_index == string_length)
            {
                string_found_index = i - string_length + 1;
                found_string = 1;
                break;
            }
        }
        else
        {
            // printf(" NO\n");
            string_index = 0;
        }
    }

    *found_index = string_found_index;
    return found_string;
}

// Get the substring from array and return it as allocated memory of char
char *get_substring(uint8_t *buffer, uint16_t buffer_size, uint16_t start_index, uint16_t end_index){
    // Check for out of bounds
    if (end_index > buffer_size - 1)
    {
        return NULL;
    }

    uint16_t length = end_index - start_index + 1 + 1; // +1 for index calculation and +1 for the \0 char
    char *string = malloc(length);
    uint16_t string_index = 0;

    for (uint16_t i = start_index; i < buffer_size + length; i++)
    {
        string[string_index] = buffer[i];
        string_index++;
    }
    string[length - 1] = '\0';

    return string;
}

// Print char array pointer
void print_char(char *data){
    for (uint16_t i = 0; i < 1000; i++)
    {
        if (data[i] == '\0')
        {
            break;
        }
        else
        {
            printf("%c", (char *)data[i]);
        }
    }
    printf("\n");
}

void print_binary(uint8_t value) {
    printf("0b");
    for (int i = 7; i >= 0; i--) {
        printf("%d", (value >> i) & 1); // Check each bit from MSB to LSB
    }
}
// Print uint16_t in binary
void print_binary16(uint16_t value) {
    printf("0b");
    for (int i = 15; i >= 0; i--) {
        printf("%d", (value >> i) & 1);
    }
}

// Print uint32_t in binary
void print_binary32(uint32_t value) {
    printf("0b");
    for (int i = 31; i >= 0; i--) {
        printf("%d", (value >> i) & 1);
    }
}
