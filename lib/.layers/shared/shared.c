#include "./shared.h"

// Map value from a specified range to a new range
float map_value(float value, float input_min, float input_max, float output_min, float output_max) {
    // Calculate the input and output ranges' lengths
    // Normalize the input value relative to the input range
    // Scale the normalized value according to the output range
    // Add the output range's minimum value to the scaled value
    return output_min + (((value - input_min) / (input_max - input_min)) * (output_max - output_min));
}

uint8_t is_in_dead_zone(float value, float max_value, float min_value, float dead_zone) {
    float mid_point = (max_value + min_value) / 2;
    float half_range = (max_value - min_value) / 2;
    float normalized_value = (value - mid_point) / half_range; // this will be -1 at min_value, +1 at max_value

    float dead_zone_normalized = dead_zone / half_range;

    // Check if the value is within the dead zone
    if (normalized_value >= -dead_zone_normalized && normalized_value <= dead_zone_normalized) {
        return 1;
    } else {
        return 0;
    }
}

float get_sensor_fusion_altitude(float gps_altitude, float barometer_altitude){
    return gps_altitude + barometer_altitude;
}

float mapValue(float value, float input_min, float input_max, float output_min, float output_max) {
    // Calculate the input and output ranges' lengths
    float input_range = input_max - input_min;
    float output_range = output_max - output_min;

    // Normalize the input value relative to the input range
    float normalized_value = (value - input_min) / input_range;

// Scale the normalized value according to the output range
    float scaled_value = normalized_value * output_range;

    // Add the output range's minimum value to the scaled value
    float output_value = output_min + scaled_value;

    return output_value;
}

