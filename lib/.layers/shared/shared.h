#pragma once
#include "../common.h"

float map_value(float value, float input_min, float input_max, float output_min, float output_max);
uint8_t is_in_dead_zone(float value, float max_value, float min_value, float dead_zone);
float get_sensor_fusion_altitude(float gps_altitude, float barometer_altitude);
float mapValue(float value, float input_min, float input_max, float output_min, float output_max);
