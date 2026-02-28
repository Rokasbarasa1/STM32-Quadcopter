#pragma once
#include "../common.h"
#include "../time_keeping/time_keeping.h"
#include "../../wmm/wmm.h"

void handle_get_and_calculate_sensor_values();
void handle_filtering_of_sensor_data();
float get_yaw_sensor_fusion(float selected_magnetometer_yaw);
float map_value_to_expo_range(float value, float min_expo, float max_expo, uint8_t expo_curve);
float map_value_to_expo_range_inverted(float value, float min_expo, float max_expo, uint8_t expo_curve);
void reset_array_data(float *array, uint8_t array_size);
void invert_axies(float *data);