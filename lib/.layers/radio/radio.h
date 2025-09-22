#pragma once
#include "../common.h"
#include "../startup/startup.h"
#include "../time_keeping/time_keeping.h"
#include "../logging/logging.h"
#include "../shared/shared.h"

void handle_radio_communication();
void post_remote_control_step();
void extract_joystick_request_values_uint(char *request, uint8_t request_size, uint8_t *throttle, uint8_t *yaw, uint8_t *roll, uint8_t *pitch);
void extract_joystick_request_values_float(char *request, uint8_t request_size, float *throttle, float *yaw, float *roll, float *pitch);
void extract_request_type(char *request, uint8_t request_size, char *type_output);
void extract_pid_request_values(char *request, uint8_t request_size, float *added_proportional, float *added_integral, float *added_derivative, float *added_master);
void extract_accelerometer_offsets(char *request, uint8_t request_size, float *added_x_axis_offset, float *added_y_axis_offset);
void extract_request_values(char *request, uint8_t request_size, uint8_t *throttle, uint8_t *yaw, uint8_t *pitch, uint8_t *roll);
void extract_flight_mode(char *request, uint8_t request_size, uint8_t *new_flight_mode);
void send_pid_added_info_to_remote();
float apply_dead_zone(float value, float max_value, float min_value, float dead_zone);
char* generate_message_pid_values_nrf24(float base_proportional, float base_integral, float base_derivative, float base_master);
void send_pid_base_info_to_remote();
