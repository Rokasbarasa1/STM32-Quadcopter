#pragma once
#include "../common.h"
#include "../sensors/sensors.h"
#include "../time_keeping/time_keeping.h"

void startup_procedure();
uint8_t init_drivers();
void initialize_control_abstractions();
void check_calibrations();
void set_flight_mode(uint8_t mode);
void get_initial_position();
void calibrate_gyro();
void switch_x_and_y_axis(float *data);
float sawtooth_sin(float x_radian);
float sawtooth_cos(float x_radian);
float triangle_wave(float x);
float triangle_sin(float x);
float triangle_cos(float x);
void calibrate_escs();