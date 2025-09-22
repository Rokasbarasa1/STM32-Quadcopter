#pragma once
#include "../common.h"
#include "../time_keeping/time_keeping.h"

void handle_pid_and_motor_control();
uint16_t setServoActivationPercent(float percent, uint16_t minValue, uint16_t maxValue);
float limit_max_value(float value, float min_value, float max_value);
void initialize_motor_communication();
