#pragma once
#include "stdint.h"

float complementary_filter_calculate(float ratio, float value1, float value2);
float complementary_filter_angle_calculate(float ratio, float value1, float value2, float min_angle, float max_angle);