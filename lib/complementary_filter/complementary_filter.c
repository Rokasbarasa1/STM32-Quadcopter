#include "./complementary_filter.h"

#include "math.h"

float angle_difference_range(float a, float b, float min_angle, float max_angle) {
    float range = max_angle - min_angle;
    float diff = fmodf(fmodf((b - a), range) + range, range);  // wrap to [0, range)
    if (diff > range / 2.0f)
        diff -= range;
    return diff;
}


float complementary_filter_calculate(float ratio, float value1, float value2){
    return (1.0 - ratio) * value1 + ratio * value2;
}

float complementary_filter_angle_calculate(float ratio, float value1, float value2, float min_angle, float max_angle) {
    float correction = angle_difference_range(value1, value2, min_angle, max_angle);
    float corrected_value2 = value1 + correction;
    return complementary_filter_calculate(ratio, value1, corrected_value2);
}

// 360 - 0 = 360

// 100 - 10 = 90

// 90 % 360 = 90

// (90 + 360) % 360 = 90





// 360 - 0 = 360

// 200 - 10 = 190

// 190 % 360 = 190

// (190 + 360) % 360 = 190


// 10 - 200 = -190

// -190 % 360 = 170

// 170 % 



// 350 - 10 = 340

// 340 % 360 = 340


// 10 - 350 = -340