#include "./complementary_filter.h"

float complementary_filter_calculate(float ratio, float value1, float value2){
    return (1.0 - ratio) * value1 + ratio * value2;
}