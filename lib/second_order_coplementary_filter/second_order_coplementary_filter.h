#pragma once

struct second_order_complementary_filter {
    float ratio;        // Complementary ratio (alpha)
    float beta;         // Tuning parameter for derivative
    float previous_filtered; // Previous filtered value
};

struct second_order_complementary_filter init_second_order_coplementary_filter(float ratio, float beta);
float second_order_complementary_filter_calculate(struct second_order_complementary_filter* filter, float value1, float value2, float delta_time);