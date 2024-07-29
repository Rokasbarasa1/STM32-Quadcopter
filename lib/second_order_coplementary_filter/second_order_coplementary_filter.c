#include "./second_order_coplementary_filter.h"


struct second_order_complementary_filter init_second_order_coplementary_filter(float ratio, float beta){
    struct second_order_complementary_filter new_filter;
    new_filter.ratio = ratio;
    new_filter.beta = beta;
    new_filter.previous_filtered = 0;
    return new_filter;
}

float second_order_complementary_filter_calculate(struct second_order_complementary_filter* filter, float value1, float value2, float delta_time){

    float first_order = (1.0 - filter->ratio) * value1 + filter->ratio * value2;

    float derivative = (first_order - filter->previous_filtered) / delta_time;

    float second_order = first_order + filter->beta * derivative;

    filter->previous_filtered = second_order;

    return second_order;
}
