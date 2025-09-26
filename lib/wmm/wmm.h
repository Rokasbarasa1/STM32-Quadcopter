#pragma once

void wmm_init();
void wmm_compute_elements(double latitude, double longitude, double height_above_geoid_kilometers, int full_year, int month, int day);
double wmm_get_declination_degrees();
void wmm_print_declination();
void wmm_destroy_computed_elements();
