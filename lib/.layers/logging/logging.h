#pragma once
#include "../common.h"
#include "../time_keeping/time_keeping.h"

void handle_logging();
void setup_logging_to_sd(uint8_t use_updated_file_name);
void indicate_sd_logging_ok();
void indicate_mistake_on_sd_card();
void indicate_mistake_on_logger();
void uint64_to_str(uint64_t value, char *buf);