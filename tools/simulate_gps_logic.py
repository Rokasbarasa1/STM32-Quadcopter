import time
import math

# Constants from the provided macros
M_PI = 3.1415927
M_PI_2 = 6.2831853
M_PI_DIV_BY_180 = 0.0174533
OPTIMIZE_COEFFICIENT_A0_DIVISION = 1.0 / (2.0 * 0.707)

# Simulation state
sim_gps_latitude = 55.686516
sim_gps_longitude = 12.567108
sim_got_gps = 0
sim_last_got_gps_timestamp_microseconds = 0
sim_start_time = 0

# Control loop state
gps_fix_type = 0
gps_satellites_count = 0
got_gps = 0
got_gps_multiple = 0
gps_can_be_used = 0

gps_latitude = 0.0
gps_longitude = 0.0
gps_real_longitude = 0.0

old_gps_latitude = 0.0
old_gps_longitude = 0.0
newest_gps_latitude = 0.0
newest_gps_longitude = 0.0

real_gps_latitude = 0.0
real_gps_longitude = 0.0
real_gps_real_longitude = 0.0

use_gps_multiply = 1
use_gps_filtering = 1

gps_latitude_micro = 0.0
gps_longitude_micro = 0.0
delta_gps_lat = 0.0
delta_gps_lon = 0.0

multiply_gps_frequency = 10
gps_micro_bits_added = 0

last_got_gps_timestamp_microseconds = 0
last_gps_microstep_timestamp_microseconds = 0
time_between_gps_refresh_microseconds = 0

last_fake_got_gps_timestamp_microseconds = 0

# Filter parameters
# delays with multi x10 
# 8 - with multi x10 delay is 4 iterations
# 1 - is always present and is one or two whole gps reading of delay
# 2 - is 13 iterations
# 3 - is 9 iterations
# 4 - is 6 iterations
# 5 - is 5.5 iterations
# 6 - is 4.5 iterations
# 7 - is 4.5 iterations
# 8 - is 3.5 iterations
# 9 - is 3.5 iterations
# 10 - is 3 iterations
# 11 - is 3 iterations
# 12 - is 3 iterations
# 13 - is 2.5 iterations
# 14 - is 2.5 iterations
# 15 - is 2 iterations
# 16 - is 1.5 iterations
# 17 - is 2 iterations
# 18 - is 2 iterations

gps_filtering_min_cutoff = 18

class LowPassBiquadFilter:
    def __init__(self, cutoff_frequency, sample_rate):
        self.sample_rate_division = 1.0 / sample_rate
        omega = M_PI_2 * cutoff_frequency * self.sample_rate_division
        sin_omega = math.sin(omega)
        cos_omega = math.cos(omega)
        coefficient_a0 = sin_omega * OPTIMIZE_COEFFICIENT_A0_DIVISION

        self.coefficient_b0 = (1 - cos_omega) * 0.5
        self.coefficient_b1 = 1 - cos_omega
        self.coefficient_b2 = (1 - cos_omega) * 0.5
        self.coefficient_a1 = -2.0 * cos_omega
        self.coefficient_a2 = 1.0 - coefficient_a0

        a0_normalization = 1.0 / (1.0 + coefficient_a0)
        self.coefficient_b0 *= a0_normalization
        self.coefficient_b1 *= a0_normalization
        self.coefficient_b2 *= a0_normalization
        self.coefficient_a1 *= a0_normalization
        self.coefficient_a2 *= a0_normalization

        self.previous_input1 = 0.0
        self.previous_input2 = 0.0
        self.previous_output1 = 0.0
        self.previous_output2 = 0.0

    def set_initial_values(self, initial_value):
        self.previous_input1 = initial_value
        self.previous_input2 = initial_value
        self.previous_output1 = initial_value
        self.previous_output2 = initial_value

    def read(self, current_value):
        output = (self.coefficient_b0 * current_value +
                  self.coefficient_b1 * self.previous_input1 +
                  self.coefficient_b2 * self.previous_input2 -
                  self.coefficient_a1 * self.previous_output1 -
                  self.coefficient_a2 * self.previous_output2)

        self.previous_input2 = self.previous_input1
        self.previous_input1 = current_value
        self.previous_output2 = self.previous_output1
        self.previous_output1 = output

        return output

biquad_filter_gps_lat = LowPassBiquadFilter(gps_filtering_min_cutoff, 10 * multiply_gps_frequency)
biquad_filter_gps_lon = LowPassBiquadFilter(gps_filtering_min_cutoff, 10 * multiply_gps_frequency)

# Simulated GPS data functions
def bn357_parse_data():
    global sim_gps_latitude, sim_gps_longitude, sim_got_gps, sim_start_time
    global sim_last_got_gps_timestamp_microseconds

    # Simulate GPS data arrival at 10 Hz
    if get_absolute_time() - sim_last_got_gps_timestamp_microseconds >= 100_000:  # 100,000 microseconds = 0.1 seconds
        sim_got_gps = 1
        sim_last_got_gps_timestamp_microseconds = get_absolute_time()

        # Simulate rhombus pattern with starting values
        base_latitude = 55.686516
        base_longitude = 12.567108

        t = get_absolute_time() % 40_000_000  # 40 seconds to complete one rhombus cycle
        if t < 10_000_000:
            sim_gps_latitude = base_latitude + t * 0.0001 / 1_000_000
            sim_gps_longitude = base_longitude
        elif t < 20_000_000:
            sim_gps_latitude = base_latitude + 0.001
            sim_gps_longitude = base_longitude + (t - 10_000_000) * 0.0001 / 1_000_000
        elif t < 30_000_000:
            sim_gps_latitude = base_latitude + 0.001 - (t - 20_000_000) * 0.0001 / 1_000_000
            sim_gps_longitude = base_longitude + 0.001
        else:
            sim_gps_latitude = base_latitude
            sim_gps_longitude = base_longitude + 0.001 - (t - 30_000_000) * 0.0001 / 1_000_000
    else:
        sim_got_gps = 0

def bn357_get_status_up_to_date(reset_afterwards):
    global sim_got_gps, sim_start_time
    temp_status = sim_got_gps
    if reset_afterwards:
        sim_got_gps = 0
    return temp_status

def bn357_get_latitude_decimal_format():
    return sim_gps_latitude * 1_000_000

def bn357_get_longitude_decimal_format():
    return sim_gps_longitude * 1_000_000

def bn357_get_linear_longitude_decimal_format():
    linear_longitude = (sim_gps_longitude * math.cos(sim_gps_latitude * M_PI_DIV_BY_180))
    return linear_longitude * 1_000_000

def bn357_get_fix_type():
    return 3

def bn357_get_satellites_quantity():
    return 5

def get_absolute_time():
    return int(time.perf_counter() * 1_000_000)

def busy_wait(duration_us):
    start_time = time.perf_counter()
    end_time = start_time + duration_us / 1_000_000
    while time.perf_counter() < end_time:
        pass

def main():
    global gps_latitude, gps_longitude, gps_real_longitude, got_gps, got_gps_multiple
    global old_gps_latitude, old_gps_longitude, newest_gps_latitude, newest_gps_longitude
    global real_gps_latitude, real_gps_longitude, real_gps_real_longitude
    global gps_latitude_micro, gps_longitude_micro, delta_gps_lat, delta_gps_lon
    global last_got_gps_timestamp_microseconds, last_gps_microstep_timestamp_microseconds
    global time_between_gps_refresh_microseconds, gps_micro_bits_added, gps_can_be_used

    control_loop_frequency = 520  # Hz
    control_loop_period_us = int(1_000_000 / control_loop_frequency)

    loop_counter = 0
    start_time = time.perf_counter()
    sim_start_time = get_absolute_time()

    while True:
        loop_start_time = time.perf_counter()
        loop_counter += 1

        # GPS stuff
        bn357_parse_data()  # Try to parse GPS

        # Simulate GPS startup delay
        if get_absolute_time() - sim_start_time >= 2_000_000:  # 2 seconds delay
            got_gps = bn357_get_status_up_to_date(1)
        else:
            got_gps = 0

        # Debug print for control loop frequency
        if loop_counter % 520 == 0:  # Print every second
            elapsed_time = loop_start_time - start_time
            print(f"Control loop frequency: {loop_counter / elapsed_time:.2f} Hz")
            start_time = loop_start_time
            loop_counter = 0

        filter_gps = 0
        got_gps_multiple = 0
        if got_gps:
            old_gps_latitude = real_gps_latitude
            old_gps_longitude = real_gps_longitude

            gps_latitude = bn357_get_latitude_decimal_format()
            gps_longitude = bn357_get_linear_longitude_decimal_format()  # Linear for PID
            real_gps_latitude = gps_latitude
            real_gps_longitude = gps_longitude
            real_gps_real_longitude = bn357_get_longitude_decimal_format()

            newest_gps_latitude = gps_latitude
            newest_gps_longitude = gps_longitude

            gps_real_longitude = bn357_get_longitude_decimal_format()
            gps_fix_type = bn357_get_fix_type()
            gps_satellites_count = bn357_get_satellites_quantity()

            print(f"old_gps_latitude != 0.0 : {old_gps_latitude != 0.0}")

            if use_gps_multiply and old_gps_latitude != 0.0:
                time_between_gps_refresh_microseconds = get_absolute_time() - last_got_gps_timestamp_microseconds
                last_got_gps_timestamp_microseconds = get_absolute_time()

                delta_gps_lat = (gps_latitude - old_gps_latitude) / multiply_gps_frequency
                delta_gps_lon = (gps_longitude - old_gps_longitude) / multiply_gps_frequency

                gps_latitude_micro = old_gps_latitude
                gps_longitude_micro = old_gps_longitude

                last_gps_microstep_timestamp_microseconds = get_absolute_time()
                gps_micro_bits_added = 0
            elif old_gps_latitude == 0.0:
                gps_latitude_micro = gps_latitude
                gps_longitude_micro = gps_longitude
                last_got_gps_timestamp_microseconds = get_absolute_time()


            filter_gps = 1
            gps_can_be_used = 1

        if use_gps_multiply and old_gps_latitude != 0.0:
            if (gps_can_be_used and 
                gps_micro_bits_added != multiply_gps_frequency - 1 and
                get_absolute_time() - last_gps_microstep_timestamp_microseconds >= time_between_gps_refresh_microseconds / (multiply_gps_frequency + 1)):
                last_gps_microstep_timestamp_microseconds = get_absolute_time()
                gps_latitude_micro += delta_gps_lat
                gps_longitude_micro += delta_gps_lon
                gps_micro_bits_added += 1
                filter_gps = 1
                got_gps_multiple = 1

            gps_latitude = gps_latitude_micro
            gps_longitude = gps_longitude_micro

        if filter_gps and use_gps_filtering:
            if gps_longitude - old_gps_longitude > 1000 :
                biquad_filter_gps_lon.set_initial_values(gps_longitude)
                biquad_filter_gps_lat.set_initial_values(gps_latitude)
            else:
                gps_longitude = biquad_filter_gps_lon.read(gps_longitude)
                gps_latitude = biquad_filter_gps_lat.read(gps_latitude)

        # Debug print for got_gps flag
        # print(f"got_gps: {got_gps} got_gps_multiple: {got_gps_multiple}")
        if got_gps or got_gps_multiple:
            # print(f"{real_gps_latitude:.1f};   {real_gps_longitude:.1f};   {real_gps_real_longitude:.1f};        {gps_latitude:.1f};   {gps_longitude:.1f};   {gps_real_longitude:.1f};")
            if got_gps:
                print(f"{real_gps_latitude:.1f};   {real_gps_longitude:.1f};        {gps_latitude:.1f};   {gps_longitude:.1f};  got_gps")
            else:
                print(f"{real_gps_latitude:.1f};   {real_gps_longitude:.1f};        {gps_latitude:.1f};   {gps_longitude:.1f};  got_gps_multiple")

        # Busy-wait to maintain the control loop frequency
        elapsed_time_us = int((time.perf_counter() - loop_start_time) * 1_000_000)
        busy_wait(control_loop_period_us - elapsed_time_us)

if __name__ == "__main__":
    main()
