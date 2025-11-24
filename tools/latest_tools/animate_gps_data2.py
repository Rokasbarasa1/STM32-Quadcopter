import numpy as np
import matplotlib.pyplot as plt
import tilemapbase
import warnings
import matplotlib.cbook
# warnings.filterwarnings("ignore", category=matplotlib.cbook.mplDeprecation)
# import shapely.speedups
# shapely.speedups.enable()
from matplotlib.animation import FuncAnimation
import argparse
import json
import math
import time

has_real_gps = True


skip_every_few_rows = False

recalculate_errors = False
recalculate_errors_on_yaw = False


use_old_gps_error_calc = False

disable_roll_and_pitch_arrows = False

plot_mag_primary_yaw_arrow = True
plot_mag_secondary_yaw_arrow = True
plot_gps_yaw_arrow = False
plot_gyro_yaw_arrow = False



has_yaw_extra_data = True
map_buffer = 0.00010
# map_buffer = 0.0000

# map_buffer = 0.003

def decimate_array(arr, rows_per_second=520, keep_rows=10):
    step = rows_per_second // keep_rows
    # Compute 10 evenly spaced indices in each block of 520 rows
    indices = np.round(np.linspace(0, rows_per_second-1, keep_rows)).astype(int)
    num_windows = arr.shape // rows_per_second
    # Gather the selected rows for each window
    decimated = np.vstack([
        arr[window*rows_per_second + indices]
        for window in range(num_windows)
    ])
    return decimated


class PID:
    CONVERT_MICROSECONDS_TO_SECONDS = 1e-6  # This is equivalent to 1.0/1000000.0

    def __init__(self, gain_proportional, gain_integral, gain_derivative, desired_value, time, max_value, min_value, stop_windup_integral, stop_windup_derivative):
        self.m_gain_proportional = gain_proportional
        self.m_gain_integral = gain_integral
        self.m_gain_derivative = gain_derivative
        self.m_integral_sum = 0
        self.m_last_error_for_d_term = 0
        self.m_desired_value = desired_value
        self.m_previous_time = time
        self.m_max_value = max_value
        self.m_min_value = min_value
        self.m_stop_windup_integral = stop_windup_integral
        self.m_stop_windup_derivative = stop_windup_derivative
        self.m_last_proportional_error = 0
        self.m_last_integral_error = 0
        self.m_last_derivative_error = 0

    def pid_set_desired_value(self, value):
        self.m_desired_value = value

    def pid_get_error_own_error(self, error, time):
        error_p = 0
        error_i = 0
        error_d = 0
        elapsed_time_sec = (time - self.m_previous_time) * self.CONVERT_MICROSECONDS_TO_SECONDS

        # Proportional
        error_p = error

        # Integral
        self.m_integral_sum += (error * elapsed_time_sec)

        if self.m_stop_windup_integral == 1:
            # Clamp the integral if it is getting out of bounds
            if (self.m_integral_sum * self.m_gain_integral) > self.m_max_value:
                self.m_integral_sum = self.m_max_value / self.m_gain_integral
            elif self.m_integral_sum * self.m_gain_integral < self.m_min_value:
                self.m_integral_sum = self.m_min_value / self.m_gain_integral

        error_i = self.m_integral_sum

        # Derivative
        error_d = (error_p - self.m_last_error_for_d_term) / elapsed_time_sec

        # Don't let it get out of bounds
        if error_d * self.m_gain_derivative > self.m_max_value:
            error_d = self.m_max_value
        elif error_d * self.m_gain_derivative < self.m_min_value:
            error_d = self.m_min_value

        # Set the previous error for the next iteration
        self.m_last_error_for_d_term = error

        self.m_last_proportional_error = self.m_gain_proportional * error_p
        self.m_last_integral_error = self.m_gain_integral * error_i
        self.m_last_derivative_error = self.m_gain_derivative * error_d

        # End result
        total_error = self.m_last_proportional_error + self.m_last_integral_error + self.m_last_derivative_error

        # Save the time for next calculation
        self.m_previous_time = time
        return total_error



def triangle_wave(x):
    # Normalize x to be within the range [0, 2*PI)
    x = x % (math.pi * 2)

    # Scale the normalized x to the range [0, 4]
    scaled_x = x / (math.pi/2)

    # Triangle wave calculation
    if scaled_x < 1.0:
        return scaled_x
    elif scaled_x < 3.0:
        return 2.0 - scaled_x
    else:
        return scaled_x - 4.0

def triangle_sin(x):
    return triangle_wave(x)

def triangle_cos(x):
    return triangle_wave(x + math.pi / 2)


def do(file_path, fast_animation=False, slow_animation=False, skip_no_action_rows=False):
    global map_buffer
    # I fucked up gps logging so this script fixes that 

    data = np.genfromtxt(file_path, delimiter=';', usecols=range(16), skip_header=1)

    # Skip rows with 0 in the first column until a real value shows up
    first_non_zero_index = np.argmax(data[:, 0] != 0)
    data = data[first_non_zero_index:]

    if skip_no_action_rows:
        # Filter out rows where error_lat and error_lon are both 0.0
        roll_error = (data[:, 10]).copy() # 0.0;
        pitch_error = (data[:, 11]).copy() # 0.0;
        non_zero_error_indices = (roll_error != 0.0) | (pitch_error != 0.0)
        data = data[non_zero_error_indices]

    if skip_every_few_rows:
        skip = 10  # 521Hz / 10Hz ≈ 52
        data = data[::skip]


    target_lat = (data[:, 0] / 1000000.0).copy() # 55651888.0;
    target_lon = (data[:, 1] / 1000000.0).copy() # 7095057.5;
    current_lat = (data[:, 2] / 1000000.0).copy() # 55651888.0;
    current_lon = (data[:, 3] / 1000000.0).copy() # 7095057.5;
    sats = (data[:, 4]).copy() # 12;
    yaw_degrees = (data[:, 5]).copy() # yaw;
    yaw_gyro = (data[:, 6]).copy() # gyro yaw
    yaw_mag1 = (data[:, 7]).copy() # mag yaw
    yaw_mag2 = (data[:, 8]).copy()# # mag2 yaw
    yaw_gps = (data[:, 9]).copy()# gps yaw
    roll_error = (data[:, 10]).copy() # 0.0;
    pitch_error = (data[:, 11]).copy() # 0.0;
    roll_degrees = (data[:, 12]).copy() # 2.2;
    pitch_degrees = (data[:, 13]).copy() # 1.8;


    # absolute_time = 2
    # lat_pid = PID(0.8, 0, 0, 0.0, absolute_time, 0, 0, 0, 0)
    # lon_pid = PID(0.8, 0, 0, 0.0, absolute_time, 0, 0, 0, 0)

    # if recalculate_errors_on_yaw:
    #      for i in range(len(data)):
    #         absolute_time = absolute_time + 1923

    #         yaw_rad = yaw_degrees[i]  * 0.0174533
            

    #         error_forward = 1.0 * cosf(yaw_rad) + 1.0 * sinf(yaw_rad);
    #         error_right   = -1.0 * sinf(yaw_rad) + 1.0 * cosf(yaw_rad);

    #         # temp_roll_effect_on_lat = roll_effect_on_lat[i]
    #         # temp_pitch_effect_on_lat = pitch_effect_on_lat[i]
    #         # temp_roll_effect_on_lon = roll_effect_on_lon[i]
    #         # temp_pitch_effect_on_lon = pitch_effect_on_lon[i]

    #         # roll_effect_on_lat[i] = math.sin(yaw_degrees[i] * 0.0174533)
    #         # pitch_effect_on_lat[i] = -math.cos(yaw_degrees[i] * 0.0174533)
    #         # roll_effect_on_lon[i] = -math.cos(yaw_degrees[i] * 0.0174533)
    #         # pitch_effect_on_lon[i] = -math.sin(yaw_degrees[i] * 0.0174533)
    #         # roll_effect_on_lat[i] = 0
    #         # pitch_effect_on_lat[i] = 0
    #         # roll_effect_on_lon[i] = -math.cos(yaw_degrees[i] * 0.0174533)
    #         # pitch_effect_on_lon[i] = 0
            
    #         print("LOCAl")
    #         print(math.sin(yaw_degrees[i] * 0.0174533))
    #         print(math.cos(yaw_degrees[i] * 0.0174533))



    # Iterate over data and calculate errors
    # if recalculate_errors:
    #     for i in range(len(data)):
    #         absolute_time = absolute_time + 1923
    #         # temp_roll_effect_on_lat = roll_effect_on_lat[i]
    #         # temp_pitch_effect_on_lat = pitch_effect_on_lat[i]
    #         # temp_roll_effect_on_lon = roll_effect_on_lon[i]
    #         # temp_pitch_effect_on_lon = pitch_effect_on_lon[i]

    #         # roll_effect_on_lat[i] = math.sin(yaw_degrees[i] * 0.0174533)
    #         # pitch_effect_on_lat[i] = -math.cos(yaw_degrees[i] * 0.0174533)
    #         # roll_effect_on_lon[i] = -math.cos(yaw_degrees[i] * 0.0174533)
    #         # pitch_effect_on_lon[i] = -math.sin(yaw_degrees[i] * 0.0174533)
    #         # roll_effect_on_lat[i] = 0
    #         # pitch_effect_on_lat[i] = 0
    #         # roll_effect_on_lon[i] = -math.cos(yaw_degrees[i] * 0.0174533)
    #         # pitch_effect_on_lon[i] = 0
            
    #         print("LOCAl")
    #         print(math.sin(yaw_degrees[i] * 0.0174533))
    #         print(math.cos(yaw_degrees[i] * 0.0174533))

    #         # # Calculate all values first
    #         # temp_value = -math.cos(yaw_degrees[i] * 0.0174533)
    #         # print("Calculation")
    #         # print(temp_value)
    #         # pitch_effect_on_lat[i] = temp_value
    #         # print(pitch_effect_on_lat[i])

    #         # print("Calculation")
    #         # print(-math.cos(yaw_degrees[i] * 0.0174533))
    #         # print(pitch_effect_on_lat[i])








    #         # pitch_effect_on_lon[i] = -math.sin(yaw_degrees[i] * 0.0174533)


    #         # roll_effect_on_lat[i] = -math.sin(yaw_degrees[i] * 0.0174533)
    #         # pitch_effect_on_lat[i] = math.cos(yaw_degrees[i] * 0.0174533)
    #         # roll_effect_on_lon[i] = math.cos(yaw_degrees[i] * 0.0174533)
    #         # pitch_effect_on_lon[i] = -math.sin(yaw_degrees[i] * 0.0174533)

    #         # pitch_effect_on_lat[i] = triangle_cos(yaw_radians)# * latitude_sign;  // + GOOD, - GOOD  // If moving north forward positive, backwards negative
    #         # roll_effect_on_lat[i] = -triangle_sin(yaw_radians)#  * latitude_sign;  // + GOOD, - GOOD  // If moving north roll right positive, roll left south negative
    #         # pitch_effect_on_lon[i] = triangle_sin(yaw_radians)# * longitude_sign; // + GOOD, - GOOD 
    #         # roll_effect_on_lon[i] = triangle_cos(yaw_radians)# * longitude_sign;  // + GOOD, - GOOD

    #         # roll_effect_on_lat[i] = -math.sin(yaw_radians)#  * latitude_sign;  // + GOOD, - GOOD  // If moving north roll right positive, roll left south negative
    #         # pitch_effect_on_lat[i] = math.sin(yaw_radians)# * latitude_sign;  // + GOOD, - GOOD  // If moving north forward positive, backwards negative
    #         # roll_effect_on_lon[i] = math.cos(yaw_radians)# * longitude_sign;  // + GOOD, - GOOD
    #         # pitch_effect_on_lon[i] = math.sin(yaw_radians)# * longitude_sign; // + GOOD, - GOOD 

    #         lat_distance_to_target_meters[i] = (current_lat[i] - target_lat[i]) * 111320.0
    #         lon_distance_to_target_meters[i] = (current_real_lon[i] - target_real_lon[i]) * 111320.0 * math.cos(current_lat[i] * 0.0174533);

    #         if math.isnan(lat_distance_to_target_meters[i]) or math.isnan(lon_distance_to_target_meters[i]):
    #             print('Error value')
    #             continue

    #         lat_error = lat_pid.pid_get_error_own_error(lat_distance_to_target_meters[i], absolute_time)
    #         lon_error = lon_pid.pid_get_error_own_error(lon_distance_to_target_meters[i], absolute_time)


    #         # print(f"lat_error[i]: {lat_error}")
    #         # print(f"lon_error[i]: {lon_error}")

    #         if not use_old_gps_error_calc:

    #             error_magnitude = math.sqrt(lat_error * lat_error + lon_error * lon_error)

    #             normalized_error_latitude = 0.0
    #             normalized_error_longitude = 0.0

    #             if error_magnitude != 0.0:
    #                 normalized_error_latitude = lat_error / error_magnitude
    #                 normalized_error_longitude = lon_error / error_magnitude


    #             gps_hold_roll_adjustment_calculation = (roll_effect_on_lat[i] * normalized_error_latitude +
    #                                                     roll_effect_on_lon[i] * normalized_error_longitude)
    #             gps_hold_pitch_adjustment_calculation = (pitch_effect_on_lat[i] * normalized_error_latitude +
    #                                                     pitch_effect_on_lon[i] * normalized_error_longitude)

    #             scale_factor = min(error_magnitude, 5.0)

    #             # print(f"Before roll_error[i]: {roll_error[i]} ")
    #             # print(f"Before pitch_error[i]: {pitch_error[i]} ")

    #             roll_error[i] = gps_hold_roll_adjustment_calculation * scale_factor
    #             pitch_error[i] = gps_hold_pitch_adjustment_calculation * scale_factor

    #             # print(f"After roll_error[i]: {roll_error[i]} ")
    #             # print(f"After pitch_error[i]: {pitch_error[i]} ")
    #             # print("")

    #         else:

    #             gps_hold_roll_adjustment_calculation = roll_effect_on_lat[i] * lat_error + roll_effect_on_lon[i] * lon_error
    #             gps_hold_pitch_adjustment_calculation = pitch_effect_on_lat[i] * lat_error + pitch_effect_on_lon[i] * lon_error
    #             print("TESTING")
    #             print(lat_error)
    #             print(lon_error)
    #             print(roll_effect_on_lat[i])
    #             print(roll_effect_on_lon[i])
    #             print(pitch_effect_on_lat[i])
    #             print(pitch_effect_on_lon[i])
    #             print(gps_hold_pitch_adjustment_calculation)

    #             if gps_hold_roll_adjustment_calculation > 5.0: 
    #                 gps_hold_roll_adjustment_calculation = 5.0
    #             elif gps_hold_roll_adjustment_calculation < -5.0: 
    #                 gps_hold_roll_adjustment_calculation = -5.0

    #             if gps_hold_pitch_adjustment_calculation > 5.0: 
    #                 gps_hold_pitch_adjustment_calculation = 5.0
    #             elif gps_hold_pitch_adjustment_calculation < -5.0: 
    #                 gps_hold_pitch_adjustment_calculation = -5.0

    #             roll_error[i] = gps_hold_roll_adjustment_calculation
    #             pitch_error[i] = gps_hold_pitch_adjustment_calculation

    #         # print(f"Yaw: {yaw_degrees[i]}")
    #         # print(f"Original Roll Effect on Lat: {temp_roll_effect_on_lat}, Calculated: {roll_effect_on_lat[i]}")
    #         # print(f"Original Pitch Effect on Lat: {temp_pitch_effect_on_lat}, Calculated: {pitch_effect_on_lat[i]}")
    #         # print(f"Original Roll Effect on Lon: {temp_roll_effect_on_lon}, Calculated: {roll_effect_on_lon[i]}")
    #         # print(f"Original Pitch Effect on Lon: {temp_pitch_effect_on_lon}, Calculated: {pitch_effect_on_lon[i]}")
    #         # print("\n")

    #         # Length of the vector of lat and lon errrors
    #         # error_magnitude = math.sqrt(error_lat[i] * error_lat[i] + error_lon[i] * error_lon[i])

    #         # if error_magnitude == 0:
    #         #     normalized_error_latitude = 0
    #         #     normalized_error_longitude = 0
    #         # else:
    #         #     normalized_error_latitude = error_lat[i] / error_magnitude
    #         #     normalized_error_longitude = error_lon[i] / error_magnitude

    #         # gps_hold_roll_adjustment_calculation = (roll_effect_on_lat[i] * normalized_error_latitude +
    #         #                                         roll_effect_on_lon[i] * normalized_error_longitude)
    #         # gps_hold_pitch_adjustment_calculation = (pitch_effect_on_lat[i] * normalized_error_latitude +
    #         #                                         pitch_effect_on_lon[i] * normalized_error_longitude)

    #         # scale_factor = min(error_magnitude, 5.0)

    #         # roll_error[i] = gps_hold_roll_adjustment_calculation * scale_factor
    #         # pitch_error[i] = gps_hold_pitch_adjustment_calculation * scale_factor

    # Iterate over data and set roll_error and pitch_error to 0.0 if error_lat and error_lon are 0.0




    # for i in range(len(data)):
    #     if error_lat[i] == 0.0 and error_lon[i] == 0.0:
    #         roll_error[i] = 0.0
    #         pitch_error[i] = 0.0

    # The plot does not know where north is, this puts the yaw when it is on north actually on north
    yaw_degrees = (yaw_degrees + 270) % 360
    yaw_gyro = (yaw_gyro + 270) % 360
    yaw_mag1 = (yaw_mag1 + 270) % 360
    yaw_mag2 = (yaw_mag2 + 270) % 360
    yaw_gps = (yaw_gps + 270) % 360

    min_lon = current_lon.min() - map_buffer
    max_lon = current_lon.max() + map_buffer
    min_lat = current_lat.min() - map_buffer
    max_lat = current_lat.max() + map_buffer
        # current_lon.min() - map_buffer, 
        # current_lon.max() + map_buffer, 
        # current_lat.min() - map_buffer, 
        # current_lat.max() + map_buffer]



    # Compute min/max longitude and latitude with buffer
    min_lon = current_lon.min() - map_buffer
    max_lon = current_lon.max() + map_buffer
    min_lat = current_lat.min() - map_buffer
    max_lat = current_lat.max() + map_buffer

    lon_range = max_lon - min_lon
    lat_range = max_lat - min_lat

    # Adjust ranges so map bounding box is square by expanding smaller range
    if lon_range > lat_range:
        lat_mid = (max_lat + min_lat) / 2
        min_lat = lat_mid - lon_range / 2
        max_lat = lat_mid + lon_range / 2
    elif lat_range > lon_range:
        lon_mid = (max_lon + min_lon) / 2
        min_lon = lon_mid - lat_range / 2
        max_lon = lon_mid + lat_range / 2

    bounding_box = [min_lon, max_lon, min_lat, max_lat]



    # Initialize tilemapbase
    tilemapbase.init(create=True)
    extent = tilemapbase.Extent.from_lonlat(bounding_box[0], bounding_box[1], bounding_box[2], bounding_box[3])  # Example coordinates (London area)
    extent = extent.to_aspect(1.0)
    tiles = tilemapbase.tiles.build_OSM()
    fig, ax = plt.subplots(figsize=(10, 10))

    # Add the background map
    plotter = tilemapbase.Plotter(extent, tiles, zoom=18)
    plotter.plot(ax, allow_large = True)

    # Project and plot the data points
    projected_points_target = np.array([tilemapbase.project(lon, lat) for lon, lat in zip(target_lon, target_lat)])

    x_target = projected_points_target[:, 0]
    y_target = projected_points_target[:, 1]

    projected_points_current = np.array([tilemapbase.project(lon, lat) for lon, lat in zip(current_lon, current_lat)])

    x_current = projected_points_current[:, 0]
    y_current = projected_points_current[:, 1]

    # Plot the points on the map
    scatter_current = ax.scatter([], [], color='blue', s=35, label='Current')
    scatter_target = ax.scatter([], [], color='red', s=35, label='Current')

    plot_current, = ax.plot([], [], color='blue', linewidth=1, label='Path')

    # Initialize the arrow to indicate yaw
    quiver_yaw = ax.quiver(x_current[0], y_current[0], np.cos(np.radians(yaw_mag1[0])) * 0.0001, np.sin(np.radians(yaw_mag1[0])) * 0.0001, scale=1, scale_units='xy', angles='xy', color='red')
    quiver_pitch_gps = ax.quiver(x_current[0], y_current[0], np.cos(np.radians(yaw_mag1[0])) * 0.0000002, np.sin(np.radians(yaw_mag1[0])) * 0.0000002, scale=1, scale_units='xy', angles='xy', color='orange')
    quiver_pitch_drone = ax.quiver(x_current[0], y_current[0], np.cos(np.radians(yaw_mag1[0])) * 0.0000002, np.sin(np.radians(yaw_mag1[0])) * 0.0000002, scale=1, scale_units='xy', angles='xy', color='purple', width=0.002)
    quiver_roll_gps = ax.quiver(x_current[0], y_current[0], np.cos(np.radians(yaw_mag1[0] + 90)) * 0.0000002, np.sin(np.radians(yaw_mag1[0] + 90)) * 0.0000002, scale=1, scale_units='xy', angles='xy', color='lightBlue')
    quiver_roll_drone = ax.quiver(x_current[0], y_current[0], np.cos(np.radians(yaw_mag1[0] + 90)) * 0.0000002, np.sin(np.radians(yaw_mag1[0] + 90)) * 0.0000002, scale=1, scale_units='xy', angles='xy', color='purple', width=0.002)

    quiver_yaw_mag2 = ax.quiver(x_current[0], y_current[0], np.cos(np.radians(yaw_mag2[0])) * 0.0001, np.sin(np.radians(yaw_mag2[0])) * 0.0001, scale=1, scale_units='xy', angles='xy', color='blue')
    quiver_yaw_gps = ax.quiver(x_current[0], y_current[0], np.cos(np.radians(yaw_gps[0])) * 0.0001, np.sin(np.radians(yaw_gps[0])) * 0.0001, scale=1, scale_units='xy', angles='xy', color='black')
    quiver_yaw_gyro = ax.quiver(x_current[0], y_current[0], np.cos(np.radians(yaw_gyro[0])) * 0.0001, np.sin(np.radians(yaw_gyro[0])) * 0.0001, scale=1, scale_units='xy', angles='xy', color='white')

    # Add title and labels if needed
    ax.set_title("Map with Points Overlay")
    ax.set_xlabel("Longitude")
    ax.set_ylabel("Latitude")

    def animate(i):
        if i < len(x_target):

            
            if plot_mag_primary_yaw_arrow:
                dx = np.cos(np.radians(yaw_mag1[i])) * 0.0000002
                dy = np.sin(np.radians(yaw_mag1[i])) * 0.0000002
            else:
                dx = 0.0
                dy = 0.0

            if plot_mag_secondary_yaw_arrow:
                dx_mag2 = np.cos(np.radians(yaw_mag2[i])) * 0.0000002
                dy_mag2 = np.sin(np.radians(yaw_mag2[i])) * 0.0000002
            else:
                dx_mag2 = 0.0
                dy_mag2 = 0.0

            if plot_gps_yaw_arrow:
                dx_gps = np.cos(np.radians(yaw_gps[i])) * 0.0000002
                dy_gps = np.sin(np.radians(yaw_gps[i])) * 0.0000002
            else:
                dx_gps = 0.0
                dy_gps = 0.0

            if plot_gyro_yaw_arrow:
                dx_gyro = np.cos(np.radians(yaw_gyro[i])) * 0.0000002
                dy_gyro = np.sin(np.radians(yaw_gyro[i])) * 0.0000002
            else:
                dx_gyro = 0.0
                dy_gyro = 0.0
            

            quiver_yaw.set_offsets([x_current[i], y_current[i]])
            quiver_yaw.set_UVC(dx, dy)

            quiver_yaw_mag2.set_offsets([x_current[i], y_current[i]])
            quiver_yaw_mag2.set_UVC(dx_mag2, dy_mag2)

            quiver_yaw_gps.set_offsets([x_current[i], y_current[i]])
            quiver_yaw_gps.set_UVC(dx_gps, dy_gps)

            quiver_yaw_gyro.set_offsets([x_current[i], y_current[i]])
            quiver_yaw_gyro.set_UVC(dx_gyro, dy_gyro)


            if(disable_roll_and_pitch_arrows):
                dx_pitch_gps = np.cos(np.radians(yaw_mag1[i])) * 0.0000001 * 0.3
                dy_pitch_gps = np.sin(np.radians(yaw_mag1[i])) * 0.0000001 * 0.3
            else:
                dx_pitch_gps = np.cos(np.radians(yaw_mag1[i])) * 0.0000001 * pitch_error[i]
                dy_pitch_gps = np.sin(np.radians(yaw_mag1[i])) * 0.0000001 * pitch_error[i]
            quiver_pitch_gps.set_offsets([x_current[i], y_current[i]])
            quiver_pitch_gps.set_UVC(dx_pitch_gps, dy_pitch_gps)



            if(disable_roll_and_pitch_arrows):
                dx_pitch_drone = np.cos(np.radians(yaw_mag1[i])) * 0.0000001 * 0.3
                dy_pitch_drone = np.sin(np.radians(yaw_mag1[i])) * 0.0000001 * 0.3
            else:
                dx_pitch_drone = np.cos(np.radians(yaw_mag1[i])) * 0.0000001 * pitch_degrees[i]
                dy_pitch_drone = np.sin(np.radians(yaw_mag1[i])) * 0.0000001 * pitch_degrees[i]
            quiver_pitch_drone.set_offsets([x_current[i], y_current[i]])
            quiver_pitch_drone.set_UVC(dx_pitch_drone, dy_pitch_drone)


            if(disable_roll_and_pitch_arrows):
                dx_roll_gps = np.cos(np.radians(yaw_mag1[i] + 90)) * 0.0000001 * 0.3
                dy_roll_gps = np.sin(np.radians(yaw_mag1[i] + 90)) * 0.0000001 * 0.3
            else:
                dx_roll_gps = np.cos(np.radians(yaw_mag1[i] + 90)) * 0.0000001 * roll_error[i]
                dy_roll_gps = np.sin(np.radians(yaw_mag1[i] + 90)) * 0.0000001 * roll_error[i]
            quiver_roll_gps.set_offsets([x_current[i], y_current[i]])
            quiver_roll_gps.set_UVC(dx_roll_gps, dy_roll_gps)

            
            if(disable_roll_and_pitch_arrows):
                dx_roll_drone = np.cos(np.radians(yaw_mag1[i] + 90)) * 0.0000001 * 0.3
                dy_roll_drone = np.sin(np.radians(yaw_mag1[i] + 90)) * 0.0000001 * 0.3
            else:
                dx_roll_drone = np.cos(np.radians(yaw_mag1[i] + 90)) * 0.0000001 * roll_degrees[i]
                dy_roll_drone = np.sin(np.radians(yaw_mag1[i] + 90)) * 0.0000001 * roll_degrees[i]
            quiver_roll_drone.set_offsets([x_current[i], y_current[i]])
            quiver_roll_drone.set_UVC(dx_roll_drone, dy_roll_drone)

            # Update the scatter plot data
            plot_current.set_data(x_current[:i+1], y_current[:i+1])
            scatter_current.set_offsets([x_current[i], y_current[i]])
            scatter_target.set_offsets([x_target[i], y_target[i]])

            if(i % 10 == 0):
                print(i)

            # Set the animation interval to match the desired frequency (521 Hz)
            # interval = int(1000 / 521)
            # ani.event_source.interval = interval

            # Return the updated objects
            return plot_current, scatter_current, scatter_target, quiver_yaw, quiver_yaw_mag2, quiver_yaw_gps, quiver_yaw_gyro, quiver_pitch_gps, quiver_roll_gps, quiver_pitch_drone, quiver_roll_drone




    if slow_animation:
        ani = FuncAnimation(fig, animate, frames=len(x_target), interval=200, blit=True)
    elif fast_animation:
        ani = FuncAnimation(fig, animate, frames=len(x_target), interval=1, blit=True)
    else:
        ani = FuncAnimation(fig, animate, frames=len(x_target), interval=33, blit=True)

    plt.show()

def main():
    parser = argparse.ArgumentParser(description="Remove lines containing 'NAN' from a file.")
    parser.add_argument('file_path', type=str, help='Path to the file to be cleaned')
    parser.add_argument('--slow', action='store_true', help='Enable slow mode processing')
    parser.add_argument('--active', action='store_true', help='Skip lines containing "inactive"')
    parser.add_argument('--fast', action='store_true', help='Enable fast mode processing')

    args = parser.parse_args()

    do(args.file_path, fast_animation=args.fast, slow_animation=args.slow, skip_no_action_rows=args.active)

if __name__ == "__main__":
    main()