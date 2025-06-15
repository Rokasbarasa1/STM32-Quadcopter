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

skip_no_action_rows = True

skip_every_few_rows = False

recalculate_errors = False

use_old_gps_error_calc = False

slow_animation = False

disable_roll_and_pitch_arrows = True

map_buffer = 0.00010
# map_buffer = 0.003

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

def do(file_path):
    global map_buffer
    # I fucked up gps logging so this script fixes that 
    if has_real_gps:
        data = np.genfromtxt(file_path, delimiter=';', usecols=range(21))
    else:
        data = np.genfromtxt(file_path, delimiter=';', usecols=range(16))

    # Skip rows with 0 in the first column until a real value shows up
    first_non_zero_index = np.argmax(data[:, 0] != 0)
    data = data[first_non_zero_index:]


    if has_real_gps:

        real_gps_lat = (data[:, 0] / 1000000.0).copy() # 55651888.0;
        real_gps_lon = (data[:, 1] / 1000000.0).copy() # 7095057.5
        real_gps_real_lon = (data[:, 2] / 1000000.0).copy() # 12575003.0;
        target_lat = (data[:, 3] / 1000000.0).copy() # 55651888.0;
        target_lon = (data[:, 4] / 1000000.0).copy() # 7095057.5;
        target_real_lon = (data[:, 5] / 1000000.0).copy() # 12575003.0;
        current_lat = (data[:, 6] / 1000000.0).copy() # 55651888.0;
        current_lon = (data[:, 7] / 1000000.0).copy() # 7095057.5;
        current_real_lon = (data[:, 8] / 1000000.0).copy() # 12575003.0;
        sats = (data[:, 9]).copy() # 12;
        yaw_degrees = (data[:, 10]).copy() # 37.4;
        roll_effect_on_lat = (data[:, 11]).copy() # -0.61;
        pitch_effect_on_lat = (data[:, 12]).copy() # 0.79;
        roll_effect_on_lon = (data[:, 12]).copy()
        pitch_effect_on_lon = (data[:, 11] * -1).copy()
        PID_proportional = (data[:, 13]).copy() # 0.00;
        PID_derivative = (data[:, 14]).copy() # 0.00;
        roll_error = (data[:, 15]).copy() # 0.0;
        pitch_error = (data[:, 16]).copy() # 0.0;
        roll_degrees = (data[:, 17]).copy() # 2.2;
        pitch_degrees = (data[:, 18]).copy() # 1.8;

        lat_distance_to_target_meters = (data[:, 19]).copy()
        lon_distance_to_target_meters = (data[:, 20]).copy()

        target_real_lon = (target_lon).copy()
        current_real_lon = (current_lon).copy()

    else:
        real_gps_lat = (data[:, 0] / 1000000.0).copy()
        real_gps_lon = (data[:, 1] / 1000000.0).copy()
        target_lat = (data[:, 2] / 1000000.0).copy()
        target_lon = (data[:, 3] / 1000000.0).copy()
        current_lat = (data[:, 4] / 1000000.0).copy()
        current_lon = (data[:, 5] / 1000000.0).copy()
        sats = (data[:, 6]).copy()
        yaw_degrees = (data[:, 7]).copy()
        roll_effect_on_lat = (data[:, 8]).copy()
        pitch_effect_on_lat = (data[:, 9]).copy()
        roll_effect_on_lon = (data[:, 9]).copy()
        pitch_effect_on_lon = (data[:, 8] * -1).copy()
        PID_proportional = (data[:, 10]).copy()
        PID_derivative = (data[:, 11]).copy()
        roll_error = (data[:, 12]).copy()
        pitch_error = (data[:, 13]).copy()
        roll_degrees = (data[:, 14]).copy()
        pitch_degrees = (data[:, 15]).copy()
    

    if skip_no_action_rows:

        # Filter out rows where error_lat and error_lon are both 0.0

        if has_real_gps:
            non_zero_error_indices = (roll_error != 0.0) | (pitch_error != 0.0)
            data = data[non_zero_error_indices]
            real_gps_lat = real_gps_lat[non_zero_error_indices]
            real_gps_lon = real_gps_lon[non_zero_error_indices]
            real_gps_real_lon = real_gps_real_lon[non_zero_error_indices]
            target_lat = target_lat[non_zero_error_indices]
            target_lon = target_lon[non_zero_error_indices]
            target_real_lon = target_real_lon[non_zero_error_indices]
            current_lat = current_lat[non_zero_error_indices]
            current_lon = current_lon[non_zero_error_indices]
            current_real_lon = current_real_lon[non_zero_error_indices]
            sats = sats[non_zero_error_indices]
            yaw_degrees = yaw_degrees[non_zero_error_indices]
            roll_effect_on_lat = roll_effect_on_lat[non_zero_error_indices]
            pitch_effect_on_lat = pitch_effect_on_lat[non_zero_error_indices]
            roll_effect_on_lon = roll_effect_on_lon[non_zero_error_indices]
            pitch_effect_on_lon = pitch_effect_on_lon[non_zero_error_indices]
            PID_proportional = PID_proportional[non_zero_error_indices]
            PID_derivative = PID_derivative[non_zero_error_indices]
            roll_error = roll_error[non_zero_error_indices]
            pitch_error = pitch_error[non_zero_error_indices]
            roll_degrees = roll_degrees[non_zero_error_indices]
            pitch_degrees = pitch_degrees[non_zero_error_indices]
            lat_distance_to_target_meters = lat_distance_to_target_meters[non_zero_error_indices]
            lon_distance_to_target_meters = lon_distance_to_target_meters[non_zero_error_indices]
        else:
            non_zero_error_indices = (roll_error != 0.0) | (roll_degrees != 0.0)
            data = data[non_zero_error_indices]
            real_gps_lat = real_gps_lat[non_zero_error_indices]
            real_gps_lon = real_gps_lon[non_zero_error_indices]
            target_lat = target_lat[non_zero_error_indices]
            target_lon = target_lon[non_zero_error_indices]
            current_lat = current_lat[non_zero_error_indices]
            current_lon = current_lon[non_zero_error_indices]
            sats = sats[non_zero_error_indices]
            yaw_degrees = yaw_degrees[non_zero_error_indices]
            roll_effect_on_lat = roll_effect_on_lat[non_zero_error_indices]
            pitch_effect_on_lat = pitch_effect_on_lat[non_zero_error_indices]
            roll_effect_on_lon = roll_effect_on_lon[non_zero_error_indices]
            pitch_effect_on_lon = pitch_effect_on_lon[non_zero_error_indices]
            PID_proportional = PID_proportional[non_zero_error_indices]
            PID_derivative = PID_derivative[non_zero_error_indices]
            roll_error = roll_error[non_zero_error_indices]
            pitch_error = pitch_error[non_zero_error_indices]
            roll_degrees = roll_degrees[non_zero_error_indices]
            pitch_degrees = pitch_degrees[non_zero_error_indices]

    if skip_every_few_rows:
        skip = 10  # 521Hz / 10Hz ≈ 52
        if has_real_gps: 
            real_gps_lat = real_gps_lat[::skip]
            real_gps_lon = real_gps_lon[::skip]
            real_gps_real_lon = real_gps_real_lon[::skip]
            target_lat = target_lat[::skip]
            target_lon = target_lon[::skip]
            target_real_lon = target_real_lon[::skip]
            current_lat = current_lat[::skip]
            current_lon = current_lon[::skip]
            current_real_lon = current_real_lon[::skip]
            sats = sats[::skip]
            yaw_degrees = yaw_degrees[::skip]
            roll_effect_on_lat = roll_effect_on_lat[::skip]
            pitch_effect_on_lat = pitch_effect_on_lat[::skip]
            roll_effect_on_lon = roll_effect_on_lon[::skip]
            pitch_effect_on_lon = pitch_effect_on_lon[::skip]
            PID_proportional = PID_proportional[::skip]
            PID_derivative = PID_derivative[::skip]
            roll_error = roll_error[::skip]
            pitch_error = pitch_error[::skip]
            roll_degrees = roll_degrees[::skip]
            pitch_degrees = pitch_degrees[::skip]
            target_real_lon = target_real_lon[::skip]
            current_real_lon = current_real_lon[::skip]
            target_real_lon = target_real_lon[::skip]
            lat_distance_to_target_meters = lat_distance_to_target_meters[::skip]
            lon_distance_to_target_meters = lon_distance_to_target_meters[::skip]
        else:
            real_gps_lat = real_gps_lat[::skip]
            real_gps_lon = real_gps_lon[::skip]
            target_lat = target_lat[::skip]
            target_lon = target_lon[::skip]
            current_lat = current_lat[::skip]
            current_lon = current_lon[::skip]
            sats = sats[::skip]
            yaw_degrees = yaw_degrees[::skip]
            roll_effect_on_lat = roll_effect_on_lat[::skip]
            pitch_effect_on_lat = pitch_effect_on_lat[::skip]
            roll_effect_on_lon = roll_effect_on_lon[::skip]
            pitch_effect_on_lon = pitch_effect_on_lon[::skip]
            PID_proportional = PID_proportional[::skip]
            PID_derivative = PID_derivative[::skip]
            roll_error = roll_error[::skip]
            pitch_error = pitch_error[::skip]
            roll_degrees = roll_degrees[::skip]
            pitch_degrees = pitch_degrees[::skip]


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


    absolute_time = 2
    lat_pid = PID(0.8, 0, 0, 0.0, absolute_time, 0, 0, 0, 0)
    lon_pid = PID(0.8, 0, 0, 0.0, absolute_time, 0, 0, 0, 0)

    # Iterate over data and calculate errors
    if recalculate_errors:
        for i in range(len(data)):
            absolute_time = absolute_time + 1923
            # temp_roll_effect_on_lat = roll_effect_on_lat[i]
            # temp_pitch_effect_on_lat = pitch_effect_on_lat[i]
            # temp_roll_effect_on_lon = roll_effect_on_lon[i]
            # temp_pitch_effect_on_lon = pitch_effect_on_lon[i]

            roll_effect_on_lat[i] = math.sin(yaw_degrees[i] * 0.0174533)
            pitch_effect_on_lat[i] = -math.cos(yaw_degrees[i] * 0.0174533)
            roll_effect_on_lon[i] = -math.cos(yaw_degrees[i] * 0.0174533)
            pitch_effect_on_lon[i] = -math.sin(yaw_degrees[i] * 0.0174533)
            # roll_effect_on_lat[i] = 0
            # pitch_effect_on_lat[i] = 0
            # roll_effect_on_lon[i] = -math.cos(yaw_degrees[i] * 0.0174533)
            # pitch_effect_on_lon[i] = 0
            
            print("LOCAl")
            print(math.sin(yaw_degrees[i] * 0.0174533))
            print(math.cos(yaw_degrees[i] * 0.0174533))

            # # Calculate all values first
            # temp_value = -math.cos(yaw_degrees[i] * 0.0174533)
            # print("Calculation")
            # print(temp_value)
            # pitch_effect_on_lat[i] = temp_value
            # print(pitch_effect_on_lat[i])

            # print("Calculation")
            # print(-math.cos(yaw_degrees[i] * 0.0174533))
            # print(pitch_effect_on_lat[i])








            # pitch_effect_on_lon[i] = -math.sin(yaw_degrees[i] * 0.0174533)


            # roll_effect_on_lat[i] = -math.sin(yaw_degrees[i] * 0.0174533)
            # pitch_effect_on_lat[i] = math.cos(yaw_degrees[i] * 0.0174533)
            # roll_effect_on_lon[i] = math.cos(yaw_degrees[i] * 0.0174533)
            # pitch_effect_on_lon[i] = -math.sin(yaw_degrees[i] * 0.0174533)

            # pitch_effect_on_lat[i] = triangle_cos(yaw_radians)# * latitude_sign;  // + GOOD, - GOOD  // If moving north forward positive, backwards negative
            # roll_effect_on_lat[i] = -triangle_sin(yaw_radians)#  * latitude_sign;  // + GOOD, - GOOD  // If moving north roll right positive, roll left south negative
            # pitch_effect_on_lon[i] = triangle_sin(yaw_radians)# * longitude_sign; // + GOOD, - GOOD 
            # roll_effect_on_lon[i] = triangle_cos(yaw_radians)# * longitude_sign;  // + GOOD, - GOOD

            # roll_effect_on_lat[i] = -math.sin(yaw_radians)#  * latitude_sign;  // + GOOD, - GOOD  // If moving north roll right positive, roll left south negative
            # pitch_effect_on_lat[i] = math.sin(yaw_radians)# * latitude_sign;  // + GOOD, - GOOD  // If moving north forward positive, backwards negative
            # roll_effect_on_lon[i] = math.cos(yaw_radians)# * longitude_sign;  // + GOOD, - GOOD
            # pitch_effect_on_lon[i] = math.sin(yaw_radians)# * longitude_sign; // + GOOD, - GOOD 

            lat_distance_to_target_meters[i] = (current_lat[i] - target_lat[i]) * 111320.0
            lon_distance_to_target_meters[i] = (current_real_lon[i] - target_real_lon[i]) * 111320.0 * math.cos(current_lat[i] * 0.0174533);

            if math.isnan(lat_distance_to_target_meters[i]) or math.isnan(lon_distance_to_target_meters[i]):
                print('Error value')
                continue

            lat_error = lat_pid.pid_get_error_own_error(lat_distance_to_target_meters[i], absolute_time)
            lon_error = lon_pid.pid_get_error_own_error(lon_distance_to_target_meters[i], absolute_time)


            # print(f"lat_error[i]: {lat_error}")
            # print(f"lon_error[i]: {lon_error}")

            if not use_old_gps_error_calc:

                error_magnitude = math.sqrt(lat_error * lat_error + lon_error * lon_error)

                normalized_error_latitude = 0.0
                normalized_error_longitude = 0.0

                if error_magnitude != 0.0:
                    normalized_error_latitude = lat_error / error_magnitude
                    normalized_error_longitude = lon_error / error_magnitude


                gps_hold_roll_adjustment_calculation = (roll_effect_on_lat[i] * normalized_error_latitude +
                                                        roll_effect_on_lon[i] * normalized_error_longitude)
                gps_hold_pitch_adjustment_calculation = (pitch_effect_on_lat[i] * normalized_error_latitude +
                                                        pitch_effect_on_lon[i] * normalized_error_longitude)

                scale_factor = min(error_magnitude, 5.0)

                # print(f"Before roll_error[i]: {roll_error[i]} ")
                # print(f"Before pitch_error[i]: {pitch_error[i]} ")

                roll_error[i] = gps_hold_roll_adjustment_calculation * scale_factor
                pitch_error[i] = gps_hold_pitch_adjustment_calculation * scale_factor

                # print(f"After roll_error[i]: {roll_error[i]} ")
                # print(f"After pitch_error[i]: {pitch_error[i]} ")
                # print("")

            else:

                gps_hold_roll_adjustment_calculation = roll_effect_on_lat[i] * lat_error + roll_effect_on_lon[i] * lon_error
                gps_hold_pitch_adjustment_calculation = pitch_effect_on_lat[i] * lat_error + pitch_effect_on_lon[i] * lon_error
                print("TESTING")
                print(lat_error)
                print(lon_error)
                print(roll_effect_on_lat[i])
                print(roll_effect_on_lon[i])
                print(pitch_effect_on_lat[i])
                print(pitch_effect_on_lon[i])
                print(gps_hold_pitch_adjustment_calculation)

                if gps_hold_roll_adjustment_calculation > 5.0: 
                    gps_hold_roll_adjustment_calculation = 5.0
                elif gps_hold_roll_adjustment_calculation < -5.0: 
                    gps_hold_roll_adjustment_calculation = -5.0

                if gps_hold_pitch_adjustment_calculation > 5.0: 
                    gps_hold_pitch_adjustment_calculation = 5.0
                elif gps_hold_pitch_adjustment_calculation < -5.0: 
                    gps_hold_pitch_adjustment_calculation = -5.0

                roll_error[i] = gps_hold_roll_adjustment_calculation
                pitch_error[i] = gps_hold_pitch_adjustment_calculation

            # print(f"Yaw: {yaw_degrees[i]}")
            # print(f"Original Roll Effect on Lat: {temp_roll_effect_on_lat}, Calculated: {roll_effect_on_lat[i]}")
            # print(f"Original Pitch Effect on Lat: {temp_pitch_effect_on_lat}, Calculated: {pitch_effect_on_lat[i]}")
            # print(f"Original Roll Effect on Lon: {temp_roll_effect_on_lon}, Calculated: {roll_effect_on_lon[i]}")
            # print(f"Original Pitch Effect on Lon: {temp_pitch_effect_on_lon}, Calculated: {pitch_effect_on_lon[i]}")
            # print("\n")

            # Length of the vector of lat and lon errrors
            # error_magnitude = math.sqrt(error_lat[i] * error_lat[i] + error_lon[i] * error_lon[i])

            # if error_magnitude == 0:
            #     normalized_error_latitude = 0
            #     normalized_error_longitude = 0
            # else:
            #     normalized_error_latitude = error_lat[i] / error_magnitude
            #     normalized_error_longitude = error_lon[i] / error_magnitude

            # gps_hold_roll_adjustment_calculation = (roll_effect_on_lat[i] * normalized_error_latitude +
            #                                         roll_effect_on_lon[i] * normalized_error_longitude)
            # gps_hold_pitch_adjustment_calculation = (pitch_effect_on_lat[i] * normalized_error_latitude +
            #                                         pitch_effect_on_lon[i] * normalized_error_longitude)

            # scale_factor = min(error_magnitude, 5.0)

            # roll_error[i] = gps_hold_roll_adjustment_calculation * scale_factor
            # pitch_error[i] = gps_hold_pitch_adjustment_calculation * scale_factor

    # Iterate over data and set roll_error and pitch_error to 0.0 if error_lat and error_lon are 0.0




    # for i in range(len(data)):
    #     if error_lat[i] == 0.0 and error_lon[i] == 0.0:
    #         roll_error[i] = 0.0
    #         pitch_error[i] = 0.0

    yaw_degrees = (yaw_degrees + 270) % 360


    # "<target lat>;<target lon>;<current lat>;<current lon>;<roll error>;<pitch error>;<roll degrees>;<pitch degrees>;<yaw degrees>;"



    if has_real_gps:
        bounding_box = [current_real_lon.min() - map_buffer, current_real_lon.max() + map_buffer, current_lat.min() - map_buffer, current_lat.max() + map_buffer]
    else:
        bounding_box = [current_lon.min() - map_buffer, current_lon.max() + map_buffer, current_lat.min() - map_buffer, current_lat.max() + map_buffer]

    print(bounding_box)

    # Lon delta 0.000685
    # lat delta 0.000544

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
    if has_real_gps:
        projected_points_target = np.array([tilemapbase.project(lon, lat) for lon, lat in zip(target_real_lon, target_lat)])
    else:
        projected_points_target = np.array([tilemapbase.project(lon, lat) for lon, lat in zip(target_lon, target_lat)])

    x_target = projected_points_target[:, 0]
    y_target = projected_points_target[:, 1]

    if has_real_gps:
        projected_points_current = np.array([tilemapbase.project(lon, lat) for lon, lat in zip(current_real_lon, current_lat)])
    else:
        projected_points_current = np.array([tilemapbase.project(lon, lat) for lon, lat in zip(current_lon, current_lat)])

    x_current = projected_points_current[:, 0]
    y_current = projected_points_current[:, 1]

    # Plot the points on the map
    # ax.scatter(x_target, y_target, color='red', s=5, label='Targets')

    scatter_current = ax.scatter([], [], color='blue', s=35, label='Current')
    scatter_target = ax.scatter([], [], color='red', s=35, label='Current')

    plot_current, = ax.plot([], [], color='blue', linewidth=1, label='Path')

    # Initialize the arrow to indicate yaw
    quiver_yaw = ax.quiver(x_current[0], y_current[0], np.cos(np.radians(yaw_degrees[0])) * 0.0001, np.sin(np.radians(yaw_degrees[0])) * 0.0001, scale=1, scale_units='xy', angles='xy', color='red')
    quiver_pitch = ax.quiver(x_current[0], y_current[0], np.cos(np.radians(yaw_degrees[0])) * 0.0000002, np.sin(np.radians(yaw_degrees[0])) * 0.0000002, scale=1, scale_units='xy', angles='xy', color='purple')
    quiver_roll = ax.quiver(x_current[0], y_current[0], np.cos(np.radians(yaw_degrees[0] + 90)) * 0.0000002, np.sin(np.radians(yaw_degrees[0] + 90)) * 0.0000002, scale=1, scale_units='xy', angles='xy', color='orange')

    # Add title and labels if needed
    ax.set_title("Map with Points Overlay")
    ax.set_xlabel("Longitude")
    ax.set_ylabel("Latitude")

    def animate(i):
        if i < len(x_target):
            dx = np.cos(np.radians(yaw_degrees[i])) * 0.0000002
            dy = np.sin(np.radians(yaw_degrees[i])) * 0.0000002
            quiver_yaw.set_offsets([x_current[i], y_current[i]])
            quiver_yaw.set_UVC(dx, dy)



            if(disable_roll_and_pitch_arrows):
                dx_pitch = np.cos(np.radians(yaw_degrees[i])) * 0.0000001 * 0.3
                dy_pitch = np.sin(np.radians(yaw_degrees[i])) * 0.0000001 * 0.3
            else:
                dx_pitch = np.cos(np.radians(yaw_degrees[i])) * 0.0000001 * pitch_error[i]
                dy_pitch = np.sin(np.radians(yaw_degrees[i])) * 0.0000001 * pitch_error[i]
            quiver_pitch.set_offsets([x_current[i], y_current[i]])
            quiver_pitch.set_UVC(dx_pitch, dy_pitch)


            if(disable_roll_and_pitch_arrows):
                dx_roll = np.cos(np.radians(yaw_degrees[i] + 90)) * 0.0000001 * 0.3
                dy_roll = np.sin(np.radians(yaw_degrees[i] + 90)) * 0.0000001 * 0.3
            else:
                dx_roll = np.cos(np.radians(yaw_degrees[i] + 90)) * 0.0000001 * roll_error[i]
                dy_roll = np.sin(np.radians(yaw_degrees[i] + 90)) * 0.0000001 * roll_error[i]
            quiver_roll.set_offsets([x_current[i], y_current[i]])
            quiver_roll.set_UVC(dx_roll, dy_roll)

            # Update the scatter plot data
            plot_current.set_data(x_current[:i+1], y_current[:i+1])
            scatter_current.set_offsets([x_current[i], y_current[i]])
            scatter_target.set_offsets([x_target[i], y_target[i]])

            if(i % 100 == 0):
                print(i)

            # Set the animation interval to match the desired frequency (521 Hz)
            # interval = int(1000 / 521)
            # ani.event_source.interval = interval

            # Return the updated objects
            return plot_current, scatter_current, scatter_target, quiver_yaw, quiver_pitch, quiver_roll

    if slow_animation:
        ani = FuncAnimation(fig, animate, frames=len(x_target), interval=100, blit=True)
        # ani = FuncAnimation(fig, animate, frames=len(x_target), interval=1000, blit=True)

    else:
        ani = FuncAnimation(fig, animate, frames=len(x_target), interval=1, blit=True)

    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Remove lines containing 'NAN' from a file.")
    parser.add_argument('file_path', type=str, help='Path to the file to be cleaned')

    args = parser.parse_args()
    do(args.file_path)

if __name__ == "__main__":
    main()