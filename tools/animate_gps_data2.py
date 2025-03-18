import numpy as np
# import pandas as pd
# import geopandas as gpd
# from scipy.stats import gaussian_kde
import matplotlib.pyplot as plt
# from matplotlib.font_manager import FontProperties
# from matplotlib.path import Path
# from matplotlib.textpath import TextToPath
import tilemapbase
import warnings
import matplotlib.cbook
warnings.filterwarnings("ignore", category=matplotlib.cbook.mplDeprecation)
# import os
# import seaborn as sns
# import shapely.speedups
# shapely.speedups.enable()
from matplotlib.animation import FuncAnimation

import math

# I fucked up gps logging so this script fixes that 
data = np.genfromtxt('../misc/gps_logs/gps_targeting33.txt', delimiter=';', usecols=range(16))

# Skip rows with 0 in the first column until a real value shows up
first_non_zero_index = np.argmax(data[:, 0] != 0)
data = data[first_non_zero_index:]


target_lat = data[:, 0]
target_lon = data[:, 1]
current_lat = data[:, 2]
current_lon = data[:, 3]
roll_error = data[:, 4]
pitch_error = data[:, 5]
roll_degrees = data[:, 6]
pitch_degrees = data[:, 7]
yaw_degrees = data[:, 8]
sats = data[:, 9]
roll_effect_on_lat = data[:, 10]
pitch_effect_on_lat = data[:, 11]
roll_effect_on_lon = data[:, 12]
pitch_effect_on_lon = data[:, 13]
error_lat = data[:, 14]
error_lon = data[:, 15]

# skip = 1  # 521Hz / 10Hz ≈ 52
# target_lat = target_lat[::skip]
# target_lon = target_lon[::skip]
# current_lat = current_lat[::skip]
# current_lon = current_lon[::skip]
# roll_error = roll_error[::skip]
# pitch_error = pitch_error[::skip]
# yaw_degrees = yaw_degrees[::skip]


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


# Iterate over data and calculate errors
# for i in range(len(data)):
#     yaw_radians = yaw_degrees[i] * 0.0174533 
#     # roll_effect_on_lat[i] = -math.sin(yaw_degrees[i] * 0.0174533)
#     # roll_effect_on_lon[i] = math.cos(yaw_degrees[i] * 0.0174533)
#     # pitch_effect_on_lat[i] = math.cos(yaw_degrees[i] * 0.0174533)
#     # pitch_effect_on_lon[i] = math.sin(yaw_degrees[i] * 0.0174533)

#     temp_pitch_effect_on_lat = pitch_effect_on_lat[i]
#     temp_roll_effect_on_lat = roll_effect_on_lat[i]
#     temp_pitch_effect_on_lon = pitch_effect_on_lon[i]
#     temp_roll_effect_on_lon = roll_effect_on_lon[i]

#     # print(roll_effect_on_lat[i])

#     # pitch_effect_on_lat[i] = triangle_cos(yaw_radians)# * latitude_sign;  // + GOOD, - GOOD  // If moving north forward positive, backwards negative
#     # roll_effect_on_lat[i] = -triangle_sin(yaw_radians)#  * latitude_sign;  // + GOOD, - GOOD  // If moving north roll right positive, roll left south negative
#     # pitch_effect_on_lon[i] = triangle_sin(yaw_radians)# * longitude_sign; // + GOOD, - GOOD 
#     # roll_effect_on_lon[i] = triangle_cos(yaw_radians)# * longitude_sign;  // + GOOD, - GOOD

#     pitch_effect_on_lat[i] = math.cos(yaw_radians)# * latitude_sign;  // + GOOD, - GOOD  // If moving north forward positive, backwards negative
#     roll_effect_on_lat[i] = -math.sin(yaw_radians)#  * latitude_sign;  // + GOOD, - GOOD  // If moving north roll right positive, roll left south negative
#     pitch_effect_on_lon[i] = math.sin(yaw_radians)# * longitude_sign; // + GOOD, - GOOD 
#     roll_effect_on_lon[i] = math.cos(yaw_radians)# * longitude_sign;  // + GOOD, - GOOD


#     print(f"Yaw: {yaw_degrees[i]}")
#     print(f"Original Roll Effect on Lat: {temp_pitch_effect_on_lat}, Calculated: {pitch_effect_on_lat[i]}")
#     print(f"Original Pitch Effect on Lat: {temp_roll_effect_on_lat}, Calculated: {roll_effect_on_lat[i]}")
#     print(f"Original Roll Effect on Lon: {temp_pitch_effect_on_lon}, Calculated: {pitch_effect_on_lon[i]}")
#     print(f"Original Pitch Effect on Lon: {temp_roll_effect_on_lon}, Calculated: {roll_effect_on_lon[i]}")
#     print("\n")

#     # Length of the vector of lat and lon errrors
#     error_magnitude = math.sqrt(error_lat[i] * error_lat[i] + error_lon[i] * error_lon[i])

#     if error_magnitude == 0:
#         normalized_error_latitude = 0
#         normalized_error_longitude = 0
#     else:
#         normalized_error_latitude = error_lat[i] / error_magnitude
#         normalized_error_longitude = error_lon[i] / error_magnitude

#     gps_hold_roll_adjustment_calculation = (roll_effect_on_lat[i] * normalized_error_latitude +
#                                             roll_effect_on_lon[i] * normalized_error_longitude)
#     gps_hold_pitch_adjustment_calculation = (pitch_effect_on_lat[i] * normalized_error_latitude +
#                                              pitch_effect_on_lon[i] * normalized_error_longitude)

#     scale_factor = min(error_magnitude, 5.0)

#     roll_error[i] = gps_hold_roll_adjustment_calculation * scale_factor
#     pitch_error[i] = gps_hold_pitch_adjustment_calculation * scale_factor


yaw_degrees = (yaw_degrees + 270) % 360


# "<target lat>;<target lon>;<current lat>;<current lon>;<roll error>;<pitch error>;<roll degrees>;<pitch degrees>;<yaw degrees>;"

map_buffer = 0.0006
# map_buffer = 0.003

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
projected_points_target = np.array([tilemapbase.project(lon, lat) for lon, lat in zip(target_lon, target_lat)])
x_target = projected_points_target[:, 0]
y_target = projected_points_target[:, 1]

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


        dx_pitch = np.cos(np.radians(yaw_degrees[i])) * 0.0000001 * pitch_error[i]
        dy_pitch = np.sin(np.radians(yaw_degrees[i])) * 0.0000001 * pitch_error[i]
        quiver_pitch.set_offsets([x_current[i], y_current[i]])
        quiver_pitch.set_UVC(dx_pitch, dy_pitch)


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

ani = FuncAnimation(fig, animate, frames=len(x_target), interval=1, blit=True)

plt.show()