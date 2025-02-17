# import numpy as np
# import pandas as pd
# import geopandas as gpd
# from scipy.stats import gaussian_kde
# import matplotlib.pyplot as plt
# from matplotlib.font_manager import FontProperties
# from matplotlib.path import Path
# from matplotlib.textpath import TextToPath
# import tilemapbase
# import warnings
# import matplotlib.cbook
# warnings.filterwarnings("ignore", category=matplotlib.cbook.mplDeprecation)

# import os
# import seaborn as sns
# import shapely.speedups
# shapely.speedups.enable()
# from matplotlib.animation import FuncAnimation
# from matplotlib.widgets import Slider, Button

# # I fucked up gps logging so this script fixes that 
# # data = np.genfromtxt('../misc/gps_logs/gps_targeting7_big.txt', delimiter=';')
# # data = np.genfromtxt('../misc/gps_logs/gps_targeting7.txt', delimiter=';')
# # data = np.genfromtxt('../misc/gps_logs/gps_targeting8.txt', delimiter=';')
# # data = np.genfromtxt('../misc/gps_logs/gps_targeting9.txt', delimiter=';')
# # data = np.genfromtxt('../misc/gps_logs/gps_targeting13.txt', delimiter=';')

# # data = np.genfromtxt('../misc/gps_logs/gps_targeting15.txt', delimiter=';')
# # data = np.genfromtxt('../misc/gps_logs/gps_targeting16.txt', delimiter=';')
# # data = np.genfromtxt('../misc/gps_logs/gps_targeting17.txt', delimiter=';')

# # data = np.genfromtxt('../misc/gps_logs/gps_targeting18.txt', delimiter=';')
# # data = np.genfromtxt('../misc/gps_logs/gps_targeting19.txt', delimiter=';')
# # data = np.genfromtxt('../misc/gps_logs/gps_targeting20.txt', delimiter=';')
# data = np.genfromtxt('../misc/gps_logs/gps_targeting24.txt', delimiter=';')


# skip_lines_amount = 0
# # skip_lines_amount = 1400

# # Skip the first 100 lines of data
# data = data[skip_lines_amount:]


# # Skip rows with 0 in the first column until a real value shows up
# # first_non_zero_index = np.argmax(data[:, 0] != 0)
# # data = data[first_non_zero_index:]

# # Reverse the linearization of longitude
# # M_PI_DIV_BY_180 = 0.0174533
# # data[:, 1] = data[:, 1] / np.cos(data[:, 0] * M_PI_DIV_BY_180)
# # data[:, 3] = data[:, 3] / np.cos(data[:, 2] * M_PI_DIV_BY_180)


# target_lat = data[:, 0]
# target_lon = data[:, 1]
# current_lat = data[:, 2]
# current_lon = data[:, 3]
# roll_error = data[:, 4]
# pitch_error = data[:, 5]
# roll_degrees = data[:, 6]
# pitch_degrees = data[:, 7]
# yaw_degrees = data[:, 8]
# sats = data[:, 9]

# # Remove rows where roll_degrees or pitch_degrees are 0.0
# valid_indices = (roll_degrees != 0.0) & (pitch_degrees != 0.0)
# target_lat = target_lat[valid_indices]
# target_lon = target_lon[valid_indices]
# current_lat = current_lat[valid_indices]
# current_lon = current_lon[valid_indices]
# roll_error = roll_error[valid_indices]
# pitch_error = pitch_error[valid_indices]
# roll_degrees = roll_degrees[valid_indices]
# pitch_degrees = pitch_degrees[valid_indices]
# yaw_degrees = yaw_degrees[valid_indices]
# sats = sats[valid_indices]

# # skip = 52  # 521Hz / 10Hz ≈ 52
# # target_lat = target_lat[::skip]
# # target_lon = target_lon[::skip]
# # current_lat = current_lat[::skip]
# # current_lon = current_lon[::skip]
# # roll_error = roll_error[::skip]
# # pitch_error = pitch_error[::skip]
# # yaw_degrees = yaw_degrees[::skip]

# yaw_degrees = (yaw_degrees + 270) % 360

# # "<target lat>;<target lon>;<current lat>;<current lon>;<roll error>;<pitch error>;<roll degrees>;<pitch degrees>;<yaw degrees>;"

# # map_buffer = 0.003
# map_buffer = 0.0003


# bounding_box = [current_lon.min() - map_buffer, current_lon.max() + map_buffer, current_lat.min() - map_buffer, current_lat.max() + map_buffer]
# print(bounding_box)

# # Lon delta 0.000685
# # lat delta 0.000544

# # Initialize tilemapbase
# tilemapbase.init(create=True)
# extent = tilemapbase.Extent.from_lonlat(bounding_box[0], bounding_box[1], bounding_box[2], bounding_box[3])  # Example coordinates (London area)
# extent = extent.to_aspect(1.0)
# tiles = tilemapbase.tiles.build_OSM()


# fig, ax = plt.subplots(figsize=(10, 10))

# # Add the background map
# plotter = tilemapbase.Plotter(extent, tiles, zoom=18)
# plotter.plot(ax)

# # Project and plot the data points
# projected_points_target = np.array([tilemapbase.project(lon, lat) for lon, lat in zip(target_lon, target_lat)])
# x_target = projected_points_target[:, 0]
# y_target = projected_points_target[:, 1]

# projected_points_current = np.array([tilemapbase.project(lon, lat) for lon, lat in zip(current_lon, current_lat)])
# x_current = projected_points_current[:, 0]
# y_current = projected_points_current[:, 1]


# # scatter_target = ax.scatter([], [], color='red', s=5, label='Targets')
# # Plot the points on the map
# # ax.scatter(x_target, y_target, color='red', s=5, label='Targets')
# scatter_current = ax.scatter([], [], color='blue', s=35, label='Current')
# plot_current, = ax.plot([], [], color='blue', linewidth=1, label='Path')


# # Initialize the arrow to indicate yaw
# quiver_yaw = ax.quiver(x_current[0], y_current[0], np.cos(np.radians(yaw_degrees[0])) * 0.0001, np.sin(np.radians(yaw_degrees[0])) * 0.0001, scale=1, scale_units='xy', angles='xy', color='red')
# quiver_pitch = ax.quiver(x_current[0], y_current[0], np.cos(np.radians(yaw_degrees[0])) * 0.0000002, np.sin(np.radians(yaw_degrees[0])) * 0.0000002, scale=1, scale_units='xy', angles='xy', color='purple')
# quiver_roll = ax.quiver(x_current[0], y_current[0], np.cos(np.radians(yaw_degrees[0] + 90)) * 0.0000002, np.sin(np.radians(yaw_degrees[0] + 90)) * 0.0000002, scale=1, scale_units='xy', angles='xy', color='orange')


# # Add title and labels if needed
# ax.set_title("Map with Points Overlay")
# ax.set_xlabel("Longitude")
# ax.set_ylabel("Latitude")

# last_satalite_count = 0

# scatter_target = ax.scatter([], [], color='red', s=35, label='Targets')

# def animate(i):
#     global last_satalite_count  # Add this line

#     if i < len(x_target):
#         dx = np.cos(np.radians(yaw_degrees[i])) * 0.0000002
#         dy = np.sin(np.radians(yaw_degrees[i])) * 0.0000002
#         quiver_yaw.set_offsets([x_current[i], y_current[i]])
#         quiver_yaw.set_UVC(dx, dy)


#         dx_pitch = np.cos(np.radians(yaw_degrees[i])) * 0.0000001 * pitch_error[i]
#         dy_pitch = np.sin(np.radians(yaw_degrees[i])) * 0.0000001 * pitch_error[i]
#         quiver_pitch.set_offsets([x_current[i], y_current[i]])
#         quiver_pitch.set_UVC(dx_pitch, dy_pitch)


#         dx_roll = np.cos(np.radians(yaw_degrees[i] + 90)) * 0.0000001 * roll_error[i]
#         dy_roll = np.sin(np.radians(yaw_degrees[i] + 90)) * 0.0000001 * roll_error[i]
#         quiver_roll.set_offsets([x_current[i], y_current[i]])
#         quiver_roll.set_UVC(dx_roll, dy_roll)


#         # Update the scatter plot data
#         plot_current.set_data(x_current[:i+1], y_current[:i+1])
#         scatter_current.set_offsets([x_current[i], y_current[i]])
        
#         # Gradually reveal target points
#         scatter_target.set_offsets([[x_target[i], y_target[i]]])

#         if(last_satalite_count != sats[i]):
#             print(f"Satelite count: {sats[i]}")
#             last_satalite_count = sats[i]
        
#         # if(i % 100 == 0):
#         #     print(i)


#         # Set the animation interval to match the desired frequency (521 Hz)
#         # interval = int(1000 / 521)
#         # ani.event_source.interval = interval

#         # Return the updated objects
#         return plot_current, scatter_current, scatter_target, quiver_yaw, quiver_pitch, quiver_roll


# # Add a slider for the timeline
# ax_slider = plt.axes([0.1, 0.01, 0.65, 0.03], facecolor='lightgoldenrodyellow')
# slider = Slider(ax_slider, 'Frame', 0, len(x_target) - 1, valinit=0, valstep=1)

# # Add a pause button
# ax_button_pause = plt.axes([0.8, 0.01, 0.1, 0.04])
# button_pause = Button(ax_button_pause, 'Pause', color='lightgoldenrodyellow', hovercolor='0.975')

# # Add a continue button
# ax_button_continue = plt.axes([0.9, 0.01, 0.1, 0.04])
# button_continue = Button(ax_button_continue, 'Continue', color='lightgoldenrodyellow', hovercolor='0.975')

# # Variable to control the animation state
# is_paused = False

# def update(val):
#     frame = int(slider.val)
#     animate(frame)

# def pause(event):
#     global is_paused
#     is_paused = True

# def continue_(event):
#     global is_paused
#     is_paused = False

# slider.on_changed(update)
# button_pause.on_clicked(pause)
# button_continue.on_clicked(continue_)

# def animation_func(i):
#     if not is_paused:
#         slider.set_val(i)
#         return animate(i)
#     return []

# ani = FuncAnimation(fig, animation_func, frames=len(x_target), interval=20, blit=True)
# ax_slider = plt.axes([0.1, 0.01, 0.65, 0.03], facecolor='lightgoldenrodyellow')
# slider = Slider(ax_slider, 'Frame', 0, len(x_target) - 1, valinit=0, valstep=1)

# # Add a pause button
# ax_button_pause = plt.axes([0.8, 0.01, 0.1, 0.04])
# button_pause = Button(ax_button_pause, 'Pause', color='lightgoldenrodyellow', hovercolor='0.975')

# # Add a continue button
# ax_button_continue = plt.axes([0.9, 0.01, 0.1, 0.04])
# button_continue = Button(ax_button_continue, 'Continue', color='lightgoldenrodyellow', hovercolor='0.975')

# # Variable to control the animation state
# is_paused = False

# def update(val):
#     frame = int(slider.val)
#     animate(frame)

# def pause(event):
#     global is_paused
#     is_paused = True

# def continue_(event):
#     global is_paused
#     is_paused = False

# slider.on_changed(update)
# button_pause.on_clicked(pause)
# button_continue.on_clicked(continue_)

# def animation_func(i):
#     if not is_paused:
#         slider.set_val(i)
#         return animate(i)
#     return []

# ani = FuncAnimation(fig, animation_func, frames=len(x_target), interval=20, blit=True)

# # ani = FuncAnimation(fig, animate, frames=len(x_target), interval=20, blit=True)

# plt.show()

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider, Button
import tilemapbase

import numpy as np
import time

class PID:
    def __init__(self, kp, ki, kd, desired_value, max_value, min_value, stop_windup_integral=False, stop_windup_derivative=False):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.desired_value = desired_value
        self.integral_sum = 0
        self.last_error = 0
        self.last_time = 0  # Initialize with 0 for simulation
        self.max_value = max_value
        self.min_value = min_value
        self.stop_windup_integral = stop_windup_integral
        self.stop_windup_derivative = stop_windup_derivative

    def calculate_error(self, current_value, current_time):
        elapsed_time = current_time - self.last_time

        error = self.desired_value - current_value

        # Proportional term
        p_term = error

        # Integral term
        self.integral_sum += error * elapsed_time
        if self.stop_windup_integral:
            if self.integral_sum * self.ki > self.max_value:
                self.integral_sum = self.max_value / self.ki
            elif self.integral_sum * self.ki < self.min_value:
                self.integral_sum = self.min_value / self.ki
        i_term = self.integral_sum

        # Derivative term
        d_term = (error - self.last_error) / elapsed_time if elapsed_time != 0 else 0
        if self.stop_windup_derivative:
            if d_term * self.kd > self.max_value:
                d_term = self.max_value
            elif d_term * self.kd < self.min_value:
                d_term = self.min_value

        self.last_error = error
        self.last_time = current_time

        # Total error
        total_error = (self.kp * p_term) + (self.ki * i_term) + (self.kd * d_term)
        return total_error

    def set_desired_value(self, desired_value):
        self.desired_value = desired_value
# -----------------------------
# 1. Load & Prepare Your Data
# -----------------------------
data = np.genfromtxt('../misc/gps_logs/gps_targeting26.txt', delimiter=';')

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

yaw_degrees = (yaw_degrees + 270) % 360



# Define PID parameters
gps_hold_outer_master_gain = 1.0
gps_hold_outer_gain_p = 100000.0
gps_hold_outer_gain_i = 0.0
gps_hold_outer_gain_d = 0.0

# Initialize PID controllers
gps_latitude_outer_pid = PID(
    kp=gps_hold_outer_master_gain * gps_hold_outer_gain_p,
    ki=gps_hold_outer_master_gain * gps_hold_outer_gain_i,
    kd=gps_hold_outer_master_gain * gps_hold_outer_gain_d,
    desired_value=0.0,
    max_value=0,
    min_value=0
)

gps_longitude_outer_pid = PID(
    kp=gps_hold_outer_master_gain * gps_hold_outer_gain_p,
    ki=gps_hold_outer_master_gain * gps_hold_outer_gain_i,
    kd=gps_hold_outer_master_gain * gps_hold_outer_gain_d,
    desired_value=0.0,
    max_value=0,
    min_value=0
)


# Calculate time increment in microseconds
refresh_rate = 521  # Hz
time_increment_us = int(1e6 / refresh_rate)  # Convert seconds to microseconds

# Initialize time variable
current_time_us = 0
# Iterate over data and calculate errors
for i in range(len(data)):
    # Set desired values
    gps_latitude_outer_pid.set_desired_value(target_lat[i])
    gps_longitude_outer_pid.set_desired_value(target_lon[i])

    # Calculate errors
    error_lat[i] = gps_latitude_outer_pid.calculate_error(current_lat[i], current_time_us)
    error_lon[i] = gps_longitude_outer_pid.calculate_error(current_lon[i], current_time_us)

    # Increment time
    current_time_us += time_increment_us

start_index = 0 # Default


# The LON adjustments will try to go left or right on the map, THEY DONT KNOW IF THE DRONE IS FACING RIGHT AHEAD
# It tries to make the shortest path to reach that longitude and going straight to the left or right is the correct way

# Testing lon pitch forward GOOD
# start_index = 1668 # Facing forwards towards +lon

# Testing lon pitch back GOOD
# start_index = 1584 # Facing backwards towards +lon

# Testing lon roll right GOOD
# start_index = 1792 # Facing north with -lon  on its right

# Testing lon roll left GOOD
# start_index = 1622 # Facing south with +lon on its left

# GOOD
# roll_error = pitch_effect_on_lon * error_lon
# pitch_error = roll_effect_on_lon * error_lon




# The goal of this one is to reach the target lat nothing else, it just wants to go up or down

# Testing lat pitch forward GOOD
# start_index = 1792 # Facing north with front towards towards +lat

# Testing lat pitch back GOOD
# start_index = 1967 # Facing north with back towards -lat

# Testing lat roll right 
# start_index = 2122 # Facing west with right towards +lat

# Testing lat roll left GOOD
# start_index = 1668 # Facing east with left towards +lat

# GOOD
# roll_error = -pitch_effect_on_lat * error_lat
# pitch_error = -roll_effect_on_lat * error_lat



# The goal is to reach the target point not only lat or lon individually
# Testing lat lon pitch forward
start_index = 1799 # Facing north towards taget 

# Testing lat lon pitch back 
# start_index = 1585 # Facing west with target right behind

# Testing lat lon roll right
# start_index = 1891 # Facing south with target to the left of it

# Testing lat lon roll left
# start_index = 1605 # Facing south with target to the left of it

roll_error = -pitch_effect_on_lat * error_lat + pitch_effect_on_lon * error_lon
pitch_error = -roll_effect_on_lat * error_lat + roll_effect_on_lon * error_lon




# Ensure the calculations are correct

# roll_error = roll_effect_on_lat * error_lat
# roll_error = roll_effect_on_lon * error_lon

# pitch_error = pitch_effect_on_lat * error_lat
# pitch_error = pitch_effect_on_lon * error_lon

# roll_error = 0.0
# pitch_error = 0.0

# Calculate roll_error and pitch_error


# roll_error = roll_effect_on_lat * error_lat
# roll_error = pitch_effect_on_lon * error_lon

# pitch_error = pitch_effect_on_lat * error_lat
# pitch_error = roll_effect_on_lon * error_lon

# Limit the values of roll_error and pitch_error
roll_error = np.clip(roll_error, -5.0, 5.0)
pitch_error = np.clip(pitch_error, -5.0, 5.0)

map_buffer = 0.0003
bounding_box = [
    current_lon.min() - map_buffer, 
    current_lon.max() + map_buffer, 
    current_lat.min() - map_buffer, 
    current_lat.max() + map_buffer
]

tilemapbase.init(create=True)
extent = tilemapbase.Extent.from_lonlat(
    bounding_box[0], bounding_box[1],
    bounding_box[2], bounding_box[3]
).to_aspect(1.0)
tiles = tilemapbase.tiles.build_OSM()

# --------------------------------
# 2. Create the Figure & Axes
# --------------------------------
fig, ax = plt.subplots(figsize=(10, 10))
plotter = tilemapbase.Plotter(extent, tiles, zoom=18)
plotter.plot(ax)

# 2D projections
proj_target = np.array([
    tilemapbase.project(lon, lat) 
    for lon, lat in zip(target_lon, target_lat)
])
x_target, y_target = proj_target[:, 0], proj_target[:, 1]

proj_current = np.array([
    tilemapbase.project(lon, lat) 
    for lon, lat in zip(current_lon, current_lat)
])
x_current, y_current = proj_current[:, 0], proj_current[:, 1]

# ---------------------------------------
# 3. Artists (Scatter, Plot, Quivers...)
# ---------------------------------------
plot_current, = ax.plot([], [], color='blue', linewidth=1, label='Path')
scatter_current = ax.scatter([], [], color='blue', s=35, label='Current')
scatter_target = ax.scatter([], [], color='red',  s=35, label='Targets')

# Initialize quivers with dummy offsets
quiver_yaw = ax.quiver(x_current[0], y_current[0], 0, 0, 
                       scale=1, scale_units='xy', angles='xy', color='red')
quiver_pitch = ax.quiver(x_current[0], y_current[0], 0, 0, 
                         scale=1, scale_units='xy', angles='xy', color='purple')
quiver_roll = ax.quiver(x_current[0], y_current[0], 0, 0, 
                        scale=1, scale_units='xy', angles='xy', color='orange')

ax.set_title("Map with Points Overlay")
ax.set_xlabel("Longitude")
ax.set_ylabel("Latitude")
ax.legend()

# -----------------------------------
# 4. Global Control Variables
# -----------------------------------
current_frame = 0  # will track current frame index
is_paused = False
last_satalite_count = None








# Clamp it if needed
start_index = min(start_index, len(x_current) - 1)

# -------------------------------------------------
# 5. Update Plot Function (like your animate())
# -------------------------------------------------
def update_plot(frame):
    """Update all artists for the given 'frame' index."""
    global last_satalite_count

    # Safety clamp
    frame = min(frame, len(x_current) - 1)

    # Quiver updates
    dx_yaw = np.cos(np.radians(yaw_degrees[frame])) * 0.0000002
    dy_yaw = np.sin(np.radians(yaw_degrees[frame])) * 0.0000002
    quiver_yaw.set_offsets([x_current[frame], y_current[frame]])
    quiver_yaw.set_UVC(dx_yaw, dy_yaw)

    dx_pitch = np.cos(np.radians(yaw_degrees[frame])) * 0.0000001 * pitch_error[frame]
    dy_pitch = np.sin(np.radians(yaw_degrees[frame])) * 0.0000001 * pitch_error[frame]
    quiver_pitch.set_offsets([x_current[frame], y_current[frame]])
    quiver_pitch.set_UVC(dx_pitch, dy_pitch)

    dx_roll = np.cos(np.radians(yaw_degrees[frame] + 90)) * 0.0000001 * roll_error[frame]
    dy_roll = np.sin(np.radians(yaw_degrees[frame] + 90)) * 0.0000001 * roll_error[frame]
    quiver_roll.set_offsets([x_current[frame], y_current[frame]])
    quiver_roll.set_UVC(dx_roll, dy_roll)

    # Scatter & path updates
    plot_current.set_data(x_current[:frame+1], y_current[:frame+1])
    scatter_current.set_offsets([x_current[frame], y_current[frame]])
    scatter_target.set_offsets([x_target[frame], y_target[frame]])

    # Satellite count check
    if last_satalite_count != sats[frame]:
        print(f"Satellite count: {sats[frame]}")
        last_satalite_count = sats[frame]

    # Return artists for blitting
    return (
        plot_current, scatter_current, scatter_target,
        quiver_yaw, quiver_pitch, quiver_roll
    )

# --------------------------------------------------
# 6. Slider and Buttons
# --------------------------------------------------
ax_slider = plt.axes([0.1, 0.01, 0.65, 0.03], facecolor='lightgoldenrodyellow')
slider = Slider(ax_slider, 'Frame', 0, len(x_current) - 1, valinit=0, valstep=1)

ax_button_pause = plt.axes([0.8, 0.01, 0.07, 0.04])
button_pause = Button(ax_button_pause, 'Pause', color='lightgoldenrodyellow', hovercolor='0.975')

ax_button_resume = plt.axes([0.88, 0.01, 0.1, 0.04])
button_resume = Button(ax_button_resume, 'Continue', color='lightgoldenrodyellow', hovercolor='0.975')

def on_slider_change(val):
    """Handle slider moves (user drags timeline)."""
    global current_frame
    current_frame = int(val)
    update_plot(current_frame)

def pause(event):
    """Pause animation."""
    global is_paused
    is_paused = True

def resume(event):
    """Resume animation from current frame."""
    global is_paused
    is_paused = False

slider.on_changed(on_slider_change)
button_pause.on_clicked(pause)
button_resume.on_clicked(resume)

# --------------------------------------------
# 7. Main Animation Function
# --------------------------------------------
def animation_func(i):
    """This is called by FuncAnimation for each frame i."""
    global current_frame
    if not is_paused:
        # Advance the frame index by 1
        current_frame += 1
        # Loop or clamp the frame index
        if current_frame >= len(x_current):
            current_frame = 0  # or current_frame = len(x_current)-1 to freeze at end
        # Update slider to reflect new frame
        slider.set_val(current_frame)
        return update_plot(current_frame)
    else:
        # While paused, *do not* return empty list. Return the last state
        return (
            plot_current, scatter_current, scatter_target,
            quiver_yaw, quiver_pitch, quiver_roll
        )

# --------------------------------------------
# 8. Initialize & Start the Animation
# --------------------------------------------
# 1) Set the frame to the desired start_index
current_frame = start_index

# 2) Optionally start paused at that frame
is_paused = True  # comment out if you want it to play immediately

# 3) Update slider & plot to show initial frame
slider.set_val(current_frame)  
update_plot(current_frame)

# Create the FuncAnimation
ani = FuncAnimation(
    fig, 
    animation_func, 
    frames=len(x_current),
    interval=20, 
    blit=True, 
    repeat=False
)

plt.show()