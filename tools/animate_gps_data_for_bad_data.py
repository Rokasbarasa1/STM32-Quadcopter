import math
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
data = np.genfromtxt('../misc/gps_logs/gps_targeting31.txt', delimiter=';', usecols=range(16))

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




# Define PID parameters
gps_hold_outer_master_gain = 1000.0
gps_hold_outer_gain_p = 100.0
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

    yaw_radians = yaw_degrees[i] * 0.0174533 

    temp_roll_effect_on_lat = roll_effect_on_lat[i]
    temp_pitch_effect_on_lat = pitch_effect_on_lat[i]
    temp_roll_effect_on_lon = roll_effect_on_lon[i]
    temp_pitch_effect_on_lon = pitch_effect_on_lon[i]

    roll_effect_on_lat[i] = -math.sin(yaw_radians)#  * latitude_sign;  // + GOOD, - GOOD  // If moving north roll right positive, roll left south negative
    pitch_effect_on_lat[i] = math.cos(yaw_radians)# * latitude_sign;  // + GOOD, - GOOD  // If moving north forward positive, backwards negative
    roll_effect_on_lon[i] = math.cos(yaw_radians)# * longitude_sign;  // + GOOD, - GOOD
    pitch_effect_on_lon[i] = math.sin(yaw_radians)# * longitude_sign; // + GOOD, - GOOD 

    print(f"Yaw: {yaw_degrees[i]}")
    print(f"Original Roll Effect on Lat: {temp_roll_effect_on_lat}, Calculated: {roll_effect_on_lat[i]}")
    print(f"Original Pitch Effect on Lat: {temp_pitch_effect_on_lat}, Calculated: {pitch_effect_on_lat[i]}")
    print(f"Original Roll Effect on Lon: {temp_roll_effect_on_lon}, Calculated: {roll_effect_on_lon[i]}")
    print(f"Original Pitch Effect on Lon: {temp_pitch_effect_on_lon}, Calculated: {pitch_effect_on_lon[i]}")
    print("\n")

    # Length of the vector of lat and lon errrors
    error_magnitude = math.sqrt(error_lat[i] * error_lat[i] + error_lon[i] * error_lon[i])

    if error_magnitude == 0:
        normalized_error_latitude = 0
        normalized_error_longitude = 0
    else:
        normalized_error_latitude = error_lat[i] / error_magnitude
        normalized_error_longitude = error_lon[i] / error_magnitude

    gps_hold_roll_adjustment_calculation = (roll_effect_on_lat[i] * normalized_error_latitude +
                                            roll_effect_on_lon[i] * normalized_error_longitude)
    gps_hold_pitch_adjustment_calculation = (pitch_effect_on_lat[i] * normalized_error_latitude +
                                             pitch_effect_on_lon[i] * normalized_error_longitude)

    scale_factor = min(error_magnitude, 5.0)

    roll_error[i] = gps_hold_roll_adjustment_calculation * scale_factor
    pitch_error[i] = gps_hold_pitch_adjustment_calculation * scale_factor


    # Increment time
    current_time_us += time_increment_us

yaw_degrees = (yaw_degrees + 270) % 360

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
# start_index = 1799 # Facing north towards taget 

# Testing lat lon pitch back 
# start_index = 1585 # Facing west with target right behind

# Testing lat lon roll right
# start_index = 1891 # Facing south with target to the left of it

# Testing lat lon roll left
# start_index = 1605 # Facing south with target to the left of it

# roll_error = -pitch_effect_on_lat * error_lat + pitch_effect_on_lon * error_lon
# pitch_error = -roll_effect_on_lat * error_lat + roll_effect_on_lon * error_lon




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

map_buffer = 0.0007
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