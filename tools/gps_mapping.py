import numpy as np
import pandas as pd
import geopandas as gpd
from scipy.stats import gaussian_kde
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.path import Path
from matplotlib.textpath import TextToPath
import tilemapbase
import warnings
import matplotlib.cbook
warnings.filterwarnings("ignore", category=matplotlib.cbook.mplDeprecation)

import os
import seaborn as sns
import shapely.speedups
shapely.speedups.enable()
from matplotlib.animation import FuncAnimation



data = np.genfromtxt('gps_targeting2.txt', delimiter=';')

target_lat = data[:, 0] 
target_lon = data[:, 1]
current_lat = data[:, 2]
current_lon = data[:, 3]
roll_error = data[:, 4]
pitch_error = data[:, 5]
roll_degrees = data[:, 6]
pitch_degrees = data[:, 7]
yaw_degrees = data[:, 8]

yaw_degrees = (yaw_degrees + 270) % 360

# "<target lat>;<target lon>;<current lat>;<current lon>;<roll error>;<pitch error>;<roll degrees>;<pitch degrees>;<yaw degrees>;"

map_buffer = 0.0003

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
plotter.plot(ax)

# Project and plot the data points
projected_points_target = np.array([tilemapbase.project(lon, lat) for lon, lat in zip(target_lon, target_lat)])
x_target = projected_points_target[:, 0]
y_target = projected_points_target[:, 1]

projected_points_current = np.array([tilemapbase.project(lon, lat) for lon, lat in zip(current_lon, current_lat)])
x_current = projected_points_current[:, 0]
y_current = projected_points_current[:, 1]



# Plot the points on the map
ax.scatter(x_target, y_target, color='red', s=5, label='Targets')
scatter_current = ax.scatter([], [], color='blue', s=35, label='Current')
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

        print(i)

        # Return the updated objects
        return plot_current, scatter_current, quiver_yaw, quiver_pitch, quiver_roll

ani = FuncAnimation(fig, animate, frames=len(x_target), interval=int(1000/1000), blit=True)

plt.show()














# import numpy as np
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation
# import cartopy.crs as ccrs
# import cartopy.feature as cfeature

# # Function to read the file and return data as a list of tuples
# def load_data_from_file(filename):
#     data_points = []
#     with open(filename, 'r') as file:
#         lines = file.readlines()
#         for line in lines:
#             line = line.strip()
#             if line:
#                 data = line.split(';')
#                 target_lat = float(data[0])
#                 target_lon = float(data[1])
#                 current_lat = float(data[2])
#                 current_lon = float(data[3])
#                 yaw = float(data[8])
#                 data_points.append((target_lat, target_lon, current_lat, current_lon, yaw))
#     return data_points

# # Load data at the start
# data_points = load_data_from_file('gps_targeting2.txt')

# # Set up the map projection and the figure
# fig = plt.figure()
# ax = plt.axes(projection=ccrs.PlateCarree())

# # Add features to the map (coastlines, borders, etc.)
# ax.add_feature(cfeature.COASTLINE)
# ax.add_feature(cfeature.BORDERS, linestyle=':')
# ax.add_feature(cfeature.LAND, facecolor='lightgray')
# ax.add_feature(cfeature.OCEAN, facecolor='lightblue')

# # Plot the target location
# target_point, = ax.plot([], [], 'go', markersize=10, transform=ccrs.PlateCarree())  # Green point for the target

# # Quadcopter location marker
# quadcopter_point, = ax.plot([], [], 'ro', markersize=10, transform=ccrs.PlateCarree())  # Red point for the quadcopter
# orientation_line, = ax.plot([], [], 'r-', lw=2, transform=ccrs.PlateCarree())  # Line to show orientation

# # Path line for the quadcopter movement
# path_line, = ax.plot([], [], 'b-', lw=1, transform=ccrs.PlateCarree())  # Blue line for the path

# # List to store the path coordinates
# path_coords = []

# def update(frame):
#     # Get the data for the current frame
#     target_lat, target_lon, current_lat, current_lon, yaw = data_points[frame]
    
#     # Update the target and quadcopter positions
#     target_point.set_data(target_lon, target_lat)
#     quadcopter_point.set_data(current_lon, current_lat)
    
#     # Add current position to path and update path line
#     path_coords.append((current_lon, current_lat))
#     path_lon, path_lat = zip(*path_coords)
#     path_line.set_data(path_lon, path_lat)
    
#     # Calculate orientation line based on yaw
#     yaw_rad = np.deg2rad(yaw)
#     orientation_x = [current_lon, current_lon + 0.0005 * np.cos(yaw_rad)]
#     orientation_y = [current_lat, current_lat + 0.0005 * np.sin(yaw_rad)]
#     orientation_line.set_data(orientation_x, orientation_y)
    
#     # Set the extent (viewing area) around the current position
#     extent = [current_lon - 0.001, current_lon + 0.001, current_lat - 0.001, current_lat + 0.001]
#     ax.set_extent(extent, crs=ccrs.PlateCarree())
    
#     return target_point, quadcopter_point, orientation_line, path_line

# # Animation function, iterating over the data points
# ani = animation.FuncAnimation(fig, update, frames=len(data_points), interval=20, blit=True)

# # Show the animation
# plt.show()