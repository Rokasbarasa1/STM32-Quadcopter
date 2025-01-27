import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# Lists to store the first two columns
altitude_barometer = []
vertical_acceleration_old = []
vertical_acceleration = []
altitude_sensor_fusion = []
vertical_velocity = []

# file_path = "../misc/testing_vertical_stabilization/5long_test.txt"
# file_path = "../misc/testing_vertical_stabilization/6long_test.txt"
file_path = "../misc/testing_vertical_stabilization/7long_test.txt"

# file_path = "../misc/testing_vertical_stabilization/3short_test.txt"

# Read the file and parse the first two columns
with open(file_path, 'r') as file:
    for line in file:
        # Split the line into columns based on the semicolon delimiter
        columns = line.strip().split(';')
        
        # Ensure there are enough columns to avoid errors
        if len(columns) > 1:
            try:
                # Append the first two columns (convert to floats)
                altitude_barometer.append(float(columns[0]))
                vertical_acceleration_old.append(float(columns[1]))
                vertical_acceleration.append(float(columns[2]))
                altitude_sensor_fusion.append(float(columns[3]))
                # vertical_velocity.append(float(columns[4]))

            except ValueError:
                # Handle lines that cannot be converted to float
                print(f"Skipping line due to conversion error: {line.strip()}")

# Display the loaded data
# print("Altitudes:", altitudes)
# print("Vertical Speeds:", vertical_speeds)

# df = pd.read_csv('./data_examples/24Quadcopter.csv', skipinitialspace=True)

# Extract required columns and divide by 131 to get real values

# Plotting the data
plt.figure(figsize=(10, 6))

plt.plot(altitude_barometer, label='Altitude barometer')
# plt.plot(altitude_sensor_fusion, label='Altitude kalman')

# plt.plot(vertical_acceleration_old, label='vertical_acceleration_old')
# plt.plot(vertical_acceleration, label='vertical_acceleration')

plt.plot(vertical_velocity, label='Vertical Speed')


# plt.plot(column1Y, label='Y')
# plt.plot(column1A, label='Y target')


# Adding titles and labels
plt.title('Data from new_data.txt')
plt.xlabel('Sample Number')
plt.ylabel('Value')
# plt.ylim(-3, 3)
plt.legend()
plt.grid(True)

# Calculate tick positions based on the length of column1X
num_samples = len(altitude_barometer)
tick_positions = np.arange(0, num_samples, 521)
# Set custom tick labels
tick_labels = np.arange(1, len(tick_positions) + 1)
plt.xticks(tick_positions, tick_labels)


# num_samples = len(column1X)
# tick_positions = np.arange(0, num_samples, 521)
# plt.xticks(tick_positions)

# Show the plot
plt.show()