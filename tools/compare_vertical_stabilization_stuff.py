import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# Lists to store the first two columns
altitude_barometer = []
target_altitude_barometer = []
altitude_barometer_rate_of_change = []
target_altitude_barometer_rate_of_change = []

# file_path = "../misc/testing_vertical_stabilization/5long_test.txt"
# file_path = "../misc/testing_vertical_stabilization/6long_test.txt"
# file_path = "../misc/testing_vertical_stabilization/7long_test.txt"
# file_path = "../misc/testing_vertical_stabilization/8long_test.txt"
# file_path = "../misc/testing_vertical_stabilization/10long_test.txt"
# file_path = "../misc/testing_vertical_stabilization/11long_test.txt"
# file_path = "../misc/testing_vertical_stabilization/13long_test.txt"
file_path = "../misc/testing_vertical_stabilization/15long_test.txt"
file_path = "../misc/testing_vertical_stabilization/1Quadcopter.txt"
file_path = "../misc/testing_vertical_stabilization/2Quadcopter.txt"


skip_lines_from_start = 300

# file_path = "../misc/testing_vertical_stabilization/3short_test.txt"

# Read the file and parse the first two columns
with open(file_path, 'r') as file:
    index = 0
    for line in file:
        # Split the line into columns based on the semicolon delimiter
        columns = line.strip().split(';')
        index += 1

        if(index < skip_lines_from_start):
            continue

        # Ensure there are enough columns to avoid errors
        if len(columns) > 1:
            try:
                # Append the first two columns (convert to floats)
                altitude_barometer.append(float(columns[0]))
                target_altitude_barometer.append(float(columns[1]))
                altitude_barometer_rate_of_change.append(float(columns[2]))
                target_altitude_barometer_rate_of_change.append(float(columns[3]))
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

plt.plot(altitude_barometer, label='altitude_barometer')
plt.plot(target_altitude_barometer, label='target_altitude_barometer')
# plt.plot(altitude_barometer_rate_of_change, label='altitude_barometer_rate_of_change')
# plt.plot(target_altitude_barometer_rate_of_change, label='target_altitude_barometer_rate_of_change')



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
tick_positions = np.arange(0, num_samples, 50)
# Set custom tick labels
tick_labels = np.arange(1, len(tick_positions) + 1)
plt.xticks(tick_positions, tick_labels)


# num_samples = len(column1X)
# tick_positions = np.arange(0, num_samples, 521)
# plt.xticks(tick_positions)

# Show the plot
plt.show()