import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# Lists to store the first two columns
file_path = "../misc/gps_logs/gps_targeting23.txt"

data1 = []
data2 = []
data3 = []
data4 = []

skip_lines_from_start = 300

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
                data1.append(float(columns[10]))
                data2.append(float(columns[11]))
                data3.append(float(columns[12]))
                data4.append(float(columns[13]))

                # vertical_velocity.append(float(columns[4]))

            except ValueError:
                # Handle lines that cannot be converted to float
                print(f"Skipping line due to conversion error: {line.strip()}")

# Plotting the data
plt.figure(figsize=(10, 6))

# plt.plot(data1, label='data1')
plt.plot(data2, label='data2')
# plt.plot(data3, label='data3')
# plt.plot(data4, label='data4')

# Adding titles and labels
plt.title('Data from new_data.txt')
plt.xlabel('Sample Number')
plt.ylabel('Value')
# plt.ylim(-3, 3)
plt.legend()
plt.grid(True)

# Calculate tick positions based on the length of column1X
num_samples = len(data1)
tick_positions = np.arange(0, num_samples, 10)
# Set custom tick labels
tick_labels = np.arange(1, len(tick_positions) + 1)
plt.xticks(tick_positions, tick_labels)

# Show the plot
plt.show()