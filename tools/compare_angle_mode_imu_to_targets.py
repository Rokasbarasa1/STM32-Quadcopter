import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


# Read data
# df = pd.read_csv('./imu_data/1QuadcopterNew.01.csv', skipinitialspace=True)
# df = pd.read_csv('./imu_data/2QuadcopterNew.01.csv', skipinitialspace=True)
# df = pd.read_csv('./imu_data/3QuadcopterNew.01.csv', skipinitialspace=True)
# df = pd.read_csv('./imu_data/4QuadcopterNew-Long-flight.01.csv', skipinitialspace=True)
# df = pd.read_csv('./imu_data/5QuadcopterDrift.01.csv', skipinitialspace=True)
# df = pd.read_csv('./imu_data/434Quadcopter.04.csv', skipinitialspace=True)
# df = pd.read_csv('./imu_data/drift/6Quadcopter.02.csv', skipinitialspace=True)
# df = pd.read_csv('./imu_data/drift/8Quadcopter.03.csv', skipinitialspace=True)
# df = pd.read_csv('./imu_data/drift/9Quadcopter.03.csv', skipinitialspace=True)

# df = pd.read_csv('./imu_data/drift/f35/1Quadcopter.10.csv', skipinitialspace=True)
# df = pd.read_csv('./imu_data/drift/f35/2Quadcopter.06.csv', skipinitialspace=True)
# df = pd.read_csv('./imu_data/drift/f35/3Quadcopter.03.csv', skipinitialspace=True)

# df = pd.read_csv('./imu_data/drift/f36/8Quadcopter.04.csv', skipinitialspace=True)

df = pd.read_csv('./imu_data/drift/f37/3Quadcopter.01.csv', skipinitialspace=True)
# df = pd.read_csv('./data_examples/24Quadcopter.csv', skipinitialspace=True)

# Extract required columns and divide by 131 to get real values
column1X = df['debug[0]'] / 100 # it is already converted by the .csv conversion
column1Y = df['debug[1]'] / 100 - 10
column1Z = df['debug[2]'] / 100
column1A = df['debug[3]'] / 100 - 10

angleModeRoll = df['axisI[0]'] / 10

# Plotting the data
plt.figure(figsize=(10, 6))

plt.plot(column1X, label='X')
plt.plot(column1Z, label='X target')
plt.plot(angleModeRoll, label='X integral')


# plt.plot(column1Y, label='Y')
# plt.plot(column1A, label='Y target')


# Adding titles and labels
plt.title('Data from new_data.txt')
plt.xlabel('Sample Number')
plt.ylabel('Value')
plt.ylim(-3, 3)
plt.legend()
plt.grid(True)

# Calculate tick positions based on the length of column1X
num_samples = len(column1X)
tick_positions = np.arange(0, num_samples, 521)
# Set custom tick labels
tick_labels = np.arange(1, len(tick_positions) + 1)
plt.xticks(tick_positions, tick_labels)


# num_samples = len(column1X)
# tick_positions = np.arange(0, num_samples, 521)
# plt.xticks(tick_positions)

# Show the plot
plt.show()