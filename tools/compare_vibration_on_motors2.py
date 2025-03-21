import matplotlib.pyplot as plt
import numpy as np

# Read data from new_data.txt
# data = np.genfromtxt('../misc/motors_off_no_filtering_data.txt', delimiter=';')
# data = np.genfromtxt('../misc/raw_yaw_vs_sensor_fused.txt', delimiter=';')

# data = np.genfromtxt('../misc/motor_2212_bl_with_nuts_data.txt', delimiter=';')
# data = np.genfromtxt('../misc/motor_2212_br_with_nuts_data.txt', delimiter=';')
# data = np.genfromtxt('../misc/motor_2212_fl_with_nuts_data.txt', delimiter=';')
# data = np.genfromtxt('../misc/motor_2212_fr_with_nuts_data.txt', delimiter=';')

# data = np.genfromtxt('../misc/motor_2807_bl_no_nuts_data.txt', delimiter=';')
# data = np.genfromtxt('../misc/motor_2807_br_no_nuts_data.txt', delimiter=';')
# data = np.genfromtxt('../misc/motor_2807_fl_no_nuts_data.txt', delimiter=';')
# data = np.genfromtxt('../misc/motor_2807_fr_no_nuts_data.txt', delimiter=';')
data = np.genfromtxt('../misc/motor_2807_bl_3_blade_propeller.txt', delimiter=';')


# data2 = np.genfromtxt('../misc/motor_2807_bl_no_nuts_data.txt', delimiter=';')
# data2 = np.genfromtxt('../misc/motor_2807_br_no_nuts_data.txt', delimiter=';')
# data2 = np.genfromtxt('../misc/motor_2807_fl_no_nuts_data.txt', delimiter=';')
# data2 = np.genfromtxt('motor_2807_fr_no_nuts_data.txt', delimiter=';')
# data2 = np.genfromtxt('../misc/motor_2807_bl_3_blade_propeller.txt', delimiter=';')








# data2 = np.genfromtxt('../misc/motors_on_no_filtering_data.txt', delimiter=';')
# data2 = np.genfromtxt('../misc/motors_on_no_filtering_data_new_props.txt', delimiter=';')
# data2 = np.genfromtxt('../misc/motors_on_new_props_tape_applied.txt', delimiter=';')
# data2 = np.genfromtxt('../misc/motors_on_low_pass_10Hz.txt', delimiter=';')
# data2 = np.genfromtxt('../misc/motors_on_low_pass_48Hz.txt', delimiter=';')

# data2 = np.genfromtxt('../misc/no_propellers_no_filtering_only_tape_dampening.txt', delimiter=';')
# data2 = np.genfromtxt('../misc/no_propellers_no_filtering_tape_and_rubber_spacers.txt', delimiter=';')
# data2 = np.genfromtxt('../misc/no_propellers_no_filtering_tape_rubber_standoffs_ruber_spacers.txt', delimiter=';')
# data2 = np.genfromtxt('../misc/no_propellers_no_filtering_tape_rubber_standoffs_ruber_spacers.txt', delimiter=';')

# data2 = np.genfromtxt('../misc/propellers_no_filtering_tape_rubber_standoffs_ruber_spacers.txt', delimiter=';')
# data2 = np.genfromtxt('../misc/propellers_no_filtering_tape_and_rubber_spacers.txt', delimiter=';')
# data2 = np.genfromtxt('../misc/propellers_no_filtering_tape_rubber_spacers_yaw_pid_match.txt', delimiter=';')
# data2 = np.genfromtxt('../misc/propellers_no_filtering_tape_rubber_spacers_yaw_pid_match_21Hz.txt', delimiter=';')
# data2 = np.genfromtxt('../misc/propellers_no_filtering_tape_rubber_spacers_yaw_pid_match_45Hz.txt', delimiter=';')

data2 = np.genfromtxt('../misc/motor_2212_bl_no_nuts_data.txt', delimiter=';')
# data2 = np.genfromtxt('../misc/motor_2212_br_no_nuts_data.txt', delimiter=';')
# data2 = np.genfromtxt('../misc/motor_2212_fl_no_nuts_data.txt', delimiter=';')
# data2 = np.genfromtxt('../misc/motor_2212_fr_no_nuts_data.txt', delimiter=';')

# Check if the data is read correctly
if data.size == 0:
    print("No data found in new_data.txt")
    exit(1)

# Extracting each column for plotting
# column1 = data[:, 0]
# column2 = data[:, 1]



column11 = data[:, 0]
column12 = data[:, 1] - 7
column13 = data[:, 2] - 14
column14 = data[:, 3] + 200
column15 = data[:, 4] + 100
column16 = data[:, 5]


column21 = data2[:, 0] - 3
column22 = data2[:, 1] - 10
column23 = data2[:, 2] - 17
column24 = data2[:, 3] + 150
column25 = data2[:, 4] + 50
column26 = data2[:, 5] - 50


# Plotting the data
plt.figure(figsize=(10, 6))

plt.plot(column11, label='Ax OFF')
plt.plot(column21, label='Ax ON')
plt.plot(column12, label='Ay OFF')
plt.plot(column22, label='Ay ON')
plt.plot(column13, label='Az OFF')
plt.plot(column23, label='Az ON')


# plt.plot(column14, label='Gx OFF')
# plt.plot(column24, label='Gx ON')
# plt.plot(column15, label='Gy OFF')
# plt.plot(column25, label='Gy ON')
# plt.plot(column16, label='Gz OFF')
# plt.plot(column26, label='Gz ON')


# plt.plot(column1, label='Yaw raw')
# plt.plot(column2, label='Yaw fused')

# Adding titles and labels
plt.title('Data from new_data.txt')
plt.xlabel('Sample Number')
plt.ylabel('Value')
plt.legend()
plt.grid(True)

num_samples = len(column11)
# num_samples = len(column1)

tick_positions = np.arange(0, num_samples, 200)
plt.xticks(tick_positions)

# Show the plot
plt.show()