import argparse
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


parser = argparse.ArgumentParser(description='Plot data from a file.')
parser.add_argument('file', help='Path to the input data file')

# Add the boolean flag
parser.add_argument('--plot-calibrated-data', action='store_true',
                    help='If set, also plot the calibrated magnetometer data')

parser.add_argument('--compare', action='store_true',
                    help='If set, also plot data for second sensor')

parser.add_argument('--in_csv', action='store_true',
                    help='If set, read input file as CSV with comma delimiter')

args = parser.parse_args()

# # Load data
# with open(args.file, 'r') as f:
#     lines = f.readlines()
# data = [list(map(float, line.strip().split())) for line in lines if line.strip()]
# data = np.array(data)




# # Load data
# # Load data and check columns count per line
# with open(args.file, 'r') as f:
#     lines = f.readlines()

# lines_with_too_many_columns = []
# filtered_lines = []

# for idx, line in enumerate(lines):
#     stripped_line = line.strip()
#     if stripped_line:
#         columns = stripped_line.split()
#         if len(columns) > 3:
#             for column in columns:
#                 if(column.__len__ > 11):
#                     print(idx)
#             print(f"Row index {idx} has more than 3 columns: {len(columns)} columns")
#             lines_with_too_many_columns.append(idx)
#         else:
#             filtered_lines.append(columns)

# # Convert only valid lines to float numpy array
# data = np.array([list(map(float, line)) for line in filtered_lines])



# Load data lines
with open(args.file, 'r', encoding='utf-8', errors='replace') as f:
    lines = f.readlines()


def split_line(line, csv=False):
    if csv == True:
        return [c for c in line.split(';') if c != '']
    else:
        return line.split()


# 1) Check for rows with more than 3 columns and print their indices
print("Checking for rows with more than 3 columns:")
lines_with_too_many_columns = []
for idx, line in enumerate(lines):
    if idx == 0:
        continue
    
    stripped_line = line.strip()
    if stripped_line:
        columns = split_line(stripped_line, csv=args.in_csv)
        if args.compare == True and len(columns) > 6:
            print(f"Row index {idx} has more than 6 columns: {len(columns)} columns")
            lines_with_too_many_columns.append(idx)
        elif args.compare == False and len(columns) > 3:
            
            print(args.compare)
            print(f"Row index {idx} has more than 3 columns: {len(columns)} columns")
            lines_with_too_many_columns.append(idx)


# 2) Check for column values with string length > 11 chars
print("\nChecking for column values longer than 11 characters:")
lines_with_long_values = []
for idx, line in enumerate(lines):
    if idx == 0:
        continue

    stripped_line = line.strip()
    if stripped_line:
        columns = split_line(stripped_line, csv=args.in_csv)
        for col_idx, val in enumerate(columns):
            
            if len(val) > 14:

                print(f"Row {idx}, Column {col_idx} has a value longer than 11 chars: '{val}' len() = {len(val)}")
                lines_with_long_values.append((idx, col_idx, val))

# 3) Check which columns do not parse as floats and which parse floats above 10000
print("\nChecking for non-float values and floats above 10000:")
lines_with_non_float = []
lines_with_large_float = []
for idx, line in enumerate(lines):
    if idx == 0:
        continue

    stripped_line = line.strip()
    if stripped_line:
        columns = split_line(stripped_line, csv=args.in_csv)
        for col_idx, val in enumerate(columns):
            try:
                fval = float(val)
                if fval > 10000:
                    print(f"Row {idx}, Column {col_idx} has a float value above 10000: {fval}")
                    lines_with_large_float.append((idx, col_idx, fval))
            except ValueError:
                print(f"Row {idx}, Column {col_idx} has a non-float value: '{val}'")
                lines_with_non_float.append((idx, col_idx, val))

# Optionally, filter lines to keep only those with max 3 columns for further processing:
filtered_lines = [
    split_line(line.strip(), csv=args.in_csv)
    for i, line in enumerate(lines)
    if i not in lines_with_too_many_columns and line.strip()
]

# Build filtered_lines with their original indices (line numbers)
filtered_lines_with_indices = [
    (i, split_line(line.strip(), csv=args.in_csv))
    for i, line in enumerate(lines)
    if i not in lines_with_too_many_columns and line.strip()
]

data = []
for original_idx, line_values in filtered_lines_with_indices:
    float_line = []
    parse_error = False
    for val in line_values:
        try:
            fval = float(val)
            float_line.append(fval)
        except ValueError:
            print(f"Error parsing float on original line {original_idx}: value '{val}'")
            parse_error = True
            break  # Stop processing this line on first error
    if not parse_error:
        data.append(float_line)
    else:
        print(f"Skipping original line {original_idx} due to parse errors.")

data = np.array(data)


if data.size > 0:
    for col_idx in range(data.shape[1]):
        col_min = np.min(data[:, col_idx])
        col_max = np.max(data[:, col_idx])
        print(f"Column {col_idx + 1}: min = {col_min}, max = {col_max}")
else:
    print("No valid data to compute min/max.")

# 2D plots: all column combinations (original)
fig2d, axes = plt.subplots(1, 3, figsize=(15, 5))
# num_cols = data.shape[1]
num_cols = 3

plot_count = 0
for i in range(num_cols):
    for j in range(i + 1, num_cols):
        ax = axes[plot_count]
        ax.plot(data[:, i], data[:, j], 'o', markersize=0.5)  # Smaller dots
        if args.compare:
            ax.plot(data[:, i+3], data[:, j+3], 'o', markersize=0.5)  # Smaller dots


        if args.compare:
            all_x = np.concatenate([data[:, i], data[:, i+3]])
            all_y = np.concatenate([data[:, j], data[:, j+3]])
        else:
            all_x = data[:, i]
            all_y = data[:, j]

        min_lim = min(all_x.min(), all_y.min())
        max_lim = max(all_x.max(), all_y.max())
        center = (min_lim + max_lim) / 2
        max_range = max(all_x.max() - all_x.min(), all_y.max() - all_y.min()) / 2

        ax.set_xlim(center - max_range, center + max_range)
        ax.set_ylim(center - max_range, center + max_range)
        ax.set_aspect('equal', adjustable='box')


        

        ax.set_xlabel(f'Column {i+1}')
        ax.set_ylabel(f'Column {j+1}')
        ax.set_title(f'Col {i+1} vs Col {j+1}')
        plot_count += 1
fig2d.tight_layout()


# # 3D plot: all columns (original)
# fig3d = plt.figure()
# ax3d = fig3d.add_subplot(111, projection='3d')
# ax3d.scatter(data[:, 0], data[:, 1], data[:, 2], s=0.5)  # Smaller dots
# ax3d.set_xlabel('Column 1')
# ax3d.set_ylabel('Column 2')
# ax3d.set_zlabel('Column 3')
# ax3d.set_title('3D plot of Columns 1, 2, and 3')


if args.plot_calibrated_data:
    # Hard iron correction vector


    if args.compare == True:
        # magnetometer_hard_iron_correction_1 = np.array([
        #     -37.536070, -10.123957, -12.278410
        # ])

        # # Soft iron correction matrix
        # magnetometer_soft_iron_correction_1 = np.array([
        #     [0.984543, -0.006678, -0.008087],
        #     [-0.006678, 0.998121, -0.007749],
        #     [-0.008087, -0.007749, 1.062088]
        # ])

        magnetometer_hard_iron_correction_1 = np.array([
            -25.827434, -1.773559, -6.239976
        ])

        # Soft iron correction matrix
        magnetometer_soft_iron_correction_1 = np.array([
            [0.987158, -0.007043, -0.007839],
            [-0.007043, 0.997832, -0.006877],
            [-0.007839, -0.006877, 1.063742]
        ])

        

        
        
        

        magnetometer_hard_iron_correction_2 = np.array([
            3277.392264, 3275.273523, 3271.048347
        ])

        # Soft iron correction matrix
        magnetometer_soft_iron_correction_2 = np.array([
            [1.001581, -0.002604, -0.008801],
            [-0.002604, 0.980725, 0.063952],
            [-0.008801, 0.063952, 1.046160]
        ])

        data1, data2 = np.split(data, 2, axis=1)
        print(data1.shape)
        print(data2.shape)


        # Apply hard iron correction: subtract offsets from each sample
        data_hard_corrected_1 = data1 - magnetometer_hard_iron_correction_1
        data_hard_corrected_2 = data2 - magnetometer_hard_iron_correction_2

        # Apply soft iron correction: matrix multiplication for each sample
        data_calibrated_1 = np.dot(data_hard_corrected_1, magnetometer_soft_iron_correction_1.T)
        data_calibrated_2 = np.dot(data_hard_corrected_2, magnetometer_soft_iron_correction_2.T)

        print(data_calibrated_1.shape)
        print(data_calibrated_2.shape)

        xy_lim = np.concatenate([data[:, i], data[:, j]])
        # Plot calibrated data: 2D plots of all column combinations
        fig2d_cal, axes_cal = plt.subplots(1, 3, figsize=(15, 5))
        plot_count = 0
        for i in range(3):
            for j in range(i + 1, 3):
                ax = axes_cal[plot_count]
                ax.plot(data_calibrated_1[:, i], data_calibrated_1[:, j], 'o', markersize=1, color='r')
                ax.plot(data_calibrated_2[:, i], data_calibrated_2[:, j], 'o', markersize=1, color='g')

                # Combine data from both datasets for axis limits
                all_x = np.concatenate([data_calibrated_1[:, i], data_calibrated_2[:, i]])
                all_y = np.concatenate([data_calibrated_1[:, j], data_calibrated_2[:, j]])
                min_lim = min(all_x.min(), all_y.min())
                max_lim = max(all_x.max(), all_y.max())
                center = (min_lim + max_lim) / 2
                max_range = max(all_x.max() - all_x.min(), all_y.max() - all_y.min()) / 2
                
                ax.set_xlim(center - max_range, center + max_range)
                ax.set_ylim(center - max_range, center + max_range)
                ax.set_aspect('equal', adjustable='box')

                ax.set_xlabel(f'Calibrated Column {i+1}')
                ax.set_ylabel(f'Calibrated Column {j+1}')
                ax.set_title(f'Calibrated Col {i+1} vs Calibrated Col {j+1}')
                plot_count += 1
        fig2d_cal.tight_layout()

    else:
        magnetometer_hard_iron_correction = np.array([
            -36.533517, -7.833620, -10.153782
        ])

        # Soft iron correction matrix
        magnetometer_soft_iron_correction = np.array([
            [0.986775, -0.007845, -0.004856],
            [-0.007845, 0.997795, -0.007557],
            [-0.004856, -0.007557, 1.063928]
        ])

        print(data.shape)

        # Apply hard iron correction: subtract offsets from each sample
        data_hard_corrected = data - magnetometer_hard_iron_correction

        # Apply soft iron correction: matrix multiplication for each sample
        data_calibrated = np.dot(data_hard_corrected, magnetometer_soft_iron_correction.T)

        print(data_calibrated.shape)

        # Plot calibrated data: 2D plots of all column combinations
        fig2d_cal, axes_cal = plt.subplots(1, 3, figsize=(15, 5))
        plot_count = 0
        for i in range(3):
            for j in range(i + 1, 3):
                ax = axes_cal[plot_count]
                ax.plot(data_calibrated[:, i], data_calibrated[:, j], 'o', markersize=1, color='r')

                # Compute combined limits just from this dataset
                x = data_calibrated[:, i]
                y = data_calibrated[:, j]
                min_lim = min(x.min(), y.min())
                max_lim = max(x.max(), y.max())
                center = (min_lim + max_lim) / 2
                max_range = max(x.max() - x.min(), y.max() - y.min()) / 2
                
                ax.set_xlim(center - max_range, center + max_range)
                ax.set_ylim(center - max_range, center + max_range)
                ax.set_aspect('equal', adjustable='box')

                ax.set_xlabel(f'Calibrated Column {i+1}')
                ax.set_ylabel(f'Calibrated Column {j+1}')
                ax.set_title(f'Calibrated Col {i+1} vs Calibrated Col {j+1}')
                plot_count += 1
        fig2d_cal.tight_layout()

# Show all plots together
plt.show()
