import numpy as np
import pandas as pd
import argparse
import matplotlib.pyplot as plt


def is_numeric(s):
    if not s:
        return False
    try:
        float(s)
        return True
    except ValueError:
        return False


def is_header_line(line):
    fields = line.strip().split(';')
    if len(fields) <= 1:
        return False
    non_numeric_count = 0
    for field in fields:
        field = field.strip()
        if not field:
            continue
        try:
            float(field)
        except ValueError:
            non_numeric_count += 1
    if non_numeric_count > len(fields) / 2 or (len(fields) > 0 and not is_numeric(fields[0].strip())):
        return True
    return False


def compute_baseline(df, axis_names, zero_rpm_mask):
    return {axis: df.loc[zero_rpm_mask, axis].mean() for axis in axis_names}


def smooth_values(values, window=5):
    smoothed = values.copy()
    half = window // 2
    n = len(values)
    for i in range(half, n - half):
        smoothed[i] = np.mean(values[i - half:i + half + 1])
    return smoothed


def main():
    parser = argparse.ArgumentParser(description="Generate lookup arrays and plot magnetometer offsets vs RPM with smoothing.")
    parser.add_argument("csvfile", help="CSV file path")
    args = parser.parse_args()

    with open(args.csvfile, 'r') as f:
        first_line = f.readline()

    has_header = is_header_line(first_line)

    col_names = ["motor0 rpm", "motor1 rpm", "motor2 rpm", "motor3 rpm",
                 "mag x", "mag y", "mag z", "mag yaw",
                 "mag2 x", "mag2 y", "mag2 z", "mag2 yaw",
                 "gps yaw", "gyro yaw", "roll", "pitch"]

    if has_header:
        df = pd.read_csv(args.csvfile, sep=';')
    else:
        df = pd.read_csv(args.csvfile, sep=';', header=None, names=col_names)

    mean_rpm = df[['motor0 rpm', 'motor1 rpm', 'motor2 rpm', 'motor3 rpm']].mean(axis=1)
    zero_rpm_mask = (df[['motor0 rpm', 'motor1 rpm', 'motor2 rpm', 'motor3 rpm']] == 0).all(axis=1)

    mag1_axes = ['mag x', 'mag y', 'mag z']
    mag2_axes = ['mag2 x', 'mag2 y', 'mag2 z']

    baseline_mag1 = compute_baseline(df, mag1_axes, zero_rpm_mask)
    baseline_mag2 = compute_baseline(df, mag2_axes, zero_rpm_mask)

    offsets_mag1 = {axis: df[axis] - baseline_mag1[axis] for axis in mag1_axes}
    offsets_mag2 = {axis: df[axis] - baseline_mag2[axis] for axis in mag2_axes}

    rpm_min = mean_rpm.min()
    rpm_max = mean_rpm.max()
    n_samples = 100

    rpm_values = np.linspace(rpm_min, rpm_max, n_samples)

    mag1_samples = {axis: [] for axis in mag1_axes}
    mag2_samples = {axis: [] for axis in mag2_axes}

    for rpm in rpm_values:
        idx = (np.abs(mean_rpm - rpm)).idxmin()
        for axis in mag1_axes:
            mag1_samples[axis].append(offsets_mag1[axis].iloc[idx])
        for axis in mag2_axes:
            mag2_samples[axis].append(offsets_mag2[axis].iloc[idx])

    def zero_noise_before_sign_change(values):
        last_sign = np.sign(values[-1])
        for i in range(len(values) - 2, -1, -1):
            if np.sign(values[i]) != last_sign and np.sign(values[i]) != 0:
                return [0.0 if j < i + 1 else values[j] for j in range(len(values))]
        return values

    for axis in mag1_axes:
        mag1_samples[axis] = zero_noise_before_sign_change(mag1_samples[axis])
    for axis in mag2_axes:
        mag2_samples[axis] = zero_noise_before_sign_change(mag2_samples[axis])

    mag1_smoothed = {axis: smooth_values(mag1_samples[axis], window=5) for axis in mag1_axes}
    mag2_smoothed = {axis: smooth_values(mag2_samples[axis], window=5) for axis in mag2_axes}


    print("RPM_samples = [")
    for v in rpm_values:
        print(f"    {v:.6f},")
    print("]\n")

    for axis in mag1_axes:
        print(f"{axis.replace(' ', '_')}_offsets = [")
        for v in mag1_smoothed[axis]:
            print(f"    {v:.6f},")
        print("]\n")

    for axis in mag2_axes:
        print(f"{axis.replace(' ', '_')}_offsets = [")
        for v in mag2_smoothed[axis]:
            print(f"    {v:.6f},")
        print("]\n")

    print(f"Minimum RPM in dataset: {rpm_min:.6f}")
    print(f"Maximum RPM in dataset: {rpm_max:.6f}")

        # Plot before and after smoothing for mag1 axes
    # for axis in mag1_axes:
    #     plt.figure(figsize=(10, 5))
    #     plt.plot(rpm_values, mag1_samples[axis], label='Original')
    #     plt.plot(rpm_values, mag1_smoothed[axis], label='Smoothed')
    #     plt.title(f'{args.csvfile} Mag1 {axis} offset vs RPM')
    #     plt.xlabel('RPM')
    #     plt.ylabel('Offset')
    #     plt.legend()
    #     plt.grid(True)

    # # Plot before and after smoothing for mag2 axes
    # for axis in mag2_axes:
    #     plt.figure(figsize=(10, 5))
    #     plt.plot(rpm_values, mag2_samples[axis], label='Original')
    #     plt.plot(rpm_values, mag2_smoothed[axis], label='Smoothed')
    #     plt.title(f'{args.csvfile} Mag2 {axis} offset vs RPM')
    #     plt.xlabel('RPM')
    #     plt.ylabel('Offset')
    #     plt.legend()
    #     plt.grid(True)

    # plt.show()


    def interpolate_offset(rpm, rpm_values, offset_values):
        if rpm_values is None or offset_values is None or len(rpm_values) == 0:
            return 0.0
        if rpm <= rpm_values[0]:
            return 0.0
        if rpm >= rpm_values[-1]:
            return offset_values[-1]
        # Linear search for interval
        for i in range(1, len(rpm_values)):
            if rpm < rpm_values[i]:
                x0, x1 = rpm_values[i-1], rpm_values[i]
                y0, y1 = offset_values[i-1], offset_values[i]
                t = (rpm - x0) / (x1 - x0)
                return y0 + t * (y1 - y0)
        return offset_values[-1]

    # Example usage test for mag1_x_offsets
    print("\nInterpolation test for mag1_x_offsets:")
    min_rpm = rpm_values[0]
    mid_rpm = (rpm_values[0] + rpm_values[-1]) / 2
    max_rpm = rpm_values[-1]
    below_min_rpm = min_rpm - 10.0

    for test_rpm in [below_min_rpm, min_rpm, mid_rpm, max_rpm]:
        offset = interpolate_offset(test_rpm, rpm_values, mag1_smoothed['mag x'])
        print(f"RPM: {test_rpm:.2f} => Interpolated Offset: {offset:.6f}")

if __name__ == "__main__":
    main()
