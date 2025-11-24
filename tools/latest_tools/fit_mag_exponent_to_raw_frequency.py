import numpy as np
import pandas as pd
from scipy.optimize import curve_fit
import argparse

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

def exp_saturation_model(x, a, b, c):
    return a * (1 - np.exp(-b * x)) + c

def compute_baseline(df, axis_names, zero_rpm_mask):
    return {axis: df.loc[zero_rpm_mask, axis].mean() for axis in axis_names}

def main():
    parser = argparse.ArgumentParser(description="Fit mag offsets for X, Y, Z axes (both sensors) vs RPM, with 0 RPM baseline correction.")
    parser.add_argument("csvfile", help="CSV file path")
    args = parser.parse_args()

    with open(args.csvfile, 'r') as f:
        first_line = f.readline()

    has_header = is_header_line(first_line)

    col_names = ["motor0 rpm","motor1 rpm","motor2 rpm","motor3 rpm",
                 "mag x","max y","mag z","mag yaw",
                 "mag2 x","mag2 y","mag2 z","mag2 yaw",
                 "gps yaw","gyro yaw","roll","pitch"]

    if has_header:
        df = pd.read_csv(args.csvfile, sep=';')
    else:
        df = pd.read_csv(args.csvfile, sep=';', header=None, names=col_names)

    mean_rpm = df[['motor0 rpm', 'motor1 rpm', 'motor2 rpm', 'motor3 rpm']].mean(axis=1)
    zero_rpm_mask = (df[['motor0 rpm', 'motor1 rpm', 'motor2 rpm', 'motor3 rpm']] == 0).all(axis=1)

    mag1_axes = ['mag x', 'max y', 'mag z']
    mag2_axes = ['mag2 x', 'mag2 y', 'mag2 z']

    baseline_mag1 = compute_baseline(df, mag1_axes, zero_rpm_mask)
    baseline_mag2 = compute_baseline(df, mag2_axes, zero_rpm_mask)

    offsets_mag1 = {axis: df[axis] - baseline_mag1[axis] for axis in mag1_axes}
    offsets_mag2 = {axis: df[axis] - baseline_mag2[axis] for axis in mag2_axes}

    print("Fitted magnetometer offset model parameters (corrected for 0 RPM baseline):")
    params_dict = {}

    # Fit and print params for mag1 axes
    for axis in mag1_axes:
        params, _ = curve_fit(exp_saturation_model, mean_rpm, offsets_mag1[axis], maxfev=5000)
        params_dict[axis] = params
        print(f"{axis} offset = {params[0]:.15f} * (1 - exp(-{params[1]:.15f} * RPM)) + {params[2]:.15f}")

    # Fit and print params for mag2 axes
    for axis in mag2_axes:
        params, _ = curve_fit(exp_saturation_model, mean_rpm, offsets_mag2[axis], maxfev=5000)
        params_dict[axis] = params
        print(f"{axis} offset = {params[0]:.15f} * (1 - exp(-{params[1]:.15f} * RPM)) + {params[2]:.15f}")

    # Optionally, print predicted values for the first axis as example
    rpm_min = mean_rpm.min()
    rpm_max = mean_rpm.max()
    rpm_values = np.linspace(rpm_min, rpm_max, 100)

    print("\nRPM;" + ";".join([f"{a}_pred" for a in mag1_axes + mag2_axes]))
    for i, rpm in enumerate(rpm_values):
        preds = [exp_saturation_model(rpm, *params_dict[axis]) for axis in mag1_axes + mag2_axes]
        print("{:.2f};{}".format(rpm, ";".join([f"{p:.6f}" for p in preds])))

    # Compare predictions vs actual data for zero RPM samples
    print("\nRPM;Mag1_x_Pred;Mag1_y_Pred;Mag1_z_Pred;Mag1_x_Actual;Mag1_y_Actual;Mag1_z_Actual;Mag2_x_Pred;Mag2_y_Pred;Mag2_z_Pred;Mag2_x_Actual;Mag2_y_Actual;Mag2_z_Actual")

    for rpm in rpm_values:
        # Model predictions for this RPM
        preds = [exp_saturation_model(rpm, *params_dict[axis]) for axis in mag1_axes + mag2_axes]

        # Find index of closest RPM in original data
        closest_idx = np.abs(mean_rpm - rpm).idxmin()

        # Get measured offsets at closest index
        actual_mag1 = [offsets_mag1[axis].iloc[closest_idx] for axis in mag1_axes]
        actual_mag2 = [offsets_mag2[axis].iloc[closest_idx] for axis in mag2_axes]

        # Print RPM, predicted vals, and actual offset vals
        row_str = (
            f"{rpm:.2f};" +
            ";".join(f"{p:.6f}" for p in preds) + ";     " +
            ";".join(f"{v:.6f}" for v in actual_mag1) + ";" +
            ";".join(f"{v:.6f}" for v in actual_mag2)
        )
        print(row_str)
    
if __name__ == "__main__":
    main()
