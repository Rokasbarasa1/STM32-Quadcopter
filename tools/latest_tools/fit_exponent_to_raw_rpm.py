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

def main():
    parser = argparse.ArgumentParser(description="Fit yaw offset based on RPM with 0 RPM baseline correction.")
    parser.add_argument("csvfile", help="CSV file path")
    args = parser.parse_args()

    with open(args.csvfile, 'r') as f:
        first_line = f.readline()

    has_header = is_header_line(first_line)

    if has_header:
        df = pd.read_csv(args.csvfile, sep=';')
    else:
        col_names = ["motor0 rpm","motor1 rpm","motor2 rpm","motor3 rpm",
                     "mag x","max y","mag z","mag yaw",
                     "mag2 x","mag2 y","mag2 z","mag2 yaw",
                     "gps yaw","gyro yaw","roll","pitch"]
        df = pd.read_csv(args.csvfile, sep=';', header=None, names=col_names)

    mean_rpm = df[['motor0 rpm', 'motor1 rpm', 'motor2 rpm', 'motor3 rpm']].mean(axis=1)

    # Get yaw at zero RPM as baseline/reference: average of rows where all motors rpm == 0
    zero_rpm_mask = (df[['motor0 rpm', 'motor1 rpm', 'motor2 rpm', 'motor3 rpm']] == 0).all(axis=1)
    baseline_mag_yaw = df.loc[zero_rpm_mask, 'mag yaw'].mean()
    baseline_mag2_yaw = df.loc[zero_rpm_mask, 'mag2 yaw'].mean()

    # Calculate yaw offsets vs baseline
    mag_yaw_offset = df['mag yaw'] - baseline_mag_yaw
    mag2_yaw_offset = df['mag2 yaw'] - baseline_mag2_yaw

    # Fit the offset curves (yaw offset vs mean RPM)
    params_mag1, _ = curve_fit(exp_saturation_model, mean_rpm, mag_yaw_offset, maxfev=5000)
    params_mag2, _ = curve_fit(exp_saturation_model, mean_rpm, mag2_yaw_offset, maxfev=5000)

    print("Fitted yaw offset model parameters (corrected for 0 RPM baseline):")
    print(f"Mag yaw offset = {params_mag1[0]:.15f} * (1 - exp(-{params_mag1[1]:.15f} * RPM)) + {params_mag1[2]:.15f}")
    print(f"Mag2 yaw offset = {params_mag2[0]:.15f} * (1 - exp(-{params_mag2[1]:.15f} * RPM)) + {params_mag2[2]:.15f}")

    rpm_min = mean_rpm.min()
    rpm_max = mean_rpm.max()

    rpm_values = np.linspace(rpm_min, rpm_max, 100)
    mag1_preds = exp_saturation_model(rpm_values, *params_mag1)
    mag2_preds = exp_saturation_model(rpm_values, *params_mag2)

    print("\nRPM;Mag1_predicted_yaw_offset;Mag2_predicted_yaw_offset")
    for rpm, m1, m2 in zip(rpm_values, mag1_preds, mag2_preds):
        print(f"{rpm:.2f};{m1:.2f};{m2:.2f}")

if __name__ == "__main__":
    main()
