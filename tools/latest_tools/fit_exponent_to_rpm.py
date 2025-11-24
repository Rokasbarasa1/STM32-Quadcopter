import numpy as np
import pandas as pd
from scipy.optimize import curve_fit
import argparse

# Exponential saturation model: y = a * (1 - exp(-b * x)) + c
def exp_saturation_model(x, a, b, c):
    return a * (1 - np.exp(-b * x)) + c

def main():
    parser = argparse.ArgumentParser(description="Fit exponential yaw offset models for mag1 and mag2")
    parser.add_argument("csvfile", help="CSV file path with columns including 'mag1 yaw','mag2 yaw', and motor RPMs")
    args = parser.parse_args()

    # Read CSV with semicolon separator
    df = pd.read_csv(args.csvfile, sep=';')

    # Compute mean RPM across motors per row
    mean_rpm = df[['rpm motor0', 'rpm motor1', 'rpm motor2', 'rpm motor3']].mean(axis=1)

    # Extract mag1 and mag2 yaw columns
    mag1_yaw = df['mag1 yaw']
    mag2_yaw = df['mag2 yaw']

    # Fit exp saturation to mag1 yaw vs mean RPM
    params_mag1, _ = curve_fit(exp_saturation_model, mean_rpm, mag1_yaw, maxfev=5000)
    a1, b1, c1 = params_mag1

    # Fit exp saturation to mag2 yaw vs mean RPM
    params_mag2, _ = curve_fit(exp_saturation_model, mean_rpm, mag2_yaw, maxfev=5000)
    a2, b2, c2 = params_mag2

    print("Fitted curve parameters:")
    print(f"Mag1 yaw offset = {a1:.4f} * (1 - exp(-{b1:.4f} * RPM)) + {c1:.4f}")
    print(f"Mag2 yaw offset = {a2:.4f} * (1 - exp(-{b2:.4f} * RPM)) + {c2:.4f}")

    # Example calculation at mean RPM=100
    rpm_example = 100
    offset1 = exp_saturation_model(rpm_example, a1, b1, c1)
    offset2 = exp_saturation_model(rpm_example, a2, b2, c2)
    print(f"\nAt mean RPM={rpm_example}:")
    print(f"Mag1 predicted yaw offset: {offset1:.2f} degrees")
    print(f"Mag2 predicted yaw offset: {offset2:.2f} degrees")

if __name__ == "__main__":
    main()
