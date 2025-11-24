import numpy as np
import pandas as pd
import sys
import os


def calculate_yaw_tilt_compensated(mx, my, mz, roll, pitch, yaw_offset):
    # Convert to radians
    roll_r = np.deg2rad(roll)
    pitch_r = np.deg2rad(pitch)

    # Tilt compensation
    Xh = mx * np.cos(pitch_r) + mz * np.sin(pitch_r)
    Yh = mx * np.sin(roll_r) * np.sin(pitch_r) + my * np.cos(roll_r) - mz * np.sin(roll_r) * np.cos(pitch_r)

    yaw = np.degrees(np.arctan2(Yh, Xh)) + yaw_offset

    # Normalize to [0, 360)
    yaw = np.mod(yaw, 360)
    return yaw


def undo_rotation(input_file):
    mag1_rotation = np.array([
        [0.866503,  0.432242, -0.249678],
        [-0.456176,  0.482611, -0.747656],
        [-0.202671,  0.761744,  0.615363]
    ])

    mag2_rotation = np.array([
        [-0.417500, -0.889518,  0.185610],
        [-0.503418,  0.396475,  0.767709],
        [-0.756481,  0.227080, -0.613328]
    ])

    # Load CSV with the correct delimiter
    df = pd.read_csv(input_file, delimiter=';')

    # Compute inverse rotation matrices
    mag1_inv = np.linalg.inv(mag1_rotation)
    mag2_inv = np.linalg.inv(mag2_rotation)

    mag1_data = df[['magx', 'magy', 'magz']].values
    mag2_data = df[['mag2x', 'mag2y', 'mag2z']].values

    # Apply inverse rotations
    mag1_unrot = (mag1_inv @ mag1_data.T).T
    mag2_unrot = (mag2_inv @ mag2_data.T).T

    # Add corrected values to new columns
    df['magx'] = mag1_unrot[:, 0]
    df['magy'] = mag1_unrot[:, 1]
    df['magz'] = mag1_unrot[:, 2]
    df['mag2x'] = mag2_unrot[:, 0]
    df['mag2y'] = mag2_unrot[:, 1]
    df['mag2z'] = mag2_unrot[:, 2]

    # Compute new yaw values
    yaw_offset = 5.117134
    df['magnetometer_yaw'] = calculate_yaw_tilt_compensated(
        df['magx'], df['magy'], df['magz'], df['roll'], df['pitch'], yaw_offset
    )

    df['magnetometer_yaw_secondary'] = calculate_yaw_tilt_compensated(
        df['mag2x'], df['mag2y'], df['mag2z'], df['roll'], df['pitch'], yaw_offset
    )

    # Save new CSV with suffix "_corrected"
    filename, ext = os.path.splitext(input_file)
    output_file = f"{filename}_fixed{ext}"
    df.to_csv(output_file, index=False, sep=';')
    print(f"Corrected data saved to {output_file}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python undo_magnetometer_rotation.py <input_csv_file>")
        sys.exit(1)
    input_csv = sys.argv[1]
    undo_rotation(input_csv)
