import numpy as np
from scipy.linalg import svd, cholesky, inv, norm
import logging
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

def generate_synthetic_motion_data(num_samples=20000, accel_rot_deg=-30, gravity=1, mag_strength=50732.4e-9):
    # Generate random orientations (rotations)
    orientations = R.random(num_samples)

    # Gravity vector in inertial frame with magnitude of Earth gravity
    gravity_vec = np.array([0, 0, -gravity])

    # Magnetometer true direction vectors (random unit vectors on sphere) scaled to Earth's magnetic field strength
    phi = np.random.uniform(0, 2 * np.pi, num_samples)
    costheta = np.random.uniform(-1, 1, num_samples)
    theta = np.arccos(costheta)
    mag_unit = np.vstack((np.sin(theta) * np.cos(phi),
                          np.sin(theta) * np.sin(phi),
                          np.cos(theta))).T
    mag_data = mag_unit * mag_strength

    # Rotate gravity vector by orientations + offset for accelerometer data
    offset_rotation = R.from_euler('z', accel_rot_deg, degrees=True)
    accel_data = offset_rotation.apply(orientations.apply(gravity_vec))

    # Magnetometer data rotated only by orientations (no offset)
    mag_data = orientations.apply(mag_data)

    return mag_data, accel_data

mag_syn, accel_syn = generate_synthetic_motion_data()

print("Sample Magnetometer data (first 5):")
print(mag_syn[:5])
print("\nSample Accelerometer data (first 5):")
print(accel_syn[:5])



fig2d, axes = plt.subplots(1, 3, figsize=(15, 5))
# num_cols = data.shape[1]
num_cols = 3
plot_count = 0
for i in range(num_cols):
    for j in range(i + 1, num_cols):
        ax = axes[plot_count]
        ax.plot(mag_syn[:, i], mag_syn[:, j], 'o', markersize=0.5, label="Mag", color='r')  # Smaller dots
        ax.plot(accel_syn[:, i], accel_syn[:, j], 'o', markersize=0.5, label="MAG rotated selected", color='g')  # Smaller dots
        ax.set_aspect('equal', adjustable='box')

        ax.set_xlabel(f'Column {i+1}')
        ax.set_ylabel(f'Column {j+1}')
        ax.set_title(f'Col {i+1} vs Col {j+1}')
        plot_count += 1
fig2d.tight_layout()



plt.show()