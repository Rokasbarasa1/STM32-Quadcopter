import numpy as np
from scipy.linalg import svd, cholesky, inv, norm, eigvalsh, lstsq, polar, eigh
import logging
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from scipy.optimize import minimize
import time
import argparse
import random
import itertools

# ----------------- Logging setup -----------------
logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
logger = logging.getLogger(__name__)
np.set_printoptions(precision=6, suppress=False, floatmode='fixed')


def add_synced_edge_circles(mag_data, accel_data, margin_fraction=0.05, num_points=200):
    """
    Adds ring data on sphere edges at X±, Y±, Z± for both mag and accel data in sync.
    margin_fraction defines how far away from max/min edges the ring is placed.
    num_points is number of points in each ring.
    """
    new_mag_points = []
    new_accel_points = []

    # For axis 0,1,2 (X,Y,Z)
    for axis in range(3):
        # Compute min/max for mag and accel on this axis
        mag_axis_data = mag_data[:, axis]
        mag_min, mag_max = np.min(mag_axis_data), np.max(mag_axis_data)
        mag_range = mag_max - mag_min
        mag_margin_max = mag_max - margin_fraction * mag_range
        mag_margin_min = mag_min + margin_fraction * mag_range

        accel_axis_data = accel_data[:, axis]
        accel_min, accel_max = np.min(accel_axis_data), np.max(accel_axis_data)
        accel_range = accel_max - accel_min
        accel_margin_max = accel_max - margin_fraction * accel_range
        accel_margin_min = accel_min + margin_fraction * accel_range

        def create_ring_points(center_value, data, axis, num_points, is_accel):
            ring_points = []
            other_axes = [i for i in range(3) if i != axis]

            # Calculate max vector magnitude from data (norm of vectors)
            max_mag = np.max(np.linalg.norm(data, axis=1))

            # Calculate radius of circle orthogonal to axis so points lie on sphere's surface
            radius = np.sqrt(max_mag**2 - center_value**2)

            for angle in np.linspace(0, 2*np.pi, num_points, endpoint=False):
                point = np.zeros(3)
                point[axis] = center_value
                point[other_axes[0]] = np.cos(angle) * radius
                point[other_axes[1]] = np.sin(angle) * radius
                ring_points.append(point)
            return np.array(ring_points)

        mag_ring_max = create_ring_points(mag_margin_max, mag_data, axis, num_points, False)
        mag_ring_min = create_ring_points(mag_margin_min, mag_data, axis, num_points, False)
        accel_ring_max = create_ring_points(accel_margin_max, accel_data, axis, num_points, True)
        accel_ring_min = create_ring_points(accel_margin_min, accel_data, axis, num_points, True)

        new_mag_points.append(mag_ring_max)
        new_mag_points.append(mag_ring_min)
        new_accel_points.append(accel_ring_max)
        new_accel_points.append(accel_ring_min)

    mag_augmented = np.vstack([mag_data] + new_mag_points)
    accel_augmented = np.vstack([accel_data] + new_accel_points)

    return mag_augmented, accel_augmented

def generate_synthetic_motion_data(num_samples=2000,
                                   accel_rot_x_deg=15,
                                   accel_rot_y_deg=15,
                                   accel_rot_z_deg=15,
                                   gravity=1.0,
                                   mag_strength=50732.4e-9):
    """
    Generates synthetic accelerometer and magnetometer data using the same
    direction vectors but different magnitudes. Then applies independent
    rotation offsets to the accelerometer around X, Y, and Z axes.

    accel_rot_x_deg/y_deg/z_deg:
        Rotation angles in degrees around the respective axes.
    """

    # 1. Generate uniform random direction vectors on a sphere
    phi = np.random.uniform(0, 2 * np.pi, num_samples)
    costheta = np.random.uniform(-1, 1, num_samples)
    theta = np.arccos(costheta)

    unit_vectors = np.vstack((
        np.sin(theta) * np.cos(phi),
        np.sin(theta) * np.sin(phi),
        np.cos(theta)
    )).T

    # 2. Scale vectors for magnetometer and accelerometer
    mag_data = unit_vectors * mag_strength
    accel_data = unit_vectors * gravity

    # 3. Augment both datasets (add circle outlines, etc.)
    mag_augmented, accel_augmented = add_synced_edge_circles(mag_data, accel_data)

    # 4. Apply independent rotations to accelerometer data AFTER augmentation
    offset_rotation = R.from_euler(
        'xyz',
        [accel_rot_x_deg, accel_rot_y_deg, accel_rot_z_deg],
        degrees=True
    )
    accel_rotated = offset_rotation.apply(accel_augmented)

    # 5. Return the modified data
    return mag_augmented, accel_rotated

def select_valid_accel_samples(accel, mag, mag_thresh=0.15, var_thresh=0.001, window_size=20):
    accel_magnitude = np.linalg.norm(accel, axis=1)
    mean_val = np.mean(accel_magnitude)
    std_val = np.std(accel_magnitude)
    logger.info(f"Accel magnitude mean={mean_val:.4f}, std={std_val:.4f}")

    # Auto-detect if accelerometer data is in g-units or m/s²
    if mean_val < 2:
        g_ref = 1.0
        logger.info("Detected accelerometer data in g-units (~1.0).")
    else:
        g_ref = 9.81
        logger.info("Detected accelerometer data in m/s² (~9.8).")

    # Magnitude-based filtering
    low_bound = g_ref * (1 - mag_thresh)
    high_bound = g_ref * (1 + mag_thresh)
    valid_mag_mask = (accel_magnitude > low_bound) & (accel_magnitude < high_bound)
    logger.debug(f"Magnitude range accepted: {low_bound:.3f}-{high_bound:.3f}")
    logger.info("Samples passing magnitude check: %d / %d",
                np.sum(valid_mag_mask), len(accel_magnitude))

    # Variance-based filtering (smoothness within local window)
    N = len(accel_magnitude)
    valid_var_mask = np.zeros(N, dtype=bool)
    half_win = window_size // 2
    for i in range(half_win, N - half_win):
        window = accel_magnitude[i - half_win:i + half_win + 1]
        if np.var(window) < var_thresh:
            valid_var_mask[i] = True

    valid_mask = valid_mag_mask & valid_var_mask
    logger.info("Valid samples after both checks: %d / %d",
                np.sum(valid_mask), N)

    if np.sum(valid_mask) == 0:
        logger.warning("No valid samples found -> try increasing mag_thresh or var_thresh.")
        sample_info = f"min={np.min(accel_magnitude):.4f}, max={np.max(accel_magnitude):.4f}"
        logger.warning("Observed accelerometer magnitude range: %s", sample_info)
    else:
        logger.debug("Example accepted magnitude: %.4f", np.mean(accel_magnitude[valid_mask]))

    return mag[valid_mask], accel[valid_mask]

def load_data(filename):
    mag1 = []
    mag2 = []
    accel = []
    print(f"Loading data file: {filename}")
    with open(filename, 'r') as f:
        line_index = 0
        for line in f:
            parts = line.strip().strip(';').split(';')
            parts = list(map(float, parts))
            mag1.append(parts[0:3])
            mag2.append(parts[3:6])
            accel.append(parts[6:9])
            line_index = line_index + 1

    mag1 = np.array(mag1)
    mag2 = np.array(mag2)
    accel = np.array(accel)

    return mag1, mag2, accel


# ------------------------------------------------------------------
# Step 1: Fix magnetometer data soft iron and hard iron errors
# ------------------------------------------------------------------
def fit_ellipsoid(ym):
    ym = np.asarray(ym, dtype=float)
    # Separate x, y, z components from the input data points
    x, y, z = ym[:, 0], ym[:, 1], ym[:, 2]

    # Construct the design matrix D for least squares ellipsoid fitting.
    # Each row corresponds to [x^2, y^2, z^2, xy, xz, yz, x, y, z, 1]
    D = np.column_stack([x*x, y*y, z*z, x*y, x*z, y*z, x, y, z, np.ones_like(x)])

    # Solve the homogeneous system D * sol = 0 using SVD.
    # The solution vector sol corresponds to the ellipsoid parameters,
    # with the smallest singular value vector (last row of V^T).
    _, _, vh = np.linalg.svd(D, full_matrices=False)
    sol = vh[-1, :] / norm(vh[-1, :])  # Normalize solution vector

    # Extract the quadratic form matrix A from the solution vector.
    # A represents the ellipsoid shape and orientation.
    A = np.array([
        [sol[0], sol[3]/2, sol[4]/2],
        [sol[3]/2, sol[1], sol[5]/2],
        [sol[4]/2, sol[5]/2, sol[2]]
    ])

    # Extract vector b and scalar c corresponding to linear and constant terms.
    b_vec = sol[6:9] / 2
    c = sol[9]

    # Ensure A is positive definite (the ellipsoid matrix).
    # If all eigenvalues are negative, flip the signs to make positive definite.
    eigvals = eigvalsh(A)
    if np.all(eigvals < 0):
        A, b_vec, c = -A, -b_vec, -c

    # Calculate the ellipsoid center (offset) as -A^{-1} * b_vec.
    # This is the hard iron offset representing the magnetometer bias.
    o0 = -inv(A) @ b_vec

    # Return the ellipsoid shape matrix A and center offset o0.
    return A, o0

def compute_soft_iron_from_D0(D0):
    # Perform eigendecomposition of the distortion matrix D0.
    # D0 represents the soft iron distortion in quadratic form.
    eigvals, eigvecs = eigh(D0)

    # Clamp eigenvalues to a small positive threshold to avoid negative or zero values,
    # which ensures the square root calculation remains valid and numerical stability.
    threshold = 1e-8
    eigvals_clamped = np.where(eigvals > threshold, eigvals, threshold)

    # Construct the soft iron correction matrix by taking the square root of the clamped eigenvalues.
    # This creates a scaling matrix that linearizes the effect of soft iron distortion.
    sqrt_eigvals = np.sqrt(eigvals_clamped)

    # Recompose the soft iron matrix from eigenvectors and the scaled eigenvalues.
    # This matrix will be used to correct distorted magnetometer data.
    soft_iron_matrix = eigvecs @ np.diag(sqrt_eigvals) @ eigvecs.T

    # Enforce symmetry to mitigate numerical asymmetry from eigen decomposition.
    soft_iron_matrix = (soft_iron_matrix + soft_iron_matrix.T) / 2

    return soft_iron_matrix

def fix_scale_of_soft_iron(hard_iron, soft_iron_matrix, original_data, magnetic_field_strength):

    # Apply calibrations as they are
    data_hard_corrected = original_data.copy() - hard_iron
    data_calibrated = np.dot(data_hard_corrected, soft_iron_matrix.T)

    # Compute magnitudes of each vector
    magnitudes = np.linalg.norm(data_calibrated, axis=1)

    # Compute mean magnitude
    mean_magnitude = np.mean(magnitudes)

    # Scale the measured magnitude by the magnetic field to find how to correct it
    magnitude_scale_fix =  magnetic_field_strength / mean_magnitude

    # Apply correction
    soft_iron_matrix = soft_iron_matrix * magnitude_scale_fix

    return soft_iron_matrix


# ------------------------------------------------------------------
# Step 2: Find alignment
# ------------------------------------------------------------------

def find_alignment_with_calibrated_magnetometer(ym, yz):
    ym_n = ym / (np.linalg.norm(ym, axis=1, keepdims=True) + 1e-12)
    yz_n = yz / (np.linalg.norm(yz, axis=1, keepdims=True) + 1e-12)

    def objective(params):
        q = params[:4]
        d = params[4]
        q = q / np.linalg.norm(q)
        Rmat = R.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
        transformed = ym_n @ Rmat
        dots = np.einsum('ij,ij->i', yz_n, transformed)
        resids = d - dots
        return np.dot(resids, resids)

    x0 = np.array([0., 0., 0., 1., 0.])  # identity quaternion + d=0 initial

    start_time = time.perf_counter()
    res = minimize(objective, x0, method='BFGS')
    end_time = time.perf_counter()

    q_opt = res.x[:4]
    q_opt /= np.linalg.norm(q_opt)
    d_opt = res.x[4]
    rotation_matrix = R.from_quat([q_opt[1], q_opt[2], q_opt[3], q_opt[0]]).as_matrix()

    # print(f"Seconds to find rotation matrix: {end_time - start_time:.4f} seconds for {len(ym)} samples")

    return rotation_matrix

def print_soft_and_hard_iron_calibrations(hard_iron, soft_iron_matrix, print_c=True, print_python=True):

    if print_python:
        print("magnetometer_hard_iron_correction = np.array(")
        print(f"    [{hard_iron.tolist()[0]:.6f}, {hard_iron.tolist()[1]:.6f}, {hard_iron.tolist()[2]:.6f}]")
        print(")")
        print("magnetometer_soft_iron_correction = np.array([")
        for row in soft_iron_matrix:
            print("  [" + ", ".join(f"{x:.6f}" for x in row) + "],")
        print("])")
        
    if print_c:
        print()

        print("float hard_iron_offset[3] = {" + ", ".join(f"{x:.6f}f" for x in hard_iron) + "};")
        print("float soft_iron_offset[3][3] = {")
        for row in soft_iron_matrix:
            print("  {" + ", ".join(f"{x:.6f}f" for x in row) + "},")
        print("};")

def print_rotation_matrix(rotation_matrix, print_c=True, print_python=True):

    if print_python:
        print("magnetometer_rotation_correction = np.array([")
        for row in rotation_matrix:
            print("  [" + ", ".join(f"{x:.6f}" for x in row) + "],")
        print("])")

    if print_c:
        print()

        print("float magnetometer_rotation_correction[3][3] = {")
        for row in rotation_matrix:
            print("  {" + ", ".join(f"{x:.6f}f" for x in row) + "},")
        print("};")

def calibrate_magnetometer_data(magnetometer_data, magnetic_field_strength, perform_calibration):
    magnetometer_data_copy = np.array(magnetometer_data, copy=True)

    if perform_calibration:
        # print("Performing calibration of magnetometer data")

        distortion_matrix, hard_iron_correction = fit_ellipsoid(magnetometer_data_copy)
        soft_iron_correction = compute_soft_iron_from_D0(distortion_matrix)
        soft_iron_correction = fix_scale_of_soft_iron(hard_iron_correction, soft_iron_correction, magnetometer_data_copy, magnetic_field_strength)

        magnetometer_data_hard_iron_corrected = magnetometer_data.copy() - hard_iron_correction
        magnetometer_data_calibrated = np.dot(magnetometer_data_hard_iron_corrected, soft_iron_correction.T)
        magnetometer_data_copy = magnetometer_data_calibrated

        print_soft_and_hard_iron_calibrations(hard_iron_correction, soft_iron_correction, print_c=True, print_python=False)
        
    else:
        # print("NOT performing calibration of magnetometer data")

        soft_iron_correction = np.eye(3)
        hard_iron_correction = np.zeros(3)
    
    return magnetometer_data_copy, soft_iron_correction, hard_iron_correction



def find_rotation_matrix(magnetometer_data, accelerometer_data):
    rotation_matrix = find_alignment_with_calibrated_magnetometer(magnetometer_data, accelerometer_data)
    print_rotation_matrix(rotation_matrix.T, print_c=True, print_python=False)

    return rotation_matrix

def apply_rotation(magnetometer_data, rotation_matrix):
    """
    Applies ellipsoid calibration and alignment rotation to raw magnetometer data.
    Ensures output is scaled to unit sphere.
    """

    return (rotation_matrix.T @ magnetometer_data.T).T


def plot_magnetometer_data(list_of_data_sets, list_of_labels):
    """
    Plots the magnetometer data in 2D plots for all column combinations.
    """

    color_map = plt.get_cmap('tab10')

    # 2D plots: all column combinations
    fig2d, axes = plt.subplots(1, 3, figsize=(15, 5))
    num_cols = 3
    plot_count = 0
    for i in range(num_cols):
        for j in range(i + 1, num_cols):
            ax = axes[plot_count]
            for k, data in enumerate(list_of_data_sets):
                ax.plot(data[:, i], data[:, j], 'o', markersize=0.5, label=f"{list_of_labels[k]}", color=color_map(k % len(list_of_data_sets)))  # Smaller dots

            ax.set_aspect('equal', adjustable='box')
            ax.set_xlabel(f'Column {i+1}')
            ax.set_ylabel(f'Column {j+1}')
            ax.set_title(f'Col {i+1} vs Col {j+1}')
            ax.legend(fontsize='small', markerscale=4)  # Show legend with scaled markers
            plot_count += 1
    fig2d.tight_layout()

def plot_unit_vector_arrows_2d(list_of_vectors, list_of_labels):
    """
    Plot unit vectors as arrows on 2D plots for XY, XZ, and YZ planes.
    Each vector is drawn from origin (0,0) to (x,y) in the respective plane.
    list_of_vectors: list of numpy arrays with shape (N,3) for N vectors.
    list_of_labels: list of labels for each vector set.
    """
    color_map = plt.get_cmap('tab10')
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))

    planes = [(0, 1), (0, 2), (1, 2)]  # XY, XZ, YZ
    plane_titles = ['XY plane', 'XZ plane', 'YZ plane']

    for plot_idx, (i, j) in enumerate(planes):
        ax = axes[plot_idx]
        for k, vectors in enumerate(list_of_vectors):
            for vec in vectors:
                ax.arrow(0, 0, vec[i], vec[j],
                         head_width=0.05, head_length=0.1,
                         fc=color_map(k % 10), ec=color_map(k % 10),
                         length_includes_head=True)

        ax.set_xlim(-1.2, 1.2)
        ax.set_ylim(-1.2, 1.2)
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel(f"Axis {i}")
        ax.set_ylabel(f"Axis {j}")
        ax.set_title(plane_titles[plot_idx])
        ax.legend(list_of_labels, fontsize='small')

    fig.tight_layout()


def extract_and_normalize_random_sample(mag1, mag2, accel):
    idx = random.randint(0, len(mag1) - 1)
    print(f"Random sample index chosen: {idx}")

    mag1_unit = mag1[idx] / np.linalg.norm(mag1[idx])
    mag2_unit = mag2[idx] / np.linalg.norm(mag2[idx])
    accel_unit = accel[idx] / np.linalg.norm(accel[idx])

    print(f"MAG1 unit vector: {mag1_unit}")
    print(f"MAG2 unit vector: {mag2_unit}")
    print(f"ACCEL unit vector: {accel_unit}")

    # Wrap single vectors in 2D arrays for compatibility with plotting function
    return np.array([mag1_unit]), np.array([mag2_unit]), np.array([accel_unit])

def check_axis_errors(mag1, mag2):
    axes = [0, 1, 2]
    axis_names = ['X', 'Y', 'Z']
    best_error = np.inf
    best_config = None

    # Try all permutations of axes and sign flips on BMM sensor data
    for perm in itertools.permutations(axes):
        for signs in itertools.product([-1, 1], repeat=3):
            corrected = mag1[:, perm] * signs
            error = np.sum((corrected - mag2) ** 2)
            if error < best_error:
                best_error = error
                best_config = (perm, signs)

    perm, signs = best_config

    print("Best axis matching from MMC5603 to BMM350 axes:")
    for i, axis in enumerate(perm):
        sign = signs[i]
        sign_str = '' if sign == 1 else '-'
        print(f"MMC5603 {axis_names[i]} axis matches BMM350 {sign_str}{axis_names[axis]} axis")

def flip_sensor_data_upside_down_np(data):
    # data is (N,3) numpy array

    # Create an output array
    flipped = np.zeros_like(data)

    flipped[:, 0] = -data[:, 1]  # -y becomes x
    flipped[:, 1] = -data[:, 0]  # -x becomes y
    flipped[:, 2] = -data[:, 2]  # -z stays z
    return flipped

def fix_sensor_data_180roll(data):
    # data is (N,3) numpy array

    # Create an output array
    flipped = np.zeros_like(data)

    flipped[:, 0] = data[:, 0]  # -y becomes x
    flipped[:, 1] = -data[:, 1]  # -x becomes y
    flipped[:, 2] = -data[:, 2]  # -z stays z
    return flipped

def filter_data_using_corrected_accelerometer_data(mag_array_list, accel_array, accel_threshold=0.3, outlier_std=3, sudden_jump_threshold=0.5):
    """
    Filters multiple magnetometer data arrays by shared accelerometer data.

    Parameters:
    - mag_array_list: list of numpy arrays with shape (N, 3) for magnetometers
    - accel_array: numpy array with shape (N, 3) for accelerometer data in g units
    - accel_threshold: allowed deviation from 1g (recommended ~0.3 for tolerance)
    - outlier_std: z-score threshold for outlier removal
    - sudden_jump_threshold: threshold to detect sudden changes in accel

    Returns:
    - filtered_mag_array_list: list of filtered magnetometer numpy arrays
    - filtered_accel_array: filtered accelerometer numpy array
    """
    # Calculate magnitude of accelerometer data
    accel_mag = np.linalg.norm(accel_array, axis=1)

    # Filter: acceleration magnitude near 1g ± threshold
    valid_mask = (accel_mag > (1 - accel_threshold)) & (accel_mag < (1 + accel_threshold))

    # Outlier removal on filtered acceleration magnitudes
    filtered_mag = accel_mag[valid_mask]
    mean_mag = np.mean(filtered_mag)
    std_mag = np.std(filtered_mag)
    z_scores = (filtered_mag - mean_mag) / std_mag
    outlier_mask = np.abs(z_scores) < outlier_std

    # Compose final mask indices
    valid_indices = np.where(valid_mask)[0]
    filtered_indices = valid_indices[outlier_mask]

    # Removing sudden jumps - based on difference between consecutive accelerometer vectors
    accel_diff = np.linalg.norm(np.diff(accel_array, axis=0), axis=1)
    sudden_jump_mask = np.insert(accel_diff < sudden_jump_threshold, 0, True)  # Keep the first sample

    # Combine masks to get final indices
    final_indices = np.intersect1d(filtered_indices, np.where(sudden_jump_mask)[0])

    # Apply final indices to accelerometer data
    filtered_accel = accel_array[final_indices]

    # Apply final indices to each magnetometer data array
    filtered_mag_data_list = [mag_data[final_indices] for mag_data in mag_array_list]

    return filtered_mag_data_list, filtered_accel

def main():
    parser = argparse.ArgumentParser(description="Magnetometer calibration with hard and soft iron correction")
    parser.add_argument("filename", help="Path to raw magnetometer data file")
    parser.add_argument("magfield", type=float, help="Local magnetic field strength in microteslas (µT)")
    parser.add_argument('--simulate', action='store_true', help='i dont care')
    parser.add_argument('--plot', action='store_true', help='i dont care')
    parser.add_argument('--calibrate', action='store_true', help='i dont care')
    parser.add_argument('--dont-rotate', action='store_true', help='i dont care')
    parser.add_argument('--error-check', action='store_true', help='i dont care')

    args = parser.parse_args()

    if args.simulate:
        mag, accel = generate_synthetic_motion_data()
        mag2 = mag.copy()
    else:
        mag, mag2, accel = load_data(args.filename)

    print("#################################### MAG1")
    calibrated_magnetometer_data1, soft_iron_correction1, hard_iron_correction1 = calibrate_magnetometer_data(mag, args.magfield, args.calibrate)

    
    if args.dont_rotate == False:
        filtered_mag1, filtered_accelerometer = filter_data_using_corrected_accelerometer_data([calibrated_magnetometer_data1], accel)
        calibrated_magnetometer_data1 = filtered_mag1[0]

        rotation_matrix1 = find_rotation_matrix(calibrated_magnetometer_data1, filtered_accelerometer)
        calibrated_rotated_magnetometer_data1 = apply_rotation(calibrated_magnetometer_data1, rotation_matrix1)

        euler_angles_deg1 = R.from_matrix(rotation_matrix1.T).as_euler('xyz', degrees=True)

        print(f"\nCalibration rotation angles around axes for MAG1:")
        print(f"  X axis: {euler_angles_deg1[0]:.2f} degrees")
        print(f"  Y axis: {euler_angles_deg1[1]:.2f} degrees")
        print(f"  Z axis: {euler_angles_deg1[2]:.2f} degrees")

    else:
        calibrated_rotated_magnetometer_data1 = calibrated_magnetometer_data1
    print()
    print("######### END MAG1")

    print("#################################### MAG2")
    calibrated_magnetometer_data2, soft_iron_correction2, hard_iron_correction2 = calibrate_magnetometer_data(mag2, args.magfield, args.calibrate)

    if args.dont_rotate == False:
        filtered_mag2, filtered_accelerometer = filter_data_using_corrected_accelerometer_data([calibrated_magnetometer_data2], accel)
        calibrated_magnetometer_data2 = filtered_mag2[0]
        accel = filtered_accelerometer

        rotation_matrix2 = find_rotation_matrix(calibrated_magnetometer_data2, accel)
        calibrated_rotated_magnetometer_data2 = apply_rotation(calibrated_magnetometer_data2, rotation_matrix2)

        euler_angles_deg2 = R.from_matrix(rotation_matrix2.T).as_euler('xyz', degrees=True)

        print(f"\nCalibration rotation angles around axes for MAG2:")
        print(f"  X axis: {euler_angles_deg2[0]:.2f} degrees")
        print(f"  Y axis: {euler_angles_deg2[1]:.2f} degrees")
        print(f"  Z axis: {euler_angles_deg2[2]:.2f} degrees")

    else:
        calibrated_rotated_magnetometer_data2 = calibrated_magnetometer_data2
    print()
    print("######### END MAG2")

    if args.error_check:
        print()
        print("Unrotated data:")
        check_axis_errors(calibrated_magnetometer_data1, calibrated_magnetometer_data2)
        print()
        print("Rotated data:")
        check_axis_errors(calibrated_rotated_magnetometer_data1, calibrated_rotated_magnetometer_data2)

    mag1_unit, mag2_unit, accel_unit = extract_and_normalize_random_sample(calibrated_rotated_magnetometer_data1, calibrated_rotated_magnetometer_data2, accel)
    plot_unit_vector_arrows_2d([mag1_unit, mag2_unit, accel_unit], ["mag1_unit", "mag2_unit", "accel_unit"])
 
    if args.plot:
        plot_magnetometer_data([mag, calibrated_magnetometer_data1, calibrated_rotated_magnetometer_data1], ["mag", "cal_mag_data1", "cal_rot_mag_data1"])
        plot_magnetometer_data([mag2, calibrated_magnetometer_data2, calibrated_rotated_magnetometer_data2], ["mag", "cal_mag_data2", "cal_rot_mag_data2"])

        calibrated_rotated_magnetometer_data1_unit = calibrated_rotated_magnetometer_data1 / np.median(np.linalg.norm(calibrated_rotated_magnetometer_data1, axis=1))
        calibrated_rotated_magnetometer_data2_unit = calibrated_rotated_magnetometer_data2 / np.median(np.linalg.norm(calibrated_rotated_magnetometer_data2, axis=1))
        accel_unit = accel / np.median(np.linalg.norm(accel, axis=1))

        plot_magnetometer_data([calibrated_rotated_magnetometer_data1_unit, calibrated_rotated_magnetometer_data2_unit, accel_unit], ["calibrated_rotated_magnetometer_data1_unit", "calibrated_rotated_magnetometer_data2_unit", "accel_unit"])
        
        plt.show()

if __name__ == "__main__":
    main()
