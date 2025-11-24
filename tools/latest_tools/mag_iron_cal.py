import numpy as np
import argparse
from numpy.linalg import eigvalsh, norm, inv, eigh, eig
from scipy.linalg import sqrtm
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

use_simulated_data = False

def add_synced_edge_circles(mag_data, margin_fraction=0.05, num_points=200):
    """
    Adds ring data on sphere edges at X±, Y±, Z±.
    margin_fraction defines how far away from max/min edges the ring is placed.
    num_points is number of points in each ring.
    """
    new_mag_points = []
    # For axis 0,1,2 (X,Y,Z)
    for axis in range(3):
        # Compute min/max for mag on this axis
        mag_axis_data = mag_data[:, axis]
        mag_min, mag_max = np.min(mag_axis_data), np.max(mag_axis_data)
        mag_range = mag_max - mag_min
        mag_margin_max = mag_max - margin_fraction * mag_range
        mag_margin_min = mag_min + margin_fraction * mag_range

        def create_ring_points(center_value, data, axis, num_points):
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

        mag_ring_max = create_ring_points(mag_margin_max, mag_data, axis, num_points)
        mag_ring_min = create_ring_points(mag_margin_min, mag_data, axis, num_points)
        new_mag_points.append(mag_ring_max)
        new_mag_points.append(mag_ring_min)

    mag_augmented = np.vstack([mag_data] + new_mag_points)
    return mag_augmented

def generate_synthetic_mag_data(num_samples=2000, hard_iron_offset=None, soft_iron_matrix=None, mag_strength=50.0):
    """
    Generates synthetic magnetometer data with a perfect sphere, then adds edge circles,
    and finally applies hard iron offset and soft iron transformation.

    Parameters:
    - num_samples: Number of points on the sphere.
    - hard_iron_offset: The hard iron offset vector (e.g., [x, y, z]).
    - soft_iron_matrix: The soft iron transformation matrix (3x3).
    - mag_strength: The strength of the magnetic field.
    """
    if hard_iron_offset is None:
        hard_iron_offset = np.array([0.0, 0.0, 0.0])
    if soft_iron_matrix is None:
        soft_iron_matrix = np.eye(3)  # Identity matrix by default

    # 1. Generate uniform random direction vectors on a sphere
    phi = np.random.uniform(0, 2 * np.pi, num_samples)
    costheta = np.random.uniform(-1, 1, num_samples)
    theta = np.arccos(costheta)
    unit_vectors = np.vstack((
        np.sin(theta) * np.cos(phi),
        np.sin(theta) * np.sin(phi),
        np.cos(theta)
    )).T

    # 2. Scale vectors for magnetometer
    mag_data = unit_vectors * mag_strength

    # 3. Add edge circles to the sphere
    mag_data_with_circles = add_synced_edge_circles(mag_data)

    # 4. Apply soft iron matrix
    mag_data_transformed = np.dot(mag_data_with_circles, soft_iron_matrix.T)

    # 5. Apply hard iron offset
    mag_data_final = mag_data_transformed + hard_iron_offset

    return mag_data_final, mag_data_with_circles

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

def load_data_file(filename):
    data = []
    with open(filename, 'r') as f:
        for line in f:
            parts = line.strip().strip(';').split(';')
            if len(parts) == 3:
                try:
                    x, y, z = map(float, parts)
                    data.append([x, y, z])
                except ValueError:
                    pass
    data = np.array(data)

    return data

def print_arrays(hard_iron, distortion_matrix, print_c=True, print_python=True):
    if print_python:
        print("magnetometer_hard_iron_correction = np.array(")
        print(f"    [{hard_iron.tolist()[0]:.6f}, {hard_iron.tolist()[1]:.6f}, {hard_iron.tolist()[2]:.6f}]")
        print(")")
        print("magnetometer_soft_iron_correction = np.array([")
        for row in distortion_matrix:
            print("  [" + ", ".join(f"{x:.6f}" for x in row) + "],")
        print("])")
        
    if print_c:
        print()

        print("float hard_iron_offset[3] = {" + ", ".join(f"{x:.6f}f" for x in hard_iron) + "};")
        print("float distortion_matrix[3][3] = {")
        for row in distortion_matrix:
            print("  {" + ", ".join(f"{x:.6f}f" for x in row) + "},")
        print("};")


def main():
    parser = argparse.ArgumentParser(description="Magnetometer calibration with hard and soft iron correction")
    parser.add_argument("datafile", help="Path to raw magnetometer data file")
    parser.add_argument("magfield", type=float, help="Local magnetic field strength in microteslas (µT)")
    parser.add_argument('--simulate', action='store_true', help='i dont care')
    parser.add_argument('--plot', action='store_true', help='i dont care')
    parser.add_argument('--magneto', action='store_true', help='i dont care')
    parser.add_argument('--simulate-save', action='store_true', help='i dont care')


    args = parser.parse_args()

    expected_soft_iron = np.eye(3)
    expected_hard_iron = np.zeros(3)

    if args.simulate:
        hard_iron_offset = np.array([200.0, 300.0, 193.0])
        soft_iron_matrix = np.array([
            [1.3, 0.05, 0.02],
            [0.05, 1.2, 0.08],
            [0.02, 0.08, 1.1]
        ])

        # Generate synthetic magnetometer data
        data, data_clean = generate_synthetic_mag_data(
            num_samples=2000,
            hard_iron_offset=hard_iron_offset,
            soft_iron_matrix=soft_iron_matrix,
            mag_strength=args.magfield
        )

        if args.simulate_save:
            # Save to file
            with open('simulated.txt', 'w') as f:
                for point in data:
                    f.write("%f\t%f\t%f\t\n" % (point[0], point[1], point[2]))
        

    else:
        data = load_data_file(args.datafile)


    distortion_matrix, hard_iron = fit_ellipsoid(data)
    soft_iron = compute_soft_iron_from_D0(distortion_matrix)
    soft_iron = fix_scale_of_soft_iron(hard_iron, soft_iron, data, args.magfield)

    if args.simulate:
        print("======================== EXPECTED USING SIMULATION")
        expected_soft_iron = inv(soft_iron_matrix)
        expected_soft_iron = (expected_soft_iron + expected_soft_iron.T) / 2
        expected_hard_iron = hard_iron_offset
        print_arrays(-hard_iron_offset, expected_soft_iron, print_c=False)
        print("======================== EXPECTED USING SIMULATION")


    print_arrays(hard_iron, soft_iron, print_c=False)

    if args.plot:
        if args.simulate:


            data_hard_corrected1 = data.copy() - hard_iron
            data_calibrated1 = np.dot(data_hard_corrected1, soft_iron.T)

            data_hard_corrected2 = data.copy() - expected_hard_iron
            data_calibrated2 = np.dot(data_hard_corrected2, expected_soft_iron.T)

            plot_magnetometer_data([data_calibrated1, data_calibrated2, data_clean], ["calculated correction", "correction from simulation", "clean data"])

            if args.magneto:
                
                hard_iron3 = np.array([200.000000, 300.000000, 193.000000])
                soft_iron3 = np.array([
                    [0.770616, -0.031327, -0.011733],
                    [-0.031327, 0.838667, -0.060424],
                    [-0.011733, -0.060424, 0.913699]
                ])

                data_hard_corrected3 = data - hard_iron3
                data_calibrated3 = np.dot(data_hard_corrected3, soft_iron3.T)

                plot_magnetometer_data([data_calibrated1, data_calibrated3, data_clean], ["calculated correction", "correction from magneto", "clean data"])

            plt.show()
        else:
            data_hard_corrected1 = data.copy() - hard_iron
            data_calibrated1 = np.dot(data_hard_corrected1, soft_iron.T)

            plot_magnetometer_data([data_calibrated1], ["calculated correction"])
            
            if args.magneto:
                
                hard_iron3 = np.array([-24.470167, -2.145625, -5.809672])
                soft_iron3 = np.array([
                    [0.982082, -0.007454, -0.003948],
                    [-0.007454, 0.997291, -0.010888],
                    [-0.003948, -0.010888, 1.064393]
                ])

                data_hard_corrected3 = data.copy() - hard_iron3
                data_calibrated3 = np.dot(data_hard_corrected3, soft_iron3.T)

                plot_magnetometer_data([data_calibrated1, data_calibrated3], ["calculated correction", "correction from magneto"])
            plt.show()


if __name__ == "__main__":
    main()
