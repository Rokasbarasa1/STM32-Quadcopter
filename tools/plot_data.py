import numpy as np
import matplotlib.pyplot as plt
import argparse

def do(file_path, min_column=0, max_column=None):
    # Load all columns using ; delimiter
    data = np.genfromtxt(file_path, delimiter=';')

    if data.ndim == 1:
        data = data.reshape(-1, len(data))  # Handle single-line case

    num_columns = data.shape[1]

    # Validate column bounds
    min_column = max(0, min_column)
    max_column = num_columns if max_column is None else min(num_columns, max_column)

    # Slice the relevant columns
    data = data[:, min_column:max_column]

    # Plot all selected columns
    plt.figure(figsize=(12, 6))
    for i in range(data.shape[1]):
        plt.plot(data[:, i], label=f"Column {min_column + i}")

    plt.title("CSV Column Plot")
    plt.xlabel("Row Index")
    plt.ylabel("Value")
    plt.legend(loc='upper right')
    plt.grid(True)
    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Plot selected columns from a semicolon-separated CSV.")
    parser.add_argument('file_path', type=str, help='Path to the CSV file')
    parser.add_argument('--min-column', type=int, default=0,
                        help='Start plotting from this column index (inclusive)')
    parser.add_argument('--max-column', type=int, default=None,
                        help='Only plot columns up to this index (exclusive)')

    args = parser.parse_args()
    do(args.file_path, args.min_column, args.max_column)

if __name__ == "__main__":
    main()