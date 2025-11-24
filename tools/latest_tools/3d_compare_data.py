import pandas as pd
import matplotlib.pyplot as plt
import argparse

def is_header_line(line):
    """Determine if a line is likely a header based on its content."""
    fields = line.strip().split(';')
    # If the line has only one field, it's probably not a header
    if len(fields) <= 1:
        return False

    # Count how many fields are non-numeric
    non_numeric_count = 0
    for field in fields:
        field = field.strip()
        if not field:  # empty field
            continue
        # Try to convert to float
        try:
            float(field)
            # If successful, it's numeric
        except ValueError:
            non_numeric_count += 1

    # If more than half the fields are non-numeric, it's probably a header
    # Also, if the first field is non-numeric, it's more likely to be a header
    if non_numeric_count > len(fields) / 2 or (len(fields) > 0 and not is_numeric(fields[0].strip())):
        return True
    return False

def is_numeric(s):
    """Check if a string is numeric."""
    if not s:
        return False
    try:
        float(s)
        return True
    except ValueError:
        return False

def do(file_path, use_cols=None, use_cols2=None):
    # First, read the first line to check if it's a header
    with open(file_path, 'r') as f:
        first_line = f.readline()

    has_header = is_header_line(first_line)

    # Now read the entire file with pandas
    if has_header:
        df = pd.read_csv(file_path, delimiter=';')
    else:
        # No header, so we'll use default column names and skip no rows
        df = pd.read_csv(file_path, delimiter=';', header=None)


    # Check if all indices are within the valid range
    max_index = df.shape[1] - 1
    for idx in use_cols:
        if idx > max_index:
            print(f"Column index {idx} does not exist.")
            return

    if use_cols2 is not None:
            
        for idx in use_cols2:
            if idx > max_index:
                print(f"Column index {idx} does not exist.")
                return

        # Extract header names and make them one string
        selected_headers1 = [df.columns[i] for i in use_cols]
        vector_name1 = ", ".join(selected_headers1)

        selected_headers2 = [df.columns[i] for i in use_cols2]
        vector_name2 = ", ".join(selected_headers2)

        # Extract only the desired columns
        vectors1 = df.iloc[:, use_cols].to_numpy()
        vectors2 = df.iloc[:, use_cols2].to_numpy()


        # print(vectors1)
        # print(vectors2)
        plot_data_3d([vectors1, vectors2], [vector_name1, vector_name2])
    else:

        # Extract header names and make them one string
        selected_headers = [df.columns[i] for i in use_cols]
        vector_name = ", ".join(selected_headers)


        # Extract only the desired columns
        vectors = df.iloc[:, use_cols].to_numpy()


        # print(vectors)
        plot_data_3d([vectors], [vector_name])
    plt.show()

def plot_data_3d(list_of_data_sets, list_of_labels):
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

def main():
    parser = argparse.ArgumentParser(description="Plot selected columns from a semicolon-separated CSV.")
    parser.add_argument('file_path', type=str, help='Path to the CSV file')
    parser.add_argument('--use-cols', type=str, default='', help='Comma-separated list of column indices to skip (e.g., 6,7)')
    parser.add_argument('--use-cols2', type=str, default='', help='Comma-separated list of column indices to skip (e.g., 6,7)')

    args = parser.parse_args()
    # Convert skip-cols string to list of ints
    use_cols = [int(x) for x in args.use_cols.split(',')] if args.use_cols else None
    use_cols2 = [int(x) for x in args.use_cols2.split(',')] if args.use_cols2 else None

    if len(use_cols) != 3:
        print("Too little columns selected")
        return

    do(args.file_path, use_cols, use_cols2)




if __name__ == "__main__":
    main()
