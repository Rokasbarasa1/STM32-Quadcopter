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

def do(file_path, min_column=0, max_column=None, skip_cols=None):
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

    # Get the number of columns
    num_columns = len(df.columns)

    # Validate column bounds
    min_column = max(0, min_column)
    max_column = num_columns if max_column is None else min(num_columns, max_column)

    # Determine columns to keep, excluding skipped ones
    if skip_cols is None:
        skip_cols = []
    selected_columns = [i for i in range(min_column, max_column) if i not in skip_cols]

    # Extract only the desired columns
    data_selected = df.iloc[:, selected_columns]

    # Plot all selected columns
    plt.figure(figsize=(12, 6))
    for i, col_index in enumerate(selected_columns):
        if has_header and col_index < len(df.columns):
            label = "#" + str(col_index) + " " + df.columns[col_index] 
        else:
            label = f"Column {col_index}"
        plt.plot(data_selected.iloc[:, i], label=label)

    plt.title(file_path)
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
    parser.add_argument('--skip-cols', type=str, default='',
                        help='Comma-separated list of column indices to skip (e.g., 6,7)')
    args = parser.parse_args()
    # Convert skip-cols string to list of ints
    skip_cols = [int(x) for x in args.skip_cols.split(',')] if args.skip_cols else None
    do(args.file_path, args.min_column, args.max_column, skip_cols)

if __name__ == "__main__":
    main()
