import argparse

def main():
    parser = argparse.ArgumentParser(description='Modify CSV file data.')
    parser.add_argument('input_file', type=str, help='Input CSV file path')
    parser.add_argument('output_file', type=str, help='Output CSV file path')
    args = parser.parse_args()

    # Read the CSV file line by line
    with open(args.input_file, 'r') as f:
        lines = f.readlines()

    data = []
    for line in lines:
        row = line.strip().split(';')
        # Convert each element to float if possible, otherwise keep as string
        converted_row = []
        for item in row:
            try:
                converted_row.append(float(item))
            except ValueError:
                converted_row.append(item)  # Keep as string if conversion fails
        data.append(converted_row)

    if not data:
        print("Warning: The input file is empty.")
        return

    # Target column 6 (index 5)
    main_column_index = 5  # The 6th column (0-based index)

    # Secondary column is column 5 (index 4)
    secondary_column_index = 4

    # Check if the specified columns exist
    max_columns = len(data[0]) if len(data) > 0 else 0
    if max_columns <= main_column_index or max_columns <= secondary_column_index:
        # If there are fewer columns than needed, find the best we can do
        actual_main_index = min(main_column_index, max_columns-1) if max_columns > 0 else 0
        actual_secondary_index = min(secondary_column_index, max_columns-1) if max_columns > 0 else 0
        if actual_main_index != main_column_index or actual_secondary_index != secondary_column_index:
            print(f"Warning: Data has only {max_columns} columns. Using columns {actual_main_index+1} and {actual_secondary_index+1} instead of 6 and 5.")
            main_column_index = actual_main_index
            secondary_column_index = actual_secondary_index

    # Initialize variables to store the reference values
    stored_main_col_value = 0
    stored_secondary_col_value = 0

    # Get the initial stored values once at the beginning
    if len(data) > 600:
        # For stored_main_col_value: use value from row 600, main_column_index (column 6)
        if main_column_index < len(data[600]) and isinstance(data[600][main_column_index], (int, float)):
            stored_main_col_value = data[600][main_column_index]
            print(f"Using stored main column value from row 600: {stored_main_col_value}")
        else:
            print("Warning: Could not read stored main column value from row 600, using default 0")
            stored_main_col_value = 0

        # For stored_secondary_col_value: use value from row 600, secondary_column_index (column 5)
        if secondary_column_index < len(data[600]) and isinstance(data[600][secondary_column_index], (int, float)):
            stored_secondary_col_value = data[600][secondary_column_index]
            print(f"Using stored secondary column value from row 600: {stored_secondary_col_value}")
        else:
            print("Warning: Could not read stored secondary column value from row 600, using default 0")
            stored_secondary_col_value = 0

        # Process each row after 600 (rows 601 onwards)
        for row in range(601, len(data)):
            if main_column_index < len(data[row]):
                current_value = data[row][main_column_index]
                if isinstance(current_value, (int, float)):
                    # Calculate the new value using the fixed stored values
                    new_value = (current_value - stored_main_col_value) + stored_secondary_col_value
                    # Apply modulo 360 to keep within range
                    new_value %= 360
                    data[row][main_column_index] = new_value

    # Write the modified data to the output CSV file with semicolon delimiter
    with open(args.output_file, 'w') as f:
        for row in data:
            # Convert each item to string, handling both numeric and string values
            str_row = []
            for item in row:
                if isinstance(item, (int, float)):
                    # Format floats to avoid scientific notation and maintain precision
                    if isinstance(item, float):
                        # Convert to string and remove trailing .0 for whole numbers
                        str_val = f"{item:.10f}".rstrip('0').rstrip('.') if '.' in f"{item:.10f}" else str(int(item))
                        str_row.append(str_val)
                    else:
                        str_row.append(str(item))
                else:
                    str_row.append(str(item))
            line = ';'.join(str_row) + '\n'
            f.write(line)

if __name__ == '__main__':
    main()
