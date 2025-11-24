import argparse

def is_numeric(s: str) -> bool:
    """Check if a string is numeric."""
    if not s:
        return False
    try:
        float(s)
        return True
    except ValueError:
        return False

def is_header_line(line: str) -> bool:
    """Determine if a line is likely a header based on its content."""
    fields = line.strip().split(';')
    if len(fields) <= 1:
        return False

    non_numeric_count = 0
    for field in fields:
        field = field.strip()
        if not field:  # empty field
            continue
        if not is_numeric(field):
            non_numeric_count += 1

    if non_numeric_count > len(fields) / 2 or (fields and not is_numeric(fields[0])):
        return True
    return False

def read_csv_like(file_path: str):
    """Read semicolon CSV into list-of-lists of strings, ignoring header if present."""
    rows = []
    with open(file_path, "r") as f:
        first_line = f.readline()
        has_header = is_header_line(first_line)

    with open(file_path, "r") as f:
        for idx, line in enumerate(f):
            line = line.strip()
            if not line:
                continue
            if line.endswith(";"):
                line = line[:-1]  # strip trailing ;
            fields = line.split(";")

            if idx == 0 and has_header:
                continue
            rows.append(fields)
    return rows

def main():
    parser = argparse.ArgumentParser(description="Compute column averages for selected rows in a semicolon CSV.")
    parser.add_argument("file", help="Path to the CSV file")
    parser.add_argument("--start-index", type=int, required=True, help="Start row index (0-based, after header)")
    parser.add_argument("--end-index", type=int, required=True, help="End row index (inclusive, after header)")
    args = parser.parse_args()

    rows = read_csv_like(args.file)

    # Select rows
    selected = rows[args.start_index : args.end_index + 1]

    # Debug: print selected rows
    # for row in selected:
    #     print(";".join(row))

    # Compute averages
    num_cols = len(selected[0])
    sums = [0.0] * num_cols
    counts = [0] * num_cols

    for row in selected:
        for i, val in enumerate(row):
            try:
                num = float(val)
                sums[i] += num
                counts[i] += 1
            except ValueError:
                pass  # ignore non-numeric

    averages = []
    for i in range(num_cols):
        if counts[i] > 0:
            averages.append(f"{sums[i] / counts[i]:.2f}")  # 2 decimal places
        else:
            averages.append("")

    print(";".join(averages))

if __name__ == "__main__":
    main()
