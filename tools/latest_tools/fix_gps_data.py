import argparse

ILLEGAL_CHARS = set("!\"#$&'*,/:<=>?@[\\^${|}~`")  # Illegal characters, no space/tab/newline

def has_illegal_characters(line):
    for ch in line:
        if ch in ILLEGAL_CHARS:
            return True
        if ch < ' ' and ch not in ('\t', '\n', '\r'):
            return True
    return False

def count_columns(line):
    return len(line.strip().split(';'))

def count_digits_before_after_decimal(value):
    if '.' in value:
        before_dec, after_dec = value.split('.', 1)
    else:
        before_dec, after_dec = value, ''
    digits_before = sum(c.isdigit() for c in before_dec)
    digits_after = sum(c.isdigit() for c in after_dec)
    return digits_before, digits_after

def get_most_common_digit_count(freq_dict):
    # Find the key with the highest frequency and its count
    max_freq = 0
    most_common_digit = None
    total = 0
    for digit_count, freq in freq_dict.items():
        total += freq
        if freq > max_freq:
            max_freq = freq
            most_common_digit = digit_count
    return most_common_digit, max_freq, total

def remove_invalid_lines_in_place(file_path):
    try:
        with open(file_path, 'r', encoding='utf-8', errors='replace') as file:
            lines = file.readlines()
        if not lines:
            print("File is empty.")
            return

        header = lines[0]
        data_lines = lines[1:]

        column_counts = []
        digits_before_freq = [{} for _ in range(4)]  # list of dicts for counts
        digits_after_freq = [{} for _ in range(4)]

        # First pass - gather data
        for line in data_lines:
            line_lower = line.lower()
            if 'nan' in line_lower or has_illegal_characters(line):
                continue
            cols = line.strip().split(';')
            col_count = len(cols)
            if col_count == 0:
                continue
            column_counts.append(col_count)

            for i in range(min(4, col_count)):
                val = cols[i]
                bef, aft = count_digits_before_after_decimal(val)
                # Count digits_before freq
                digits_before_freq[i][bef] = digits_before_freq[i].get(bef, 0) + 1
                # Count digits_after freq
                digits_after_freq[i][aft] = digits_after_freq[i].get(aft, 0) + 1

        if not column_counts:
            print("No valid lines to determine column count.")
            return

        # Determine expected column count (most common)
        col_count_freq = {}
        for c in column_counts:
            col_count_freq[c] = col_count_freq.get(c, 0) + 1
        expected_col_count = max(col_count_freq, key=col_count_freq.get)

        # Determine expected digits before decimal for first 4 columns:
        expected_before = []
        for i in range(4):
            most_common_digit, freq, total = get_most_common_digit_count(digits_before_freq[i])
            if most_common_digit is not None:
                if freq / total > 0.5:
                    expected_before.append(most_common_digit)
                else:
                    expected_before.append(8)  # default if no majority
            else:
                expected_before.append(8)  # default if no data

        expected_after = 1  # always expect 1 digit after decimal

        # Second pass - filter lines
        cleaned_lines = []
        for line in data_lines:
            line_lower = line.lower()
            if 'nan' in line_lower or has_illegal_characters(line):
                continue
            cols = line.strip().split(';')
            if len(cols) != expected_col_count:
                continue

            remove_line = False
            for i in range(4):
                if i >= len(cols):
                    continue
                bef, aft = count_digits_before_after_decimal(cols[i])
                if bef != expected_before[i] or aft != expected_after:
                    remove_line = True
                    break
            if remove_line:
                continue
            cleaned_lines.append(line)

        with open(file_path, 'w', encoding='utf-8') as file:
            file.write(header)
            file.writelines(cleaned_lines)

        print(f"Cleaned {file_path}: kept {len(cleaned_lines)} valid lines out of {len(data_lines)}")

    except Exception as e:
        print(f"An error occurred: {e}")

def main():
    parser = argparse.ArgumentParser(description="Clean CSV, remove nan, illegal chars, bad column count and corrupted lat/lon digit lengths.")
    parser.add_argument('file_path', type=str, help='Path to the file to be cleaned')
    args = parser.parse_args()
    remove_invalid_lines_in_place(args.file_path)

if __name__ == "__main__":
    main()
