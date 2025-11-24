import argparse
import string

def is_printable_line(line):
    # Return False if any character is not printable (except newline/tab)
    return all(ch in string.printable or ch in '\n\r\t' for ch in line)

def count_columns(line):
    # Count semicolon-separated values
    return len([col for col in line.strip().split(';') if col])

def remove_invalid_lines_in_place(file_path):
    try:
        with open(file_path, 'r', encoding='utf-8', errors='replace') as file:
            lines = file.readlines()

        if not lines:
            print("File is empty.")
            return

        # Preserve the first line (assumed to be a comment or header)
        header = lines[0]
        data_lines = lines[1:]

        # Determine the expected column count from the first *valid* line
        expected_col_count = None
        for line in data_lines:
            if "NAN" in line or not is_printable_line(line):
                continue
            col_count = count_columns(line)
            if col_count > 0:
                expected_col_count = col_count
                break

        if expected_col_count is None:
            print("No valid lines to determine column count.")
            return

        cleaned_lines = []
        for line in data_lines:
            if "NAN" in line:
                continue
            if not is_printable_line(line):
                continue
            if count_columns(line) != expected_col_count:
                continue
            cleaned_lines.append(line)

        with open(file_path, 'w', encoding='utf-8') as file:
            file.write(header)
            file.writelines(cleaned_lines)

        print(f"Cleaned {file_path}: kept {len(cleaned_lines)} valid lines out of {len(data_lines)}")

    except Exception as e:
        print(f"An error occurred: {e}")

def main():
    parser = argparse.ArgumentParser(description="Clean a CSV-like file from 'NAN's, invalid characters, and bad column counts.")
    parser.add_argument('file_path', type=str, help='Path to the file to be cleaned')
    args = parser.parse_args()
    remove_invalid_lines_in_place(args.file_path)

if __name__ == "__main__":
    main()
