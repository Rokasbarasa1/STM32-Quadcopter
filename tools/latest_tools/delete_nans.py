import argparse

def remove_lines_with_nan_in_place(file_path):
    try:
        # Read the contents of the file
        with open(file_path, 'r', encoding='utf-8', errors='replace') as file:
            lines = file.readlines()

        # Filter out lines containing "NAN"
        cleaned_lines = [line for line in lines if "NAN" not in line]

        # Write the cleaned lines back to the same file
        with open(file_path, 'w', encoding='utf-8') as file:
            file.writelines(cleaned_lines)

        print(f"Lines containing 'NAN' removed from {file_path}")

    except Exception as e:
        print(f"An error occurred: {e}")

def main():
    parser = argparse.ArgumentParser(description="Remove lines containing 'NAN' from a file.")
    parser.add_argument('file_path', type=str, help='Path to the file to be cleaned')

    args = parser.parse_args()
    remove_lines_with_nan_in_place(args.file_path)

if __name__ == "__main__":
    main()