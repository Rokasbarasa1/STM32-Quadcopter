import os
import sys

# Append bytes to the end of a betaflight file that signifies the end of the file as the drone cant do that before it is turned off.

def find_and_append(file_name, append_text):
    found = False  # Flag to check if file is found
    # Search for the file in the current directory
    for root, dirs, files in os.walk('.'):
        for file in files:
            if file.lower() == file_name.lower():  # Case-insensitive comparison
                file_path = os.path.join(root, file)
                print(f"File found: {file_path}")
                found = True
                # Open the file and append text
                with open(file_path, 'ab') as file:
                    file.write(append_text.encode())
                    print("Text appended successfully.")
                break
        if found:
            break

    if not found:
        print("File not found. Make sure you are in the correct directory.")

def main():
    if len(sys.argv) < 2:
        print("Usage: python script.py <filename>")
        sys.exit(1)
    
    file_name = sys.argv[1]
    append_text = "End of log\n"
    
    find_and_append(file_name, append_text)

if __name__ == "__main__":
    
    main()?