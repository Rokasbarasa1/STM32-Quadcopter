import sys
import os
import argparse

def split_file(input_file, output_prefix, in_csv=False, out_csv=False):

    printed_line = False
    if in_csv:
        if out_csv:
            output_file1 = f"{output_prefix}1.csv"
            output_file2 = f"{output_prefix}2.csv"
        else:
            output_file1 = f"{output_prefix}1.txt"
            output_file2 = f"{output_prefix}2.txt"

        with open(input_file, "r") as infile, \
            open(output_file1, "w") as out1, \
            open(output_file2, "w") as out2:
            
            for line in infile:
                if not printed_line:
                    print(line)
                    printed_line = True
                parts = line.split(';')
                
                if len(parts) >= 6:
                    first_three = parts[0:3]
                    second_three = parts[3:6]
                    
                    if out_csv:
                        # Add an extra tab before the newline
                        out1.write(";".join(first_three) + ";\n")
                        out2.write(";".join(second_three) + ";\n")
                    else:
                        # Add an extra tab before the newline
                        out1.write("\t".join(first_three) + "\t\n")
                        out2.write("\t".join(second_three) + "\t\n")

    
    else:
        if out_csv:
            output_file1 = f"{output_prefix}1.csv"
            output_file2 = f"{output_prefix}2.csv"
        else:
            output_file1 = f"{output_prefix}1.txt"
            output_file2 = f"{output_prefix}2.txt"
        with open(input_file, "r") as infile, \
            open(output_file1, "w") as out1, \
            open(output_file2, "w") as out2:
            
            for line in infile:
                if not printed_line:
                    print(line)
                    printed_line = True
                parts = line.strip().split()
                
                if len(parts) >= 6:
                    first_three = parts[0:3]
                    second_three = parts[3:6]

                    if out_csv:
                        # Add an extra tab before the newline
                        out1.write(";".join(first_three) + ";\n")
                        out2.write(";".join(second_three) + ";\n")
                    else:
                        # Add an extra tab before the newline
                        out1.write("\t".join(first_three) + "\t\n")
                        out2.write("\t".join(second_three) + "\t\n")
    
    print(f"[✔] Created: {output_file1}, {output_file2}")

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="")
    parser.add_argument("input_file", help="")
    parser.add_argument("output_prefix", help="")
    parser.add_argument('--in_csv', action='store_true', help='')
    parser.add_argument('--out_csv', action='store_true', help='')

    args = parser.parse_args()

    input_file = args.input_file
    output_prefix = args.output_prefix

    if not os.path.exists(input_file):
        print(f"Error: input file '{input_file}' not found.")
        sys.exit(1)
    
    split_file(input_file, output_prefix, args.in_csv, args.out_csv)
