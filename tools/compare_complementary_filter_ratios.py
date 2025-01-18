import matplotlib.pyplot as plt
import os

# Function to load and parse data from a file
def load_data(file_path):
    x_data = []
    y_data = []
    with open(file_path, 'r') as file:
        for line in file:
            if line.strip():  # Ignore empty lines
                try:
                    x, y = map(float, line.strip().split(';')[:2])
                    x_data.append(x)
                    y_data.append(y)
                except ValueError:
                    print(f"Skipping invalid line in {file_path}: {line.strip()}")
    return x_data, y_data

# List of file names
file_names = [
    # "../misc/complementary_filter_ratio_testing/complementary_1.txt", 
    # "../misc/complementary_filter_ratio_testing/complementary_2.txt", 
    # "../misc/complementary_filter_ratio_testing/complementary_3.txt", 
    # "../misc/complementary_filter_ratio_testing/complementary_4.txt", 
    # "../misc/complementary_filter_ratio_testing/complementary_5.txt",
    # "../misc/complementary_filter_ratio_testing/complementary_6.txt",
    "../misc/complementary_filter_ratio_testing/complementary_7.txt",
    "../misc/complementary_filter_ratio_testing/complementary_8.txt",
    "../misc/complementary_filter_ratio_testing/complementary_9.txt",
    "../misc/complementary_filter_ratio_testing/complementary_10.txt",
    "../misc/complementary_filter_ratio_testing/complementary_11.txt",
    "../misc/complementary_filter_ratio_testing/complementary_12.txt",
    "../misc/complementary_filter_ratio_testing/complementary_13.txt",
    "../misc/complementary_filter_ratio_testing/complementary_14.txt",
    # "../misc/complementary_filter_ratio_testing/complementary_15.txt",
    # "../misc/complementary_filter_ratio_testing/complementary_20.txt"
]

# Check if files exist and load data
all_data = []
for file_name in file_names:
    if os.path.exists(file_name):
        x, y = load_data(file_name)
        all_data.append((x, y))
    else:
        print(f"File not found: {file_name}")

# Plotting the data
plt.figure(figsize=(10, 6))

for index, (x, y) in enumerate(all_data):
    plt.plot(range(len(x)), x, label=f"Data from {file_names[index]}")

plt.axhline(y=-60, color='r', linestyle='--', label="Static line at -60")
plt.title("X")
plt.xlabel("Index")
plt.ylabel("X")
plt.legend()
plt.grid()

# for index, (x, y) in enumerate(all_data):
#     plt.plot(range(len(x)), y, label=f"Data from {file_names[index]}")

# plt.title("Y")
# plt.xlabel("Index")
# plt.ylabel("Y")
# plt.legend()
# plt.grid()

plt.show()
