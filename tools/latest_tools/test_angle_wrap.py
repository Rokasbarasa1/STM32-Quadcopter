# import math

# def angle_difference_range(a, b, min_angle, max_angle):
#     range_ = max_angle - min_angle
#     diff = (b - a) % range_
#     diff = (diff + range_) % range_
#     if diff > range_ / 2.0:
#         diff -= range_
#     return diff

# def angle_difference_range_original(a, b, min_angle, max_angle):
#     L = max_angle - min_angle
#     a_shifted = a - min_angle
#     b_shifted = b - min_angle
#     diff = ( (b_shifted - a_shifted + 0.5 * L) % L ) - 0.5 * L
#     return diff

# def complementary_filter_calculate(ratio, value1, value2):
#     return (1.0 - ratio) * value1 + ratio * value2

# def complementary_filter_angle_calculate(ratio, value1, value2, min_angle, max_angle):
#     correction = angle_difference_range(value1, value2, min_angle, max_angle)
#     corrected_value2 = value1 + correction
#     return complementary_filter_calculate(ratio, value1, corrected_value2)

# def complementary_filter_angle_calculate_old(ratio, value1, value2, min_angle, max_angle):
#     correction = angle_difference_range_original(value1, value2, min_angle, max_angle)
#     corrected_value2 = value1 + correction
#     return complementary_filter_calculate(ratio, value1, corrected_value2)


# # ---- Simulation using hardcoded values ----
# # Yqw       gyro yaw    mag yaw      roll       pitch
# 346.5;      -0.1;      12.6;       -2.9;      -2.0;
# 345.9;      -0.2;      14.9;       -1.8;      -1.5;
# 340.3;      -0.4;      5.8;        -0.7;      -1.5;
# 336.2;      -0.5;      1.6;         0.4;      -0.9;
# 334.3;      -0.4;      343.1;       2.3;      -1.1;
# 334.2;       0.0;      334.5;       3.3;      -1.6;
# 334.5;       0.2;      322.0;       4.8;      -1.4;
# 334.8;       0.2;      302.8;       6.9;      -0.6;
# 335.4;       0.6;      297.5;       7.8;      -0.3;


# # Example values
# ratio = 0.001  # How much to trust value2 (magnetometer), vs value1 (gyro-integrated yaw)
# gyro_yaw = 346.0  # Simulated yaw from gyroscope (in degrees)
# magnetometer_yaw = 14.0  # Simulated yaw from magnetometer (in degrees)

# min_angle = 0.0
# max_angle = 360.0

# # Calculate filtered yaw
# filtered_yaw = complementary_filter_angle_calculate(
#     ratio, 
#     gyro_yaw, 
#     magnetometer_yaw, 
#     min_angle, 
#     max_angle
# )

# filtered_yaw_old = complementary_filter_angle_calculate_old(
#     ratio, 
#     gyro_yaw, 
#     magnetometer_yaw, 
#     min_angle, 
#     max_angle
# )

# print(f"Filtered Yaw: {filtered_yaw:.2f}°")
# print(f"Filtered Yaw old: {filtered_yaw_old:.2f}°")




import math

# Original angle difference function
def angle_difference_range_original(a, b, min_angle, max_angle):
    L = max_angle - min_angle
    a_shifted = a - min_angle
    b_shifted = b - min_angle
    diff = (b_shifted - a_shifted + 0.5 * L) % L - 0.5 * L
    return diff

# Safer alternative version
def angle_difference_range_safe(a, b, min_angle, max_angle):
    range_ = max_angle - min_angle
    diff = ((b - a) % range_ + range_) % range_
    if diff > range_ / 2:
        diff -= range_
    return diff

# Input data
data = [
    # yaw, delta_gyro, mag_yaw
    (346.5, -0.1, 12.6),
    (345.9, -0.2, 14.9),
    (340.3, -0.4, 5.8),
    (336.2, -0.5, 1.6),
    (334.3, -0.4, 343.1),
    (334.2,  0.0, 334.5),
    (334.5,  0.2, 322.0),
    (334.8,  0.2, 302.8),
    (335.4,  0.6, 297.5),
]

min_angle = 0.0
max_angle = 360.0
previous_yaw = 346.6

print(f"{'Step':>2} | {'GyroYaw':>8} | {'MagYaw':>7} | {'OriginalDiff':>14} | {'SafeDiff':>10}")
print("-"*60)

for i, (fused_yaw, delta_gyro, mag) in enumerate(data):
    gyro_yaw = previous_yaw + delta_gyro
    
    # Normalize gyro_yaw to [0, 360)
    gyro_yaw = gyro_yaw % 360.0

    diff_orig = angle_difference_range_original(gyro_yaw, mag, min_angle, max_angle)
    diff_safe = angle_difference_range_safe(gyro_yaw, mag, min_angle, max_angle)

    print(f"{i+1:>2} | {gyro_yaw:8.2f} | {mag:7.2f} | {diff_orig:14.2f} | {diff_safe:10.2f}")

    previous_yaw = fused_yaw  # Update for next step