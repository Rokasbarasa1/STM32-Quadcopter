import math


M_RAD_TO_DEG = 180.0 / math.pi

def calculate_yaw_using_magnetometer_data(magnetometer_data, yaw_offset=0.0):
    """
    Simple yaw calculation from magnetometer data without tilt compensation.

    Args:
        magnetometer_data: list or tuple with at least two elements [mx, my]
        yaw_offset: angle offset in degrees to add to final yaw

    Returns:
        yaw in degrees, normalized to [0, 360)
    """
    x = magnetometer_data[0]
    y = magnetometer_data[1]

    yaw = math.atan2(y, x) * M_RAD_TO_DEG
    yaw += yaw_offset

    if yaw < 0:
        yaw += 360

    return yaw


def calculate_yaw_tilt_compensated_using_magnetometer_data(magnetometer_data, roll, pitch, yaw_offset=0.0):
    """
    Tilt compensated yaw calculation from 3-axis magnetometer and roll/pitch angles.

    Args:
        magnetometer_data: list or tuple with three elements [mx, my, mz]
        roll: roll angle in degrees
        pitch: pitch angle in degrees
        yaw_offset: angle offset in degrees to add to final yaw

    Returns:
        tilt compensated yaw in degrees, normalized to [0, 360)
    """
    roll_rad = math.radians(roll)
    pitch_rad = math.radians(pitch)

    mx = magnetometer_data[0]
    my = magnetometer_data[1]
    mz = magnetometer_data[2]

    # Tilt compensation projection
    Xh = mx * math.cos(pitch_rad) + mz * math.sin(pitch_rad)
    Yh = (mx * math.sin(roll_rad) * math.sin(pitch_rad) +
          my * math.cos(roll_rad) -
          mz * math.sin(roll_rad) * math.cos(pitch_rad))

    yaw = math.atan2(Yh, Xh) * M_RAD_TO_DEG
    yaw += yaw_offset

    while yaw >= 360:
        yaw -= 360
    while yaw < 0:
        yaw += 360

    return yaw

yaw_offset = -90.0

mag_x = 15.4
mag_y = -18.9
mag_z = -44.8

mags = [mag_x,mag_y,mag_z]

aligned_mx =  mag_y
aligned_my = -mag_x
aligned_mz =  mag_z

aligned_mags = [aligned_mx,aligned_my,aligned_mz]

roll = 26.1
pitch = 0.66
goal = 152


yaw_tilt_compensated = calculate_yaw_tilt_compensated_using_magnetometer_data(mags, roll, pitch, yaw_offset)
yaw = calculate_yaw_using_magnetometer_data(mags, yaw_offset)


rotated_yaw_tilt_compensated = calculate_yaw_tilt_compensated_using_magnetometer_data(aligned_mags, roll, pitch, 0)
rotated_yaw = calculate_yaw_using_magnetometer_data(aligned_mags, 0)

print(yaw)
print(yaw_tilt_compensated)
print(goal)

print()

print(rotated_yaw)
print(rotated_yaw_tilt_compensated)
print(goal)
