import numpy as np
import math

# Create array
pitch_effect_on_lat = np.zeros(10, dtype=np.float64)

# Test values
yaw_degrees = np.array([0.0, 90.0, 180.0, 270.0])

for i in range(len(yaw_degrees)):
    temp_value = -math.cos(yaw_degrees[i] * 0.0174533)
    print(f"Calculated: {temp_value}")
    pitch_effect_on_lat[i] = temp_value
    print(f"Stored: {pitch_effect_on_lat[i]}")
    print(f"Match: {temp_value == pitch_effect_on_lat[i]}")