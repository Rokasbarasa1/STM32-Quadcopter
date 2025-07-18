import numpy as np
import matplotlib.pyplot as plt

# --- Parameters ---
start_deg = 1.00
end_deg = 7.0
expo_curve = 18
step_deg = 0.01

full_range = np.arange(start_deg-0.5, end_deg+0.5, step_deg)
alphas = []
for tilt in full_range:
    if tilt <= start_deg:
        alpha = 1.0
    elif tilt >= end_deg:
        alpha = 0.0
    else:
        percent = ((tilt - start_deg) / (end_deg - start_deg))
        percent = np.clip(percent, 0.0, 1.0)
        alpha = (1.0 - percent) ** expo_curve
    alphas.append(alpha)

# Show the mapping table
for deg, alpha in zip(full_range, alphas):
    print(f"{deg:.2f} deg -> alpha: {alpha:.4f}")


plt.figure(figsize=(8,5))
plt.plot(full_range, alphas, label=f'Alpha (expo={expo_curve})')
plt.axvline(start_deg, color='green', linestyle=':', label='Mag gate')
plt.axvline(end_deg, color='red', linestyle=':', label='Gyro gate')
plt.xlabel('Tilt (degrees)')
plt.ylabel('Alpha')
plt.title('Alpha vs Tilt - Fast initial exponential drop')
plt.ylim(-0.05, 1.05)
plt.legend()
plt.grid(True)
plt.show()