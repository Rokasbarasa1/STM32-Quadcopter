import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter

# Function to create a low-pass Butterworth filter
def butter_lowpass(cutoff, fs, order=5):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

# Function to apply the low-pass filter
def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y

# Sample rate and duration
fs = 500  # Sample rate, Hz
T = 1.0   # Seconds (shorter time duration)
n = int(T * fs)  # Total number of samples

# Generate a sample time vector
t = np.linspace(0, T, n, endpoint=False)

# Generate a simple signal with low and high frequency components
frequencies = [5, 50]  # Hz
signal = sum(np.sin(2 * np.pi * f * t) for f in frequencies)

# Apply low-pass filter
cutoff = 10  # Cutoff frequency in Hz
filtered_signal = butter_lowpass_filter(signal, cutoff, fs, order=6)

# Plot the original and filtered signals on the same scale
plt.figure(figsize=(14, 7))

# Determine the amplitude range for consistent y-axis scaling
y_min = min(np.min(signal), np.min(filtered_signal))
y_max = max(np.max(signal), np.max(filtered_signal))

plt.plot(t, signal, label='Original Signal')
plt.plot(t, filtered_signal, label='Filtered Signal (Low-Pass)', color='orange')
plt.ylim([y_min, y_max])
plt.title('Original vs Filtered Signal')
plt.xlabel('Time [s]')
plt.ylabel('Amplitude')
plt.grid(True)
plt.legend()

# Highlight a specific point to show delay
highlight_time = 0.4  # seconds
plt.axvline(x=highlight_time, color='red', linestyle='--', label='Highlight Time')
plt.legend()

plt.tight_layout()
plt.show()