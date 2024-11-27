import numpy as np
import matplotlib.pyplot as plt

class NotchFilterQ:
    def __init__(self, center_frequency, q, refresh_rate):
        self.input_array = [0.0, 0.0]
        self.output_array = [0.0, 0.0]
        self.q = q

        self.precalculated_refresh_rate_division = 1.0 / refresh_rate
        self.precalculated_q_division = 1.0 / (2.0 * q)

        self.set_center_frequency(center_frequency)

    def set_center_frequency(self, new_center_frequency):
        M_PI_2 = 2 * np.pi  # Equivalent to 6.2831853f

        omega = M_PI_2 * new_center_frequency * self.precalculated_refresh_rate_division
        cos_omega = np.cos(omega)
        sin_omega = np.sin(omega)
        alpha = sin_omega * self.precalculated_q_division

        b0 = 1.0
        b1 = -2.0 * cos_omega
        b2 = 1.0
        a0 = 1.0 + alpha
        a1 = -2.0 * cos_omega
        a2 = 1.0 - alpha

        normalize_a0 = 1.0 / a0

        self.b0 = b0 * normalize_a0
        self.b1 = b1 * normalize_a0
        self.b2 = b2 * normalize_a0
        self.a1 = a1 * normalize_a0
        self.a2 = a2 * normalize_a0

    def set_center_frequency_using_reference_filter(self, reference_filter):
        self.b0 = reference_filter.b0
        self.b1 = reference_filter.b1
        self.b2 = reference_filter.b2
        self.a1 = reference_filter.a1
        self.a2 = reference_filter.a2

    def read(self, input_value):
        output_value = (self.b0 * input_value +
                        self.b1 * self.input_array[0] +
                        self.b2 * self.input_array[1] -
                        self.a1 * self.output_array[0] -
                        self.a2 * self.output_array[1])

        # Update state
        self.input_array[1] = self.input_array[0]
        self.input_array[0] = input_value

        self.output_array[1] = self.output_array[0]
        self.output_array[0] = output_value

        return output_value





# Test parameters
fs = 1000  # Sampling frequency in Hz
t = np.arange(0, 1, 1 / fs)  # Time vector

# Create a signal with 50 Hz and 100 Hz components
signal = np.sin(2 * np.pi * 50 * t) + np.sin(2 * np.pi * 100 * t)

# Initialize the notch filter to remove 100 Hz component
center_frequency = 100  # Hz
q = 30  # Quality factor

notch_filter = NotchFilterQ(center_frequency, q, fs)

# Apply the filter to the signal
filtered_signal = np.array([notch_filter.read(x) for x in signal])

# Plot the original and filtered signals
plt.figure(figsize=(12, 6))

# Original signal
plt.subplot(2, 1, 1)
plt.plot(t, signal)
plt.title('Original Signal (50 Hz and 100 Hz)')
plt.xlabel('Time [s]')
plt.ylabel('Amplitude')
plt.grid(True)

# Filtered signal
plt.subplot(2, 1, 2)
plt.plot(t, filtered_signal)
plt.title('Filtered Signal (Notch at 100 Hz)')
plt.xlabel('Time [s]')
plt.ylabel('Amplitude')
plt.grid(True)

plt.tight_layout()
plt.show()

# Frequency-domain analysis
from scipy.fft import fft, fftfreq

N = len(signal)
yf = fft(signal)
yf_filtered = fft(filtered_signal)
xf = fftfreq(N, 1 / fs)

plt.figure(figsize=(12, 6))

# Spectrum of original signal
plt.subplot(2, 1, 1)
plt.plot(xf[:N // 2], (2 / N) * np.abs(yf[:N // 2]))
plt.title('Original Signal Spectrum')
plt.xlabel('Frequency [Hz]')
plt.ylabel('Magnitude')
plt.grid(True)

# Spectrum of filtered signal
plt.subplot(2, 1, 2)
plt.plot(xf[:N // 2], (2 / N) * np.abs(yf_filtered[:N // 2]))
plt.title('Filtered Signal Spectrum')
plt.xlabel('Frequency [Hz]')
plt.ylabel('Magnitude')
plt.grid(True)

plt.tight_layout()
plt.show()