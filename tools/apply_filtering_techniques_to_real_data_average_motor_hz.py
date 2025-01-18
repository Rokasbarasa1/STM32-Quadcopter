import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq
from scipy.signal import spectrogram
from scipy.interpolate import interp1d
from scipy.signal import get_window

class NotchFilterQ:
    def __init__(self, center_frequency, q, refresh_rate):
        self.input_array = [0.0, 0.0]
        self.output_array = [0.0, 0.0]
        self.q = q / 100.0

        self.precalculated_refresh_rate_division = 1.0 / refresh_rate
        self.precalculated_q_division = 1.0 / (2.0 * self.q)

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
    
class LowPassPT1Filter:
    def __init__(self, cutoff_frequency, sample_rate):
        self.alpha = 1.0 / (1.0 + (2 * np.pi * cutoff_frequency / sample_rate))
        self.previous_filter_output = 0.0

    def read(self, current_value):
        self.previous_filter_output = self.alpha * self.previous_filter_output + (1 - self.alpha) * current_value
        return self.previous_filter_output

# Replace 'data.csv' with the path to your CSV file
# Ensure the CSV file uses ',' as the delimiter and that there are no extra spaces after commas



# df = pd.read_csv('24Quadcopter.csv', skipinitialspace=True)

# # Extract required columns and divide by 131 to get real values
# gyro_adc_0 = df['gyroADC[2]'] / 131  # Gyro values in degrees/sec
# debug_3 = df['debug[3]'] / 131 / 7       # Center frequency in Hz
# throttle_setpoint = (df['setpoint[3]'] - 48) * (100.0 / (1547 - 48))
# time_us = df['time (us)']
# # Time vector in seconds
# time_s = time_us * 1e-6





df = pd.read_csv('1Quadcopter.csv', skipinitialspace=True)

# Extract required columns and divide by 131 to get real values
gyro_adc_0 = df['gyroADC[2]'] / 131  # Gyro values in degrees/sec
debug_3 = df['debug[3]'] / 131       # Center frequency in Hz
throttle_setpoint = (df['setpoint[3]'] - 48) * (100.0 / (1547 - 48))
time_us = df['time (us)']
# Time vector in seconds
time_s = time_us * 1e-6



# Compute sampling frequency from time differences
dt = np.diff(time_us) * 1e-6  # Convert microseconds to seconds
# fs = 1.0 / np.median(dt)      # Sampling frequency in Hz
fs = 460
nyquist_limit = fs/2 - 6 # -6 just to not get too close to nyquist

print(f"Estimated sampling frequency: {fs:.2f} Hz")

divide_filter_rate = 2

# Initialize the notch filter with a default center frequency
q_value = 200  # Quality factor
notch_filter = NotchFilterQ(0, q_value, fs)

notch_filter_harmonic_1 = NotchFilterQ(0, q_value, fs)
notch_filter_harmonic_2 = NotchFilterQ(0, q_value, fs)
notch_filter_harmonic_3 = NotchFilterQ(0, q_value, fs)

cutoff_frequency = 105.0  # Adjust as needed
low_pass_filter = LowPassPT1Filter(cutoff_frequency, fs)


motor_frequency_low_pass_filter_cutoff = 150.0  # Adjust as needed
motor_frequency_low_pass_filter = LowPassPT1Filter(motor_frequency_low_pass_filter_cutoff, fs)


filtered_gyro = []
filtered_low_pass_gyro = []

# notch_filter.set_center_frequency(150)

# Apply the notch filter to gyro_adc_0 using center frequency from debug_3
for idx in range(len(gyro_adc_0)):
    gyro_value = gyro_adc_0.iloc[idx]
    center_frequency = debug_3.iloc[idx]

    # center_frequency = motor_frequency_low_pass_filter.read(debug_3.iloc[idx])

    # center_frequency = motor_frequency_low_pass_filter.read(debug_3.iloc[idx])

    if center_frequency > 50:
        new_gyro_value = gyro_value
        if center_frequency< nyquist_limit:
            # notch_filter.set_center_frequency(center_frequency)
            # new_gyro_value = notch_filter.read(new_gyro_value)
            notch_filter_harmonic_1.set_center_frequency(center_frequency)
            new_gyro_value = notch_filter_harmonic_1.read(new_gyro_value)
        
        if center_frequency*2< nyquist_limit:
            notch_filter_harmonic_2.set_center_frequency(center_frequency*2)
            new_gyro_value = notch_filter_harmonic_2.read(new_gyro_value)

        # The third harmonic can get too close to nyquist
        if center_frequency*3< nyquist_limit:
            notch_filter_harmonic_3.set_center_frequency(center_frequency*3)
            new_gyro_value = notch_filter_harmonic_3.read(new_gyro_value)

        filtered_gyro.append(new_gyro_value)
    else:
        filtered_gyro.append(gyro_value)

    filtered_low_pass_gyro.append(low_pass_filter.read(gyro_value))

# Convert filtered_gyro to a NumPy array
filtered_gyro = np.array(filtered_gyro)
filtered_low_pass_gyro = np.array(filtered_low_pass_gyro)



# Plot the original and filtered gyro data
plt.figure(figsize=(14, 7))
plt.plot(time_us * 1e-6, gyro_adc_0, label='Original')
# plt.plot(time_us * 1e-6, filtered_low_pass_gyro, label='Low pass filter')
plt.plot(time_us * 1e-6, filtered_gyro, label='Notch filter')
plt.xlabel('Time [s]')
plt.ylabel('Gyro Value [deg/s]')
plt.title('GyroADC[0] Before and After Notch Filtering')
plt.legend()
plt.grid(True)
# plt.show()

# --- Frequency Domain Analysis (Corrected Code) ---

# Convert Pandas Series to NumPy arrays
gyro_adc_0_np = gyro_adc_0.values  # Convert to NumPy array
filtered_gyro_np = filtered_gyro   # Already a NumPy array
filtered_low_pass_gyro_np = filtered_low_pass_gyro   # Already a NumPy array


# Compute the FFT of the original signal
N = len(gyro_adc_0_np)
yf_original = fft(gyro_adc_0_np)
xf = fftfreq(N, 1 / fs)

# Only take the positive frequencies (one-sided spectrum)
xf = xf[:N // 2]
yf_original = yf_original[:N // 2]

# Plot the frequency spectrum of the original signal
# plt.figure(figsize=(14, 7))
# plt.plot(xf, 2.0 / N * np.abs(yf_original))
# plt.xlabel('Frequency [Hz]')
# plt.ylabel('Amplitude')
# plt.title('Frequency Spectrum of Original GyroADC[0]')
# plt.grid(True)
# plt.show()

# Compute the FFT of the filtered signal
yf_filtered = fft(filtered_gyro_np)
yf_filtered = yf_filtered[:N // 2]

# Compute the FFT of the filtered signal
yfl_filtered = fft(filtered_low_pass_gyro_np)
yfl_filtered = yfl_filtered[:N // 2]

# Plot the frequency spectrum of the original and filtered signals
plt.figure(figsize=(14, 7))
plt.plot(xf, 2.0 / N * np.abs(yf_original), label='Original')
# plt.plot(xf, 2.0 / N * np.abs(yfl_filtered), label='Low pass filter')
plt.plot(xf, 2.0 / N * np.abs(yf_filtered), label='Notch filter')
plt.xlabel('Frequency [Hz]')
plt.ylabel('Amplitude')
plt.title('Frequency Spectrum Before and After Notch Filtering')
plt.legend()
plt.grid(True)
# plt.show()



# --- More Frequency Domain Analysis ---

gyro_data = gyro_adc_0.values
gyro_data_filtered = filtered_gyro
gyro_data_filtered_low_pass = filtered_low_pass_gyro

throttle_data = throttle_setpoint.values

# --- More More Frequency Domain Analysis ---
# Assuming 'gyro_adc_0_np' is your gyro data as a NumPy array
# 'fs' is the sampling frequency
# 'throttle_data' is your throttle data as a NumPy array
# 'time_s' is your time vector in seconds

# Compute the spectrogram
f, t, Sxx = spectrogram(gyro_adc_0_np, fs, window='hann', nperseg=1024, noverlap=512)
Sxx_dB = 10 * np.log10(Sxx + 1e-6)  # Convert power to dB scale

# Create an interpolation function for throttle data
interp_func = interp1d(time_s, throttle_data, kind='linear', bounds_error=False, fill_value='extrapolate')

# Interpolate throttle data to match spectrogram time bins
throttle_interp = interp_func(t)

# Plot the spectrogram with frequency on X-axis and time on Y-axis
plt.figure(figsize=(14, 7))
plt.pcolormesh(f, t, Sxx_dB.T, shading='gouraud', cmap='viridis')
plt.xlabel('Frequency [Hz]')
plt.ylabel('Time [s]')
plt.title('Original')
plt.colorbar(label='Intensity [dB]')

# Invert Y-axis if you want time to start from the top
# plt.gca().invert_yaxis()

# Plot the throttle data along the Y-axis (time)
# Since time is on the Y-axis, we need to plot throttle values against time
# We'll create a new axis sharing the Y-axis
# ax1 = plt.gca()
# ax2 = ax1.twiny()  # Create a new X-axis on top
# ax2.plot(throttle_interp, t, color='white', alpha=0.7, label='Throttle')
# ax2.set_xlabel('Throttle [%]', color='white')
# ax2.xaxis.set_label_position('top')
# ax2.xaxis.tick_top()
# ax2.tick_params(axis='x', colors='white')
# ax2.set_xlim(0, 100)  # Adjust based on your throttle range
# ax2.set_ylim(t.min(), t.max())  # Ensure the Y-axis limits match
# ax2.invert_yaxis()  # Invert Y-axis if needed
# ax2.legend(loc='upper right')

# plt.show()

# Compute the spectrogram for filtered data
f_filtered, t_filtered, Sxx_filtered = spectrogram(gyro_data_filtered, fs, window='hann', nperseg=1024, noverlap=512)
Sxx_filtered_dB = 10 * np.log10(Sxx_filtered + 1e-6)

# Interpolate throttle data to match spectrogram time bins
throttle_interp_filtered = interp_func(t_filtered)

# Plot the spectrogram with frequency on X-axis and time on Y-axis
plt.figure(figsize=(14, 7))
plt.pcolormesh(f_filtered, t_filtered, Sxx_filtered_dB.T, shading='gouraud', cmap='viridis')
plt.xlabel('Frequency [Hz]')
plt.ylabel('Time [s]')
plt.title('Notch filter')
plt.colorbar(label='Intensity [dB]')

# Overlay throttle data
# ax1 = plt.gca()
# ax2 = ax1.twiny()
# ax2.plot(throttle_interp_filtered, t_filtered, color='white', alpha=0.7, label='Throttle')
# ax2.set_xlabel('Throttle [%]', color='white')
# ax2.xaxis.set_label_position('top')
# ax2.xaxis.tick_top()
# ax2.tick_params(axis='x', colors='white')
# ax2.set_xlim(0, 100)
# ax2.set_ylim(t_filtered.min(), t_filtered.max())
# ax2.invert_yaxis()
# ax2.legend(loc='upper right')

# plt.show()


# f_filtered, t_filtered, Sxx_filtered = spectrogram(gyro_data_filtered_low_pass, fs, window='hann', nperseg=1024, noverlap=512)
# Sxx_filtered_dB = 10 * np.log10(Sxx_filtered + 1e-6)

# # Interpolate throttle data to match spectrogram time bins
# throttle_interp_filtered = interp_func(t_filtered)

# # Plot the spectrogram with frequency on X-axis and time on Y-axis
# plt.figure(figsize=(14, 7))
# plt.pcolormesh(f_filtered, t_filtered, Sxx_filtered_dB.T, shading='gouraud', cmap='viridis')
# plt.xlabel('Frequency [Hz]')
# plt.ylabel('Time [s]')
# plt.title('Low pass filter')
# plt.colorbar(label='Intensity [dB]')

# # Overlay throttle data
# ax1 = plt.gca()
# ax2 = ax1.twiny()
# ax2.plot(throttle_interp_filtered, t_filtered, color='white', alpha=0.7, label='Throttle')
# ax2.set_xlabel('Throttle [%]', color='white')
# ax2.xaxis.set_label_position('top')
# ax2.xaxis.tick_top()
# ax2.tick_params(axis='x', colors='white')
# ax2.set_xlim(0, 100)
# ax2.set_ylim(t_filtered.min(), t_filtered.max())
# ax2.invert_yaxis()
# ax2.legend(loc='upper right')














plt.show()