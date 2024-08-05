import numpy as np
import matplotlib.pyplot as plt
import scipy.io as sio
from scipy import *
import matplotlib.pyplot as plt
import scipy.io as sio
import scipy.fftpack
import scipy.signal
import scipy.io.wavfile
from matplotlib.pyplot import figure
import copy

data = np.genfromtxt('motors_on_new_props_tape_applied.txt', delimiter=';')

column1 = data[:, 0]
column2 = data[:, 1]
column3 = data[:, 2]
column4 = data[:, 3]
column5 = data[:, 4]
column6 = data[:, 5]


signal = column4
npnts = len(signal)

time = []
srate = 200

for idx, x in enumerate(signal):
    # print(idx * (1/srate))
    time.append(idx * (1/srate))

figure(figsize=(20, 5), dpi=80)
plt.plot(time,signal)
plt.show()













# winlength = int(0.5*srate)

# winOnsets = np.arange(0, int(len(time)-winlength), int(winlength))

# hzW = np.linspace(0,srate/2, int(np.floor(winlength)))

# hannWin = 0.5 - np.cos(2*np.pi*np.linspace(0,1,int(winlength)))/2

# signalPow = np.zeros(len(hzW))

# listOfFTs = []

# matrix = np.zeros((len(winOnsets),len(hzW)))

# for wi in range(0, len(winOnsets)):
#     dataChunk = signal[winOnsets[wi]:winOnsets[wi]+winlength]
#     dataChunk = dataChunk * hannWin
#     tmpPow = np.abs(scipy.fftpack.fft(dataChunk)/winlength)**2

#     matrix[wi,:] = matrix[wi,:] + tmpPow[0:len(hzW)]
    
#     signalPow = signalPow + tmpPow[0:len(hzW)]

# signalPow = signalPow / len(winOnsets)

# # plotting
# plt.plot(hzW,signalPow,'r',label='Welch''s method')
# plt.xlim([0,40])
# plt.xlabel('Frequency (Hz)')
# plt.legend()

# times = np.arange(-3.0,3.0,0.5)

# print(times.shape)
# print(hzW.shape)
# print(matrix.shape)
# print(np.transpose(matrix).shape)

# plt.pcolormesh(times, hzW, np.transpose(matrix))
# plt.ylim([0,30])
# plt.xlabel('Time (s)'), plt.ylabel('Frequency (Hz)')







# focused on 200HZ and bellow
frex,time,pwr = scipy.signal.spectrogram(signal, npnts)
slice = pwr[0:20,:]
plt.pcolormesh(time,frex[0:20],slice)
plt.xlabel('Time (s)'), plt.ylabel('Frequency (Hz)')
plt.show()

# focused on all frequencies

frex,time,pwr = scipy.signal.spectrogram(signal, npnts)
slice = pwr
plt.pcolormesh(time,frex,slice)
plt.xlabel('Time (s)'), plt.ylabel('Frequency (Hz)')
plt.show()