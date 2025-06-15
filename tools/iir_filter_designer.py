from scipy.signal import butter

sampling_rate = 520
cutoff = 20
order = 4

b, a = butter(order, cutoff/(sampling_rate/2), btype='low')
print("b =", b)
print("a =", a)