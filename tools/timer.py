# helper script for making pwm calculations

timer_peripheral_clock = 75000000
pre_scaler = 72
counter_period = 1000

print("Timer peripheral clock: " +str(timer_peripheral_clock) + "Hz")
print("Pre scaler value: " +str(pre_scaler))
print("Counter period: " +str(counter_period))


timer_clock = timer_peripheral_clock /pre_scaler
print("Timer clock divided by pre scaler: " +str(timer_clock) + "Hz")

frequency = timer_clock/counter_period
print("Frequency: " + str(frequency) + "Hz")
print("Max duty cycle: " + str(1000/ frequency) + "ms")
print("Pwm value range: 0-" + str(counter_period))
print("Resolution per pwm step: " + str((1000 / frequency)/counter_period) +"ms")

#$used = 50
#duty_cycle = used / counter_period

