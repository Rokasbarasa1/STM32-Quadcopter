import serial

# Configure the serial connection
ser = serial.Serial('COM18', 115200, timeout=1)

# Open/create the log file
with open('arduino_log.txt', 'a', encoding='utf-8') as logfile:
    print("Logging started. Press Ctrl+C to stop.")
    try:
        while True:
            if ser.in_waiting:
                data = ser.readline()
                try:
                    # Decode and write line to log
                    line = data.decode('utf-8', errors='replace')
                    logfile.write(line)
                    logfile.flush()
                    print(line, end='')  # Optional: also show in terminal
                except Exception as e:
                    # Handle any decode/write errors gracefully
                    print(f"Error writing line: {e}")
    except KeyboardInterrupt:
        print("\nLogging stopped.")
    finally:
        ser.close()
