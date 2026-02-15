import serial
import matplotlib.pyplot as plt
import threading

# ===== CHANGE THIS TO YOUR PORT =====
# PORT = 'COM3'
PORT = '/dev/ttyACM0'
BAUD = 115200

ser = serial.Serial(PORT, BAUD)

data = []
stop_flag = False


def keyboard_listener():
    global stop_flag
    while True:
        key = input()
        if key.lower() == 'p':
            stop_flag = True


# Start keyboard thread
thread = threading.Thread(target=keyboard_listener, daemon=True)
thread.start()

print("Recording... Type 'p' + Enter to plot. Recording resumes automatically.")

while True:
    if stop_flag:
        if data:
            plt.figure()
            plt.plot(data)
            plt.xlabel("Sample")
            plt.ylabel("Value")
            plt.title("Output")
            plt.show()

        # Reset for next capture
        data.clear()
        stop_flag = False
        print("Recording restarted...")

    line = ser.readline().decode(errors='ignore').strip()

    if line:
        try:
            value = int(float(line))
            data.append(value)
        except ValueError:
            pass
