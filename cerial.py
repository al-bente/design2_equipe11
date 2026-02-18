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
mode_flag = False


def mode_select():

    mode = input("Entrer le mode voulu (Courant : 0, Pose : 1, Commande : 2, Erreur : 3) ---> ")

    cntr = 0

    while(cntr < 10):

        ser.write(mode.encode('utf-8'))
        cntr += 1
        
    print(f"Mode envoyé: {mode}")



def keyboard_listener():
    global stop_flag
    global mode_flag
    while True:
        key = input().strip().lower()
        if key == 'p':
            stop_flag = True
            print("Stopping recording and plotting...")
        if key == 'm':
            mode_select()




# Start keyboard thread
thread = threading.Thread(target=keyboard_listener, daemon=True)
thread.start()

print("Recording... Type 'p' + Enter pour printer les données et 'm' + enter pour changer le mode")

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
