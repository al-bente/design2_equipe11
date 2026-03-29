import serial
import matplotlib.pyplot as plt
import threading
import os
import csv
import json
from datetime import datetime
import platform
import sys
import serial.tools.list_ports
import time
import statistics
import numpy as np
from collections import deque

# ===== CONFIGURATION =====
BAUD = 115200
DIR_PATH = r"./Recordings"

HANDSHAKE_START = 255
HANDSHAKE_STOP = 254

# ===== MOVING AVERAGE CONFIGURATION =====
MASS_AVG_SIZE = 400  # Nombre de samples pour la moyenne mobile (augmenter = plus lisse, diminuer = plus réactif)

# ===== LOW-PASS FILTER CONFIGURATION =====
LOWPASS_ALPHA = 0.05  # Facteur du filtre passe-bas (0.0 = très lisse, 1.0 = pas de lissage)
                       # Bloque les spikes rapides en temps réel

# Configure which data streams to plot (True = plot, False = ignore)
PLOT_CONFIG = {
    'masse': False,     # Mass (g)
    'pose': True,       # Position (mm)
    'pid': False,       # PID output
    'error': False      # Error signal
}

# Mode mapping: 0=masse, 1=pose, 2=pid, 3=error, 4=all
MODE_MAP = {
    0: ['masse'],
    1: ['pose'],
    2: ['pid'],
    3: ['error'],
    4: ['masse', 'pose', 'pid', 'error']
}

PLOT_INFO = {
    'masse': ("Mass (g)", 'steelblue'),
    'pose': ("Position (mm)", 'forestgreen'),
    'pid': ("PID Output", 'coral'),
    'error': ("Error", 'mediumpurple')
}

# Calibration
INDEX_BIT = 0
N_AVG = 500
CALIBRATION_POINTS = [0.0, 5.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 80.0, 90.0, 100.0]

# Coefficients polynomiaux (quadratique) pour conversion bits -> masse
coeffs = None

# Variable de tare
tare_mass = 0.0
tare_bits = None

# Deque de samples pour calibration / live
cal_samples = deque(maxlen=1000)

# Calibration state
calibration_done = False

# Fichier de sauvegarde calibration
CALIBRATION_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibration.json")


# ===== CLASSE FILTRE PASSE-BAS =====
class LowPassFilter:
    """Filtre passe-bas de premier ordre pour bloquer les spikes rapides."""
    def __init__(self, alpha):
        self.alpha = alpha
        self.filtered_value = None
    
    def update(self, value):
        if self.filtered_value is None:
            self.filtered_value = value
        else:
            self.filtered_value = self.alpha * value + (1.0 - self.alpha) * self.filtered_value
        return self.filtered_value
    
    def reset(self):
        self.filtered_value = None


# ===== CLASSE MOYENNE MOBILE =====
class MovingAverage:
    """Implémentation optimisée en O(1) d'une moyenne mobile simple."""
    def __init__(self, size):
        self.size = size
        self.window = deque(maxlen=size)
        self.total = 0.0

    def update(self, value):
        if len(self.window) == self.size:
            # Soustrait la valeur la plus ancienne qui s'apprête à sortir
            self.total -= self.window[0]
        
        self.window.append(value)
        self.total += value
        return self.total / len(self.window)

    def reset(self):
        self.window.clear()
        self.total = 0.0

# Initialisation du filtre
mass_filter = MovingAverage(MASS_AVG_SIZE)
mass_lowpass = LowPassFilter(LOWPASS_ALPHA)


def save_calibration():
    """Sauvegarde les coefficients quadratiques dans un fichier JSON"""
    if coeffs is None:
        return
    data_cal = {"coeffs": coeffs.tolist()}
    with open(CALIBRATION_FILE, 'w') as f:
        json.dump(data_cal, f, indent=2)
    print(f"  [SAVE] Calibration sauvegardée dans {CALIBRATION_FILE}")


def load_calibration():
    """Charge les coefficients depuis le fichier JSON. Retourne True si chargé."""
    global coeffs, calibration_done
    if not os.path.exists(CALIBRATION_FILE):
        return False
    try:
        with open(CALIBRATION_FILE, 'r') as f:
            data_cal = json.load(f)
        coeffs = np.array(data_cal["coeffs"])
        calibration_done = True
        print(f"  [LOAD] Calibration chargée depuis {CALIBRATION_FILE}")
        print(f"         m = {coeffs[0]:.10f}*x² + {coeffs[1]:.10f}*x + {coeffs[2]:.10f}")
        return True
    except Exception as e:
        print(f"  [ERREUR] Impossible de charger la calibration : {e}")
        return False


def get_arduino_port():
    """Auto-detect Arduino port by scanning available ports"""
    print("\n" + "="*60)
    print("Arduino Port Detection")
    print("="*60)
    
    ports = serial.tools.list_ports.comports()
    
    if not ports:
        print("No serial ports found!")
        print("Please check:")
        print("  - The Arduino is connected")
        print("  - The USB driver is installed")
        sys.exit(1)
    
    print(f"Found {len(ports)} serial port(s):\n")
    
    arduino_ports = []
    for i, port in enumerate(ports):
        port_name = port.device
        port_desc = port.description
        print(f"  [{i}] {port_name}: {port_desc}")
        
        # Look for Arduino-like devices
        if "Arduino" in port_desc or "CH340" in port_desc or "USB" in port_desc:
            arduino_ports.append(port_name)
    
    print()
    
    # If exactly one likely Arduino port found, use it
    if len(arduino_ports) == 1:
        selected_port = arduino_ports[0]
        print(f"✓ Auto-detected Arduino on: {selected_port}")
        return selected_port
    
    # If multiple Arduino-like ports, ask user to choose
    if len(arduino_ports) > 1:
        print("Multiple Arduino-like ports detected:")
        for i, port in enumerate(arduino_ports):
            print(f"  [{i}] {port}")
        choice = input("Select port number (0-{0}): ".format(len(arduino_ports)-1)).strip()
        try:
            idx = int(choice)
            if 0 <= idx < len(arduino_ports):
                return arduino_ports[idx]
        except ValueError:
            pass
    
    # Fallback: ask user to select from all ports
    print("Select a port number (0-{0}): ".format(len(ports)-1))
    while True:
        try:
            choice = input("Enter port number: ").strip()
            idx = int(choice)
            if 0 <= idx < len(ports):
                return ports[idx].device
            else:
                print("Invalid selection. Try again.")
        except ValueError:
            print("Please enter a valid number.")


ser = None

data = {
    'masse': [],
    'pose': [],
    'pid': [],
    'error': [],
    'sample': []
}

stop_flag = False
sample_count = 0
current_mode = None
selected_channels = []
recording = False  # Recording state
target_samples = 1000  # Default target number of samples
recording_enabled = False  # Flag to enable/disable recording


def create_csv_file():
    """Create a new CSV file for recording"""
    if not os.path.exists(DIR_PATH):
        os.makedirs(DIR_PATH)
    
    i = 0
    filename = f"Arduino_recording_{i}.csv"
    while os.path.exists(os.path.join(DIR_PATH, filename)):
        i += 1
        filename = f"Arduino_recording_{i}.csv"
    
    csv_path = os.path.join(DIR_PATH, filename)
    return csv_path


def save_to_csv(csv_path):
    """Save collected data to CSV file based on selected channels, including calculated mass"""
    if not data['sample']:
        print("No data to save")
        return
    
    with open(csv_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        
        # Create header based on selected channels
        header = ["Sample"]
        for channel in selected_channels:
            header.append(f"{channel.capitalize()}")
        # Always add calculated mass column when masse is selected
        if 'masse' in selected_channels:
            header.append("Masse_calculee_g")
        writer.writerow(header)
        
        # Write data for selected channels only
        for i in range(len(data['sample'])):
            row = [data['sample'][i]]
            for channel in selected_channels:
                row.append(data[channel][i] if i < len(data[channel]) else '')
            # The masse column already contains the calculated mass from convert_courant
            # Duplicate it explicitly for clarity
            if 'masse' in selected_channels and i < len(data['masse']):
                row.append(data['masse'][i])
            writer.writerow(row)
    
    print(f"Data saved to: {csv_path}")


def convert_courant(bits):
    """Convert bits to mass using quadratic polyfit coefficients"""
    if coeffs is None:
        return float("nan")
    return float(np.polyval(coeffs, bits))


def displayed_mass(bits_val):
    """Return mass corrected for tare"""
    m_abs = convert_courant(bits_val)
    if tare_bits is None:
        return m_abs
    return m_abs - convert_courant(tare_bits)


def plot_data():
    """Create and display plots for selected channels only"""
    if not data['sample']:
        print("No data to plot")
        return
    
    num_plots = len(selected_channels)
    
    if num_plots == 0:
        print("No channels selected for plotting")
        return
    
    fig, axes = plt.subplots(num_plots, 1, figsize=(12, 4 * num_plots))
    if num_plots == 1:
        axes = [axes]
    
    for idx, (channel_name, ax) in enumerate(zip(selected_channels, axes)):
        title, color = PLOT_INFO[channel_name]
        ax.plot(data['sample'], data[channel_name], lw=2, color=color, marker='o', markersize=3)
        ax.set_title(title, fontsize=12, fontweight='bold')
        ax.set_xlabel("Sample")
        ax.set_ylabel(title)
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    print(f"\nDisplaying {num_plots} plot(s) with {len(data['sample'])} samples...")
    plt.show()


def cal_wait_for_samples(n=N_AVG):
    """Wait until at least n samples are in cal_samples"""
    while len(cal_samples) < n:
        time.sleep(0.05)


def cal_get_avg(n=N_AVG):
    """Return mean of last n calibration samples"""
    return statistics.mean(list(cal_samples)[-n:])


def cal_get_std(n=N_AVG):
    """Return std dev of last n calibration samples"""
    d = list(cal_samples)[-n:]
    return statistics.stdev(d) if len(d) >= 2 else 0.0


def cal_reader_thread():
    """Background thread that feeds cal_samples during calibration"""
    global stop_flag
    while not stop_flag:
        try:
            if ser is None or not ser.is_open:
                time.sleep(0.01)
                continue
            byte = ser.read(1)
            if not byte or byte[0] != HANDSHAKE_START:
                continue

            line = ser.readline().decode(errors='ignore').strip()
            if not line:
                continue

            parts = line.split(',')
            if len(parts) <= INDEX_BIT:
                continue

            try:
                valeur = float(parts[INDEX_BIT])
                cal_samples.append(valeur)
            except ValueError:
                pass

            # Wait for stop handshake
            while not stop_flag:
                byte = ser.read(1)
                if byte and byte[0] == HANDSHAKE_STOP:
                    break

        except Exception as e:
            print(f"[ERREUR serie cal] {e}")
            break


def calibrate():
    """Perform quadratic calibration with predefined masses, save CSV and plot"""
    global coeffs, calibration_done, stop_flag

    print("\n" + "=" * 60)
    print("  CALIBRATION QUADRATIQUE")
    print("=" * 60)
    print(f"  {len(CALIBRATION_POINTS)} points de calibration : {CALIBRATION_POINTS}")
    input("\nAppuyez sur ENTREE pour commencer...\n")

    # Start calibration reader thread
    old_stop = stop_flag
    stop_flag = False
    t_cal = threading.Thread(target=cal_reader_thread, daemon=True)
    t_cal.start()

    masses = []
    bits_list = []

    for i, masse_cible in enumerate(CALIBRATION_POINTS):
        if masse_cible == 0.0:
            print(f"  [{i+1}/{len(CALIBRATION_POINTS)}] Plateau VIDE (0 g)")
        else:
            print(f"  [{i+1}/{len(CALIBRATION_POINTS)}] Deposez {masse_cible:.0f} g sur le plateau")

        input("         -> Appuyez sur ENTREE quand stable...")

        cal_samples.clear()
        mass_filter.reset()
        mass_lowpass.reset()
        print(f"         -> Acquisition de {N_AVG} echantillons...", end=" ", flush=True)
        cal_wait_for_samples(N_AVG)
        avg = cal_get_avg()
        std = cal_get_std()
        print(f"bits = {avg:.3f}  (std = {std:.3f})\n")

        masses.append(masse_cible)
        bits_list.append(avg)

    # Stop cal reader thread
    stop_flag = True
    time.sleep(0.1)
    stop_flag = old_stop

    # Fit quadratique : masse = f(bits)
    masses_arr = np.array(masses)
    bits_arr = np.array(bits_list)
    coeffs = np.polyfit(bits_arr, masses_arr, 2)
    calibration_done = True

    print("=" * 60)
    print("  RÉSULTAT DU FIT QUADRATIQUE")
    print(f"  m = {coeffs[0]:.10f}*x² + {coeffs[1]:.10f}*x + {coeffs[2]:.10f}")
    print("=" * 60)

    # Verification table
    calculated_masses = []
    print(f"\n  {'Masse ref':>10} {'Bits moy':>10} {'Masse calc':>12} {'Erreur':>10}")
    print("  " + "-" * 46)
    for masse_ref, bits_moy in zip(masses, bits_list):
        masse_calc = float(np.polyval(coeffs, bits_moy))
        erreur = masse_calc - masse_ref
        calculated_masses.append(masse_calc)
        ok = "OK" if abs(erreur) <= 1.0 else "ATTENTION"
        print(f"  {masse_ref:>10.1f} {bits_moy:>10.3f} {masse_calc:>12.3f} {erreur:>+10.3f} g  {ok}")
    print()

    # ---- Save calibration to CSV ----
    cal_csv = os.path.join(DIR_PATH, "calibration.csv")
    if not os.path.exists(DIR_PATH):
        os.makedirs(DIR_PATH)
    with open(cal_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Bits_moyen", "Masse_reelle_g", "Masse_calculee_g", "Erreur_g"])
        for bits_moy, masse_ref, masse_calc in zip(bits_list, masses, calculated_masses):
            writer.writerow([f"{bits_moy:.4f}", f"{masse_ref:.2f}", f"{masse_calc:.4f}", f"{abs(masse_calc - masse_ref):.4f}"])
    print(f"  ✓ Calibration sauvegardée dans : {cal_csv}")

    # ---- Plot calibration ----
    fit_x = np.linspace(min(bits_arr) * 0.9, max(bits_arr) * 1.1, 200)
    fit_y = np.polyval(coeffs, fit_x)

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    # Plot 1 : calibration curve
    ax1 = axes[0]
    ax1.scatter(bits_arr, masses_arr, color='steelblue', s=80, zorder=5, label='Points réels')
    ax1.scatter(bits_arr, np.array(calculated_masses), color='coral', marker='x', s=80, zorder=5, label='Masses calculées')
    ax1.plot(fit_x, fit_y, '--', color='gray', lw=1.5,
             label=f'Fit quad. ({coeffs[0]:.6f}x² + {coeffs[1]:.4f}x + {coeffs[2]:.2f})')
    ax1.set_xlabel("Courant (bits)")
    ax1.set_ylabel("Masse (g)")
    ax1.set_title("Courbe de calibration (quadratique)")
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # Plot 2 : error bar
    ax2 = axes[1]
    errors = [abs(mc - mr) for mc, mr in zip(calculated_masses, masses)]
    ax2.bar(range(len(errors)), errors, color='mediumpurple', edgecolor='black')
    ax2.set_xticks(range(len(errors)))
    ax2.set_xticklabels([f"{m:.0f}g" for m in masses], rotation=45)
    ax2.set_ylabel("Erreur absolue (g)")
    ax2.set_title("Erreur de calibration par point")
    ax2.grid(True, alpha=0.3, axis='y')

    plt.tight_layout()
    plt.show()

    save_calibration()

    print("\n✓ Calibration quadratique terminée avec succès!\n")
    return True


def receive_packet():
    """Receive a data packet in format: 255, courant,pose,pid,error, 254"""
    global sample_count, target_samples, recording_enabled, tare_bits
    
    try:
        # Wait for start handshake
        while True:
            byte = ser.read(1)
            if byte and byte[0] == HANDSHAKE_START:
                break
        
        # Read the data line
        line = ser.readline().decode(errors='ignore').strip()
        if not line:
            return False
        
        # Parse the comma-separated values
        parts = line.split(',')
        if len(parts) >= 4:
            try:
                courant_bits = float(parts[0])
                
                # Filtrage par moyenne mobile (utilise MASS_AVG_SIZE)
                mass_smoothed = mass_filter.update(courant_bits)
                
                # Filtrage passe-bas pour bloquer les spikes rapides (utilise LOWPASS_ALPHA)
                mass_filtered = mass_lowpass.update(mass_smoothed)
                
                # Apply tare-corrected mass via quadratic polyval
                mass_taree = displayed_mass(mass_filtered)
                pose = float(parts[1])
                pid = float(parts[2])
                error = float(parts[3])
                
                # Wait for stop handshake
                while True:
                    byte = ser.read(1)
                    if byte and byte[0] == HANDSHAKE_STOP:
                        break
                
                # Store data
                data['sample'].append(sample_count)
                data['masse'].append(mass_taree)
                data['pose'].append(pose)
                data['pid'].append(pid)
                data['error'].append(error)
                sample_count += 1
                
                # Print progress feedback
                if recording_enabled:
                    # Calculate progress percentage
                    progress = (sample_count / target_samples) * 100
                    # Print progress on the same line
                    print(f"\rRecording: {sample_count}/{target_samples} samples ({progress:.1f}%)", end='', flush=True)
                
                return True
            except ValueError:
                return False
    except Exception as e:
        print(f"Error receiving packet: {e}")
        return False
    
    return False


def parse_mode_input(user_input):
    """Parse mode input: single digit (0-4) or combined (e.g., '1;2')"""
    try:
        # Check if it's a single mode (0-4)
        if len(user_input) == 1 and user_input.isdigit():
            mode = int(user_input)
            if 0 <= mode <= 4:
                return MODE_MAP[mode]
            else:
                print("Invalid mode. Please enter 0-4 or combinations like '1;2'")
                return None
        
        # Check if it's a combined mode (e.g., '1;2')
        elif ';' in user_input:
            channels = []
            parts = user_input.split(';')
            channel_names = ['courant', 'pose', 'pid', 'error']
            
            for part in parts:
                part = part.strip()
                if part.isdigit():
                    idx = int(part)
                    if 0 <= idx <= 3:
                        channel_name = channel_names[idx]
                        if channel_name not in channels:
                            channels.append(channel_name)
                    else:
                        print(f"Invalid channel index: {idx}. Use 0-3")
                        return None
                else:
                    print(f"Invalid format: {part}")
                    return None
            
            if channels:
                return channels
            else:
                print("No valid channels specified")
                return None
        
        else:
            print("Invalid input format. Use single digit (0-4) or combined (e.g., '1;2')")
            return None
    
    except Exception as e:
        print(f"Error parsing mode: {e}")
        return None


def print_main_menu():
    """Print the main menu"""
    print("\nSerial Plotter - Data Visualization with Sample Recording")
    print("========================================================")
    print(f"Current Mode: {', '.join(selected_channels)}")
    print(f"Target Samples: {target_samples}")
    print("\nCommands:")
    print("  Press 'c' + Enter: Calibration quadratique (11 points, 0-100g)")
    print("  Press '0' + Enter: Tare (set current mass as zero reference)")
    print("  Press 'r' + Enter: Lecture masse en continu (Ctrl+C pour arrêter)")
    print("  Press 'p' + Enter: Start recording (will record until target samples reached)")
    print("  Press 't' + Enter: Change target number of samples")
    print("  Press 'm' + Enter: Change recording/plotting mode")
    print("\nMode Options:")
    print("  0: Mass only")
    print("  1: Pose (Position) only")
    print("  2: PID only")
    print("  3: Error only")
    print("  4: All channels")
    print("  Combinations: '0;1', '1;2', '0;1;2', etc.")
    print("\nWaiting for commands...\n")


def read_mass_live():
    """Read mass continuously and print to terminal. Press Enter to stop."""
    global stop_flag

    if not calibration_done:
        print("\n❌ Calibration non effectuée. Faites d'abord 'c'.\n")
        return

    print("\n" + "=" * 60)
    print("  LECTURE MASSE EN CONTINU")
    print("  Appuyez sur ENTREE pour arrêter")
    print("=" * 60)
    print(f"  {'Bits':>10}  {'Std':>8}  {'Masse brute':>14}  {'Masse tarée':>14}")
    print("  " + "-" * 52)

    # Start reader thread
    old_stop = stop_flag
    stop_flag = False
    cal_samples.clear()
    t_read = threading.Thread(target=cal_reader_thread, daemon=True)
    t_read.start()

    # Background thread to wait for Enter key
    enter_pressed = threading.Event()
    def wait_enter():
        try:
            input()
        except EOFError:
            pass
        enter_pressed.set()

    t_enter = threading.Thread(target=wait_enter, daemon=True)
    t_enter.start()

    try:
        while not enter_pressed.is_set():
            if len(cal_samples) >= 10:
                avg = statistics.mean(list(cal_samples)[-10:])
                std = statistics.stdev(list(cal_samples)[-10:]) if len(cal_samples) >= 10 else 0.0
                m_brut = convert_courant(avg)
                m_tare = displayed_mass(avg)
                print(f"\r  {avg:10.2f}  {std:8.3f}  {m_brut:12.3f} g  {m_tare:12.3f} g", end="", flush=True)
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass

    stop_flag = True
    time.sleep(0.1)
    stop_flag = old_stop
    print("\n\n✓ Lecture arrêtée.\n")


def keyboard_listener():
    """Listen for keyboard input"""
    global stop_flag, current_mode, selected_channels, recording, target_samples, recording_enabled, tare_bits
    while True:
        try:
            key = input()
            if key.lower() == 'c':
                # Perform calibration
                calibrate()
                print_main_menu()
            elif key.lower() == 'r':
                # Read mass continuously
                read_mass_live()
                print_main_menu()
            elif key.lower() == '0':
                # Perform tare
                if not calibration_done:
                    print("\n❌ Calibration non effectuée. Faites d'abord 'c'.\n")
                    continue
                print("\n⏳ Taring... collecting samples")
                cal_samples.clear()
                # Start temp cal reader for tare
                old_stop = stop_flag
                stop_flag = False
                t_tare = threading.Thread(target=cal_reader_thread, daemon=True)
                t_tare.start()
                cal_wait_for_samples(N_AVG)
                tare_bits = cal_get_avg()
                stop_flag = True
                time.sleep(0.1)
                stop_flag = old_stop
                masse_offset = convert_courant(tare_bits)
                print(f"✓ Tare completed ({N_AVG} samples)")
                print(f"  Tare bits: {tare_bits:.3f}  |  offset = {masse_offset:.3f} g  |  Masse affichée = 0.00 g\n")
                
            elif key.lower() == 'p':
                # Start recording
                recording = True
                recording_enabled = True
                print(f"\n✓ Recording started. Target: {target_samples} samples\n")
            elif key.lower() == 't':
                # Change target number of samples
                print("\nSample Configuration:")
                while True:
                    try:
                        samples_input = input("Enter target number of samples: ").strip()
                        samples = int(samples_input)
                        if samples > 0:
                            target_samples = samples
                            print(f"✓ Target set to: {target_samples} samples\n")
                            break
                        else:
                            print("Please enter a positive number")
                    except ValueError:
                        print("Invalid input. Please enter a valid number.")
            elif key.lower() == 'm':
                print("\nMode Selection:")
                print("  0: Mass")
                print("  1: Pose (Position)")
                print("  2: PID")
                print("  3: Error")
                print("  4: All channels")
                print("  Or enter combination: '0;1' for mass+pose, '1;2' for pose+pid, etc.")
                
                while True:
                    mode_input = input("Enter mode (0-4 or combination): ").strip()
                    channels = parse_mode_input(mode_input)
                    
                    if channels is not None:
                        selected_channels = channels
                        current_mode = mode_input
                        print(f"Mode set to: {', '.join(selected_channels)}")
                        print("Recording will save and plot only selected channels.\n")
                        break
        except:
            pass


def main():
    """Main program loop"""
    global stop_flag, sample_count, selected_channels, current_mode, recording, target_samples, recording_enabled, ser
    
    # Get Arduino port from user
    port = get_arduino_port()
    
    # Try to connect to Arduino
    try:
        ser = serial.Serial(port, BAUD)
        print(f"✓ Connected to Arduino on {port} at {BAUD} baud")
    except Exception as e:
        print(f"\n✗ Error connecting to Arduino on {port}: {e}")
        print("Please check:")
        print("  - The port number is correct")
        print("  - The Arduino is connected")
        print("  - The Arduino driver is installed")
        sys.exit(1)
    
    # Initialize with default mode (all channels)
    selected_channels = MODE_MAP[4]
    current_mode = "4"
    recording = False
    recording_enabled = False
    
    # Charger calibration précédente si disponible
    if load_calibration():
        print("✓ Calibration précédente chargée automatiquement.\n")
    else:
        print("⚠ Aucune calibration sauvegardée. Utilisez 'c' pour calibrer.\n")
    
    # Start keyboard thread
    thread = threading.Thread(target=keyboard_listener, daemon=True)
    thread.start()
    
    print_main_menu()
    
    csv_path = create_csv_file()
    
    while True:
        # Auto-stop recording when target samples reached
        if recording_enabled and sample_count >= target_samples:
            print(f"\n✓ Target samples reached ({sample_count} samples collected)")
            
            # Calculate final mass value
            if data['masse']:
                final_mass = data['masse'][-1]
                print(f"\n{'='*60}")
                print(f"FINAL MASS: {final_mass:.2f} g")
                print(f"{'='*60}\n")
            
            print("Generating CSV and plots...")
            
            # Save data to CSV
            save_to_csv(csv_path)
            
            # Plot data
            plot_data()
            
            # Reset for next recording
            data['masse'].clear()
            data['pose'].clear()
            data['pid'].clear()
            data['error'].clear()
            data['sample'].clear()
            sample_count = 0
            recording = False
            recording_enabled = False
            
            # Réinitialise les filtres de masse pour ne pas conserver de vieilles valeurs
            mass_filter.reset()
            mass_lowpass.reset()
            
            csv_path = create_csv_file()
            print(f"Recording stopped. Ready for new recording.")
            
            # Reprint main menu
            print_main_menu()
        
        # Collect data only if recording is enabled
        if recording_enabled:
            try:
                receive_packet()
            except:
                pass
        else:
            # Small delay to prevent busy waiting
            time.sleep(0.01)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nClosing serial connection...")
        if ser is not None:
            ser.close()
        print("Done.")