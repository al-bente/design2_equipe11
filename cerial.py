import serial
import matplotlib.pyplot as plt
import threading
import os
import csv
from datetime import datetime
import platform
import sys

# ===== CONFIGURATION =====
PORT = '/dev/ttyACM0'
BAUD = 115200
DIR_PATH = r"./Recordings"

HANDSHAKE_START = 255
HANDSHAKE_STOP = 254

# Configure which data streams to plot (True = plot, False = ignore)
PLOT_CONFIG = {
    'courant': False,   # Current (amps)
    'pose': True,      # Position (mm)
    'pid': False,       # PID output
    'error': False      # Error signal
}

# Mode mapping: 0=courant, 1=pose, 2=pid, 3=error, 4=all
MODE_MAP = {
    0: ['courant'],
    1: ['pose'],
    2: ['pid'],
    3: ['error'],
    4: ['courant', 'pose', 'pid', 'error']
}

PLOT_INFO = {
    'courant': ("Current (A)", 'steelblue'),
    'pose': ("Position (mm)", 'forestgreen'),
    'pid': ("PID Output", 'coral'),
    'error': ("Error", 'mediumpurple')
}


def get_arduino_port():
    """Get Arduino port from user with OS-specific format guidance"""
    system = platform.system()
    
    print("\n" + "="*60)
    print("Arduino Port Configuration")
    print("="*60)
    
    if system == "Linux":
        print("Linux detected")
        print("Common formats: /dev/ttyACM0, /dev/ttyUSB0, /dev/ttyUSB1, etc.")
        example = "/dev/ttyACM0"
    elif system == "Darwin":
        print("macOS detected")
        print("Common formats: /dev/tty.usbserial-*, /dev/cu.usbserial-*, etc.")
        example = "/dev/tty.usbserial-14110"
    elif system == "Windows":
        print("Windows detected")
        print("Common formats: COM3, COM4, COM5, etc.")
        example = "COM3"
    else:
        print("Unknown OS")
        example = "/dev/ttyACM0"
    
    while True:
        port = input(f"Enter Arduino port (example: {example}): ").strip()
        
        if not port:
            print("Port cannot be empty. Please try again.")
            continue
        
        # Basic validation
        if system == "Windows":
            if port.upper().startswith("COM") and port.upper()[3:].isdigit():
                return port.upper()
            else:
                print(f"Invalid Windows port format. Please use COMxx (e.g., COM3)")
                continue
        else:
            if port.startswith("/dev/"):
                return port
            else:
                print(f"Invalid port format. Please use /dev/ttyXXX format")
                continue


ser = None

data = {
    'courant': [],
    'pose': [],
    'pid': [],
    'error': [],
    'sample': []
}

stop_flag = False
sample_count = 0
current_mode = None
selected_channels = []
command_sent = None  # Track which command was sent to PWM ('a' for asservissement, 'e' for échelon, None if none)
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
    """Save collected data to CSV file based on selected channels"""
    if not data['sample']:
        print("No data to save")
        return
    
    with open(csv_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        
        # Create header based on selected channels
        header = ["Sample"]
        for channel in selected_channels:
            header.append(f"{channel.capitalize()}")
        writer.writerow(header)
        
        # Write data for selected channels only
        for i in range(len(data['sample'])):
            row = [data['sample'][i]]
            for channel in selected_channels:
                row.append(data[channel][i] if i < len(data[channel]) else '')
            writer.writerow(row)
    
    print(f"Data saved to: {csv_path}")


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


def receive_packet():
    """Receive a data packet in format: 255, courant,pose,pid,error, 254"""
    global sample_count, target_samples, recording_enabled
    
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
                courant = float(parts[0])
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
                data['courant'].append(courant)
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
    print("  Press 'p' + Enter: Start recording (will record until target samples reached)")
    print("  Press 't' + Enter: Change target number of samples")
    print("  Press 'm' + Enter: Change recording/plotting mode")
    print("\nMode Options:")
    print("  0: Courant (Current) only")
    print("  1: Pose (Position) only")
    print("  2: PID only")
    print("  3: Error only")
    print("  4: All channels")
    print("  Combinations: '0;1', '1;2', '0;1;2', etc.")
    print("\nWaiting for commands...\n")



def keyboard_listener():
    """Listen for keyboard input"""
    global stop_flag, current_mode, selected_channels, recording, target_samples, recording_enabled
    while True:
        try:
            key = input()
            if key.lower() == 'p':
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
                print("  0: Courant (Current)")
                print("  1: Pose (Position)")
                print("  2: PID")
                print("  3: Error")
                print("  4: All channels")
                print("  Or enter combination: '0;1' for courant+pose, '1;2' for pose+pid, etc.")
                
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
    
    # Start keyboard thread
    thread = threading.Thread(target=keyboard_listener, daemon=True)
    thread.start()
    
    print_main_menu()
    
    csv_path = create_csv_file()
    
    while True:
        # Auto-stop recording when target samples reached
        if recording_enabled and sample_count >= target_samples:
            print(f"\n✓ Target samples reached ({sample_count} samples collected)")
            print("Generating CSV and plots...")
            
            # Save data to CSV
            save_to_csv(csv_path)
            
            # Plot data
            plot_data()
            
            # Reset for next recording
            data['courant'].clear()
            data['pose'].clear()
            data['pid'].clear()
            data['error'].clear()
            data['sample'].clear()
            sample_count = 0
            recording = False
            recording_enabled = False
            
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
            import time
            time.sleep(0.01)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nClosing serial connection...")
        if ser is not None:
            ser.close()
        print("Done.")
