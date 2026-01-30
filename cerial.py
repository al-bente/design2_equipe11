import serial
import struct

PORT = "/dev/ttyACM0"
BAUD = 115200

HEADER = b'\xCD\xAB'   # little-endian 0xABCD
PACKET_SIZE = 9


ser = serial.Serial(PORT, BAUD, timeout=1)

buffer = b""

print("Listening...")

while True:
    buffer += ser.read(64)

    while len(buffer) >= PACKET_SIZE:
        idx = buffer.find(HEADER)

        if idx == -1:
            buffer = buffer[-1:]  # keep last byte only
            break

        if len(buffer) < idx + PACKET_SIZE:
            break

        packet = buffer[idx:idx + PACKET_SIZE]
        buffer = buffer[idx + PACKET_SIZE:]

        _, pose, current, cmd = struct.unpack('<HHfB', packet)

        print(f"Pose: {pose}, Current: {current:.2f}, Cmd: {cmd}")
