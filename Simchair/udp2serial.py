import socket
import serial
import time
import sys

# --- UDP setup ---
listen_port = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', listen_port))

# --- Serial setup ---
# Update this to match your Teensy's serial port
# Windows: "COM3", "COM4", etc.
# Linux/Mac: "/dev/ttyUSB0", "/dev/ttyACM0", etc.
SERIAL_PORT = "/dev/ttyACM0"  # Change this to your Teensy's port
SERIAL_BAUD = 115200

try:
    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
    time.sleep(2)  # Give the serial connection time to establish
    print(f"Serial connection established on {SERIAL_PORT}")
except serial.SerialException as e:
    print(f"Failed to open serial port {SERIAL_PORT}: {e}")
    sys.exit(1)

print(f"UDP receiver listening on port {listen_port}")
print("Waiting for sim chair data...")

# --- Main loop ---
while True:
    try:
        # Receive UDP data
        data, addr = sock.recvfrom(1024)
        message = data.decode()
        
        # Parse the received data: "steering,throttle,brake,driving_mode"
        try:
            parts = message.split(',')
            if len(parts) == 4:
                steering = float(parts[0])      # -1 to 1
                throttle = float(parts[1])      # 0 to 1
                brake = float(parts[2])         # 0 to 1
                driving_mode = int(parts[3])    # 0, 1, 2, 3
                
                # Map driving mode to letters (same as your existing Master CAN expects)
                mode_map = {0: 'N', 1: 'D', 2: 'S', 3: 'R'}
                mode_char = mode_map.get(driving_mode, 'N')
                
                # Send commands to Teensy via serial using the exact format your Master CAN expects
                ser.write(f"T {throttle:.3f}\n".encode())
                ser.write(f"B {brake:.3f}\n".encode())
                ser.write(f"S {steering:.3f}\n".encode())
                ser.write(f"M {mode_char}\n".encode())
                
                # Print for debugging
                print(f"UDP->Serial: S={steering:.2f} T={throttle:.2f} B={brake:.2f} M={mode_char}")
                
        except (ValueError, IndexError) as e:
            print(f"Error parsing UDP data '{message}': {e}")
            continue
            
    except KeyboardInterrupt:
        print("\nShutting down...")
        break
    except Exception as e:
        print(f"Error: {e}")
        continue

# Cleanup
ser.close()
sock.close()
print("UDP receiver stopped.")