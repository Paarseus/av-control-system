#!/usr/bin/env python3
"""
Go-Kart Keyboard Control with Data Logging
Requires: pip install pyserial pynput opencv-python

Logs: throttle, steering, brake, timestamps, and dual camera images
Creates: CSV log file and timestamped image folders

Controls:
  UP ARROW    - Full throttle (hold for throttle, release stops)
  LEFT ARROW  - Full steer left (hold)
  RIGHT ARROW - Full steer right (hold)
  SPACE       - Toggle brake ON/OFF
  ENTER       - Cycle drive mode (N -> D -> S -> R -> N)
  E           - Toggle E-STOP
  ESC         - Exit
"""

import serial
import time
import sys
import threading
import csv
import os
from datetime import datetime
from pathlib import Path
import cv2
from pynput import keyboard

# ===== CONFIGURATION =====
SERIAL_PORT = '/dev/ttyACM0'  # Change to COM3, COM4, etc. on Windows
BAUD_RATE = 115200

# Camera configuration
CAMERA_1_ID = 0  # First camera (usually built-in or first USB camera)
CAMERA_2_ID = 1  # Second camera (second USB camera)
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
FPS = 30

# Logging configuration
LOG_RATE_HZ = 20  # Data logging frequency
DATA_DIR = 'gokart_data'

# ===== STATE =====
state = {
    'throttle': 0.0,
    'steer': 0.0,
    'brake': 0.0,
    'mode': 'N',
    'estop': False,
    'modes': ['N', 'D', 'S', 'R']
}

ser = None
keys_held = set()
running = True

# Logging variables
csv_file = None
csv_writer = None
cam1 = None
cam2 = None
log_counter = 0
session_dir = None

# ===== SETUP LOGGING =====
def setup_logging():
    """Create directories and files for logging session"""
    global csv_file, csv_writer, session_dir
    
    # Create main data directory
    Path(DATA_DIR).mkdir(exist_ok=True)
    
    # Create session directory with timestamp
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    session_dir = Path(DATA_DIR) / f'session_{timestamp}'
    session_dir.mkdir(exist_ok=True)
    
    # Create subdirectories for images
    (session_dir / 'camera1').mkdir(exist_ok=True)
    (session_dir / 'camera2').mkdir(exist_ok=True)
    
    # Create CSV file
    csv_path = session_dir / 'control_log.csv'
    csv_file = open(csv_path, 'w', newline='')
    csv_writer = csv.writer(csv_file)
    
    # Write header
    csv_writer.writerow([
        'frame_id',
        'timestamp',
        'time_seconds',
        'throttle',
        'steering',
        'brake',
        'mode',
        'estop',
        'camera1_image',
        'camera2_image'
    ])
    csv_file.flush()
    
    print(f"\nLogging to: {session_dir}")
    return session_dir

# ===== CAMERA SETUP =====
def setup_cameras():
    """Initialize both cameras"""
    global cam1, cam2
    
    try:
        cam1 = cv2.VideoCapture(CAMERA_1_ID)
        cam1.set(cv2.CAP_PROP_FRAME_WIDTH, IMAGE_WIDTH)
        cam1.set(cv2.CAP_PROP_FRAME_HEIGHT, IMAGE_HEIGHT)
        cam1.set(cv2.CAP_PROP_FPS, FPS)
        
        cam2 = cv2.VideoCapture(CAMERA_2_ID)
        cam2.set(cv2.CAP_PROP_FRAME_WIDTH, IMAGE_WIDTH)
        cam2.set(cv2.CAP_PROP_FRAME_HEIGHT, IMAGE_HEIGHT)
        cam2.set(cv2.CAP_PROP_FPS, FPS)
        
        # Test cameras
        ret1, _ = cam1.read()
        ret2, _ = cam2.read()
        
        if ret1 and ret2:
            print("✓ Both cameras initialized successfully")
            return True
        else:
            print("⚠ Warning: One or both cameras failed to initialize")
            if not ret1:
                print(f"  Camera 1 (ID={CAMERA_1_ID}) failed")
            if not ret2:
                print(f"  Camera 2 (ID={CAMERA_2_ID}) failed")
            return False
            
    except Exception as e:
        print(f"⚠ Camera setup error: {e}")
        return False

# ===== SERIAL COMMUNICATION =====
def send_command(cmd):
    """Send command to Teensy via serial"""
    if ser and ser.is_open:
        try:
            ser.write(f"{cmd}\n".encode())
            ser.flush()
        except:
            pass

# ===== KEYBOARD HANDLING =====
def on_press(key):
    """Handle key press events"""
    global running
    
    try:
        keys_held.add(key)
        
        if key == keyboard.Key.space:
            state['brake'] = 0.0 if state['brake'] > 0 else 1.0
            send_command(f"B {state['brake']:.0f}")
            print(f"Brake: {'ON' if state['brake'] > 0 else 'OFF'}")
            
        elif key == keyboard.Key.enter:
            idx = state['modes'].index(state['mode'])
            state['mode'] = state['modes'][(idx + 1) % len(state['modes'])]
            send_command(f"M {state['mode']}")
            print(f"Mode: {state['mode']}")
            
        elif key == keyboard.Key.esc:
            print("\nExiting...")
            running = False
            return False
            
        elif hasattr(key, 'char') and key.char:
            c = key.char.upper()
            if c == 'E':
                state['estop'] = not state['estop']
                send_command(f"E {1 if state['estop'] else 0}")
                print(f"E-STOP: {'ON' if state['estop'] else 'OFF'}")
                
    except Exception as e:
        print(f"Error: {e}")

def on_release(key):
    """Handle key release events"""
    try:
        if key in keys_held:
            keys_held.remove(key)
    except:
        pass

# ===== CONTINUOUS UPDATE =====
def continuous_update():
    """Update vehicle state based on held keys"""
    global running
    
    while running:
        try:
            # Throttle
            if keyboard.Key.up in keys_held:
                if state['throttle'] != 1.0:
                    state['throttle'] = 1.0
                    send_command("T 1.0")
            else:
                if state['throttle'] != 0.0:
                    state['throttle'] = 0.0
                    send_command("T 0.0")
            
            # Steering
            new_steer = 0.0
            if keyboard.Key.left in keys_held:
                new_steer = -1.0
            elif keyboard.Key.right in keys_held:
                new_steer = 1.0
            
            if state['steer'] != new_steer:
                state['steer'] = new_steer
                send_command(f"S {state['steer']:.1f}")
            
            time.sleep(0.05)
            
        except Exception as e:
            pass

# ===== DATA LOGGING =====
def logging_loop():
    """Main logging loop - captures images and writes CSV data"""
    global running, log_counter, csv_writer
    
    start_time = time.time()
    
    while running:
        try:
            current_time = time.time()
            elapsed = current_time - start_time
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            
            # Capture images from both cameras
            img1_path = None
            img2_path = None
            
            if cam1 is not None and cam1.isOpened():
                ret1, frame1 = cam1.read()
                if ret1:
                    img1_filename = f'frame_{log_counter:06d}.jpg'
                    img1_path = session_dir / 'camera1' / img1_filename
                    cv2.imwrite(str(img1_path), frame1)
                    img1_path = f'camera1/{img1_filename}'
            
            if cam2 is not None and cam2.isOpened():
                ret2, frame2 = cam2.read()
                if ret2:
                    img2_filename = f'frame_{log_counter:06d}.jpg'
                    img2_path = session_dir / 'camera2' / img2_filename
                    cv2.imwrite(str(img2_path), frame2)
                    img2_path = f'camera2/{img2_filename}'
            
            # Write data to CSV
            csv_writer.writerow([
                log_counter,
                timestamp,
                f'{elapsed:.3f}',
                f'{state["throttle"]:.3f}',
                f'{state["steer"]:.3f}',
                f'{state["brake"]:.3f}',
                state['mode'],
                int(state['estop']),
                img1_path if img1_path else '',
                img2_path if img2_path else ''
            ])
            
            # Flush every 10 frames
            if log_counter % 10 == 0:
                csv_file.flush()
                if log_counter % 100 == 0:
                    print(f"Logged {log_counter} frames ({elapsed:.1f}s)")
            
            log_counter += 1
            
            # Sleep to maintain logging rate
            time.sleep(1.0 / LOG_RATE_HZ)
            
        except Exception as e:
            print(f"Logging error: {e}")
            time.sleep(0.1)

# ===== CLEANUP =====
def cleanup():
    """Clean up resources"""
    global running, csv_file, cam1, cam2, ser
    
    print("\nCleaning up...")
    running = False
    time.sleep(0.2)
    
    # Close CSV file
    if csv_file:
        csv_file.close()
        print(f"✓ Saved {log_counter} log entries")
    
    # Release cameras
    if cam1 is not None:
        cam1.release()
    if cam2 is not None:
        cam2.release()
    print("✓ Released cameras")
    
    # Send safe commands and close serial
    if ser and ser.is_open:
        send_command("E 1")
        send_command("T 0")
        send_command("B 1")
        time.sleep(0.1)
        ser.close()
        print("✓ Closed serial connection")
    
    cv2.destroyAllWindows()
    print(f"\nSession saved to: {session_dir}")

# ===== MAIN =====
def main():
    global ser, running
    
    print("=" * 60)
    print("Go-Kart Control with Data Logging")
    print("=" * 60)
    
    # Setup logging
    setup_logging()
    
    # Setup cameras
    cameras_ok = setup_cameras()
    if not cameras_ok:
        print("\nProceed without cameras? (y/n): ", end='')
        if input().lower() != 'y':
            print("Exiting...")
            sys.exit(1)
    
    # Open serial connection
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        time.sleep(2)
        print(f"✓ Connected to {SERIAL_PORT}")
    except serial.SerialException as e:
        print(f"\n✗ ERROR: Could not open {SERIAL_PORT}")
        print(f"Details: {e}")
        cleanup()
        sys.exit(1)
    
    # Initialize vehicle
    send_command("E 0")
    send_command("T 0")
    send_command("B 0")
    send_command("S 0")
    send_command("M N")
    
    print("\n" + "=" * 60)
    print("READY! Logging started.")
    print("=" * 60)
    print(__doc__)
    
    # Start threads
    update_thread = threading.Thread(target=continuous_update, daemon=True)
    update_thread.start()
    
    logging_thread = threading.Thread(target=logging_loop, daemon=True)
    logging_thread.start()
    
    # Start keyboard listener
    try:
        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()
    except KeyboardInterrupt:
        print("\nInterrupted")
    
    cleanup()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\nFatal error: {e}")
        cleanup()
        sys.exit(1)
