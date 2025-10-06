#!/usr/bin/env python3
"""
Simple Keyboard Control for Teensy 4.1 CAN Master
Requires: pip install pyserial pynput

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
from pynput import keyboard

# ===== CONFIGURATION =====
SERIAL_PORT = '/dev/ttyACM1'  # Change to COM3, COM4, etc. on Windows
BAUD_RATE = 115200

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
        # Add to held keys
        keys_held.add(key)
        
        # One-time actions
        if key == keyboard.Key.space:
            # Toggle brake
            state['brake'] = 0.0 if state['brake'] > 0 else 1.0
            send_command(f"B {state['brake']:.0f}")
            print(f"Brake: {'ON' if state['brake'] > 0 else 'OFF'}")
            
        elif key == keyboard.Key.enter:
            # Cycle drive mode
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
            # Throttle: UP arrow = full throttle
            if keyboard.Key.up in keys_held:
                if state['throttle'] != 1.0:
                    state['throttle'] = 1.0
                    send_command("T 1.0")
            else:
                if state['throttle'] != 0.0:
                    state['throttle'] = 0.0
                    send_command("T 0.0")
            
            # Steering: LEFT = -1, RIGHT = +1, neither = 0
            new_steer = 0.0
            if keyboard.Key.left in keys_held:
                new_steer = -1.0
            elif keyboard.Key.right in keys_held:
                new_steer = 1.0
            
            if state['steer'] != new_steer:
                state['steer'] = new_steer
                send_command(f"S {state['steer']:.1f}")
            
            time.sleep(0.05)  # 20Hz update
            
        except Exception as e:
            pass
    
# ===== MAIN =====
def main():
    global ser, running
    
    print("=" * 50)
    print("Simple CAN Master Keyboard Control")
    print("=" * 50)
    print(__doc__)
    
    # Open serial connection
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        time.sleep(2)
        print(f"\nConnected to {SERIAL_PORT}")
    except serial.SerialException as e:
        print(f"\nERROR: Could not open {SERIAL_PORT}")
        print(f"Details: {e}")
        sys.exit(1)
    
    # Initialize
    send_command("E 0")
    send_command("T 0")
    send_command("B 0")
    send_command("S 0")
    send_command("M N")
    
    print("\nReady! Controls active.")
    print("-" * 50)
    
    # Start update thread
    update_thread = threading.Thread(target=continuous_update, daemon=True)
    update_thread.start()
    
    # Start keyboard listener
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        try:
            listener.join()
        except KeyboardInterrupt:
            print("\nInterrupted")
    
    running = False
    time.sleep(0.1)
    
    # Cleanup
    if ser and ser.is_open:
        send_command("E 1")
        send_command("T 0")
        send_command("B 1")
        time.sleep(0.1)
        ser.close()
        print("Closed")

if __name__ == "__main__":
    main()
