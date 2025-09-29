import pygame
import time
import socket

# --- UDP setup ---
kart_ip = "100.115.20.7"   # kart's Tailscale IP
kart_port = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# --- Joystick setup ---
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No controller found!")
    exit()

controller = pygame.joystick.Joystick(0)
controller.init()
print(f"Connected to: {controller.get_name()}")

driving_mode = 0

# --- Main loop ---
while True:
    pygame.event.pump()

    left_shifter = controller.get_button(0)
    right_shifter = controller.get_button(1)
    
    steering = controller.get_axis(0)   # Steering wheel
    throttle = controller.get_axis(6)   # Gas pedal
    brake = controller.get_axis(1)      # Brake pedal

    if left_shifter or right_shifter:
        driving_mode = max(min(driving_mode - left_shifter, driving_mode), 0)
        driving_mode = min(max(driving_mode + right_shifter, driving_mode), 3)
        time.sleep(0.25)

    throttle_norm = min(-throttle + 1, 1)   # normalize 0..1
    brake_norm = min(-brake + 1, 1)         # normalize 0..1

    # Print for debugging
    print(f"S: {steering:.2f} T: {throttle_norm:.2f} "
          f"B: {brake_norm:.2f} M: {driving_mode:.2f}")

    # --- Send over UDP ---
    data = f"{steering:.2f},{throttle_norm:.2f},{brake_norm:.2f},{driving_mode}"
    sock.sendto(data.encode(), (kart_ip, kart_port))

    time.sleep(0.2)