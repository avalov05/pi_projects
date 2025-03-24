from adafruit_servokit import ServoKit #type: ignore
import time
import math

kit = ServoKit(channels=16)

# Initialize servo position
current_angle = 90
kit.servo[0].angle = current_angle

# Set movement parameters
center_angle = 90
amplitude = 90
period = 0.5  # Time for a complete cycle (in seconds)
update_rate = 50  # Updates per second

while True:
    for t in range(int(period * update_rate)):
        # Calculate target angle
        target_angle = center_angle + amplitude * math.sin(2 * math.pi * t / (period * update_rate))
        
        # Interpolate between current and target angle
        current_angle += (target_angle - current_angle) * 0.1
        
        # Set servo angle
        kit.servo[0].angle = current_angle
        
        # Wait for next update
        time.sleep(1 / update_rate)

