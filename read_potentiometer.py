import time # type: ignore
import math # type: ignore
import board # type: ignore
import busio # type: ignore
import adafruit_ads1x15.ads1115 as ADS # type: ignore
from adafruit_ads1x15.analog_in import AnalogIn # type: ignore
from luma.core.interface.serial import i2c # type: ignore
from luma.core.render import canvas # type: ignore  
from luma.oled.device import ssd1306 # type: ignore
from gpiozero import Servo # type: ignore

# Servo setup
servo = Servo(4)  # GPIO4

# Initialize I2C for ADS1115
i2c_bus = busio.I2C(board.SCL, board.SDA)

# Setup ADS1115 (Encoders)
ads = ADS.ADS1115(i2c_bus)
potentiometer = AnalogIn(ads, ADS.P0)  # Potentiometer on P0
encoder = AnalogIn(ads, ADS.P1)  # MA3 Absolute Encoder on P1

# Create the I2C interface for OLED
serial = i2c(port=1, address=0x3C)

# Create the OLED device
device = ssd1306(serial)

# Set a reasonable gain to prevent overflows
# Options: 2/3, 1, 2, 4, 8, 16
ads.gain = 1  # +/-4.096V range

# Function to convert MA3 voltage to angle (0-360 degrees)
def voltage_to_angle(voltage, v_ref=3.3):
    # MA3 outputs 0V to v_ref for 0 to 360 degrees
    # Calculate the angle based on the voltage ratio
    return (voltage / v_ref) * 360.0

# Function to convert potentiometer voltage to percentage (0-100%)
def voltage_to_percentage(voltage, v_ref=3.3):
    return (voltage / v_ref) * 100.0

# Function to move servo in a sinusoidal pattern
def move_servo_sin(t_value):
    # Use sine wave to generate smooth motion between -1 and 1
    # Scale to servo range (between -1 and 1)
    value = math.sin(t_value)
    servo.value = value
    return value  # Return the value for display purposes

try:
    # Move servo from max to min position
    print("Moving servo from max to min position...")
    
    # First move to max position
    servo.max()
    time.sleep(1)
    
    # Then move to min position
    servo.min()
    time.sleep(1)
    
    print("Servo movement complete. Starting main loop...")
    
    start_time = time.time()
    
    while True:
        try:
            # Calculate time for sinusoidal movement
            current_time = time.time() - start_time
            
            # Move servo in sinusoidal pattern
            servo_value = move_servo_sin(current_time)
            
            # Read sensor values with error handling
            try:
                # Potentiometer
                pot_value = potentiometer.value
                pot_voltage = potentiometer.voltage
                pot_percentage = voltage_to_percentage(pot_voltage)
                
                # MA3 encoder
                encoder_voltage = encoder.voltage
                encoder_angle = voltage_to_angle(encoder_voltage)
                
            except Exception as e:
                pot_value = "Error"
                pot_voltage = 0.0
                pot_percentage = 0.0
                encoder_voltage = 0.0
                encoder_angle = 0.0
                print(f"Error reading sensors: {e}")
                time.sleep(0.5)
                continue

            # Draw on the display
            with canvas(device) as draw:
                draw.rectangle(device.bounding_box, fill="black")
                
                # Display potentiometer
                draw.text((0, 0), "Potentiometer:", fill="white")
                draw.text((0, 10), f"V: {pot_voltage:.2f}V", fill="white")
                draw.text((0, 20), f"Position: {pot_percentage:.1f}%", fill="white")
                
                # Display MA3 encoder
                draw.text((0, 35), "MA3 Encoder:", fill="white")
                draw.text((0, 45), f"V: {encoder_voltage:.2f}V", fill="white")
                draw.text((0, 55), f"Angle: {encoder_angle:.1f}°", fill="white")
            
            time.sleep(0.05)  # Shorter sleep for smoother servo movement
        
        except Exception as e:
            print(f"Unexpected error: {e}")
            time.sleep(1)

except KeyboardInterrupt:
    print("Program stopped by user")
finally:
    # Clean up
    servo.close()
    print("GPIO cleaned up")