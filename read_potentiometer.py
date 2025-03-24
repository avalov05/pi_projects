import time # type: ignore
import board # type: ignore
import busio # type: ignore
import adafruit_ads1x15.ads1115 as ADS # type: ignore
from adafruit_ads1x15.analog_in import AnalogIn # type: ignore
from luma.core.interface.serial import i2c # type: ignore
from luma.core.render import canvas # type: ignore  
from luma.oled.device import ssd1306 # type: ignore

# Initialize I2C for ADS1115
i2c_bus = busio.I2C(board.SCL, board.SDA)

# Setup ADS1115 (Encoders)
ads = ADS.ADS1115(i2c_bus)
encoder1 = AnalogIn(ads, ADS.P0)  # First encoder
encoder2 = AnalogIn(ads, ADS.P1)  # MA3 Absolute Encoder

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

while True:
    try:
        # Read encoder values with error handling
        try:
            # First encoder
            encoder1_value = encoder1.value
            encoder1_voltage = encoder1.voltage
            
            # MA3 encoder
            encoder2_voltage = encoder2.voltage
            encoder2_angle = voltage_to_angle(encoder2_voltage)
            
        except Exception as e:
            encoder1_value = "Error"
            encoder1_voltage = 0.0
            encoder2_voltage = 0.0
            encoder2_angle = 0.0
            print(f"Error reading encoders: {e}")
            time.sleep(0.5)  # Wait a bit longer when there's an error
            continue

        # Draw on the display
        with canvas(device) as draw:
            draw.rectangle(device.bounding_box, fill="black")
            
            # Display first encoder
            draw.text((0, 0), "Encoder 1:", fill="white")
            draw.text((0, 10), f"Val: {encoder1_value}", fill="white")
            draw.text((0, 20), f"V: {encoder1_voltage:.2f}V", fill="white")
            
            # Display MA3 encoder
            draw.text((0, 35), "MA3 Encoder:", fill="white")
            draw.text((0, 45), f"V: {encoder2_voltage:.2f}V", fill="white")
            draw.text((0, 55), f"Angle: {encoder2_angle:.1f}°", fill="white")
        
        time.sleep(0.1)
    
    except Exception as e:
        # Catch any other errors to prevent complete program crash
        print(f"Unexpected error: {e}")
        time.sleep(1)  # Wait a bit before retrying