import time # type: ignore
import board # type: ignore
import busio # type: ignore
import adafruit_ads1x15.ads1115 as ADS # type: ignore
from adafruit_ads1x15.analog_in import AnalogIn # type: ignore
from luma.core.interface.serial import i2c # type: ignore
from luma.core.render import canvas # type: ignore  
from luma.oled.device import ssd1306 # type: ignore
from adafruit_servokit import ServoKit #type: ignore
import time
import math

kit = ServoKit(channels=16)

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
        encoder2_voltage = encoder2.voltage
        encoder2_angle = abs(voltage_to_angle(encoder2_voltage))

        # Convert angle to servo position (0-180 degrees)
        servo_position = encoder2_angle/400*180

        # Set servo position
        kit.servo[0].angle = servo_position

        with canvas(device) as draw:       
                # Display MA3 encoder
                draw.text((0, 35), "MA3 Encoder:", fill="white")
                draw.text((0, 45), f"V: {encoder2_voltage:.2f}V", fill="white")
                draw.text((0, 55), f"Angle: {encoder2_angle:.1f}°", fill="white")
    except Exception as e:
        print(servo_position)
    
    time.sleep(0.01)