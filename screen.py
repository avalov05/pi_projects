from luma.core.interface.serial import i2c # type: ignore
from luma.core.render import canvas # type: ignore  
from luma.oled.device import ssd1306 # type: ignore

# Create the I2C interface
serial = i2c(port=1, address=0x3C)

# Create the OLED device
device = ssd1306(serial)

while True:
# Draw something on the display
    with canvas(device) as draw:
        draw.rectangle(device.bounding_box, fill="black")
        draw.text((35, 30), "fuck off!", fill="white")