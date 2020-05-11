import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_SSD1306
import Python_SI1145.SI1145.SI1145 as SI1145
import subprocess
import RPi.GPIO as GPIO

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
from grove.i2c import Bus

#----------------------Display---------------------
# Raspberry Pi pin configuration:
RST = None     # on the PiOLED this pin isnt used
# Note the following are only used with SPI:
DC = 23
SPI_PORT = 0
SPI_DEVICE = 0

# 128x32 display with hardware I2C:
disp = Adafruit_SSD1306.SSD1306_128_32(rst=RST)

# Initialize library.
disp.begin()

# Clear display.
disp.clear()
disp.display()

# Create blank image for drawing.
# Make sure to create image with mode '1' for 1-bit color.
width = disp.width
height = disp.height

image = Image.new('1', (width, height))

# Get drawing object to draw on image.
draw = ImageDraw.Draw(image)

# Draw a black filled box to clear the image.
draw.rectangle((0,0,width,height), outline=0, fill=0)

# Draw some shapes.
# First define some constants to allow easy resizing of shapes.
padding = -2
top = padding
bottom = height-padding
# Move left to right keeping track of the current x position for drawing shapes.
x = 0
# Load default font.
font = ImageFont.load_default()

#------------Temperature Sensor-------------------
def CRC(data):
    crc = 0xff
    for s in data:
        crc ^= s
        for _ in range(8):
            if crc & 0x80:
                crc <<= 1
                crc ^= 0x131
            else:
                crc <<= 1
    return crc


class GroveTemperatureHumiditySensorSHT3x(object):

    def __init__(self, address=0x44, bus=None):
        self.address = address

        # I2C bus
        self.bus = Bus(bus)

    def read(self):
        # high repeatability, clock stretching disabled
        self.bus.write_i2c_block_data(self.address, 0x24, [0x00])

        # measurement duration < 16 ms
        time.sleep(0.016)

        # read 6 bytes back
        # Temp MSB, Temp LSB, Temp CRC, Humididty MSB, Humidity LSB, Humidity CRC
        data = self.bus.read_i2c_block_data(self.address, 0x00, 6)

        if data[2] != CRC(data[:2]):
            raise ValueError("temperature mhm-CRC mismatch")
        if data[5] != CRC(data[3:5]):
            raise ValueError("humidity CRC mismatch")


        temperature = data[0] * 256 + data[1]
        celsius = -45 + (175 * temperature / 65535.0)
        fahrenheit = celsius*(9/5) + 32
        humidity = 100 * (data[3] * 256 + data[4]) / 65535.0

        return fahrenheit, humidity


Grove = GroveTemperatureHumiditySensorSHT3x

#---------------Main Driver----------------------------
def main():
    
    sensor = GroveTemperatureHumiditySensorSHT3x()
    sensor_light = SI1145.SI1145()
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    
    while True:
        
        # Moisture reading
        mois = GPIO.input(23)
        if (mois == 1):
            mois_disp = 'Water: High'
            mois_command = 'Water levels stable'
        else:
            mois_disp = 'Water: Low'
            mois_command = 'Needs Water'
            
        # Light Reading
        vis = sensor_light.readVisible()
        IR = sensor_light.readIR()
        UV = sensor_light.readUV()
        uvIndex = UV / 100.0
        
        if (vis > 300):
            vis_command = 'Light levels stable'
        else:
            vis_command = 'Needs sunlight'
        
        # Temperature Reading
        temperature, humidity = sensor.read()
        if (temperature < 55):
            temp_command = 'Too cold'
        elif (temperature > 100):
            temp_command = 'Too hot'
        else:
            temp_command = 'Temperature stable'

        tempStr = ' Temp: {:.2f} F'.format(temperature)
        humStr = 'Humidity is {:.2f} %'.format(humidity)

        time.sleep(1)

        # Write first page of stats
        draw.rectangle((0,0,width,height), outline=0, fill=0)
        draw.text((x, top), str(tempStr),  font=font, fill=255)
        draw.text((x, top+8), str(humStr), font=font, fill=255)
        draw.text((x, top+16), temp_command,  font=font, fill=255)
       
        # Display image.
        disp.image(image)
        disp.display()
        time.sleep(1.5)
        
        # Display second image of stats
        draw.rectangle((0,0,width,height), outline=0, fill=0)
        draw.text((x, top), 'Vis: ' + str(vis),  font=font, fill=255)
        draw.text((x, top+8), 'IR: ' + str(IR),  font=font, fill=255)
        draw.text((x, top+16), 'UV Index: ' + str(uvIndex),  font=font, fill=255)
        draw.text((x, top+25), vis_command,  font=font, fill=255)
        
        # Display image.
        disp.image(image)
        disp.display()
        time.sleep(1)
        
        # Display third image of stats
        draw.rectangle((0,0,width,height), outline=0, fill=0)
        draw.text((x, top), mois_disp,  font=font, fill=255)
        draw.text((x, top+8), mois_command, font=font, fill=255)
        
        # Display image.
        disp.image(image)
        disp.display()
        time.sleep(.1)


if __name__ == "__main__":
    main()
