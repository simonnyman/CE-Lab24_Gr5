import smbus
import time

# Get I2C bus
bus = smbus.SMBus(1)

# ISL29125 address, 0x44(68)
# Select configuation-1register, 0x01(01)
# 0x0D(13) Operation: RGB, Range: 360 lux, Res: 16 Bits
bus.write_byte_data(0x44, 0x01, 0x05)

time.sleep(1)

print("Reading colour values and displaying them in a new window\n")

def getAndUpdateColour():
    while True:
	# Read the data from the sensor
        # Insert code here:

        # ISL29125 address, 0x44
        # Reads the data from 0x44, address of ISL29125 from register 0x09 (low green) to 0x0E (high blue)
        data = bus.read_i2c_block_data(0x44, 0x09, 6)

        # Convert the data to green, red and blue int values
        # Insert code here:
        
        green = data[1] * 256 + data[0]
        red = data[3] * 256 + data[2]
        blue = data[5] * 256 + data[4]
        blue=blue*1.8 # Compensates for the low blue readings
        
        # Output data to the console RGB values
        print("RGB(%d %d %d)" % (red, green, blue))

        # Display the most prominent color
        if (blue > red and blue > green):
            print("Blue")
        if (green > red and green > blue):
            print("green")
        else:
            print("red")

        time.sleep(2) 


for i in range (1000): 
    getAndUpdateColour()


