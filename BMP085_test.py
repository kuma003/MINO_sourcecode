import BMP085
import time

# exposed with SCL = P9_19 and SDA = P9_20.
sensor = BMP085.BMP085()

# Optionally you can override the bus number:
#sensor = BMP085.BMP085(busnum=2)

# You can also optionally change the BMP085 mode to one of BMP085_ULTRALOWPOWER,
# BMP085_STANDARD, BMP085_HIGHRES, or BMP085_ULTRAHIGHRES.  See the BMP085
# datasheet for more details on the meanings of each mode (accuracy and power
# consumption are primarily the differences).  The default mode is STANDARD.
#sensor = BMP085.BMP085(mode=BMP085.BMP085_ULTRAHIGHRES)
while True:
    #print('Temp = {0:0.2f} *C'.format(sensor.read_temperature()))
    #print('Pressure = {0:0.2f} Pa'.format(sensor.read_pressure()))
    print('Altitude = {0:0.2f} m'.format(sensor.read_altitude() + 15))
    
    #print('Sealevel Pressure = {0:0.2f} Pa'.format(sensor.read_sealevel_pressure()))
