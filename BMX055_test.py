import math
import BMX055
#from library import BMP085
import time
#import numpy as np
azimuth = 0
mag = [0, 0, 0]

def calcAzimuth():  # 方位角計算用関数
    global azimuth
    if mag[0] == 0 and mag[2] == 0:
        azimuth = 0
    else:
        azimuth = 90 - math.degrees(math.atan2(mag[1], mag[0]))
#    azimuth %= 360.0
    print(f"azimuth: {azimuth}")
    # if mag[1] == 0.0:
    #     mag[1] = 0.0000001
    # azimuth = -(180 / math.pi) * math.atan(mag[2] / mag[1])
    # if mag[1] > 0:
    #     azimuth = 90 + azimuth
    # elif mag[1] < 0:
    #     azimuth = -90 + azimuth


bmx = BMX055.BMX055()
#bmp = BMP085.BMP085()

bmx.setUp()

# a = np.array([])
# 
# for var in range(500):
#     alt = bmp.read_altitude()
#     a = np.append(alt, a)
# print(np.mean(a))
# 
# time.sleep(5)
# a = np.array([])
print("start")
    
# for var in range(500):
#     alt = bmp.read_altitude()
#     a = np.append(alt, a)
#     
# print(np.mean(a))
while True:
#     alt = bmp.read_altitude()
#     print(alt)
     acc = bmx.getAcc()
#      gyro = bmx.getGyro()
     mag = bmx.getMag()
     #print(gyro
     calcAzimuth()
     print(mag)
     print("-----------")
     time.sleep(0.2)
