import json
import serial
import time
import math
import threading
import datetime
import pigpio
import csv
import os
import cv2
import RPi.GPIO as GPIO

# import wiringpi as pi
import BMX055
import BMP085
from micropyGPS import MicropyGPS
import detect_corn as dc
from picamera2 import Picamera2

# import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
import sys
import asyncio
import websockets
import cv2
import base64


# 定数　上書きしない
PORT = 8756
SERVER_URL = "192.168.127.98"
URI = f"ws://{SERVER_URL}:{PORT}"
JPEG_QUALITY = 70  # 圧縮品質 (0-100)
ENCODE_PARAM = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
MAG_CONST = 8.9  # 地磁気補正用の偏角
CALIBRATION_MILLITIME = 20 * 1000
TARGET_LAT = 38.260720666666664
TARGET_LNG = 140.85434316666667
TARGET_ALTITUDE = 20
DATA_SAMPLING_RATE = 0.00001
ALTITUDE_CONST1 = 30
ALTITUDE_CONST2 = 5
HIGH = 1
LOW = 0
# Pin number
heating_wire = 9
M1A = 19  # Motor
M1B = 13
M4A = 5
M4B = 6


# 変数
acc = [0.0, 0.0, 0.0]
gyro = [0.0, 0.0, 0.0]
mag = [0.0, 0.0, 0.0]
calibBias = [0.0, 0.0, 0.0]
calibRange = [1.0, 1.0, 1.0]
lat = 0.0  # from GPS sensor
lng = 0.0
alt = 0.0
maxAlt = 0.0
minAlt = 0.0
pres = 0.0
distance = 0
angle = 0.0
azimuth = 0.0
direction = 0.0
frequency = 50
phase = 0
gps_detect = 0
cone_direction = 0
cone_probability = 0
restTime = 0.0
diff_rot = 1
upside_down_Flag = 0  # judge the upside down by acc(bmx)
stuck_GPS_Flag = 0  # judge the stuck by GPS : no obstacle distance_Flag = 0, if CanSat stucked distance_Flag = 1


bmx = BMX055.BMX055()
# bmp = BMP085.BMP085()

nowTime = datetime.datetime.now()
fileName = "./log/testlog_" + nowTime.strftime("%Y-%m%d-%H%M%S") + ".csv"


async def main():
    global phase
    global restTime
    global start
    global gps_detect
    time_camera_start = 0
    time_camera_detecting = 0
    n_camera_mode = 0

    phase = 0
    n = 0

    while True:
        if phase == 0:  # 投下
            print("phase0 : falling")
            start = time.time()
            while True:
                getBmxData()
                # print(fall)
                if fall > 15:
                    print("para released")
                    await asyncio.sleep(10)
                    break
                if time.time() - start > 5 * 60:
                    phase = 1
                await asyncio.sleep(0)
            phase = 1

        elif phase == 1:  # パラ分離
            print("phase1 : remove para")
            print("fire")
            GPIO.output(heating_wire, GPIO.HIGH)
            await asyncio.sleep(3)
            GPIO.output(heating_wire, GPIO.LOW)
            print("done")
            phase = 2

        elif phase == 2:  # キャリブレーション
            print("phase2 : calibration start")
            calibration()
            phase = 3

        elif phase == 3:
            print("phase3 : GPS start")
            if n_camera_mode <= 1:
                if distance < 5.0:
                    phase = 4
            elif n_camera_mode == 2:  # not using camera mode
                if distance < 1.0:
                    phase = 6  # goal
            #            if upside_down == True:
            #                phase = -2
            if distance < 1.0:  # GPS座標との距離 < m以内　　#スタック優先
                phase = 6

        elif phase == 4:
            print("phase4 : camera start")
            time_camera_start = time.time()
            cone_detect()
            if cone_probability < 1:
                phase = 5
            if n > 5:
                phase = 6
        elif phase == 5:
            print("phase5")
            while True:
                cone_detect()
                time_camera_detecting = time.time()
                if (
                    time_camera_detecting - time_camera_start >= 60
                ):  # camera mode failed
                    n_camera_mode += 1
                    phase = 3  # restart GPS mode
                    break
                if detector.is_reached:  # if rover reached cone
                    print("reached")
                    phase = 6
                    break
        #                 if cone_probability > 1: # detect cone probability
        #                     n += 1
        #                     phase = 4

        elif phase == 6:
            print("phase6 : Goal")
            await asyncio.sleep(10000)
        #         elif phase==-1:
        #             print("phase-1 : stuck")
        #             if object_distance_Flag ==0:
        #                 phase = -1
        #             elif object_distance_Flag == 1:
        #                 phase = 3
        #                 object_distance_Flag = 0
        #         elif phase ==-2:
        #             print("phase-2 : upside down")
        #             if  upside_down_Flag == 0:
        #                 phase = -2
        #             elif upside_down_Flag == 1:
        #                 phase = 3
        #

        asyncio.sleep(0.1)


def currentMilliTime():
    return round(time.time() * 1000)


def Setup():
    global detector
    bmx.setUp()

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(heating_wire, GPIO.OUT)

    with open(fileName, "a") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "MilliTime",
                "Phase",
                "AccX",
                "AccY",
                "AccZ",
                "GyroX",
                "GyroY",
                "GyroZ",
                "MagX",
                "MagY",
                "MagZ",
                "LAT",
                "LNG",
                "ALT",
                "Distance",
                "Object_Distance",
                "Azimuth",
                "Angle",
                "Direction",
                "Fall",
            ]
        )

    getThread = threading.Thread(target=moveMotor_thread, args=())
    getThread.daemon = True
    getThread.start()

    # dataThread = threading.Thread(target=setData_thread, args=())
    # dataThread.daemon = True
    # dataThread.start()

    gpsThread = threading.Thread(target=GPS_thread, args=())
    gpsThread.daemon = True
    gpsThread.start()

    detector = dc.detector()
    roi_img = cv2.imread("./log/captured.png")

    roi_img = cv2.cvtColor(roi_img, cv2.COLOR_BGR2RGB)
    detector.set_roi_img(roi_img)

    print("Setup OK")


def getBmxData():  # get BMX data
    global acc
    global gyro
    global mag
    global fall
    acc = bmx.getAcc()
    gyro = bmx.getGyro()
    mag = bmx.getMag()
    # mag[1] = mag[1]
    # mag[2] = mag[2]
    fall = math.sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2])
    for i in range(3):
        mag[i] = (mag[i] - calibBias[i]) / calibRange[i]


def getBmpData():
    global alt
    global pres
    alt = bmp.read_altitude()
    pres = bmp.read_pressure()

    alt = alt + 60  # calibration


def flying():  # 落下検知関数 :飛んでいるときはTrueを返し続ける
    # この関数は何回も繰り返されることを想定
    global maxAlt
    global minAlt

    if maxAlt < alt:
        maxAlt = alt
    if minAlt > alt:
        minAlt = alt
    subAlt = maxAlt - minAlt
    absAlt = abs(alt - minAlt)

    if subAlt > ALTITUDE_CONST1 and absAlt < ALTITUDE_CONST2:
        print("bmp : reached ground")
        time.sleep(10)
        return False

    else:
        True


def upside_down():
    global upside_down_Flag
    global acc
    if acc[0] > 0:
        upside_down_Flag = 1


def calibration():  # calibrate BMX raw data
    global calibBias
    global calibRange
    max = [0.0, 0.0, 0.0]
    min = [0.0, 0.0, 0.0]
    max[0] = mag[0]
    max[1] = mag[1]
    max[2] = mag[2]
    min[0] = mag[0]
    min[1] = mag[1]
    min[2] = mag[2]

    complete = False
    while complete == False:
        before = currentMilliTime()
        after = before
        while (after - before) < CALIBRATION_MILLITIME:
            getBmxData()
            if max[0] < mag[0]:
                max[0] = mag[0]
            elif min[0] > mag[0]:
                min[0] = mag[0]
            elif max[2] < mag[2]:
                max[2] = mag[2]
            elif min[2] > mag[2]:
                min[2] = mag[2]
            after = currentMilliTime()
        if (max[0] - min[0]) > 20 and (max[2] - min[2] > 20):
            print("calibration(): Complete!")
            complete = True
            asyncio.sleep(1)
            calibBias[0] = (max[0] + min[0]) / 2
            calibBias[2] = (max[2] + min[2]) / 2

            calibRange[0] = (max[0] - min[0]) / 2
            calibRange[2] = (max[2] - min[2]) / 2
            asyncio.sleep(2)


def calcdistance():  # 距離計算用関
    global distance
    EARTH_RADIUS = 6378136.59
    dx = (math.pi / 180) * EARTH_RADIUS * (TARGET_LNG - lng)
    dy = (math.pi / 180) * EARTH_RADIUS * (TARGET_LAT - lat)
    distance = math.sqrt(dx * dx + dy * dy)
    # print(f"distance: {distance}")


def calcAngle():  # 角度計算用関数 : north=0 east=90 west = -90
    global angle
    forEAstAngle = 0.0
    EARTH_RADIUS = 6378136.59

    dx = (math.pi / 180) * EARTH_RADIUS * (TARGET_LNG - lng)
    dy = (math.pi / 180) * EARTH_RADIUS * (TARGET_LAT - lat)
    angle = 90 - math.degrees(math.atan2(dy, dx))
    angle %= 360.0
    # if dx == 0 and dy == 0:
    #    forEastAngle = 0.0
    # else:
    #    forEastAngle = (180 / math.pi) * math.atan2(dy, dx)  # arctan
    # angle = forEastAngle - 90
    # if angle < -180:
    #    angle += 360
    # if angle > 180:
    #    angle -= 360
    # angle = -angle
    # print(f"angle: {angle}")


def calcAzimuth():  # 方位角計算用関数
    global azimuth

    azimuth = 90 - math.degrees(math.atan2(mag[2], -mag[0]))
    azimuth %= 360.0
    azimuth *= -1  # 上のazimuthはCanSatからみた北の方位
    # print(f"azimuth: {azimuth}")
    # if mag[1] == 0.0:
    #     mag[1] = 0.0000001
    # azimuth = -(180 / math.pi) * math.atan(mag[2] / mag[1])
    # if mag[1] > 0:
    #     azimuth = 90 + azimuth
    # elif mag[1] < 0:
    #     azimuth = -90 + azimuth


def GPS_thread():  # GPSモジュールを読み、GPSオブジェクトを更新する
    global lat
    global lng
    global gps_detect

    s = serial.Serial("/dev/serial0", 115200)
    s.readline()  # 最初の1行は中途半端なデーターが読めることがあるので、捨てる
    gps = MicropyGPS(9, "dd")

    while True:
        sentence = s.readline().decode("utf-8")  # GPSデーターを読み、文字列に変換する

        if s.in_waiting > 64:  # バッファを削除
            s.reset_input_buffer()
        if sentence[0] != "$":  # 先頭が'$'でなければ捨てる
            continue
        for (
            x
        ) in (
            sentence
        ):  # 読んだ文字列を解析してGPSオブジェクトにデーターを追加、更新する
            gps.update(x)
        lat = gps.latitude[0]
        lng = gps.longitude[0]

        if lat > 0:
            gps_detect = 1
        elif lat == 0.0:
            gps_detect = 0
        # print(lat)
        # print(lng)


def cone_detect():
    global detector
    global cone_direction
    global cone_probability
    global encoded_img_txt

    detector.detect_cone()
    cone_direction = 1 - detector.cone_direction
    cone_probability = detector.probability
    _, buffer = cv2.imencode(".jpg", detector.input_img, ENCODE_PARAM)
    encoded_img_txt = base64.b64encode(buffer).decode("utf-8")
    # print("direction", cone_direction)
    # print("prob.", cone_probability)


async def setData():
    counter = 0
    while True:
        try:
            print("connecting to server")
            async with websockets.connect(URI) as websocket:
                getBmxData()
                calcAngle()
                calcAzimuth()
                set_direction()
                calcdistance()
                # with open(fileName, "a", newline="") as f:
                #     writer = csv.writer(f)
                #     writer.writerow(
                #         [
                #             currentMilliTime(),
                #             round(phase, 1),
                #             acc[0],
                #             acc[1],
                #             acc[2],
                #             gyro[0],
                #             gyro[1],
                #             gyro[2],
                #             mag[1],
                #             mag[1],
                #             mag[2],
                #             lat,
                #             lng,
                #             alt,
                #             distance,
                #             azimuth,
                #             angle,
                #             direction,
                #             fall,
                #         ]
                #     )
                data = {
                    "time": currentMilliTime(),
                    "phase": round(phase, 1),
                    "acc": acc,
                    "gyro": gyro,
                    "mag": mag,
                    "lat": lat,
                    "lng": lng,
                    "alt": alt,
                    "distance": distance,
                    "azimuth": azimuth,
                    "angle": angle,
                    "direction": direction,
                    "fall": fall,
                }
                if phase >= 4 and encoded_img_txt is not None and counter % 10 == 0:
                    data.update(
                        {
                            "img": encoded_img_txt,
                            "cone_direction": cone_direction,
                            "cone_probability": cone_probability,
                            "cone_occupancy": detector.occupancy,
                            "cone_detected": detector.detected.tolist(),
                            "is_detected": bool(detector.is_detected),  # bool型に変換
                        }
                    )
                    counter += 1
                data = json.dumps(data)
                try:
                    await websocket.send(data)
                except Exception as e:
                    print(f"データ送信中にエラー発生: {e}")
                    break
                await asyncio.sleep(DATA_SAMPLING_RATE)
        except websockets.exceptions.ConnectionClosedOK:
            print("接続が正常に閉じられました。1秒後に再接続します...")
            await asyncio.sleep(1)
        except (
            websockets.exceptions.ConnectionClosedError,
            ConnectionRefusedError,
        ) as e:
            print(f"Connection lost: {e}, retrying in 1 second...")
            await asyncio.sleep(1)
        except Exception as e:
            print(f"予期せぬエラーが発生しました: {e}")
            await asyncio.sleep(0.1)


def moveMotor_thread():
    global phase
    slow = 1.0  # Slow down the motor in camera mode
    GPIO.setmode(GPIO.BCM)

    #   pi.wiringPiSetupGpio()
    GPIO.setup(M1A, GPIO.OUT)
    GPIO.setup(M1B, GPIO.OUT)
    GPIO.setup(M4A, GPIO.OUT)
    GPIO.setup(M4B, GPIO.OUT)

    M1A_pwm = GPIO.PWM(M1A, frequency)
    M1B_pwm = GPIO.PWM(M1B, frequency)
    M4A_pwm = GPIO.PWM(M4A, frequency)
    M4B_pwm = GPIO.PWM(M4B, frequency)

    M1A_pwm.start(0)
    M1B_pwm.start(0)
    M4A_pwm.start(0)
    M4B_pwm.start(0)

    while True:
        if phase < 5:
            slow = 1.0
        elif phase == 5:  # camera mode
            slow = 0.8
        if direction == 360.0:  # stop
            M1A_pwm.ChangeDutyCycle(0)
            M1B_pwm.ChangeDutyCycle(0)
            M4A_pwm.ChangeDutyCycle(0)
            M4B_pwm.ChangeDutyCycle(0)
        elif direction == -360.0:  # forward
            M1A_pwm.ChangeDutyCycle(50 * slow)
            M1B_pwm.ChangeDutyCycle(0)
            M4A_pwm.ChangeDutyCycle(50 * slow)
            M4B_pwm.ChangeDutyCycle(0)
        elif direction == -400.0:  # rotate
            M1A_pwm.ChangeDutyCycle(25)
            M1B_pwm.ChangeDutyCycle(0)
            M4A_pwm.ChangeDutyCycle(50)
            M4B_pwm.ChangeDutyCycle(0)
        elif direction == 700:  # back
            M1A_pwm.ChangeDutyCycle(0)
            M1B_pwm.ChangeDutyCycle(50)
            M4A_pwm.ChangeDutyCycle(0)
            M4B_pwm.ChangeDutyCycle(50)
        elif direction > 0.0 and direction <= 180.0:  # left
            M1A_pwm.ChangeDutyCycle(25)
            M1B_pwm.ChangeDutyCycle(0)
            M4A_pwm.ChangeDutyCycle(15)
            M4B_pwm.ChangeDutyCycle(0)
        elif direction < 0.0 and direction >= -180.0:  # right
            M1A_pwm.ChangeDutyCycle(15)
            M1B_pwm.ChangeDutyCycle(0)
            M4A_pwm.ChangeDutyCycle(25)
            M4B_pwm.ChangeDutyCycle(0)


def set_direction():  # -180<direction<180  #rover move to right while direction > 0
    global direction
    global phase
    global object_distance_Flag
    global upside_down_Flag

    if phase == 0:  # 投下
        direction = 360

    elif phase == 1:
        direction = 360

    elif phase == 2:  # キャリブレーション
        direction = -400.0  # right

    elif phase == 3:
        if (angle - azimuth) > 180:
            theta = angle - 360
        elif (azimuth - angle) > 180:
            theta = angle + 360
        else:
            theta = angle

        direction = azimuth - theta

        if abs(direction) < 8.0:
            direction = -360.0
        elif abs(direction) > 172.0:
            direction = -360.0

    elif phase == 4:
        direction = -400.0

    elif phase == 5:
        if cone_direction > 0.7:
            direction = -180
        elif cone_direction <= 0.7 and cone_direction >= 0.3:
            direction = -360
        elif cone_direction < 0.3:
            direction = 180

    elif phase == 6:
        direction = 360
    elif phase == -1:
        for _ in range(4):
            direction = 500
            time.sleep(0.3)
            direction = 600
            time.sleep(0.3)
        direction = 90
        time.sleep(2)
        direction = -360
        time.sleep(2)
        stuck_uss_Flag = 0
        stuck_GPS_Flag = 0
    elif phase == -2:
        direction = 700
        time.sleep(3)
        upside_down_Flag = 0


if __name__ == "__main__":
    GPIO.setwarnings(False)
    Setup()
    loop = asyncio.get_event_loop()
    tasks = [setData(), main()]
    loop.run_until_complete(asyncio.gather(*tasks))
    time.sleep(100)
