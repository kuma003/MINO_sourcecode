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


# 定数　上書きしない
MAG_CONST = 8.9  # 地磁気補正用の偏角
CALIBRATION_MILLITIME = 20 * 1000
TARGET_LAT = 38.260720666666664
TARGET_LNG = 140.85434316666667
TARGET_ALTITUDE = 20
DATA_SAMPLING_RATE = 0.00001
ALTITUDE_CONST1 = 30
ALTITUDE_CONST2 = 5
SERVO_PIN = 12
LED1 = 16
LED2 = 20
LED3 = 21
HIGH = 1
LOW = 0
M1A = 19
M1B = 13
M4A = 5
M4B = 6
trig_pin = 15  # GPIO 15
# echo_pin = 14  # GPIO 14
# speed_of_sound = 34370  # 音速（気温20℃）

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
object_distance = 0.0
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
stuck_uss_Flag = 0  # judge the stuck by ultrasonic sensor
stuck_GPS_Flag = 0  # judge the stuck by GPS : no obstacle distance_Flag = 0, if CanSat stucked distance_Flag = 1


bmx = BMX055.BMX055()
# bmp = BMP085.BMP085()
# servo = pigpio.pi()

nowTime = datetime.datetime.now()
fileName = "./log/testlog_" + nowTime.strftime("%Y-%m%d-%H%M%S") + ".csv"


def main():
    global phase
    global restTime
    global start
    global gps_detect

    GPIO.setwarnings(False)
    Setup()
    phase = 0
    n = 0

    while True:
        # LED_Checker()
        if phase == 0:  # 投下
            print("phase0 : falling")
            start = time.time()
            while True:
                getBmxData()
                # print(fall)
                if fall > 15:
                    print("para released")
                    time.sleep(10)
                    break
                if time.time() - start > 5 * 60:
                    phase = 1
                # time.sleep(0.1)
            phase = 1

        elif phase == 1:  # パラ分離
            print("phase1 : remove para")
            # servoMotor(160)
            time.sleep(3)
            # servoMotor(90)
            phase = 2

        elif phase == 2:  # キャリブレーション
            print("phase2 : calibration start")
            calibration()
            phase = 3

        elif phase == 3:
            print("phase3 : GPS start")
            # get_object_distance()
            #            if stuck == True:
            #                phase = -1
            #            if upside_down == True:
            #                phase = -2
            if distance < 1.0:  # GPS座標との距離 < m以内　　#スタック優先
                phase = 6
            # print(distance)
            # print(direction)
            # print(mag)

        elif phase == 4:
            print("phase4 : camera start")
            start2 = time.time()
            cone_detect()
            if cone_probability < 1:
                phase = 5
            if n > 5:
                phase = 6
        elif phase == 5:
            print("phase5")
            while True:
                cone_detect()
                #                 if detector.is_detected:
                #                     print("cone detected")
                #                     print(detector.detected[cv2.CC_STAT_AREA])
                if detector.is_reached:  # if rover reached cone
                    print("reached")
                    phase = 6
                    break
        #                 if cone_probability > 1: # detect cone probability
        #                     n += 1
        #                     phase = 4

        elif phase == 6:
            print("phase6 : Goal")
            time.sleep(10000)
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

        time.sleep(0.1)


def currentMilliTime():
    return round(time.time() * 1000)


def Setup():
    global detector
    bmx.setUp()

    GPIO.setmode(GPIO.BCM)
    #    GPIO.setup(LED1, GPIO.OUT)
    #    GPIO.setup(LED2, GPIO.OUT)
    #    GPIO.setup(LED3, GPIO.OUT)
    #    GPIO.setup(trig_pin, GPIO.OUT)  # Trigピン出力モード設定
    #    GPIO.setup(echo_pin, GPIO.IN)  # Echoピン入力モード設定

    #    servoMotor(90)

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
    # getThread.setDaemon(True)
    getThread.start()

    dataThread = threading.Thread(target=setData_thread, args=())
    dataThread.daemon = True
    #    dataThread.setDaemon(True)
    dataThread.start()

    gpsThread = threading.Thread(target=GPS_thread, args=())
    gpsThread.daemon = True
    #    gpsThread.setDaemon(True)
    gpsThread.start()

    detector = dc.detector()
    roi_img = cv2.imread("./log/captured.png")

    roi_img = cv2.cvtColor(roi_img, cv2.COLOR_BGR2RGB)
    detector.set_roi_img(roi_img)

    #    GPIO.output(LED2, HIGH)
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


def stuck_uss():
    global stuck_uss_Flag
    object_distance = [0.0, 0.0, 0.0, 0.0, 0.0]
    for i in range(5):
        object_distance[i] = object_distance
    if all(x <= 10 for x in object_distance):
        stuck_uss_Flag = 1


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
            #            GPIO.output(LED1, HIGH)
            complete = True
            time.sleep(1)
            calibBias[0] = (max[0] + min[0]) / 2
            calibBias[2] = (max[2] + min[2]) / 2

            calibRange[0] = (max[0] - min[0]) / 2
            calibRange[2] = (max[2] - min[2]) / 2
            #            GPIO.output(LED1, LOW)
            time.sleep(2)


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


def get_object_distance():  # 超音波での障害物との距離計算関数
    global object_distance
    global distance_diff
    global distance_Flag
    global while_judge_distance
    # Trigピンを10μsだけHIGHにして超音波の発信開始
    GPIO.output(trig_pin, GPIO.HIGH)
    time.sleep(0.000010)
    GPIO.output(trig_pin, GPIO.LOW)

    if while_judge_distance == 0:
        while_judge_distance = 1
        distance_diff[0] = distance
        distance_time[0] = currentMilliTime()
    elif while_judge_distance == 1:
        distance_time[1] = currentMilliTime()
        if distance_time[1] - distance_time[0] > 10 * 1000:
            distance_diff[1] = distance
            if (
                distance_diff[1] - distance_diff[0] < 1
            ):  # stack CanSat move only 1 meter while 10s
                distance_Flag = 1

    while GPIO.input(echo_pin) == GPIO.LOW:
        t1 = time.time()
    while GPIO.input(echo_pin) == GPIO.HIGH:
        t2 = time.time()
    object_distance = (t2 - t1) * speed_of_sound / 2  # 時間差から対象物までの距離計算
    print(object_distance)


def servoMotor(angle):
    assert 0 <= angle <= 180, "角度は0から180の間でなければなりません"

    pulse_width = (angle / 180) * (
        2500 - 500
    ) + 500  # 角度を500から2500のパルス幅にマッピングする
    servo.set_servo_pulsewidth(
        SERVO_PIN, pulse_width
    )  # パルス幅を設定してサーボを回転させる


def LED_Checker():
    GPIO.output(LED1, LOW)
    GPIO.output(LED2, LOW)
    GPIO.output(LED3, LOW)
    if amari[0] == 1:
        GPIO.output(LED1, HIGH)
    if amari[1] == 1:
        GPIO.output(LED2, HIGH)
    if amari[2] == 1:
        GPIO.output(LED3, HIGH)


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

    detector.detect_cone()
    cone_direction = 1 - detector.cone_direction
    cone_probability = detector.probability
    print("direction", cone_direction)
    print("prob.", cone_probability)


def setData_thread():
    while True:
        getBmxData()
        calcAngle()
        calcAzimuth()
        set_direction()
        calcdistance()
        with open(fileName, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    currentMilliTime(),
                    round(phase, 1),
                    acc[0],
                    acc[1],
                    acc[2],
                    gyro[0],
                    gyro[1],
                    gyro[2],
                    mag[1],
                    mag[1],
                    mag[2],
                    lat,
                    lng,
                    alt,
                    distance,
                    object_distance,
                    azimuth,
                    angle,
                    direction,
                    fall,
                ]
            )
        time.sleep(DATA_SAMPLING_RATE)


def moveMotor_thread():
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
        if direction == 360.0:  # stop
            M1A_pwm.ChangeDutyCycle(0)
            M1B_pwm.ChangeDutyCycle(0)
            M4A_pwm.ChangeDutyCycle(0)
            M4B_pwm.ChangeDutyCycle(0)
        elif direction == -360.0:  # forward
            M1A_pwm.ChangeDutyCycle(50)
            M1B_pwm.ChangeDutyCycle(0)
            M4A_pwm.ChangeDutyCycle(50)
            M4B_pwm.ChangeDutyCycle(0)
        elif direction == -400.0:  # rotate
            M1A_pwm.ChangeDutyCycle(25)
            M1B_pwm.ChangeDutyCycle(0)
            M4A_pwm.ChangeDutyCycle(50)
            M4B_pwm.ChangeDutyCycle(0)
        elif direction == 500.0:  # left back
            M1A_pwm.ChangeDutyCycle(0)
            M1B_pwm.ChangeDutyCycle(30)
            M4A_pwm.ChangeDutyCycle(0)
            M4B_pwm.ChangeDutyCycle(10)
        elif direction == 600.0:  # right back
            M1A_pwm.ChangeDutyCycle(0)
            M1B_pwm.ChangeDutyCycle(10)
            M4A_pwm.ChangeDutyCycle(0)
            M4B_pwm.ChangeDutyCycle(30)
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
    main()
    time.sleep(100)
