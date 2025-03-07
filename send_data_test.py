import asyncio
import math
import websockets
import BMX055
import serial
from micropyGPS import MicropyGPS
import threading
import json
import time

port = 8756
SERVER_URL = "192.168.127.98"
URI = f"ws://{SERVER_URL}:{port}"
TARGET_LAT = 38.266285
TARGET_LNG = 140.855498

bmx = BMX055.BMX055()
bmx.setUp()

lat = 0.0
lng = 0.0
gps_detect = 0
distance = 0.0
acc = [0.0, 0.0, 0.0]
gyro = [0.0, 0.0, 0.0]
mag = [0.0, 0.0, 0.0]


def get_BMX055_data():
    global acc
    global gyro
    global mag

    acc = bmx.getAcc()
    gyro = bmx.getGyro()
    mag = bmx.getMag()


def calc_distance():
    global distance
    EARTH_RADIUS = 6378136.59
    dx = (math.pi / 180) * EARTH_RADIUS * (TARGET_LNG - lng)
    dy = (math.pi / 180) * EARTH_RADIUS * (TARGET_LAT - lat)
    distance = math.sqrt(dx * dx + dy * dy)


async def send_data():
    global lat
    global lng
    global gps_detect

    while True:
        try:
            print("Connecting to server")
            async with websockets.connect(URI) as websocket:
                while True:
                    acc = bmx.getAcc()
                    gyro = bmx.getGyro()
                    mag = bmx.getMag()
                    calc_distance()
                    time.time_ns()
                    data = {
                        "lat": lat,
                        "lng": lng,
                        "acc": acc,
                        "gyro": gyro,
                        "mag": mag,
                        "distance": distance,
                        "gps_detect": gps_detect,
                    }
                    data = json.dumps(data)
                    await websocket.send(data)
        except (websockets.exceptions.ConnectionClosedError, ConnectionRefusedError):
            print("Connection lost, retrying in 1 second...")
            await asyncio.sleep(1)  # 非同期で待機


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

        if lat == 0.0:
            gps_detect = 0
            print("None gnss value")
            continue
        gps_detect = 1


async def async_main():
    global lat, lng, gps_detect
    count = 0

    try:
        while True:
            count += 1
            print(f"async_main実行回数: {count}")
            print("lat: ", lat, "lng: ", lng)
            await asyncio.sleep(1)  # 非同期sleep
    except Exception as e:
        print(f"async_main内でエラーが発生: {e}")
        # エラー発生後も継続するため再帰呼び出し
        await async_main()


if __name__ == "__main__":
    # GPSのスレッドの立ち上げ
    gpsThread = threading.Thread(target=GPS_thread, args=())
    gpsThread.daemon = True
    gpsThread.start()

    # 非同期I/O
    loop = asyncio.get_event_loop()
    # 複数のコルーチンを同時に実行
    tasks = [send_data(), async_main()]
    loop.run_until_complete(asyncio.gather(*tasks))
