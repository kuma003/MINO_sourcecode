import asyncio
import websockets


port = 8756
SERVER_URL = "192.168.127.98"
URI = f"ws://{SERVER_URL}:{port}"


async def send_data():
    while True:
        try:
            print("Connecting to server")
            async with websockets.connect(URI) as websocket:
                while True:
                    data = input("Enter data: ")
                    await websocket.send(data)
                    print(f"Sent: {data}")
        except (websockets.exceptions.ConnectionClosedError, ConnectionRefusedError):
            print("Conencton lost")
            asyncio.sleep(1)


if __name__ == "__main__":
    asyncio.run(send_data())
