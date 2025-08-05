import socket
from uarm.wrapper import SwiftAPI
import time

client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address = ("192.168.1.28", 9090)
message = "Hello, UDP Server!"
client_socket.sendto(message.encode(), server_address)

swift = SwiftAPI(port='/dev/ttyACM0')
swift.connect()
latest_distance = None

def map_value(value):
    mapping = {
        35: 1,
        34: 5,
        33: 10,
    }
    return mapping.get(int(value), None) 

def wait_for_distance():
    while True:
        data, _ = client_socket.recvfrom(1024)
        cmd = data.decode().strip()
        if cmd.startswith("Distance:"):
            try:
                value = cmd.split(":")[1].replace("cm", "").strip()
                latest_distance = float(value)
                return latest_distance
            except Exception as e:
                print(f"距离解析失败: {e}")

try:
    while True:
        latest_distance = wait_for_distance()
        mapped_result = map_value(latest_distance)
        if mapped_result is not None:
            mapped_result = max(0, min(5, mapped_result))
        swift.set_position(x=230, y=-220, z=100, wait=True)
        time.sleep(1)
        swift.set_position(x=275, y=0, z=100, wait=True)
        time.sleep(1)
        swift.set_position(x=275, y=0, z=mapped_result, wait=True)
        time.sleep(1)
        swift.set_pump(on=True)
        time.sleep(1)

        swift.set_position(x=230, y=0, z=100, wait=True)
        time.sleep(0.5)
        swift.set_position(x=230, y=-220, z=100, wait=True)
        time.sleep(0.5)
        swift.set_position(x=230, y=-220, z=20, wait=True)
        time.sleep(0.5)
        swift.set_pump(on=False)
        time.sleep(1)

        latest_distance = wait_for_distance()
        mapped_result = map_value(latest_distance)
        if mapped_result is not None:
            mapped_result = max(-1, min(0, mapped_result))
        swift.set_position(x=230, y=-220, z=100, wait=True)
        time.sleep(1)
        swift.set_position(x=275, y=0, z=100, wait=True)
        time.sleep(1)
        swift.set_position(x=275, y=0, z=mapped_result, wait=True)
        time.sleep(1)
        swift.set_pump(on=True)
        time.sleep(1)

        swift.set_position(x=230, y=0, z=100, wait=True)
        time.sleep(0.5)
        swift.set_position(x=230, y=-220, z=100, wait=True)
        time.sleep(0.5)
        swift.set_position(x=230, y=-220, z=20, wait=True)
        time.sleep(0.5)
        swift.set_pump(on=False)
        time.sleep(1)

        latest_distance = wait_for_distance()
        mapped_result = map_value(latest_distance)
        if mapped_result is not None:
            mapped_result = max(5, min(8, mapped_result))
        swift.set_position(x=230, y=-220, z=100, wait=True)
        time.sleep(1)
        swift.set_position(x=275, y=0, z=100, wait=True)
        time.sleep(1)
        swift.set_position(x=275, y=0, z=mapped_result, wait=True)
        time.sleep(1)
        swift.set_pump(on=True)
        time.sleep(1)

        swift.set_position(x=230, y=0, z=100, wait=True)
        time.sleep(0.5)
        swift.set_position(x=230, y=-220, z=100, wait=True)
        time.sleep(0.5)
        swift.set_position(x=230, y=-220, z=50, wait=True)
        time.sleep(0.5)
        swift.set_pump(on=False)
        time.sleep(1)
        swift.set_position(x=230, y=-220, z=100, wait=True)
        time.sleep(2)
        swift.disconnect()
        client_socket.sendto("动作完成".encode(), server_address)
        break
except Exception as e:
    print(f"发生异常: {e}")
finally:
    swift.disconnect()
    client_socket.close()
    print("客户端已关闭")
