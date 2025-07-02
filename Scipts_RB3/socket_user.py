import socket
from uarm.wrapper import SwiftAPI
import time

# 创建 socket 对象
client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 1. 先主动发一条消息给服务端，暴露自己的地址
server_address = ("192.168.1.28", 9090)  # 服务端实际IP
message = "Hello, UDP Server!"
client_socket.sendto(message.encode(), server_address)

# 2. 连接 uArm
swift = SwiftAPI(port='/dev/ttyACM0')
swift.connect()

print("等待服务端指令...")

latest_distance = None  # 新增变量

def map_value(value):
    mapping = {
        35: 1,
        34: 5,
        33: 10,
    }
    return mapping.get(int(value), None)  # 未知值返回 None

def wait_for_distance():
    while True:
        data, _ = client_socket.recvfrom(1024)
        cmd = data.decode().strip()
        if cmd.startswith("Distance:"):
            try:
                value = cmd.split(":")[1].replace("cm", "").strip()
                latest_distance = float(value)
                print(f"收到距离: {latest_distance} cm")
                return latest_distance
            except Exception as e:
                print(f"距离解析失败: {e}")

try:
    while True:
        # 吸电池
        latest_distance = wait_for_distance()
        mapped_result = map_value(latest_distance)
        if mapped_result is not None:
            mapped_result = max(0, min(5, mapped_result))
        print(f"第一次 mapped_result: {mapped_result}")
        swift.set_position(x=230, y=-220, z=100, wait=True)
        print("到达地面:", swift.get_position())
        time.sleep(1)

        swift.set_position(x=275, y=0, z=100, wait=True)
        print("到达地面:", swift.get_position())
        time.sleep(1)

        swift.set_position(x=275, y=0, z=mapped_result, wait=True)
        print("到达地面:", swift.get_position())
        time.sleep(1)

        swift.set_pump(on=True)
        time.sleep(1)

        swift.set_position(x=230, y=0, z=100, wait=True)
        print("抬升:", swift.get_position())
        time.sleep(0.5)

        swift.set_position(x=230, y=-220, z=100, wait=True)
        print("抬升:", swift.get_position())
        time.sleep(0.5)

        swift.set_position(x=230, y=-220, z=20, wait=True)
        print("抬升:", swift.get_position())
        time.sleep(0.5)

        swift.set_pump(on=False)
        time.sleep(1)

        # 吸绿纸
        latest_distance = wait_for_distance()
        mapped_result = map_value(latest_distance)
        if mapped_result is not None:
            mapped_result = max(-1, min(0, mapped_result))
        print(f"第二次 mapped_result: {mapped_result}")
        swift.set_position(x=230, y=-220, z=100, wait=True)
        print("到达地面:", swift.get_position())
        time.sleep(1)

        swift.set_position(x=275, y=0, z=100, wait=True)
        print("到达地面:", swift.get_position())
        time.sleep(1)

        swift.set_position(x=275, y=0, z=mapped_result, wait=True)
        print("到达地面:", swift.get_position())
        time.sleep(1)

        swift.set_pump(on=True)
        time.sleep(1)

        swift.set_position(x=230, y=0, z=100, wait=True)
        print("抬升:", swift.get_position())
        time.sleep(0.5)

        swift.set_position(x=230, y=-220, z=100, wait=True)
        print("抬升:", swift.get_position())
        time.sleep(0.5)

        swift.set_position(x=230, y=-220, z=20, wait=True)
        print("抬升:", swift.get_position())
        time.sleep(0.5)

        swift.set_pump(on=False)
        time.sleep(1)

        # 吸纸巾
        latest_distance = wait_for_distance()
        mapped_result = map_value(latest_distance)
        if mapped_result is not None:
            mapped_result = max(5, min(8, mapped_result))
        print(f"第三次 mapped_result: {mapped_result}")
        swift.set_position(x=230, y=-220, z=100, wait=True)
        print("到达地面:", swift.get_position())
        time.sleep(1)

        swift.set_position(x=275, y=0, z=100, wait=True)
        print("到达地面:", swift.get_position())
        time.sleep(1)

        swift.set_position(x=275, y=0, z=mapped_result, wait=True)
        print("到达地面:", swift.get_position())
        time.sleep(1)

        swift.set_pump(on=True)
        time.sleep(1)

        swift.set_position(x=230, y=0, z=100, wait=True)
        print("抬升:", swift.get_position())
        time.sleep(0.5)

        swift.set_position(x=230, y=-220, z=100, wait=True)
        print("抬升:", swift.get_position())
        time.sleep(0.5)

        swift.set_position(x=230, y=-220, z=50, wait=True)
        print("抬升:", swift.get_position())
        time.sleep(0.5)

        swift.set_pump(on=False)
        time.sleep(1)

        swift.set_position(x=230, y=-220, z=100, wait=True)
        print("抬升！:", swift.get_position())
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