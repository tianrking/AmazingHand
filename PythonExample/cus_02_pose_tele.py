import asyncio
import websockets
import serial
import json
import time
import numpy as np
from rustypot import Scs0009PyController

# --- 1. 定义配置 ---
# 灵巧手手指配置 (根据你的“右手”布局调整)
# ===================================================================
# 我交换了 Index 和 Ring 的 name 标签。
# 现在：
# - 11, 12 对应 'Ring' (无名指)
# - 13, 14 对应 'Middle' (中指)
# - 15, 16 对应 'Index' (食指)
# - 17, 18 对应 'Thumb' (拇指)
#
# 你可以根据实际测试，随意调换这里的 'name' 字符串。
# FINGERS = [
#     {'name': 'Ring',   'm1_id': 11, 'm2_id': 12}, # 原 'Index' Thumb
#     {'name': 'Middle', 'm1_id': 13, 'm2_id': 14}, # 保持不变 Index
#     {'name': 'Index',  'm1_id': 15, 'm2_id': 16}, # 原 'Ring' Middle
#     {'name': 'Thumb',  'm1_id': 17, 'm2_id': 18}, # 保持不变 Ring
# ]


FINGERS = [
    {'name': 'Thumb',   'm1_id': 11, 'm2_id': 12}, 
    {'name': 'Index', 'm1_id': 13, 'm2_id': 14}, 
    {'name': 'Middle',  'm1_id': 15, 'm2_id': 16},
    {'name': 'Ring',  'm1_id': 17, 'm2_id': 18}, 
]

# ===================================================================

# 舵机运动范围定义
CLOSE_ANGLE_OFFSET = 90
OPEN_ANGLE_OFFSET = -30

# 姿态识别角度输入范围 (根据前端观察到的实际值设定)
GESTURE_ANGLE_MIN = 20  # 对应 OPEN_ANGLE_OFFSET
GESTURE_ANGLE_MAX = 160 # 对应 CLOSE_ANGLE_OFFSET

# 串口号
SERVO_CONTROLLER_PORT = "/dev/ttyACM0"

# --- 2. 初始化控制器 ---
try:
    c = Scs0009PyController(
        serial_port=SERVO_CONTROLLER_PORT,
        baudrate=1000000,
        timeout=0.5,
    )
    print(f"成功连接到机械手控制器: {SERVO_CONTROLLER_PORT}")
except serial.SerialException as e:
    print(f"串口错误: {e}")
    print("请确认您的串口号设置是否正确，以及设备是否已连接。")
    exit()

def map_value(x, in_min, in_max, out_min, out_max):
    """
    核心函数：将一个范围的值线性映射到另一个范围。
    """
    x = max(in_min, min(x, in_max))
    if in_max == in_min:
        return out_min
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def setup_servos(controller, finger_config):
    """启动并初始化所有电机"""
    all_servo_ids = [id for finger in finger_config for id in (finger['m1_id'], finger['m2_id'])]
    print(f"准备控制的伺服电机ID: {all_servo_ids}")

    enable_values = [1] * len(all_servo_ids)
    controller.sync_write_torque_enable(all_servo_ids, enable_values)
    print("已为所有电机启动扭矩。")

    speeds = [6] * len(all_servo_ids)
    controller.sync_write_goal_speed(all_servo_ids, speeds)
    print("已设置所有电机速度。")
    time.sleep(0.5)
    return all_servo_ids

async def handler(websocket, controller):
    """WebSocket 服务器的处理逻辑，接收数据并控制舵机"""
    print("前端网页已连接。")
    try:
        async for message in websocket:
            try:
                # 1. 解析从前端收到的JSON数据
                data = json.loads(message)
                
                # 2. 计算每个手指的目标角度偏移
                thumb_offset = map_value(data.get('thumb', 0), GESTURE_ANGLE_MIN, GESTURE_ANGLE_MAX, OPEN_ANGLE_OFFSET, CLOSE_ANGLE_OFFSET)
                index_offset = map_value(data.get('index', 0), GESTURE_ANGLE_MIN, GESTURE_ANGLE_MAX, OPEN_ANGLE_OFFSET, CLOSE_ANGLE_OFFSET)
                middle_offset = map_value(data.get('middle', 0), GESTURE_ANGLE_MIN, GESTURE_ANGLE_MAX, OPEN_ANGLE_OFFSET, CLOSE_ANGLE_OFFSET)
                ring_offset = map_value(data.get('ring', 0), GESTURE_ANGLE_MIN, GESTURE_ANGLE_MAX, OPEN_ANGLE_OFFSET, CLOSE_ANGLE_OFFSET)

                # 3. 准备同步写入指令
                all_ids = []
                positions_deg = []
                
                # 这个字典是关键，它把网页上的数据 (key) 和 偏移量 (value) 对应
                offsets = {
                    'Thumb': thumb_offset,
                    'Index': index_offset,
                    'Middle': middle_offset,
                    'Ring': ring_offset,
                }

                # 这段循环会根据 FINGERS 列表的配置，自动把正确的偏移量(offset)
                # 分配给正确的舵机ID (m1_id, m2_id)
                for finger in FINGERS:
                    finger_name = finger['name']
                    # 从 'offsets' 字典中查找该手指应该对应的偏移量
                    current_offset = offsets.get(finger_name, 0) 
                    
                    all_ids.extend([finger['m1_id'], finger['m2_id']])
                    positions_deg.append(0 + current_offset)
                    positions_deg.append(0 - current_offset)

                # 4. 转换单位并使用传入的controller对象发送指令
                positions_rad = np.deg2rad(positions_deg).tolist()
                controller.sync_write_goal_position(all_ids, positions_rad)

            except json.JSONDecodeError:
                print(f"警告: 收到非JSON格式数据: {message}")
            except Exception as e:
                print(f"处理消息时出错: {e}")

    except websockets.exceptions.ConnectionClosed:
        print("前端网页已断开连接。")

async def main():
    """主函数：初始化舵机并启动WebSocket服务器"""
    all_servo_ids = setup_servos(c, FINGERS)
    
    handler_with_controller = lambda ws: handler(ws, c)
    
    try:
        async with websockets.serve(handler_with_controller, "0.0.0.0", 8765):
            print("\nWebSocket 服务器已在 ws://localhost:8765 启动")
            print("等待前端网页连接...")
            await asyncio.Future()  # 永久运行
    except KeyboardInterrupt:
        print("\n检测到用户中断（Ctrl+C）...")
    finally:
        # 程序结束前，安全地关闭所有电机的扭矩
        print("正在关闭所有电机的扭矩...")
        if all_servo_ids:
            disable_values = [0] * len(all_servo_ids)
            c.sync_write_torque_enable(all_servo_ids, disable_values)
        print("扭矩已关闭。程序结束。")

if __name__ == '__main__':
    asyncio.run(main())