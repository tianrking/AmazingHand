import asyncio
import websockets
import serial
import json
import time
import numpy as np
from rustypot import Scs0009PyController

# --- 1. 定义配置 ---
FINGERS = [
    {'name': 'Thumb',   'm1_id': 11, 'm2_id': 12}, 
    {'name': 'Index',   'm1_id': 13, 'm2_id': 14}, 
    {'name': 'Middle',  'm1_id': 15, 'm2_id': 16},
    {'name': 'Ring',    'm1_id': 17, 'm2_id': 18}, 
]

# --- 2. 运动范围定义 ---
CLOSE_ANGLE_OFFSET = 90
OPEN_ANGLE_OFFSET = -30
GESTURE_ANGLE_MIN = 20  # (伸直)
GESTURE_ANGLE_MAX = 160 # (握紧)

YAW_ANGLE_OFFSET_MAX = 30
YAW_GESTURE_MIN = -100 # (向左)
YAW_GESTURE_MAX = 100  # (向右)
YAW_GESTURE_CENTER = 0

# --- 3. 手势常量 (用于定义手势) ---
BEND_OPEN = GESTURE_ANGLE_MIN
BEND_CLOSED = GESTURE_ANGLE_MAX
YAW_LEFT = YAW_GESTURE_MIN
YAW_RIGHT = YAW_GESTURE_MAX
YAW_CENTER = YAW_GESTURE_CENTER

# --- 4. (核心改动) 手势库 ---

# (A) 静态手势 (只执行一次)
PRESET_GESTURES = {
    "open_hand": {
        "thumb": BEND_OPEN, "index": BEND_OPEN, "middle": BEND_OPEN, "ring": BEND_OPEN,
        "thumb_yaw": YAW_CENTER, "index_yaw": YAW_CENTER, "middle_yaw": YAW_CENTER, "ring_yaw": YAW_CENTER
    },
    "fist": {
        "thumb": BEND_CLOSED, "index": BEND_CLOSED, "middle": BEND_CLOSED, "ring": BEND_CLOSED,
        "thumb_yaw": YAW_CENTER, "index_yaw": YAW_CENTER, "middle_yaw": YAW_CENTER, "ring_yaw": YAW_CENTER
    },
    # 我们保留 "point" 和 "peace" 的中间姿态，作为动态手势的 "基础"
    "point_center": {
        "thumb": BEND_CLOSED, "index": BEND_OPEN, "middle": BEND_CLOSED, "ring": BEND_CLOSED,
        "thumb_yaw": YAW_CENTER, "index_yaw": YAW_CENTER, "middle_yaw": YAW_CENTER, "ring_yaw": YAW_CENTER
    },
    "peace_center": {
        "thumb": BEND_CLOSED, "index": BEND_OPEN, "middle": BEND_OPEN, "ring": BEND_CLOSED,
        "thumb_yaw": YAW_CENTER, "index_yaw": YAW_CENTER, "middle_yaw": YAW_CENTER, "ring_yaw": YAW_CENTER
    },
    "peace_left": { # "peace_left" 和 "right" 仍然是静态的，以便动态循环调用
        "thumb": BEND_CLOSED, "index": BEND_OPEN, "middle": BEND_OPEN, "ring": BEND_CLOSED,
        "thumb_yaw": YAW_CENTER, "index_yaw": YAW_LEFT, "middle_yaw": YAW_LEFT, "ring_yaw": YAW_CENTER
    },
    "peace_right": {
        "thumb": BEND_CLOSED, "index": BEND_OPEN, "middle": BEND_OPEN, "ring": BEND_CLOSED,
        "thumb_yaw": YAW_CENTER, "index_yaw": YAW_RIGHT, "middle_yaw": YAW_RIGHT, "ring_yaw": YAW_CENTER
    }
}

# (B) 动态手势 (循环执行)
# 这些是 async 函数，将在后台任务中运行
async def demo_beckon_loop():
    """食指勾引 往复运动"""
    base_pose = PRESET_GESTURES["point_center"]
    try:
        while True:
            # 1. 弯曲
            pose_in = base_pose.copy()
            pose_in["index"] = BEND_CLOSED
            await broadcast_state(pose_in)
            await apply_pose_to_servos(pose_in, c)
            await asyncio.sleep(0.5) # 弯曲停顿时间
            
            # 2. 伸直
            pose_out = base_pose.copy()
            pose_out["index"] = BEND_OPEN
            await broadcast_state(pose_out)
            await apply_pose_to_servos(pose_out, c)
            await asyncio.sleep(0.5) # 伸直停顿时间
    except asyncio.CancelledError:
        print("Demo 'beckon' stopped.")
        raise

async def demo_peace_wave_loop():
    """Peace手势 左右摆动"""
    try:
        while True:
            # 1. 摆向左
            pose_left = PRESET_GESTURES["peace_left"]
            await broadcast_state(pose_left)
            await apply_pose_to_servos(pose_left, c)
            await asyncio.sleep(0.7) # 摆动停顿时间
            
            # 2. 摆向右
            pose_right = PRESET_GESTURES["peace_right"]
            await broadcast_state(pose_right)
            await apply_pose_to_servos(pose_right, c)
            await asyncio.sleep(0.7) # 摆动停顿时间
    except asyncio.CancelledError:
        print("Demo 'peace_wave' stopped.")
        raise

async def demo_finger_roll_loop():
    """手指波浪 (Index -> Middle -> Ring)"""
    base_pose = PRESET_GESTURES["open_hand"].copy()
    try:
        while True:
            # 1. 全部张开
            await broadcast_state(base_pose)
            await apply_pose_to_servos(base_pose, c)
            await asyncio.sleep(0.5)
            # 2. 握食指
            pose1 = base_pose.copy()
            pose1["index"] = BEND_CLOSED
            await broadcast_state(pose1)
            await apply_pose_to_servos(pose1, c)
            await asyncio.sleep(0.3)
            # 3. 握中指
            pose2 = pose1.copy()
            pose2["middle"] = BEND_CLOSED
            await broadcast_state(pose2)
            await apply_pose_to_servos(pose2, c)
            await asyncio.sleep(0.3)
            # 4. 握无名指 (形成拳头)
            pose3 = pose2.copy()
            pose3["ring"] = BEND_CLOSED
            await broadcast_state(pose3)
            await apply_pose_to_servos(pose3, c)
            await asyncio.sleep(0.5)
    except asyncio.CancelledError:
        print("Demo 'finger_roll' stopped.")
        raise

async def demo_thumb_dance_loop():
    """拇指独舞 (在握拳基础上)"""
    base_pose = PRESET_GESTURES["fist"].copy()
    base_pose["thumb"] = BEND_OPEN # 拳头，但拇指伸出
    try:
        while True:
            # 1. 摆向左
            pose_l = base_pose.copy()
            pose_l["thumb_yaw"] = YAW_LEFT
            await broadcast_state(pose_l)
            await apply_pose_to_servos(pose_l, c)
            await asyncio.sleep(0.4)
            # 2. 摆向右
            pose_r = base_pose.copy()
            pose_r["thumb_yaw"] = YAW_RIGHT
            await broadcast_state(pose_r)
            await apply_pose_to_servos(pose_r, c)
            await asyncio.sleep(0.4)
    except asyncio.CancelledError:
        print("Demo 'thumb_dance' stopped.")
        raise

# (C) 动态手势的 "注册表"
DYNAMIC_GESTURES = {
    "demo_beckon": demo_beckon_loop,
    "demo_peace_wave": demo_peace_wave_loop,
    "demo_roll": demo_finger_roll_loop,
    "demo_thumb_dance": demo_thumb_dance_loop,
}

# --- 5. 串口和控制器初始化 ---
SERVO_CONTROLLER_PORT = "/dev/ttyACM0"
try:
    # 将控制器 'c' 设为全局变量，以便辅助函数可以访问
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

# --- 6. (核心改动) 全局状态和辅助函数 ---

# 跟踪所有连接的客户端
CONNECTED_CLIENTS = set()
# 跟踪当前运行的演示任务
current_demo_task = None

def map_value(x, in_min, in_max, out_min, out_max):
    x = max(in_min, min(x, in_max))
    if in_max == in_min: return out_min
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def setup_servos(controller, finger_config):
    all_servo_ids = [id for finger in finger_config for id in (finger['m1_id'], finger['m2_id'])]
    print(f"准备控制的伺服电机ID: {all_servo_ids}")
    controller.sync_write_torque_enable(all_servo_ids, [1] * len(all_servo_ids))
    print("已为所有电机启动扭矩。")
    controller.sync_write_goal_speed(all_servo_ids, [6] * len(all_servo_ids))
    print("已设置所有电机速度。")
    time.sleep(0.5)
    return all_servo_ids

async def broadcast_state(state):
    """将当前姿态(8个滑块值)广播给所有连接的网页"""
    if CONNECTED_CLIENTS:
        message = json.dumps(state)
        # 使用 asyncio.gather 来并行发送消息
        await asyncio.gather(
            *[client.send(message) for client in CONNECTED_CLIENTS]
        )

async def apply_pose_to_servos(pose_data, controller):
    """根据姿态数据(8个滑块值)计算并发送舵机指令"""
    try:
        # 1. 计算“弯曲”偏移量
        bend_offsets = {
            'Thumb': map_value(pose_data.get('thumb', BEND_OPEN), GESTURE_ANGLE_MIN, GESTURE_ANGLE_MAX, OPEN_ANGLE_OFFSET, CLOSE_ANGLE_OFFSET),
            'Index': map_value(pose_data.get('index', BEND_OPEN), GESTURE_ANGLE_MIN, GESTURE_ANGLE_MAX, OPEN_ANGLE_OFFSET, CLOSE_ANGLE_OFFSET),
            'Middle': map_value(pose_data.get('middle', BEND_OPEN), GESTURE_ANGLE_MIN, GESTURE_ANGLE_MAX, OPEN_ANGLE_OFFSET, CLOSE_ANGLE_OFFSET),
            'Ring': map_value(pose_data.get('ring', BEND_OPEN), GESTURE_ANGLE_MIN, GESTURE_ANGLE_MAX, OPEN_ANGLE_OFFSET, CLOSE_ANGLE_OFFSET)
        }
        # 2. 计算“摆动”偏移量
        yaw_offsets = {
            'Thumb': map_value(pose_data.get('thumb_yaw', YAW_CENTER), YAW_GESTURE_MIN, YAW_GESTURE_MAX, -YAW_ANGLE_OFFSET_MAX, YAW_ANGLE_OFFSET_MAX),
            'Index': map_value(pose_data.get('index_yaw', YAW_CENTER), YAW_GESTURE_MIN, YAW_GESTURE_MAX, -YAW_ANGLE_OFFSET_MAX, YAW_ANGLE_OFFSET_MAX),
            'Middle': map_value(pose_data.get('middle_yaw', YAW_CENTER), YAW_GESTURE_MIN, YAW_GESTURE_MAX, -YAW_ANGLE_OFFSET_MAX, YAW_ANGLE_OFFSET_MAX),
            'Ring': map_value(pose_data.get('ring_yaw', YAW_CENTER), YAW_GESTURE_MIN, YAW_GESTURE_MAX, -YAW_ANGLE_OFFSET_MAX, YAW_ANGLE_OFFSET_MAX)
        }
        # 3. 准备指令 (包含大拇指修复逻辑)
        all_ids = []
        positions_deg = []
        for finger in FINGERS:
            finger_name = finger['name']
            current_bend_offset = bend_offsets.get(finger_name, 0)
            current_yaw_offset = yaw_offsets.get(finger_name, 0)
            
            if finger_name == 'Thumb':
                pos_m1 = current_yaw_offset - current_bend_offset
                pos_m2 = current_yaw_offset + current_bend_offset
            else:
                pos_m1 = current_yaw_offset + current_bend_offset
                pos_m2 = current_yaw_offset - current_bend_offset
            
            all_ids.extend([finger['m1_id'], finger['m2_id']])
            positions_deg.extend([pos_m1, pos_m2])

        # 4. 发送指令
        positions_rad = np.deg2rad(positions_deg).tolist()
        controller.sync_write_goal_position(all_ids, positions_rad)
    except Exception as e:
        print(f"Apply pose error: {e}")

# --- 7. (核心改动) WebSocket 主处理器 ---
async def handler(websocket):
    """WebSocket 服务器的主处理逻辑"""
    global current_demo_task
    
    # 注册新客户端
    CONNECTED_CLIENTS.add(websocket)
    print(f"网页控制器已连接。当前连接数: {len(CONNECTED_CLIENTS)}")
    
    try:
        # 循环接收消息
        async for message in websocket:
            # (!!!) 关键：收到任何新消息时，立即停止当前的演示
            if current_demo_task:
                current_demo_task.cancel()
                current_demo_task = None
            
            try:
                pose_data = json.loads(message)
                
                # A. 检查是否是 "动作" 命令
                if "action" in pose_data:
                    action_name = pose_data["action"]
                    
                    # A1. 检查是否是 "静态" 手势
                    if action_name in PRESET_GESTURES:
                        print(f"收到静态动作: {action_name}")
                        static_pose = PRESET_GESTURES[action_name]
                        await broadcast_state(static_pose) # 广播UI
                        await apply_pose_to_servos(static_pose, c) # 执行动作
                    
                    # A2. 检查是否是 "动态" 手势
                    elif action_name in DYNAMIC_GESTURES:
                        print(f"启动动态演示: {action_name}")
                        demo_function = DYNAMIC_GESTURES[action_name]
                        # 启动新任务
                        current_demo_task = asyncio.create_task(demo_function())
                    else:
                        print(f"收到未知动作: {action_name}")
                
                # B. 否则，它就是 "滑块" 数据
                else:
                    # 我们不需要在这里打印，否则会刷屏
                    # print(f"收到滑块数据")
                    # 直接将滑块数据应用到舵机
                    # 并广播给其他客户端（如果有的话）
                    await broadcast_state(pose_data)
                    await apply_pose_to_servos(pose_data, c)

            except json.JSONDecodeError:
                print(f"警告: 收到非JSON格式数据: {message}")
            except Exception as e:
                print(f"处理消息时出错: {e}")

    except websockets.exceptions.ConnectionClosed:
        print("网页控制器已断开连接。")
    finally:
        # 注销客户端
        CONNECTED_CLIENTS.remove(websocket)
        print(f"一个连接已关闭。剩余连接数: {len(CONNECTED_CLIENTS)}")
        # 如果这是最后一个客户端，也停止演示
        if not CONNECTED_CLIENTS and current_demo_task:
            print("最后一个客户端断开，停止演示。")
            current_demo_task.cancel()
            current_demo_task = None

# --- 8. 主函数 ---
async def main():
    all_servo_ids = setup_servos(c, FINGERS)
    
    try:
        async with websockets.serve(handler, "0.0.0.0", 8765):
            print("\nWebSocket 服务器已在 ws://localhost:8765 启动")
            print("等待网页控制器连接...")
            await asyncio.Future()  # 永久运行
    except KeyboardInterrupt:
        print("\n检测到用户中断（Ctrl+C）...")
    finally:
        print("正在关闭所有电机的扭矩...")
        if current_demo_task:
            current_demo_task.cancel() # 关闭程序时也停止任务
        if all_servo_ids:
            disable_values = [0] * len(all_servo_ids)
            c.sync_write_torque_enable(all_servo_ids, disable_values)
        print("扭矩已关闭。程序结束。")

if __name__ == '__main__':
    asyncio.run(main())