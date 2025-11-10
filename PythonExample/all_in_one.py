# 文件名: amazinghand_server_fused.py
# (此代码与 'amazinghand_server_stoppable.py' 完全相同, 它已准备好处理融合)

import asyncio
import websockets
import serial
import json
import time
import numpy as np
from rustypot import Scs0009PyController

# --- 1. 定义配置 (无改动) ---
FINGERS = [
    {'name': 'Thumb',   'm1_id': 11, 'm2_id': 12}, 
    {'name': 'Index',   'm1_id': 13, 'm2_id': 14}, 
    {'name': 'Middle',  'm1_id': 15, 'm2_id': 16},
    {'name': 'Ring',    'm1_id': 17, 'm2_id': 18}, 
]

# --- 2. 运动范围定义 (无改动) ---
CLOSE_ANGLE_OFFSET = 90
OPEN_ANGLE_OFFSET = -30
GESTURE_ANGLE_MIN = 20
GESTURE_ANGLE_MAX = 160
YAW_ANGLE_OFFSET_MAX = 30
YAW_GESTURE_MIN = -100
YAW_GESTURE_MAX = 100
YAW_GESTURE_CENTER = 0

# --- 3. 手势常量 (无改动) ---
BEND_OPEN = GESTURE_ANGLE_MIN
BEND_CLOSED = GESTURE_ANGLE_MAX
YAW_LEFT = YAW_GESTURE_MIN
YAW_RIGHT = YAW_GESTURE_MAX
YAW_CENTER = YAW_GESTURE_CENTER

# --- 4. 手势库 (无改动) ---
PRESET_GESTURES = {
    "open_hand": {
        "thumb": BEND_OPEN, "index": BEND_OPEN, "middle": BEND_OPEN, "ring": BEND_OPEN,
        "thumb_yaw": YAW_CENTER, "index_yaw": YAW_CENTER, "middle_yaw": YAW_CENTER, "ring_yaw": YAW_CENTER
    },
    "fist": {
        "thumb": BEND_CLOSED, "index": BEND_CLOSED, "middle": BEND_CLOSED, "ring": BEND_CLOSED,
        "thumb_yaw": YAW_CENTER, "index_yaw": YAW_CENTER, "middle_yaw": YAW_CENTER, "ring_yaw": YAW_CENTER
    },
    "point_center": {
        "thumb": BEND_CLOSED, "index": BEND_OPEN, "middle": BEND_CLOSED, "ring": BEND_CLOSED,
        "thumb_yaw": YAW_CENTER, "index_yaw": YAW_CENTER, "middle_yaw": YAW_CENTER, "ring_yaw": YAW_CENTER
    },
    "peace_center": {
        "thumb": BEND_CLOSED, "index": BEND_OPEN, "middle": BEND_OPEN, "ring": BEND_CLOSED,
        "thumb_yaw": YAW_CENTER, "index_yaw": YAW_CENTER, "middle_yaw": YAW_CENTER, "ring_yaw": YAW_CENTER
    },
    "peace_left": {
        "thumb": BEND_CLOSED, "index": BEND_OPEN, "middle": BEND_OPEN, "ring": BEND_CLOSED,
        "thumb_yaw": YAW_CENTER, "index_yaw": YAW_LEFT, "middle_yaw": YAW_LEFT, "ring_yaw": YAW_CENTER
    },
    "peace_right": {
        "thumb": BEND_CLOSED, "index": BEND_OPEN, "middle": BEND_OPEN, "ring": BEND_CLOSED,
        "thumb_yaw": YAW_CENTER, "index_yaw": YAW_RIGHT, "middle_yaw": YAW_RIGHT, "ring_yaw": YAW_CENTER
    }
}

# --- 动态手势 (无改动) ---
async def demo_beckon_loop():
    base_pose = PRESET_GESTURES["point_center"]
    try:
        while True:
            pose_in = base_pose.copy(); pose_in["index"] = BEND_CLOSED
            await broadcast_state(pose_in); await apply_pose_to_servos(pose_in, c)
            await asyncio.sleep(0.5)
            
            pose_out = base_pose.copy(); pose_out["index"] = BEND_OPEN
            await broadcast_state(pose_out); await apply_pose_to_servos(pose_out, c)
            await asyncio.sleep(0.5)
    except asyncio.CancelledError:
        print("Demo 'beckon' stopped.")
        raise

async def demo_peace_wave_loop():
    try:
        while True:
            pose_left = PRESET_GESTURES["peace_left"]
            await broadcast_state(pose_left); await apply_pose_to_servos(pose_left, c)
            await asyncio.sleep(0.7)
            
            pose_right = PRESET_GESTURES["peace_right"]
            await broadcast_state(pose_right); await apply_pose_to_servos(pose_right, c)
            await asyncio.sleep(0.7)
    except asyncio.CancelledError:
        print("Demo 'peace_wave' stopped.")
        raise

async def demo_finger_roll_loop():
    base_pose = PRESET_GESTURES["open_hand"].copy()
    try:
        while True:
            await broadcast_state(base_pose); await apply_pose_to_servos(base_pose, c)
            await asyncio.sleep(0.5)
            pose1 = base_pose.copy(); pose1["index"] = BEND_CLOSED
            await broadcast_state(pose1); await apply_pose_to_servos(pose1, c)
            await asyncio.sleep(0.3)
            pose2 = pose1.copy(); pose2["middle"] = BEND_CLOSED
            await broadcast_state(pose2); await apply_pose_to_servos(pose2, c)
            await asyncio.sleep(0.3)
            pose3 = pose2.copy(); pose3["ring"] = BEND_CLOSED
            await broadcast_state(pose3); await apply_pose_to_servos(pose3, c)
            await asyncio.sleep(0.5)
    except asyncio.CancelledError:
        print("Demo 'finger_roll' stopped.")
        raise

async def demo_thumb_dance_loop():
    base_pose = PRESET_GESTURES["fist"].copy()
    base_pose["thumb"] = BEND_OPEN
    try:
        while True:
            pose_l = base_pose.copy(); pose_l["thumb_yaw"] = YAW_LEFT
            await broadcast_state(pose_l); await apply_pose_to_servos(pose_l, c)
            await asyncio.sleep(0.4)
            pose_r = base_pose.copy(); pose_r["thumb_yaw"] = YAW_RIGHT
            await broadcast_state(pose_r); await apply_pose_to_servos(pose_r, c)
            await asyncio.sleep(0.4)
    except asyncio.CancelledError:
        print("Demo 'thumb_dance' stopped.")
        raise

DYNAMIC_GESTURES = {
    "demo_beckon": demo_beckon_loop,
    "demo_peace_wave": demo_peace_wave_loop,
    "demo_roll": demo_finger_roll_loop,
    "demo_thumb_dance": demo_thumb_dance_loop,
}

# --- 5. 串口和控制器初始化 (无改动) ---
SERVO_CONTROLLER_PORT = "/dev/ttyACM0"
try:
    c = Scs0009PyController(
        serial_port=SERVO_CONTROLLER_PORT,
        baudrate=1000000,
        timeout=0.5,
    )
    print(f"成功连接到机械手控制器: {SERVO_CONTROLLER_PORT}")
except serial.SerialException as e:
    print(f"串口错误: {e}"); exit()

# --- 6. 全局状态和辅助函数 (无改动) ---
CONNECTED_CLIENTS = set()
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
    if CONNECTED_CLIENTS:
        message = json.dumps(state)
        await asyncio.gather(
            *[client.send(message) for client in CONNECTED_CLIENTS]
        )

async def apply_pose_to_servos(pose_data, controller):
    try:
        bend_offsets = {
            'Thumb': map_value(pose_data.get('thumb', BEND_OPEN), GESTURE_ANGLE_MIN, GESTURE_ANGLE_MAX, OPEN_ANGLE_OFFSET, CLOSE_ANGLE_OFFSET),
            'Index': map_value(pose_data.get('index', BEND_OPEN), GESTURE_ANGLE_MIN, GESTURE_ANGLE_MAX, OPEN_ANGLE_OFFSET, CLOSE_ANGLE_OFFSET),
            'Middle': map_value(pose_data.get('middle', BEND_OPEN), GESTURE_ANGLE_MIN, GESTURE_ANGLE_MAX, OPEN_ANGLE_OFFSET, CLOSE_ANGLE_OFFSET),
            'Ring': map_value(pose_data.get('ring', BEND_OPEN), GESTURE_ANGLE_MIN, GESTURE_ANGLE_MAX, OPEN_ANGLE_OFFSET, CLOSE_ANGLE_OFFSET)
        }
        yaw_offsets = {
            'Thumb': map_value(pose_data.get('thumb_yaw', YAW_CENTER), YAW_GESTURE_MIN, YAW_GESTURE_MAX, -YAW_ANGLE_OFFSET_MAX, YAW_ANGLE_OFFSET_MAX),
            'Index': map_value(pose_data.get('index_yaw', YAW_CENTER), YAW_GESTURE_MIN, YAW_GESTURE_MAX, -YAW_ANGLE_OFFSET_MAX, YAW_ANGLE_OFFSET_MAX),
            'Middle': map_value(pose_data.get('middle_yaw', YAW_CENTER), YAW_GESTURE_MIN, YAW_GESTURE_MAX, -YAW_ANGLE_OFFSET_MAX, YAW_ANGLE_OFFSET_MAX),
            'Ring': map_value(pose_data.get('ring_yaw', YAW_CENTER), YAW_GESTURE_MIN, YAW_GESTURE_MAX, -YAW_ANGLE_OFFSET_MAX, YAW_ANGLE_OFFSET_MAX)
        }
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

        positions_rad = np.deg2rad(positions_deg).tolist()
        controller.sync_write_goal_position(all_ids, positions_rad)
    except Exception as e:
        print(f"Apply pose error: {e}")

# --- 7. WebSocket 主处理器 (无改动) ---
async def handler(websocket):
    global current_demo_task
    
    CONNECTED_CLIENTS.add(websocket)
    print(f"网页控制器已连接。当前连接数: {len(CONNECTED_CLIENTS)}")
    
    try:
        async for message in websocket:
            # (1) 收到任何消息，先停止演示
            if current_demo_task:
                current_demo_task.cancel()
                current_demo_task = None
            
            try:
                pose_data = json.loads(message)
                
                # (2) 检查是否是 "动作" 命令
                if "action" in pose_data:
                    action_name = pose_data["action"]
                    
                    if action_name == "stop_demo":
                        print("收到停止命令 (Stop)。")
                        continue # 任务已取消，跳过
                    
                    # (3A) "静态" 手势
                    elif action_name in PRESET_GESTURES:
                        print(f"收到静态动作: {action_name}")
                        static_pose = PRESET_GESTURES[action_name]
                        await broadcast_state(static_pose) # 广播UI
                        await apply_pose_to_servos(static_pose, c) # 执行动作
                    
                    # (3B) "动态" 手势
                    elif action_name in DYNAMIC_GESTURES:
                        print(f"启动动态演示: {action_name}")
                        demo_function = DYNAMIC_GESTURES[action_name]
                        current_demo_task = asyncio.create_task(demo_function())
                    else:
                        print(f"收到未知动作: {action_name}")
                
                # (4) 否则，它就是 "滑块" 数据 (来自手动或MediaPipe)
                else:
                    await broadcast_state(pose_data)
                    await apply_pose_to_servos(pose_data, c)

            except json.JSONDecodeError:
                print(f"警告: 收到非JSON格式数据: {message}")
            except Exception as e:
                print(f"处理消息时出错: {e}")

    except websockets.exceptions.ConnectionClosed:
        print("网页控制器已断开连接。")
    finally:
        CONNECTED_CLIENTS.remove(websocket)
        print(f"一个连接已关闭。剩余连接数: {len(CONNECTED_CLIENTS)}")
        if not CONNECTED_CLIENTS and current_demo_task:
            print("最后一个客户端断开，停止演示。")
            current_demo_task.cancel()
            current_demo_task = None

# --- 8. 主函数 (无改动) ---
async def main():
    all_servo_ids = setup_servos(c, FINGERS)
    try:
        async with websockets.serve(handler, "0.0.0.0", 8765):
            print("\nWebSocket 服务器已在 ws://localhost:8765 启动")
            print("等待网页控制器连接...")
            await asyncio.Future()
    except KeyboardInterrupt:
        print("\n检测到用户中断（Ctrl+C）...")
    finally:
        print("正在关闭所有电机的扭矩...")
        if current_demo_task:
            current_demo_task.cancel()
        if all_servo_ids:
            disable_values = [0] * len(all_servo_ids)
            c.sync_write_torque_enable(all_servo_ids, disable_values)
        print("扭矩已关闭。程序结束。")

if __name__ == '__main__':
    asyncio.run(main())