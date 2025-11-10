import time
import numpy as np
from rustypot import Scs0009PyController

# --- 1. 定义手指配置和动作参数 ---
# FINGERS 列表定义了每根手指及其对应的两个电机ID
# 假设 m1_id 控制内外摆动, m2_id 控制屈伸 (或者反之，根据实际装配)
# 这里的逻辑是 m1 和 m2 相反运动来实现开合
FINGERS = [
    {'name': 'Index',  'm1_id': 1, 'm2_id': 2},
    {'name': 'Middle', 'm1_id': 3, 'm2_id': 4},
    {'name': 'Ring',   'm1_id': 5, 'm2_id': 6},
    {'name': 'Thumb',  'm1_id': 7, 'm2_id': 8},
]

# 定义开合动作的角度偏移量 (单位：度)
# 您可以调整这些值来改变手指的动作幅度
CLOSE_ANGLE_OFFSET = 90  # 合指时，一个电机+90度，另一个-90度
OPEN_ANGLE_OFFSET = 30   # 张开时，一个电机-30度，另一个+30度
# 注意：所有电机的中间位置都默认为 0 度

# --- 2. 初始化控制器 ---
# 请确保串口号 "COM11" 是正确的
c = Scs0009PyController(
    serial_port="/dev/ttyACM0",
    baudrate=1000000,
    timeout=0.5,
)

def close_all_fingers():
    """遍历所有手指配置，生成合指指令，并一次性发送。"""
    all_ids = []
    positions_deg = []

    for finger in FINGERS:
        all_ids.extend([finger['m1_id'], finger['m2_id']])
        
        # M1 和 M2 相反运动来合指
        positions_deg.append(0 + CLOSE_ANGLE_OFFSET)
        positions_deg.append(0 - CLOSE_ANGLE_OFFSET)

    # 将角度列表转换为弧度
    positions_rad = np.deg2rad(positions_deg).tolist()
    
    # 使用同步写入指令
    c.sync_write_goal_position(all_ids, positions_rad)
    print("指令：合上所有手指。")

def open_all_fingers():
    """遍历所有手指配置，生成张开指令，并一次性发送。"""
    all_ids = []
    positions_deg = []

    for finger in FINGERS:
        all_ids.extend([finger['m1_id'], finger['m2_id']])

        # M1 和 M2 相反运动来张开手指
        positions_deg.append(0 - OPEN_ANGLE_OFFSET)
        positions_deg.append(0 + OPEN_ANGLE_OFFSET)

    # 将角度列表转换为弧度
    positions_rad = np.deg2rad(positions_deg).tolist()
    
    # 使用同步写入指令
    c.sync_write_goal_position(all_ids, positions_rad)
    print("指令：张开所有手指。")


def main():
    """主函数：启动所有电机，并循环执行开合动作。"""
    # 从配置中获取所有需要控制的电机ID
    all_servo_ids = [id for finger in FINGERS for id in (finger['m1_id'], finger['m2_id'])]
    
    print(f"准备控制的伺服电机ID: {all_servo_ids}")

    try:
        # 一次性启用所有电机的扭矩
        enable_values = [1] * len(all_servo_ids)
        c.sync_write_torque_enable(all_servo_ids, enable_values)
        print("已为所有电机启动扭矩。")

        # 可以在开始时一次性设置好所有电机的速度
        speeds = [6] * len(all_servo_ids) # 6 通常是较快的速度
        c.sync_write_goal_speed(all_servo_ids, speeds)
        print("已设置所有电机速度。")
        time.sleep(0.5) # 等待电机准备好

        # 进入主循环
        while True:
            close_all_fingers()
            time.sleep(3)

            open_all_fingers()
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n检测到用户中断（Ctrl+C）...")

    finally:
        # 程序结束前，安全地关闭所有电机的扭矩
        print("正在关闭所有电机的扭矩...")
        disable_values = [0] * len(all_servo_ids)
        c.sync_write_torque_enable(all_servo_ids, disable_values)
        print("扭矩已关闭。程序结束。")

if __name__ == '__main__':
    main()