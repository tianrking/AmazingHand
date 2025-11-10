import time
import numpy as np
from rustypot import Scs0009PyController

# --- 1. 统一配置所有舵机 ---
# 在这里定义所有需要控制的舵机ID和它们的中间位置（单位：角度）
# 您可以根据您的机械手配置，随意添加、删除或修改这里的舵机
SERVOS_CONFIG = {
    # 伺服ID: 中间位置角度
    1: 0,   # 食指
    2: 0,
    3: 0,   # 中指
    4: 0,
    5: 0,   # 无名指
    6: 0,
    7: 0,   # 拇指
    8: 0,
}

# --- 2. 初始化控制器 ---
# 请确保这里的串口号 "COM11" 是正确的
c = Scs0009PyController(
    serial_port="/dev/ttyACM0",
    baudrate=1000000,
    timeout=0.5,
)

def move_all_servos_to_middle():
    """
    使用同步写入功能，将所有在配置中的舵机移动到它们的中间位置。
    """
    # 从配置字典中获取所有舵机的ID列表和位置列表
    servo_ids = list(SERVOS_CONFIG.keys())
    middle_positions_deg = list(SERVOS_CONFIG.values())

    # 将角度转换为弧度，这是控制器需要的数据格式
    middle_positions_rad = np.deg2rad(middle_positions_deg).tolist()
    
    # 假设所有舵机都使用最大速度 (6)
    speeds = [6] * len(servo_ids)

    print(f"正在发送指令，移动舵机 {servo_ids} 到中间位置...")

    # --- 使用同步写入，一次性发送指令 ---
    # c.sync_write_goal_speed(servo_ids, speeds) # 如果需要，可以取消这行注释来设置速度
    c.sync_write_goal_position(servo_ids, middle_positions_rad)


def main():
    """
    主函数：启动扭矩，并循环将舵机归中。
    """
    servo_ids = list(SERVOS_CONFIG.keys())
    
    print(f"准备控制的舵机ID: {servo_ids}")

    # 使用 try...finally 确保程序退出时能安全关闭扭矩
    try:
        # --- 3. 一次性启用所有舵机的扭矩 ---
        # 1 = 启用扭矩, 0 = 关闭扭矩
        enable_values = [1] * len(servo_ids)
        c.sync_write_torque_enable(servo_ids, enable_values)
        print(f"已为所有舵机启动扭矩。")

        # 进入主循环
        while True:
            move_all_servos_to_middle()
            # 等待3秒，再执行下一次
            time.sleep(3)

    except KeyboardInterrupt:
        print("\n检测到用户中断（Ctrl+C）...")

    finally:
        # --- 4. 一次性关闭所有舵机的扭矩 ---
        print("正在关闭所有舵机的扭矩...")
        disable_values = [0] * len(servo_ids)
        c.sync_write_torque_enable(servo_ids, disable_values)
        print("扭矩已关闭。程序结束。")


if __name__ == '__main__':
    main()