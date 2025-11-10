import time
import numpy as np
from rustypot import Scs0009PyController

# --- 1. 统一配置所有舵机 ---
# 所有舵机的新 ID，以及它们的目标“0度”位置
SERVOS_CONFIG = {
    # 伺服ID: 目标角度 (度)
    11: 0,   # 手指 1
    12: 0,
    13: 0,   # 手指 2
    14: 0,
    15: 0,   # 手指 3
    16: 0,
    17: 0,   # 手指 4
    18: 0,
}

# --- 2. 初始化控制器 ---
# 你指定的 Linux 串口号
SERIAL_PORT = "/dev/ttyACM0" 

print(f"正在连接端口 {SERIAL_PORT}...")
try:
    c = Scs0009PyController(
        serial_port=SERIAL_PORT,
        baudrate=1000000,
        timeout=0.5,
    )
except Exception as e:
    print(f"连接失败: {e}")
    print("请检查：")
    print("1. 机械手是否已连接并上电？")
    print(f"2. 串口号 {SERIAL_PORT} 是否正确？ (是否需要 sudo权限？)")
    exit()


def move_all_servos_to_zero():
    """
    使用同步写入功能，将所有在配置中的舵机移动到 0 度位置。
    """
    servo_ids = list(SERVOS_CONFIG.keys())
    zero_positions_deg = list(SERVOS_CONFIG.values())

    # 将角度 (0度) 转换为弧度
    zero_positions_rad = np.deg2rad(zero_positions_deg).tolist()
    
    # 使用一个中等速度
    speeds = [3] * len(servo_ids)

    print(f"正在发送归零指令，移动舵机 {servo_ids} 到 0 度位置...")

    # --- 使用同步写入，一次性发送指令 ---
    c.sync_write_goal_speed(servo_ids, speeds) 
    time.sleep(0.01) # 协议要求的小延迟
    c.sync_write_goal_position(servo_ids, zero_positions_rad)


def main():
    """
    主函数：启动扭矩，并循环将舵机归零。
    """
    servo_ids = list(SERVOS_CONFIG.keys())
    
    print(f"准备控制的舵机ID: {servo_ids}")

    # 使用 try...finally 确保程序退出时能安全关闭扭矩
    try:
        # --- 3. 一次性启用所有舵机的扭矩 ---
        # 1 = 启用扭矩
        enable_values = [1] * len(servo_ids)
        c.sync_write_torque_enable(servo_ids, enable_values)
        print(f"已为所有舵机启动扭矩。")

        # 进入主循环
        print("舵机将保持在 0 度位置。你现在可以安装舵盘了。")
        print("按 Ctrl+C 退出程序并关闭扭矩。")
        while True:
            # 持续发送归零指令，确保舵机在外力下也能保持位置
            move_all_servos_to_zero()
            # 每 2 秒钟重新发送一次指令
            time.sleep(2)

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