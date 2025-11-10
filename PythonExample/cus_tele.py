import time
import numpy as np
import serial  # <--- 新增导入 serial 库
from rustypot import Scs0009PyController

# --- 1. 定义手指配置和舵机参数 ---
# FINGERS列表保持不变，定义了手指和舵机的对应关系
FINGERS = [
    {'name': 'Index',  'm1_id': 1, 'm2_id': 2},
    {'name': 'Middle', 'm1_id': 3, 'm2_id': 4},
    {'name': 'Ring',   'm1_id': 5, 'm2_id': 6},
    {'name': 'Thumb',  'm1_id': 7, 'm2_id': 8}, # 大拇指由独立的ADC通道控制
]

# 动作范围定义保持不变: 张开时偏移-30度，合拢时偏移+90度
# 舵机将在这个 [-30, 90] 的区间内根据ADC值实时运动
CLOSE_ANGLE_OFFSET = 90  
OPEN_ANGLE_OFFSET = -30 # 使用负值更直观地表示张开方向的偏移

# --- 2. 初始化控制器 ---
# !!! 注意: 请确保机械手控制器和ESP32的串口号都是正确的 !!!
SERVO_CONTROLLER_PORT = "/dev/ttyACM0"      # 机械手控制器的串口号
ESP32_ADC_PORT = "/dev/ttyACM1"             # ESP32开发板的串口号

try:
    # 初始化机械手控制器
    c = Scs0009PyController(
        serial_port=SERVO_CONTROLLER_PORT,
        baudrate=1000000,
        timeout=0.5,
    )
    # 初始化ESP32数据接收串口
    adc_port = serial.Serial(ESP32_ADC_PORT, 115200, timeout=1)
    # 清空一下输入缓冲区，防止读取到旧数据
    adc_port.flushInput()
except serial.SerialException as e:
    print(f"串口错误: {e}")
    print("请确认您的串口号设置是否正确，以及设备是否已连接。")
    exit()


def map_value(x, in_min, in_max, out_min, out_max):
    """
    核心函数：将一个范围的值线性映射到另一个范围。
    用于将ADC的 0-4095 映射到舵机角度偏移 OPEN_ANGLE_OFFSET ~ CLOSE_ANGLE_OFFSET。
    """
    # 避免除以零
    if in_max == in_min:
        return out_min
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def main():
    """主函数：启动电机，并进入实时遥控循环。"""
    # 从配置中获取所有需要控制的电机ID
    all_servo_ids = [id for finger in FINGERS for id in (finger['m1_id'], finger['m2_id'])]
    
    print(f"准备控制的伺服电机ID: {all_servo_ids}")
    print(f"已连接机械手控制器: {SERVO_CONTROLLER_PORT}")
    print(f"已连接ADC数据源(ESP32): {ESP32_ADC_PORT}")

    try:
        # -- 启动和初始化所有电机 --
        enable_values = [1] * len(all_servo_ids)
        c.sync_write_torque_enable(all_servo_ids, enable_values)
        print("已为所有电机启动扭矩。")

        speeds = [6] * len(all_servo_ids) # 6 通常是较快的速度
        c.sync_write_goal_speed(all_servo_ids, speeds)
        print("已设置所有电机速度。")
        time.sleep(0.5)

        # -- 进入实时遥控主循环 --
        print("\n进入实时控制模式... 按 Ctrl+C 退出。")
        while True:
            # 仅当串口有数据时才读取，避免阻塞
            if adc_port.in_waiting > 0:
                # 1. 读取并解析来自ESP32的串口数据
                line = adc_port.readline().decode('utf-8').strip()
                
                # 必须确保数据格式是 "值1,值2"
                if ',' not in line:
                    continue # 如果格式不对，跳过此次循环

                try:
                    val1_str, val2_str = line.split(',')
                    adc_val_fingers = int(val1_str) # 第一个值控制主三指
                    adc_val_thumb = int(val2_str)   # 第二个值控制大拇指
                except ValueError:
                    print(f"警告: 无法解析数据 '{line}', 跳过。")
                    continue

                # 2. 将ADC值映射到角度偏移量
                # ADC范围 0-4095 映射到 角度范围 -30 (张开) 到 +90 (合上)
                fingers_offset = map_value(adc_val_fingers, 0, 4095, OPEN_ANGLE_OFFSET, CLOSE_ANGLE_OFFSET)
                thumb_offset = map_value(adc_val_thumb, 0, 4095, OPEN_ANGLE_OFFSET, CLOSE_ANGLE_OFFSET)
                
                # (可选) 打印当前状态用于调试
                # print(f"ADC: {adc_val_fingers},{adc_val_thumb} -> 角度偏移: 主三指={fingers_offset:.1f}, 大拇指={thumb_offset:.1f}")

                # 3. 准备同步写入指令
                all_ids = []
                positions_deg = []
                for finger in FINGERS:
                    all_ids.extend([finger['m1_id'], finger['m2_id']])
                    
                    # 根据手指名称，应用对应的角度偏移
                    if finger['name'] == 'Thumb':
                        current_offset = thumb_offset
                    else: # Index, Middle, Ring
                        current_offset = fingers_offset
                    
                    # M1 和 M2 以0度为中心，进行相反方向的运动
                    positions_deg.append(0 + current_offset)
                    positions_deg.append(0 - current_offset)

                # 4. 转换单位并发送指令到机械手
                positions_rad = np.deg2rad(positions_deg).tolist()
                c.sync_write_goal_position(all_ids, positions_rad)

    except KeyboardInterrupt:
        print("\n检测到用户中断（Ctrl+C）...")

    finally:
        # 程序结束前，安全地关闭所有电机的扭矩
        print("正在关闭所有电机的扭矩...")
        if 'all_servo_ids' in locals() and all_servo_ids:
            disable_values = [0] * len(all_servo_ids)
            c.sync_write_torque_enable(all_servo_ids, disable_values)
        print("扭矩已关闭。程序结束。")


if __name__ == '__main__':
    main()