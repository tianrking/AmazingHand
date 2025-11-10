import time
import numpy as np
import serial  # <--- Added import for serial library
from rustypot import Scs0009PyController

# --- 1. Define Finger Configuration and Servo Parameters ---
# The FINGERS list defines the mapping between fingers and their corresponding servos.
FINGERS = [
    {'name': 'Index',  'm1_id': 1, 'm2_id': 2},
    {'name': 'Middle', 'm1_id': 3, 'm2_id': 4},
    {'name': 'Ring',   'm1_id': 5, 'm2_id': 6},
    {'name': 'Thumb',  'm1_id': 7, 'm2_id': 8}, # Thumb is controlled by an independent ADC channel
]

# Defines the motion range: an offset of -30 degrees when open, and +90 degrees when closed.
# The servos will move in real-time within this [-30, 90] degree range based on the ADC value.
CLOSE_ANGLE_OFFSET = 90
OPEN_ANGLE_OFFSET = -30 # Using a negative value more intuitively represents the offset in the opening direction

# --- 2. Initialize Controllers ---
# !!! NOTE: Please ensure the serial ports for the hand controller and the ESP32 are correct !!!
SERVO_CONTROLLER_PORT = "/dev/ttyACM0"      # Serial port for the robotic hand controller
ESP32_ADC_PORT = "/dev/ttyACM1"            # Serial port for the ESP32 development board

try:
    # Initialize the robotic hand controller
    c = Scs0009PyController(
        serial_port=SERVO_CONTROLLER_PORT,
        baudrate=1000000,
        timeout=0.5,
    )
    # Initialize the serial port for receiving data from ESP32
    adc_port = serial.Serial(ESP32_ADC_PORT, 115200, timeout=1)
    # Flush the input buffer to prevent reading old data
    adc_port.flushInput()
except serial.SerialException as e:
    print(f"Serial Error: {e}")
    print("Please confirm your serial port settings are correct and the devices are connected.")
    exit()


def map_value(x, in_min, in_max, out_min, out_max):
    """
    Core function: Linearly maps a value from one range to another.
    Used to map the ADC's 0-4095 range to the servo angle offset range
    from OPEN_ANGLE_OFFSET to CLOSE_ANGLE_OFFSET.
    """
    # Avoid division by zero
    if in_max == in_min:
        return out_min
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def main():
    """Main function: Starts the motors and enters the real-time remote control loop."""
    # Get all servo IDs to be controlled from the configuration
    all_servo_ids = [id for finger in FINGERS for id in (finger['m1_id'], finger['m2_id'])]
    
    print(f"Servo IDs to be controlled: {all_servo_ids}")
    print(f"Connected to Hand Controller: {SERVO_CONTROLLER_PORT}")
    print(f"Connected to ADC Data Source (ESP32): {ESP32_ADC_PORT}")

    try:
        # -- Start and initialize all motors --
        enable_values = [1] * len(all_servo_ids)
        c.sync_write_torque_enable(all_servo_ids, enable_values)
        print("Torque enabled for all motors.")

        speeds = [6] * len(all_servo_ids) # 6 is generally a fast speed
        c.sync_write_goal_speed(all_servo_ids, speeds)
        print("All motor speeds have been set.")
        time.sleep(0.5)

        # -- Enter real-time remote control main loop --
        print("\nEntering real-time control mode... Press Ctrl+C to exit.")
        while True:
            # Read only when data is available in the serial buffer to avoid blocking
            if adc_port.in_waiting > 0:
                # 1. Read and parse the serial data from the ESP32
                line = adc_port.readline().decode('utf-8').strip()
                
                # Must ensure the data format is "value1,value2"
                if ',' not in line:
                    continue # If the format is incorrect, skip this iteration

                try:
                    val1_str, val2_str = line.split(',')
                    adc_val_fingers = int(val1_str) # The first value controls the main three fingers
                    adc_val_thumb = int(val2_str)   # The second value controls the thumb
                except ValueError:
                    print(f"Warning: Could not parse data '{line}', skipping.")
                    continue

                # 2. Map the ADC values to angle offsets
                # Map ADC range 0-4095 to angle range -30 (Open) to +90 (Close)
                fingers_offset = map_value(adc_val_fingers, 0, 4095, OPEN_ANGLE_OFFSET, CLOSE_ANGLE_OFFSET)
                thumb_offset = map_value(adc_val_thumb, 0, 4095, OPEN_ANGLE_OFFSET, CLOSE_ANGLE_OFFSET)
                
                # (Optional) Print the current status for debugging
                # print(f"ADC: {adc_val_fingers},{adc_val_thumb} -> Angle Offset: Fingers={fingers_offset:.1f}, Thumb={thumb_offset:.1f}")

                # 3. Prepare the synchronous write command
                all_ids = []
                positions_deg = []
                for finger in FINGERS:
                    all_ids.extend([finger['m1_id'], finger['m2_id']])
                    
                    # Apply the corresponding angle offset based on the finger's name
                    if finger['name'] == 'Thumb':
                        current_offset = thumb_offset
                    else: # Index, Middle, Ring
                        current_offset = fingers_offset
                    
                    # M1 and M2 move in opposite directions from the 0-degree center
                    positions_deg.append(0 + current_offset)
                    positions_deg.append(0 - current_offset)

                # 4. Convert units and send the command to the hand
                positions_rad = np.deg2rad(positions_deg).tolist()
                c.sync_write_goal_position(all_ids, positions_rad)

    except KeyboardInterrupt:
        print("\nUser interrupt detected (Ctrl+C)...")

    finally:
        # Before the program ends, safely disable the torque on all motors
        print("Disabling torque on all motors...")
        if 'all_servo_ids' in locals() and all_servo_ids:
            disable_values = [0] * len(all_servo_ids)
            c.sync_write_torque_enable(all_servo_ids, disable_values)
        print("Torque disabled. Program terminated.")


if __name__ == '__main__':
    main()