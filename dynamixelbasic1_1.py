"""
Dynamixel Basic Control - Rotate Motor and Read Operating Mode
Author: Your Name
Date: DD/MM/YYYY
"""

import time
from dynamixel_sdk import *

# Control table address
ADDR_MX_GOAL_POSITION = 30
ADDR_MX_MOVING_SPEED = 32
ADDR_MX_PRESENT_POSITION = 36
ADDR_MX_OPERATING_MODE = 11
ADDR_MX_TORQUE_ENABLE = 24

# Protocol version
PROTOCOL_VERSION = 1.0

# Default setting ID = 1
BAUDRATE = 57600
DEVICENAME = 'COM3'
DXL_ID = 1
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
DXL_MINIMUM_POSITION_VALUE = 100
DXL_MAXIMUM_POSITION_VALUE = 1020
DXL_MOVING_STATUS_THRESHOLD = 10

def move_to_position(port_handler, packet_handler, position):
    dxl_goal_position = position
    dxl_comm_result, dxl_error = packet_handler.write4ByteTxRx(
        port_handler, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position
    )
    if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
        raise Exception("Failed to move motor")

    print("Motor moved to position:", dxl_goal_position)
    time.sleep(2)  # Wait for the motor to reach the goal position

def main():
    # Initialize PortHandler instance
    port_handler = PortHandler(DEVICENAME)

    # Initialize PacketHandler instance
    packet_handler = PacketHandler(PROTOCOL_VERSION)

    try:
        # Open the serial port
        if port_handler.openPort():
            print("Serial port opened")
        else:
            raise Exception("Failed to open the serial port")

        # Set baudrate
        if port_handler.setBaudRate(BAUDRATE):
            print("Baudrate set")
        else:
            raise Exception("Failed to set baudrate")

        # Enable torque for Dynamixel motor
        dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(
            port_handler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE
        )
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            raise Exception("Failed to enable torque for Dynamixel")

        # Read and display the operating mode of the motor
        dxl_operating_mode, dxl_comm_result, dxl_error = packet_handler.read1ByteTxRx(
            port_handler, DXL_ID, ADDR_MX_OPERATING_MODE
        )
        if dxl_comm_result == COMM_SUCCESS:
            print("Operating mode:", dxl_operating_mode)
        else:
            raise Exception("Failed to read operating mode. Error:", dxl_error)

        # Move to position 1000
        move_to_position(port_handler, packet_handler, 1000)

        
        # Move to position 0
        move_to_position(port_handler, packet_handler, 0)

        # Disable torque for Dynamixel motor
        dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(
            port_handler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE
        )
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            raise Exception("Failed to disable torque for Dynamixel")

        print("Torque disabled for Dynamixel")

    except Exception as e:
        print("Error:", str(e))

    finally:
        # Close the serial port
        port_handler.closePort()
        print("Serial port closed")


if __name__ == '__main__':
    main()
