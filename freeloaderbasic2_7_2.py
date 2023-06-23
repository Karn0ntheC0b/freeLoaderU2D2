"""
freeloaderbasic2_7_2.py

Written for Python 3 via ChatGPT. 
Loosely based on work by John Amend, Nadia Cheng, and Anthony McNicoll.
Load cell data reading functionality added from python-example-3 by Mantej Dheri.

Simple Python interface for working with a Freeloader machine.
It assumes you have a simple Freeloader consisting of the following:
    - An MX-64 Dynamixel, programmed with ID = 1
    - A Loadstar USB load cell interface

The rest can be specified by the user. Usage is described below.

This code is made available under a Creative Commons
Attribution-Noncommercial-Share-Alike 3.0 license. See
<http://creativecommons.org/licenses/by-nc-sa/3.0> for details.
"""

__author__ = "Anthony McNicoll <am859@cornell.edu>, Mantej Dheri"

import time
import threading
import keyboard
import os
import serial
from serial.tools import list_ports
from dynamixel_sdk import *
from datetime import datetime
import msvcrt

# Control table address
ADDR_MX_GOAL_POSITION = 30
ADDR_MX_MOVING_SPEED = 32
ADDR_MX_PRESENT_POSITION = 36
ADDR_MX_TORQUE_ENABLE = 24

# Protocol version
PROTOCOL_VERSION = 1.0

# Default setting ID = F232237060
BAUDRATE = 57600
DEVICENAME = 'COM3'
DXL_ID = 1
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
DXL_MINIMUM_POSITION_VALUE = 0
DXL_MAXIMUM_POSITION_VALUE = 1020
DXL_MOVING_STATUS_THRESHOLD = 30

# Default Loadstar settings
LOADSTAR_BAUDRATE = 9600
LOADSTAR_COM_PORT = 'COM2'


class FreeloaderError(Exception):
    """ 
    This is an exception for errors explicitly related to a Freeloader.
    There is nothing special about it. It is instantiated with a single
    argument, a message string, which can be accessed by FreeloaderError.msg.
    """

    def __init__(self, msg):
        self.msg = msg


class Freeloader:
    """
    This class represents a classic Freeloader machine with a motor and load cell,
    and provides useful methods for interfacing with it.
    """

    def __init__(self):
        self.dyna_online = False
        self.cell_online = False
        self.interrupt_flag = False

    def connect_dynamixel(self, port, baudr):
        """ 
        Method to connect to the Dynamixel motor.
        port is a string of form "COM5" for Windows. 
        baudr is the baudrate provided as an int.
        If a Dynamixel is found, connect_dynamixel will return normally
        and the dyna_online attribute will be set to True.
        If not, a descriptive FreeloaderError will be raised.
        """
        # Make sure there is a need to connect in the first place
        if self.dyna_online:
            raise FreeloaderError("Dynamixel already connected.")

        # Initialize the port
        self.portHandler = PortHandler(port)

        # Open the port
        if self.portHandler.openPort():
            print("Succeeded to open the port.")
        else:
            raise FreeloaderError("Failed to open the port.")

        # Set port baudrate
        if self.portHandler.setBaudRate(baudr):
            print("Succeeded to change the baudrate.")
        else:
            raise FreeloaderError("Failed to change the baudrate.")

        # Initialize the packet handler
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        # Set Dynamixel ID
        self.dxl_id = DXL_ID

        # Enable Dynamixel Torque
        self.set_torque(True)

        # Set Dynamixel speed
        self.set_speed(0)

        # Set Dynamixel position to 0
        self.set_position(0)

        # Verify motor connection
        if self.ping():
            print("Dynamixel connected successfully.")
            self.dyna_online = True
        else:
            raise FreeloaderError("Failed to connect to the Dynamixel motor.")

    def on_key_release(self, event):
        if event.name == 'q':
            self.interrupt_flag = True

    def run(self):
        # ...

        keyboard.on_release(self.on_key_release)

        while True:
            # ...

            if freeloader.interrupt_flag:
                freeloader.set_torque(False)
                freeloader.set_speed(1023)
                freeloader.set_position(DXL_MINIMUM_POSITION_VALUE)
                print("Movement interrupted.")
                break


    def connect_load_cell(self, port):
        """
        Method to connect to the load cell.
        port is a string of form "COM5" for Windows.
        If a load cell is found, connect_load_cell will return normally
        and the cell_online attribute will be set to True.
        If not, a descriptive FreeloaderError will be raised.
        """
        # Make sure there is a need to connect in the first place
        if self.cell_online:
            raise FreeloaderError("Load cell already connected.")

        # Check if the specified port is available
        if port not in [comport.device for comport in list_ports.comports()]:
            raise FreeloaderError(f"Specified port '{port}' not found.")

        # Initialize the load cell communication
        self.ser = serial.Serial(port, LOADSTAR_BAUDRATE)

        # Verify load cell connection
        if self.ser.isOpen():
            print("Load cell connected successfully.")
            self.cell_online = True
        else:
            raise FreeloaderError("Failed to connect to the load cell.")

    def set_torque(self, enable):
        """
        Method to enable/disable torque on the Dynamixel motor.
        enable should be a boolean: True to enable, False to disable.
        """
        self.packetHandler.write1ByteTxRx(
            self.portHandler, self.dxl_id, ADDR_MX_TORQUE_ENABLE, enable
        )

    def tare_load_cell(self):
        """
        Method to perform the tare operation on the load cell.
        """
        if not self.cell_online:
          raise FreeloaderError("Load cell not connected.")

        try:
          self.ser.write(('TARE\r').encode('utf-8'))  # Send tare command
          self.ser.flush()
          print("Load cell tared successfully.")
        except serial.SerialException:
          raise FreeloaderError("Failed to communicate with the load cell.")



    def set_speed(self, speed):
        """
        Method to set the speed of the Dynamixel motor.
        speed is an integer between 0 and 1023.
        """
        self.packetHandler.write2ByteTxRx(
            self.portHandler, self.dxl_id, ADDR_MX_MOVING_SPEED, speed
        )

    def set_position(self, position):
        """
        Method to set the position of the Dynamixel motor.
        position is an integer between 0 and 1023.
        """
        self.packetHandler.write2ByteTxRx(
            self.portHandler, self.dxl_id, ADDR_MX_GOAL_POSITION, position
        )

    def get_position(self):
        """
        Method to get the current position of the Dynamixel motor.
        Returns an integer between 0 and 1023.
        """
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(
            self.portHandler, self.dxl_id, ADDR_MX_PRESENT_POSITION
        )
        if dxl_comm_result != COMM_SUCCESS:
            raise FreeloaderError(
                f"Failed to get the Dynamixel position (Error code: {dxl_comm_result})"
            )
        elif dxl_error != 0:
            raise FreeloaderError(
                f"Failed to get the Dynamixel position (Error code: {dxl_error})"
            )

        return dxl_present_position

    def ping(self):
        """
        Method to check if the Dynamixel motor is responding.
        Returns True if the motor responds, False otherwise.
        """
        dxl_model_number, dxl_comm_result, dxl_error = self.packetHandler.ping(
            self.portHandler, self.dxl_id
        )
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            return False
        else:
            return True

    def read_load_cell(self):
     self.ser.write(('W\r').encode('utf-8'))  # command to read the weight
     self.ser.flush()
     out = ''
     time.sleep(0.1)
     while self.ser.inWaiting() > 0:
        out += self.ser.read(1).decode("utf-8")
     try:
        out = float(out.strip())
        return out
     except ValueError:
        print("Invalid data format from the load cell.")
        return None

    def close(self):
        """
        Method to close the connections to the Dynamixel motor and load cell.
        """
        self.set_torque(False)
        self.portHandler.closePort()
        self.ser.close()


if __name__ == "__main__":
    freeloader = Freeloader()
    try:
        freeloader.connect_dynamixel(DEVICENAME, BAUDRATE)
        freeloader.connect_load_cell(LOADSTAR_COM_PORT)
        freeloader.set_position(DXL_MINIMUM_POSITION_VALUE)
        freeloader.set_speed(0)

        while True:
            print("Menu:")
            print("1. Press Enter to initiate movement from 0 to 1000. PRESS Q TO STOP TEST!!!")
            print("2. Tare load cell.")
            print("Press ESC to close the program.")
            choice = input("Enter your choice (1, 2, or ESC): ")

            if choice == "1":
                output_directory = input("Enter the directory path to save the data: ")
                output_directory = output_directory.strip('"')

                if output_directory:
                    try:
                        os.makedirs(output_directory, exist_ok=True)
                        timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
                        output_file = os.path.join(output_directory, f"motor_movement_data_{timestamp}.txt")

                        with open(output_file, "a") as file:
                            file.write("Timestamp, Position, Load\n")

                        input("Press Enter to initiate movement from 0 to 1000. PRESS Q TO STOP TEST!!!")

                        freeloader.set_torque(True)
                        freeloader.set_speed(10)
                        freeloader.set_position(DXL_MAXIMUM_POSITION_VALUE)

                        movement_started = False
                        while True:
                            position = freeloader.get_position()

                            if position > DXL_MINIMUM_POSITION_VALUE and not movement_started:
                                movement_started = True
                                print("Movement started. Reading load cell data...")

                            if movement_started:
                                load_data = freeloader.read_load_cell()
                                current_timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
                                print(f"{current_timestamp}, Position: {position}, Load: {load_data}")

                                with open(output_file, "a") as file:
                                    file.write(f"{current_timestamp}, {position}, {load_data}\n")

                            if position >= DXL_MAXIMUM_POSITION_VALUE - 10:
                                freeloader.set_torque(False)
                                freeloader.set_speed(1023)
                                freeloader.set_position(DXL_MINIMUM_POSITION_VALUE)
                                print("Movement complete.")
                                break

                            if freeloader.interrupt_flag:
                                freeloader.set_torque(False)
                                freeloader.set_speed(1023)
                                freeloader.set_position(DXL_MINIMUM_POSITION_VALUE)
                                print("Movement interrupted.")
                                break

                            keyboard.on_release(freeloader.on_key_release)  # Register the keyboard interrupt

                    except KeyboardInterrupt:
                        freeloader.set_torque(False)
                        freeloader.set_speed(1023)
                        freeloader.set_position(DXL_MINIMUM_POSITION_VALUE)
                        print("Movement interrupted.")

                else:
                    print("Please enter a valid directory path.")

            elif choice == "2":
                freeloader.tare_load_cell()  # Perform tare operation

            elif choice.upper() == "ESC":
                print("Closing the program...")
                break

            else:
                print("Invalid choice. Please try again.")

    except FreeloaderError as e:
        print("Freeloader error:", e.msg)

    finally:
        freeloader.close()

