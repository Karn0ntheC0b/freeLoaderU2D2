"""
freeLoaderGUI_5_8

Written by ChatGPT with guidance by Stephen Wood. Based off of freeLoaderbasic_2_7_2, also written by ChatGPT with Guidance by Stephen Wood. Originally based off of pyloader, U2D2 architecture prevented use of original script. Using Chat GPT, independent script developed using dynamixel-sdk library. 
"""


import tkinter as tk
from tkinter import messagebox, filedialog
import time
import threading
import os
import serial
from serial.tools import list_ports
from dynamixel_sdk import *
from datetime import datetime
import csv
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
from tkinter.ttk import Button
from tkinter.ttk import Combobox


# Control table address
ADDR_MX_GOAL_ANGLE = 30
ADDR_MX_MOVING_SPEED = 32
ADDR_MX_PRESENT_ANGLE = 36
ADDR_MX_TORQUE_ENABLE = 24
ADDR_MX_PRESENT_TEMPERATURE = 43
ADDR_MX_CW_ANGLE_LIMIT = 6
ADDR_MX_CCW_ANGLE_LIMIT = 8

# Protocol version
PROTOCOL_VERSION = 1.0

# Default setting ID = F232237060
BAUDRATE = 57600
DEVICENAME = 'COM3'
DXL_ID = 1
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

DXL_MOVING_STATUS_THRESHOLD = 30

# Default Loadstar settings
LOADSTAR_BAUDRATE = 9600
LOADSTAR_COM_PORT = 'COM2'


class FreeloaderError(Exception):
    """ Custom exception class for Freeloader errors """
    pass


class Freeloader:
 def __init__(self):
        self.dyna_online = False
        self.cell_online = False
        self.portHandler = None
        self.packetHandler = None
        self.loadstar = None
        self.interrupt_flag = False
        self.measurements = []
        self.window = None
        self.graph_frame = None

        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.machine_steps = None
        self.machine_steps = 0
        self.is_moving = False

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
            print("Succeeded to change baudrate.")
        else:
            raise FreeloaderError("Failed to change baudrate.")

        # Enable Dynamixel torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            raise FreeloaderError("Dynamixel torque enable failed.")
        elif dxl_error != 0:
            raise FreeloaderError("Dynamixel error occurred.")

        self.dyna_online = True

 def connect_loadstar(self, com_port, baudrate):
        """
        Method to connect to the Loadstar device.
        It initializes the serial connection.
        If the connection fails, it will raise a descriptive
        FreeloaderError.
        """
        try:
            import serial
        except ImportError:
            raise FreeloaderError("Failed to import serial module.")

        try:
            self.loadstar = serial.Serial(com_port, baudrate)
            self.cell_online = True
        except serial.SerialException:
            raise FreeloaderError("Failed to connect to the Loadstar device.")

 def disconnect_loadstar(self):
        """ Method to disconnect from the Loadstar device """
        if self.loadstar:
            self.loadstar.close()
            self.cell_online = False

 def get_machine_steps(self):
        # Return the current machine steps
        return self.machine_steps


 def set_machine_steps(self, steps):
        # Set the motor steps to the specified value
        self.machine_steps = steps

 def set_speed(self, speed):
        """
        Method to set the speed of the Dynamixel motor.
        speed is an integer between 0 and 1023.
        """
        self.packetHandler.write2ByteTxRx(
            self.portHandler, DXL_ID, ADDR_MX_MOVING_SPEED, speed
        )

 def disconnect_dynamixel(self):
        """ 
        Method to disconnect from the Dynamixel motor.
        If no Dynamixel is connected, a descriptive FreeloaderError will be raised.
        """
        # Make sure there is something to disconnect from
        if not self.dyna_online:
            raise FreeloaderError("No Dynamixel connected.")

        # Disable Dynamixel torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            raise FreeloaderError("Dynamixel torque disable failed.")
        elif dxl_error != 0:
            raise FreeloaderError("Dynamixel error occurred.")

        # Close the port
        self.portHandler.closePort()

        self.dyna_online = False


 def get_weight(self):
        """
        Method to read the weight from the Loadstar device.
        It sends the command to the device and waits for the response.
        If the communication fails, it will raise a descriptive
        FreeloaderError.
        """
        if not self.cell_online:
            raise FreeloaderError("Loadstar device is not connected.")

        self.loadstar.write(('W\r\n').encode('utf-8'))  # Send weigh command
        time.sleep(0.2)  # Wait for the response
        response = self.loadstar.read_all().decode('utf-8')  # Read the response

        try:
            weight = float(response)
            return weight
        except ValueError:
            raise FreeloaderError("Failed to read weight from the Loadstar device.")

 def measure(self):
        """
        Method to turn the motor for one complete revolution (360 degrees) based on motor steps.
        """
        try:
            # Store the initial steps for later restoration
            initial_steps = self.get_machine_steps()

            # Set the desired speed (adjust as needed)
            self.set_speed(2040)

            # Define the number of steps for one revolution
            steps_per_revolution = 4095 # Adjust this value based on your motor's steps per revolution
            revolutions = 40 * steps_per_revolution
            mm_per_step = 104/(4095*40)

            for _ in range(revolutions):
                if self.interrupt_flag:  # Check if the stop button was pressed
                    break

                # Move the motor by one step
                self.set_machine_steps(initial_steps + 1)
                initial_steps += 1
                
                # Calculate distance traveled in millimeters
                distance_mm = mm_per_step * initial_steps

                # Get timestamp, position, and weight values
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                position =  mm_per_step * initial_steps
                weight = self.get_weight()  # Replace this with the actual method to get weight value

                # Append measurement data to self.measurements list
                self.measurements.append((timestamp, position, weight))

                # Sleep for a short duration between steps
                time.sleep(0.001)  # Adjust this delay as needed

            # Stop the motor by setting the moving speed to 0
            self.set_speed(0)

        except FreeloaderError as e:
            messagebox.showerror("Error", str(e))

 def perform_motor_movement(self):
        """
        FOR DOING TEST Method to turn the motor for one complete revolution (360 degrees) based on motor steps.
        """
        try:
            # Store the initial steps for later restoration
            initial_steps = self.get_machine_steps()

            # Set the desired speed (adjust as needed)
            self.set_speed(2000)

            # Define the number of steps for one revolution
            steps_per_revolution = 4095  # Adjust this value based on your motor's steps per revolution
            revolutions = 40 * steps_per_revolution

            for _ in range(revolutions):
                if self.interrupt_flag:  # Check if the stop button was pressed
                    break

                # Move the motor by one step
                self.set_machine_steps(initial_steps + 1)
                initial_steps += 1

                # Sleep for a short duration between steps
                time.sleep(0.001)  # Adjust this delay as needed

            # Stop the motor by setting the moving speed to 0
            self.set_speed(0)

        except FreeloaderError as e:
            messagebox.showerror("Error", str(e))

 
 def move_motor_in_one_direction_down(self):
        """ Method to move the motor in one direction (continuous) FOR ADJUSTING MOTOR """
        if not self.is_moving:
            self.is_moving = True
            self.set_speed(1020)

 def move_motor_in_one_direction_up(self):
        """ Method to move the motor in one direction (continuous) FOR ADJUSTING MOTOR """
        if not self.is_moving:
            self.is_moving = True
            self.set_speed(2040)

 def stop_motor_movement(self):
        """ Method to stop the motor movement """
        self.interrupt_flag = True

 def continuous_motor_movement_up(self):
        """ Method to continuously move the motor in one direction """
        try:
            self.interrupt_flag = False  # Reset the interrupt flag

            while not self.interrupt_flag:
                self.move_motor_in_one_direction()
                time.sleep(0.001)  # Adjust this delay as needed

            self.set_speed(0)  # Stop the motor
            self.is_moving = False  # Update the is_moving attribute

        except FreeloaderError as e:
            messagebox.showerror("Error", str(e))

 def continuous_motor_movement_down(self):
        """ Method to continuously move the motor in one direction """
        try:
            self.interrupt_flag = False  # Reset the interrupt flag

            while not self.interrupt_flag:
                self.move_motor_in_one_direction()
                time.sleep(0.001)  # Adjust this delay as needed

            self.set_speed(0)  # Stop the motor
            self.is_moving = False  # Update the is_moving attribute

        except FreeloaderError as e:
            messagebox.showerror("Error", str(e))


 def start_measurement(self):
        """ Method to start the measurement process """
        self.interrupt_flag = False  # Reset the interrupt flag
        self.measurements = []
        motor_thread = threading.Thread(target=self.perform_motor_movement)
        motor_thread.start()

        # Perform measurements asynchronously while the motor moves
        threading.Thread(target=self.measure).start()

 def stop_measurement(self):
        """ Method to stop the measurement process """
        self.interrupt_flag = True  # Set the interrupt flag to stop motor and data collection

 def save_data(self, op_box, desc_box, mat_box, lot_box, type_combobox):
     """Method to save the measurements to a CSV file"""

     # Access the values from the entry boxes
     operator_initials = op_box.get()
     sample_description = desc_box.get()
     material_code = mat_box.get()
     lot_number = lot_box.get()
     selected_option = type_combobox.get()

     if not self.measurements:
        raise FreeloaderError("No measurements available.")

     try:
        filename = filedialog.asksaveasfilename(defaultextension=".csv")
        with open(filename, 'w', newline='') as file:
            writer = csv.writer(file)

            # Write the operator initials as a row
            writer.writerow(["freeLoaderGUI_4_0"])

            # Write the operator initials as a row
            writer.writerow(["Operator Initials", operator_initials])

            # Write the sample name as a row
            writer.writerow(["Sample Name", sample_description])

            # Write the material code as a row
            writer.writerow(["Material Code", material_code])

            # Write the lot number as a row
            writer.writerow(["Lot #", lot_number])

            # Write the selected option from the combobox as a row
            writer.writerow(["Selected Option", selected_option])

            writer.writerow(["Timestamp", "Position", "Weight"])
            writer.writerows(self.measurements)
     except IOError:
        raise FreeloaderError("Failed to save data to file.")

class FreeloaderGUI:
    def __init__(self, freeloader):
        self.freeloader = freeloader
        self.window = tk.Tk()
        self.graph_frame = tk.Frame(self.window)
        self.buttons_frame = tk.Frame(self.window)
        self.figure = figure(figsize=(6, 4), dpi=100)
        self.plot = self.figure.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.figure, master=self.graph_frame)
        self.boxes_frame = tk.Frame(self.window)
        self.type_frame = tk.Frame(self.window)
        self.is_moving = False

        # Create buttons
        button_font = ("Arial", 50)  # Set the font size
        self.start_button = Button(self.buttons_frame, text="Start", command=self.start_measurement, width=25)
        self.stop_button = Button(self.buttons_frame, text="Stop", command=self.stop_measurement)
        self.save_button = Button(self.buttons_frame, text="Save", command=self.save_data)
        self.move_up_button = Button(self.buttons_frame, text="Move Motor Up", command=self.move_motor_up)
        self.move_up_button.bind("<ButtonPress-1>", self.start_motorup)
        self.move_up_button.bind("<ButtonRelease-1>", self.stop_motor)
        self.move_down_button = Button(self.buttons_frame, text="Move Motor Down", command=self.move_motor_down)
        self.move_down_button.bind("<ButtonPress-1>", self.start_motordown)
        self.move_down_button.bind("<ButtonRelease-1>", self.stop_motor)

        # Create labels
        label_font = ("Arial", 25)  # Set the font size for labels
        self.op_label = tk.Label(self.boxes_frame, text="Operator Initials")
        self.desc_label = tk.Label(self.boxes_frame, text="Sample Description")
        self.mat_label = tk.Label(self.boxes_frame, text="Material Code")
        self.lot_label = tk.Label(self.boxes_frame, text="Lot #")
        self.type_label = tk.Label(self.type_frame, text="Sample Type")

        # Create drop-down (Combobox)
        self.type_var = tk.StringVar()
        self.type_combobox = Combobox(self.type_frame, textvariable=self.type_var, state="readonly")
        self.type_combobox["values"] = ["Monofilament", "ASTM Dog Bone", "Slit Film Yarn"]  # Add sample types
          

    def start_measurement(self):
        """ Method to start the measurement process """
        try:
            self.freeloader.start_measurement()
        except FreeloaderError as e:
            messagebox.showerror("Error", str(e))

    def stop_measurement(self):
        """ Method to stop the measurement process """
        self.freeloader.stop_measurement()

    def save_data(self):
     """ Method to save the measurements to a CSV file """
     try:
        self.freeloader.save_data(self.op_box, self.desc_box, self.mat_box, self.lot_box, self.type_combobox)
        messagebox.showinfo("Success", "Data saved successfully.")
     except FreeloaderError as e:
        messagebox.showerror("Error", str(e))


    def update_plot(self):
        """ Method to update the graph with the latest measurements """
        measurements = self.freeloader.measurements
        if measurements:
            timestamps = [m[0] for m in measurements]
            positions = [m[1] for m in measurements]
            weights = [m[2] for m in measurements]
            self.plot.cla()
            self.plot.plot(timestamps, positions, label='Distance (mm)')
            self.plot.plot(timestamps, weights, label='Tensile Load (lb.)')
            self.plot.legend()
            self.plot.set_xlabel('Timestamp')
            self.plot.set_ylabel('Distance (mm)/Load (lb.)')
            self.plot.set_title('Freeloader Tensile Data')
            self.figure.autofmt_xdate()
            self.canvas.draw()


    def start_motorup(self, event):
        """ Method to start moving the motor continuously """
        threading.Thread(target=self.continuous_motor_movement_up).start()

    def start_motordown(self, event):
        """ Method to start moving the motor continuously """
        threading.Thread(target=self.continuous_motor_movement_down).start()

    def stop_motor(self, event):
        """ Method to stop moving the motor """
        self.freeloader.stop_motor_movement()

    def move_motor_down(self):
        """ Method to move the motor when the button is pressed """
        try:
            self.freeloader.move_motor_in_one_direction_down()
        except FreeloaderError as e:
            messagebox.showerror("Error", str(e))

    def move_motor_up(self):
        """ Method to move the motor when the button is pressed """
        try:
            self.freeloader.move_motor_in_one_direction_down()
        except FreeloaderError as e:
            messagebox.showerror("Error", str(e))

    def continuous_motor_movement_up(self):
        """ Method to continuously move the motor in one direction """
        try:
            self.freeloader.interrupt_flag = False  # Reset the interrupt flag

            while not self.freeloader.interrupt_flag:
                self.freeloader.move_motor_in_one_direction_up()
                time.sleep(0.001)  # Adjust this delay as needed

            self.freeloader.set_speed(0)  # Stop the motor
            self.is_moving = False

        except FreeloaderError as e:
            messagebox.showerror("Error", str(e))

    def continuous_motor_movement_down(self):
        """ Method to continuously move the motor in one direction """
        try:
            self.freeloader.interrupt_flag = False  # Reset the interrupt flag

            while not self.freeloader.interrupt_flag:
                self.freeloader.move_motor_in_one_direction_down()
                time.sleep(0.001)  # Adjust this delay as needed

            self.freeloader.set_speed(0)  # Stop the motor
            self.is_moving = False

        except FreeloaderError as e:
            messagebox.showerror("Error", str(e))


    def start(self):
        """ Method to start the GUI """
        self.window.title("FreeloaderGUI_6_0")
        self.window.geometry("1280x800")

        # Graph frame
        self.graph_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        
        # Boxes frame
        self.boxes_frame.pack(side=tk.RIGHT, padx=50)
        button_font = ("Arial", 18)
        self.op_label = tk.Label(self.boxes_frame, text="Operator Initials", font=button_font)
        self.op_box = tk.Entry(self.boxes_frame, text="Operator Initials")
        self.desc_label = tk.Label(self.boxes_frame, text="Sample Description", font=button_font)
        self.desc_box = tk.Entry(self.boxes_frame, text="Sample Description")
        self.mat_label = tk.Label(self.boxes_frame, text="Material Code", font=button_font)
        self.mat_box = tk.Entry(self.boxes_frame, text="Material Code")
        self.lot_label = tk.Label(self.boxes_frame, text="Lot #", font=button_font)
        self.lot_box = tk.Entry(self.boxes_frame, text="Lot #")


        # Pack the labels and entry boxes in the correct order
        self.op_label.pack(side=tk.TOP, padx=20, pady=2, fill=tk.X)
        self.op_box.pack(side=tk.TOP, padx=20, pady=None, fill=tk.X)
        self.desc_label.pack(side=tk.TOP, padx=20, pady=2, fill=tk.X)
        self.desc_box.pack(side=tk.TOP, padx=20, pady=0, fill=tk.X)
        self.mat_label.pack(side=tk.TOP, padx=20, pady=2, fill=tk.X)
        self.mat_box.pack(side=tk.TOP, padx=20, pady=0, fill=tk.X)
        self.lot_label.pack(side=tk.TOP, padx=20, pady=2, fill=tk.X)
        self.lot_box.pack(side=tk.TOP, padx=20, pady=0, fill=tk.X)

        # Buttons frame
        self.buttons_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=25, pady=25)
        self.start_button.pack(side=tk.LEFT, padx=15)
        self.stop_button.pack(side=tk.LEFT, padx=15)
        self.save_button.pack(side=tk.LEFT, padx=15)
        self.move_down_button.pack(side=tk.RIGHT, padx=15)
        self.move_up_button.pack(side=tk.RIGHT, padx=15)

        # Type frame
        self.type_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=30, pady=30) 
        self.type_label.pack(anchor=tk.E, side=tk.LEFT, fill=tk.Y, padx=30, pady=30)
        self.type_combobox.pack(anchor=tk.NE, side=tk.LEFT, fill=tk.Y, padx=25, pady=25)

        # Update the graph periodically
        def update_graph():
            self.update_plot()
            self.window.after(1000, update_graph)

        update_graph()

        self.window.mainloop()





if __name__ == '__main__':
    freeloader = Freeloader()

    try:
        freeloader.connect_dynamixel(DEVICENAME, BAUDRATE)
        freeloader.connect_loadstar(LOADSTAR_COM_PORT, LOADSTAR_BAUDRATE)
    except FreeloaderError as e:
        messagebox.showerror("Error", str(e))
    else:
        gui = FreeloaderGUI(freeloader)
        gui.start()
        freeloader.disconnect_dynamixel()
        freeloader.disconnect_loadstar()
