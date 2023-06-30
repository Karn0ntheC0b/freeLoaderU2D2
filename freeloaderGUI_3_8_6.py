"""
freeLoaderGUI_3_8_6

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

# Set the maximum number of open figures
plt.rcParams["figure.max_open_warning"] = 5000  # or any other suitable value

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
    def __init__(self):
        self.dyna_online = False
        self.cell_online = False
        self.interrupt_flag = False
        self.measurements = []
        self.graph_frame = None  # Define the graph_frame attribute
        self.window = tk.Tk()
        self.window.title("Weight Measurements")
        self.dxl_id = DXL_ID

        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

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



    def plot_weight_measurements(self):
     # Extract the timestamps and weights from the measurements list
     timestamps, weights, _ = zip(*self.measurements)

     # Clear the previous graph
     if self.graph_frame is not None:
        self.graph_frame.destroy()

     # Create a new graph frame
     self.graph_frame = tk.Frame(window)
     self.graph_frame.grid(row=5, column=0, columnspan=4, padx=10, pady=10)

     # Convert the timestamps to datetime objects
     timestamps = [datetime.strptime(ts, "%Y-%m-%d %H:%M:%S") for ts in timestamps]

     # Clear the previous graph
     plt.clf()

     # Plot the weights over time
     fig, ax = plt.subplots()
     ax.plot(timestamps, weights)
     ax.set_title("Tensile Load as a Function of Time")
     ax.set_xlabel("Time")
     ax.set_ylabel("Tensile Load (LB)")
     ax.tick_params(axis='x', rotation=45)
     fig.tight_layout()

     # Convert the matplotlib figure to a Tkinter-compatible canvas
     canvas = FigureCanvasTkAgg(fig, master=self.graph_frame)
     canvas.draw()

     # Update the embedded graph in the window
     graph_widget = canvas.get_tk_widget()
     graph_widget.grid(row=0, column=0)

     # Close the figure after drawing it
     plt.close(fig)

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

    def connect_loadstar(self, com_port, baudrate):
        """ 
        Method to connect to the Loadstar scale.
        com_port is a string of form "COM5" for Windows. 
        baudrate is the baudrate provided as an int.
        If a Loadstar scale is found, connect_loadstar will return normally
        and the cell_online attribute will be set to True.
        If not, a descriptive FreeloaderError will be raised.
        """
        # Make sure there is a need to connect in the first place
        if self.cell_online:
            raise FreeloaderError("Loadstar scale already connected.")

        # Attempt to connect to the scale
        try:
            self.loadstar = serial.Serial(com_port, baudrate, timeout=0.1)
        except serial.SerialException:
            raise FreeloaderError("Failed to connect to Loadstar scale.")

        self.cell_online = True

    def disconnect_loadstar(self):
        """ 
        Method to disconnect from the Loadstar scale.
        If no Loadstar scale is connected, a descriptive FreeloaderError will be raised.
        """
        # Make sure there is something to disconnect from
        if not self.cell_online:
            raise FreeloaderError("No Loadstar scale connected.")

        # Close the scale connection
        self.loadstar.close()

        self.cell_online = False


    def disconnect(self):
        """
        Method to disconnect from the Dynamixel motor and load cell.
        """
        if self.dyna_online:
            self.portHandler.closePort()
            self.dyna_online = False

        if self.cell_online:
            self.loadstar.close()
            self.cell_online = False


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
    

    def close(self):
        """
        Method to close the connections to the Dynamixel motor and load cell.
        """
        self.set_torque(False)
        self.portHandler.closePort()
        self.ser.close()

    def set_speed(self, speed):
        """
        Method to set the speed of the Dynamixel motor.
        speed is an integer between 0 and 1023.
        """
        self.packetHandler.write2ByteTxRx(
            self.portHandler, self.dxl_id, ADDR_MX_MOVING_SPEED, speed
        )

    def tare_load_cell(self):
        """
        Method to perform the tare operation on the load cell.
        """
        if not self.cell_online:
            raise FreeloaderError("Load cell not connected.")

        try:
            self.loadstar.write(('TARE\r').encode('utf-8'))  # Send tare command
            self.loadstar.flush()

            print("Load cell tared successfully.")
        except serial.SerialException:
            raise FreeloaderError("Failed to communicate with the load cell.")

    def set_torque(self, enable):
        """
        Method to enable/disable torque on the Dynamixel motor.
        enable should be a boolean: True to enable, False to disable.
        """
        self.packetHandler.write1ByteTxRx(
            self.portHandler, self.dxl_id, ADDR_MX_TORQUE_ENABLE, enable
        )


    def start_measurement(self):
     """
     Method to start the load cell measurement loop.
     """
     if not self.cell_online:
        raise FreeloaderError("Load cell not connected.")

     self.interrupt_flag = False
     self.measurements = []  # Clear previous measurements

     # Create a new thread for the measurement loop
     self.measurement_thread = threading.Thread(target=self._measurement_thread)
     self.measurement_thread.start()

    def _measurement_thread(self):
     """
     Internal method for the load cell measurement loop.
     """
     previous_weight = None
     while not self.interrupt_flag:
         try:
            # Read load cell data
            self.loadstar.write(('W\r\n').encode('utf-8'))  # Send weigh command
            self.loadstar.flush()
            response = self.loadstar.readline().decode('utf-8').strip()
            weight = float(response)

            # Get current timestamp
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

            # Get motor position
            dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(
                self.portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION
            )

            if dxl_comm_result == COMM_SUCCESS:
                print("Timestamp:", timestamp)
                print("Motor Position:", dxl_present_position)
            else:
                print("Failed to read motor position:", self.packetHandler.getTxRxResult(dxl_comm_result))

            # Append the measurement to the measurements list
            self.measurements.append((timestamp, weight, dxl_present_position))

            # Call the plot_weight_measurements method to update the plot
            self.plot_weight_measurements()

            # Print weight
            print("Weight: {} LB".format(weight))

            # Check if weight exceeds threshold
            if weight > 200:
                # Disable torque on the Dynamixel motor
                self.set_torque(TORQUE_DISABLE)
                print("Torque disabled.")

                # Show a warning message box
                messagebox.showwarning("Warning", "Torque limit exceeded!")

            # Check if sample break detected
            if previous_weight is not None and weight > 0.50 and weight < (previous_weight * 0.1):
                self.interrupt_flag = True  # Set interrupt flag to stop the loop

            previous_weight = weight

            # Check if position reaches 1000
            if dxl_present_position >= 1000:
                self.interrupt_flag = True

         except ValueError:
            print("Invalid weight response: {}".format(response))
         except serial.SerialException:
            print("Failed to communicate with the load cell.")

         # Set Speed
         self.set_speed(10)

         # Begin movement
         self.set_position(DXL_MAXIMUM_POSITION_VALUE)

         # Sleep for 0.1 second
         time.sleep(0.01)

         # Check the interrupt flag at regular intervals
         for _ in range(100):
            if self.interrupt_flag:
                break
            time.sleep(0.01)





    def stop_measurement(self):
        """ 
        Method to stop the weight measurement process.
        """
        # Set the interrupt flag to True to stop the measurement thread
        self.interrupt_flag = True

    def export_measurements(self, filename="weight_measurements.csv"):
     """
     Export the weight measurements to a CSV file.

     Args:
        filename (str): Name ofa the CSV file to export the measurements (default: "weight_measurements.csv").
     """
     software_info = {
        "Version": "freeLoaderGUI_4_0",
    

     }

     # Open the CSV file in write mode
     with open(filename, "w", newline="") as csv_file:
        csv_writer = csv.writer(csv_file)

        # Write the software information as a header
        csv_writer.writerow(["{}: {}".format(key, value) for key, value in software_info.items()])

        # Write the name as a row
        csv_writer.writerow(["Operator Initials", op_entry.get()])

        # Write the name as a row
        csv_writer.writerow(["Sample Name", sample_entry.get()])

        # Write the name as a row
        csv_writer.writerow(["Material Code", material_entry.get()])

        # Write the name as a row
        csv_writer.writerow(["Lot #", lot_entry.get()])

        # Write the column headers
        csv_writer.writerow(["Timestamp", "Weight (LB)", "Motor Position"])

        # Write the measurement data
        for measurement in self.measurements:
            csv_writer.writerow(measurement)


    def create_gui(self):
        """ 
        Method to create the graphical user interface.
        """
        self.graph_frame = tk.Frame(self.window)
        self.graph_frame.pack()

        self.canvas = FigureCanvasTkAgg(self._create_figure(), master=self.graph_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        button_frame = tk.Frame(self.window)
        button_frame.pack()

        start_button = tk.Button(button_frame, text="Start Measurement", command=self.start_measurement)
        start_button.pack(side=tk.LEFT)

        stop_button = tk.Button(button_frame, text="Stop Measurement", command=self.stop_measurement)
        stop_button.pack(side=tk.LEFT)

    def _create_figure(self):
        """ 
        Internal method to create the figure for plotting measurements.
        This method should not be called directly.
        """
        figure(figsize=(8, 6))
        plt.xticks(rotation=45)
        plt.xlabel("Time")
        plt.ylabel("Weight (g)")
        plt.title("Weight Measurements")
        return plt.figure

    def update_plot(self):
        """ 
        Method to update the plot with the latest measurements.
        This method should be called periodically to refresh the plot.
        """
        # Clear the figure
        plt.clf()

        # Extract timestamps and weights from measurements
        timestamps, weights = zip(*self.measurements)

        # Plot the measurements
        plt.plot(timestamps, weights)

        # Update the plot
        self.canvas.draw()

    def run_gui(self):
        """ 
        Method to run the graphical user interface.
        This method will start the main GUI loop and update the plot periodically.
        """
        while True:
            # Update the plot
            self.update_plot()

            # Update the GUI
            self.window.update_idletasks()
            self.window.update()

            # Wait for a short period of time between updates
            time.sleep(0.1)

        self.window.mainloop()


if __name__ == '__main__':
    # Create an instance of Freeloader
    freeloader = Freeloader()

    try:
        # Connect to the Dynamixel motor
        freeloader.connect_dynamixel(DEVICENAME, BAUDRATE)
        print("Connected to Dynamixel motor.")

        # Enable Torque
        freeloader.set_torque(TORQUE_ENABLE)

        #
        freeloader.set_speed(1000)

        # Set the position to 0
        freeloader.set_position(0)
        print("Motor moved to position 0.")

        # Connect to the Loadstar load cell
        freeloader.connect_loadstar(LOADSTAR_COM_PORT, LOADSTAR_BAUDRATE)
        print("Connected to Loadstar load cell.")

        # Tare the load cell
        freeloader.tare_load_cell()

        #
        freeloader.set_torque(TORQUE_ENABLE)

        # Create a GUI window
        window = tk.Tk()
        window.title("Freeloader")

        def start_measurement():
            # Disable the start button and enable the stop button
            start_button.config(state=tk.DISABLED)
            stop_button.config(state=tk.NORMAL)
            disconnect_button.config(state=tk.DISABLED)
            export_button.config(state=tk.DISABLED)

            try:
                # Create a new thread for the measurement loop
                measurement_thread = threading.Thread(target=freeloader.start_measurement)

                # Set the thread as a daemon, so it will exit when the main thread exits
                measurement_thread.daemon = True

                # Start the measurement thread
                measurement_thread.start()
            except FreeloaderError as e:
                messagebox.showerror("Error", str(e))
                start_button.config(state=tk.NORMAL)
                stop_button.config(state=tk.DISABLED)
                disconnect_button.config(state=tk.NORMAL)
                export_button.config(state=tk.NORMAL)

        def stop_measurement():
            start_button.config(state=tk.NORMAL)
            stop_button.config(state=tk.DISABLED)
            disconnect_button.config(state=tk.NORMAL)
            export_button.config(state=tk.NORMAL)
            freeloader.set_speed(1000)
            freeloader.set_position(0)

            freeloader.stop_measurement()

        def disconnect():
            start_button.config(state=tk.DISABLED)
            stop_button.config(state=tk.DISABLED)
            disconnect_button.config(state=tk.DISABLED)
            export_button.config(state=tk.DISABLED)

            try:
                freeloader.disconnect()
                print("Disconnected from Dynamixel motor and load cell.")
            except FreeloaderError as e:
                messagebox.showerror("Error", str(e))

            window.destroy()

        def export_measurements():
            try:
                # Ask the user for the save location
                file_path = filedialog.asksaveasfilename(
                    defaultextension=".csv",
                    filetypes=[("CSV Files", "*.csv")]
                )

                if file_path:
                    freeloader.export_measurements(file_path)
                    messagebox.showinfo("Export", "Measurements exported to {}".format(file_path))
            except FreeloaderError as e:
                messagebox.showerror("Error", str(e))

        # Increase the window size
        window.geometry("1000x800")  # Set the desired width and height

        # Create a Start button
        start_button = tk.Button(window, text="Start Measurement", command=start_measurement)
        start_button.grid(row=0, column=0, padx=10, pady=10)

        # Create a Stop button
        stop_button = tk.Button(window, text="Stop Measurement", command=stop_measurement, state=tk.DISABLED)
        stop_button.grid(row=0, column=1, padx=10, pady=10)

        # Create a Disconnect button
        disconnect_button = tk.Button(window, text="Disconnect", command=disconnect)
        disconnect_button.grid(row=0, column=2, padx=10, pady=10)

        # Create an Export button
        export_button = tk.Button(window, text="Export Measurements", command=export_measurements, state=tk.DISABLED)
        export_button.grid(row=0, column=3, padx=10, pady=10)

        # Create a label and entry box for the operator initials
        op_label = tk.Label(window, text="Operator Initials:")
        op_label.grid(row=1, column=0, padx=10, pady=10, sticky=tk.E)
        op_entry = tk.Entry(window)
        op_entry.grid(row=1, column=1, padx=10, pady=10)

        # Create a label and entry box for the sample name
        sample_label = tk.Label(window, text="Sample Name:")
        sample_label.grid(row=2, column=0, padx=10, pady=10, sticky=tk.E)
        sample_entry = tk.Entry(window)
        sample_entry.grid(row=2, column=1, padx=10, pady=10)

        # Create a label and entry box for the material code
        material_label = tk.Label(window, text="Material Code:")
        material_label.grid(row=2, column=2, padx=10, pady=10, sticky=tk.E)
        material_entry = tk.Entry(window)
        material_entry.grid(row=2, column=3, padx=10, pady=10)

        # Create a label and entry box for the lot number
        lot_label = tk.Label(window, text="Lot #:")
        lot_label.grid(row=2, column=4, padx=10, pady=10, sticky=tk.E)
        lot_entry = tk.Entry(window)
        lot_entry.grid(row=2, column=5, padx=10, pady=10)

        # Run the GUI event loop
        window.mainloop()

    except FreeloaderError as e:
        print("An error occurred: {}".format(str(e)))
        freeloader.disconnect()


