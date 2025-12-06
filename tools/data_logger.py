import tkinter as tk
from tkinter import messagebox
import socket
import struct
import threading
import time
from datetime import datetime

# Connection settings for the data relay server
RELAY_HOST = "localhost"
RELAY_PORT = 8081

# Directory to save data
DATA_DIR = "raw_data/"

class DataCollector(threading.Thread):
    """A thread that connects to the relay server and collects data."""

    def __init__(self):
        super().__init__()
        self.daemon = True
        self._stop_event = threading.Event()
        self.collected_data = []
        self.socket = None

    def run(self):
        """Main thread loop for data collection."""
        try:
            # Connect to the relay server
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((RELAY_HOST, RELAY_PORT))
            self.socket.settimeout(1.0) # Set a timeout for recv operations
            print("Logger connected to relay server.")
        except ConnectionRefusedError:
            # Post error to the main thread's GUI
            app.master.after(0, lambda: app.show_connection_error())
            return

        buffer = bytearray()
        while not self._stop_event.is_set():
            try:
                # Read data from the socket
                data = self.socket.recv(1024)
                if not data:
                    # Connection closed by the peer
                    print("Relay server closed the connection.")
                    break
                
                buffer.extend(data)

                # Process all complete packets in the buffer
                while len(buffer) >= 32:
                    packet = buffer[:32]
                    del buffer[:32]

                    # Unpack data just like in client_test.py
                    unpacked_data = struct.unpack('<I7f', packet)
                    timestamp, ax, ay, az, gx, gy, gz, temp = unpacked_data
                    
                    # Format the data into a string
                    log_line = f"Timestamp: {timestamp}, AX: {ax:.2f}, AY: {ay:.2f}, AZ: {az:.2f}, GX: {gx:.2f}, GY: {gy:.2f}, GZ: {gz:.2f}, Temp: {temp:.2f}"
                    self.collected_data.append(log_line)
                    print(log_line)

            except socket.timeout:
                # No data received within the timeout, check if we need to stop
                if self._stop_event.is_set():
                    break # Exit loop if stop event is set
                continue # Otherwise, just continue waiting
            except socket.error as e:
                print(f"Socket error during collection: {e}")
                break
        
        # Clean up the connection
        if self.socket:
            self.socket.close()
        print("Logger disconnected.")

    def stop(self):
        """Signal the thread to stop."""
        self._stop_event.set()

class App:
    """The main GUI application."""
    def __init__(self, master):
        self.master = master
        self.master.title("Data Logger")
        self.master.geometry("300x150")

        self.collector_thread = None

        # --- Widgets ---
        self.status_label = tk.Label(master, text="Status: Idle", font=("Helvetica", 12))
        self.status_label.pack(pady=10)

        self.start_button = tk.Button(master, text="Start Recording", command=self.start_recording)
        self.start_button.pack(pady=5)

        self.stop_button = tk.Button(master, text="Stop Recording", command=self.stop_recording, state=tk.DISABLED)
        self.stop_button.pack(pady=5)
        
    def start_recording(self):
        """Handle the 'Start Recording' button click."""
        print("Starting recording...")
        self.status_label.config(text="Status: Recording...", fg="red")
        self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)

        # Start the data collection in a separate thread
        self.collector_thread = DataCollector()
        self.collector_thread.start()

    def stop_recording(self):
        """Handle the 'Stop Recording' button click."""
        if not self.collector_thread:
            return

        print("Stopping recording...")
        self.collector_thread.stop()
        self.collector_thread.join() # Wait for the thread to finish

        # Get the collected data
        data_to_save = self.collector_thread.collected_data
        
        # Reset GUI
        self.status_label.config(text="Status: Idle", fg="black")
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)

        if not data_to_save:
            print("No data was collected.")
            messagebox.showinfo("Info", "No data was collected.")
            return

        # Generate filename and save data
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{DATA_DIR}recording_{timestamp_str}.txt"
        
        try:
            with open(filename, "w") as f:
                for line in data_to_save:
                    f.write(line + "\n")
            print(f"Data saved to {filename}")
            messagebox.showinfo("Success", f"Successfully saved {len(data_to_save)} lines to:\n{filename}")
        except IOError as e:
            print(f"Error saving file: {e}")
            messagebox.showerror("Error", f"Could not save file to {filename}.\nError: {e}")
            
    def show_connection_error(self):
        """Show a connection error message in the GUI."""
        messagebox.showerror("Connection Error", "Could not connect to the data relay server on\n"
                                                 f"{RELAY_HOST}:{RELAY_PORT}\n\n"
                                                 "Please make sure `tools/data_relay.py` is running.")
        # Reset the GUI state
        self.status_label.config(text="Status: Idle", fg="black")
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)


if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()
