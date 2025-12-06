import tkinter as tk
from tkinter import messagebox, filedialog
import subprocess
import threading
import time
import os
import socket
import struct
from datetime import datetime
import signal

# --- Configuration ---
RELAY_SCRIPT = "tools/data_relay.py"
VISUALIZATION_SCRIPT = "tools/visualization_3d.py"
DATA_DIR = "demo/data/"
RELAY_HOST = "localhost"
RELAY_PORT = 8081
PACKET_SIZE = 32

class DataCollector(threading.Thread):
    """A thread that connects to the relay server and collects data."""

    def __init__(self, app_instance):
        super().__init__()
        self.daemon = True
        self._stop_event = threading.Event()
        self.collected_data = []
        self.socket = None
        self.app = app_instance

    def run(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((RELAY_HOST, RELAY_PORT))
            self.socket.settimeout(1.0)
            self.app.update_status("Logger: Connected and recording...")
        except ConnectionRefusedError:
            self.app.master.after(0, self.app.show_connection_error)
            return

        buffer = bytearray()
        while not self._stop_event.is_set():
            try:
                data = self.socket.recv(1024)
                if not data:
                    self.app.update_status("Logger: Relay server disconnected.")
                    break
                
                buffer.extend(data)

                while len(buffer) >= PACKET_SIZE:
                    packet = buffer[:PACKET_SIZE]
                    del buffer[:PACKET_SIZE]

                    unpacked_data = struct.unpack('<I7f', packet)
                    timestamp, ax, ay, az, gx, gy, gz, temp = unpacked_data
                    
                    log_line = (f"Timestamp: {timestamp}, "
                                f"AX: {ax:.2f}, AY: {ay:.2f}, AZ: {az:.2f}, "
                                f"GX: {gx:.2f}, GY: {gy:.2f}, GZ: {gz:.2f}, "
                                f"Temp: {temp:.2f}")
                    self.collected_data.append(log_line)
            
            except socket.timeout:
                continue
            except socket.error as e:
                self.app.update_status(f"Logger: Socket error: {e}")
                break
        
        if self.socket:
            self.socket.close()
        self.app.update_status("Logger: Disconnected.")

    def stop(self):
        self._stop_event.set()

class MainApp:
    def __init__(self, master):
        self.master = master
        self.master.title("Smart Ball Data Workflow")
        self.master.geometry("400x350")

        self.relay_process = None
        self.collector_thread = None
        self.last_logfile = None

        # --- Widgets ---
        # Relay Frame
        relay_frame = tk.LabelFrame(master, text="Step 1: Data Relay", padx=10, pady=10)
        relay_frame.pack(fill="x", padx=10, pady=5)

        self.relay_status_label = tk.Label(relay_frame, text="Status: Not Running", fg="red")
        self.relay_status_label.pack()
        self.start_relay_button = tk.Button(relay_frame, text="Start Relay Server", command=self.start_relay)
        self.start_relay_button.pack(side="left", expand=True, padx=5)
        self.stop_relay_button = tk.Button(relay_frame, text="Stop Relay Server", command=self.stop_relay, state=tk.DISABLED)
        self.stop_relay_button.pack(side="right", expand=True, padx=5)

        # Recording Frame
        rec_frame = tk.LabelFrame(master, text="Step 2: Record Data", padx=10, pady=10)
        rec_frame.pack(fill="x", padx=10, pady=5)
        
        self.rec_status_label = tk.Label(rec_frame, text="Status: Idle")
        self.rec_status_label.pack()
        self.start_rec_button = tk.Button(rec_frame, text="Start Recording", command=self.start_recording, state=tk.DISABLED)
        self.start_rec_button.pack(side="left", expand=True, padx=5)
        self.stop_rec_button = tk.Button(rec_frame, text="Stop Recording", command=self.stop_recording, state=tk.DISABLED)
        self.stop_rec_button.pack(side="right", expand=True, padx=5)
        
        # Analysis Frame
        vis_frame = tk.LabelFrame(master, text="Step 3: Analyze Data", padx=10, pady=10)
        vis_frame.pack(fill="x", padx=10, pady=5)

        self.last_file_label = tk.Label(vis_frame, text="Last file: None")
        self.last_file_label.pack()
        
        vis_button_frame = tk.Frame(vis_frame)
        vis_button_frame.pack(fill="x")

        self.visualize_button = tk.Button(vis_button_frame, text="Analyze Last Recording", command=self.run_visualization_last, state=tk.DISABLED)
        self.visualize_button.pack(side="left", expand=True, padx=5, pady=5)
        
        self.analyze_file_button = tk.Button(vis_button_frame, text="Analyze File...", command=self.analyze_file)
        self.analyze_file_button.pack(side="right", expand=True, padx=5, pady=5)
        
        self.master.protocol("WM_DELETE_WINDOW", self.on_closing)

    def start_relay(self):
        if self.relay_process:
            messagebox.showwarning("Warning", "Relay process is already running.")
            return

        self.update_status("Relay: Starting...")
        try:
            self.relay_process = subprocess.Popen(["python3", RELAY_SCRIPT], preexec_fn=os.setsid)
            self.relay_status_label.config(text=f"Status: Running (PID: {self.relay_process.pid})", fg="green")
            self.start_relay_button.config(state=tk.DISABLED)
            self.stop_relay_button.config(state=tk.NORMAL)
            self.start_rec_button.config(state=tk.NORMAL)
            self.update_status("Relay: Started successfully.")
        except Exception as e:
            self.update_status(f"Relay: Failed to start: {e}")
            messagebox.showerror("Error", f"Failed to start data_relay.py:\n{e}")

    def stop_relay(self):
        if self.relay_process:
            if self.collector_thread and self.collector_thread.is_alive():
                self.stop_recording()
            
            self.update_status("Relay: Stopping...")
            os.killpg(os.getpgid(self.relay_process.pid), signal.SIGTERM)
            self.relay_process.wait()
            self.relay_process = None
            
            self.relay_status_label.config(text="Status: Not Running", fg="red")
            self.start_relay_button.config(state=tk.NORMAL)
            self.stop_relay_button.config(state=tk.DISABLED)
            self.start_rec_button.config(state=tk.DISABLED)
            self.stop_rec_button.config(state=tk.DISABLED)
            self.update_status("Relay: Stopped.")
        
    def start_recording(self):
        self.rec_status_label.config(text="Status: Recording...", fg="blue")
        self.start_rec_button.config(state=tk.DISABLED)
        self.stop_rec_button.config(state=tk.NORMAL)
        self.visualize_button.config(state=tk.DISABLED)

        self.collector_thread = DataCollector(self)
        self.collector_thread.start()

    def stop_recording(self):
        if not self.collector_thread:
            return

        self.collector_thread.stop()
        self.collector_thread.join()

        data_to_save = self.collector_thread.collected_data
        self.collector_thread = None

        self.rec_status_label.config(text="Status: Idle", fg="black")
        self.start_rec_button.config(state=tk.NORMAL)
        self.stop_rec_button.config(state=tk.DISABLED)

        if not data_to_save:
            messagebox.showinfo("Info", "No data was collected.")
            return

        os.makedirs(DATA_DIR, exist_ok=True)
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.last_logfile = f"{DATA_DIR}recording_{timestamp_str}.txt"
        
        try:
            with open(self.last_logfile, "w") as f:
                for line in data_to_save:
                    f.write(line + "\n")
            
            self.last_file_label.config(text=f"Last file: {os.path.basename(self.last_logfile)}")
            self.visualize_button.config(state=tk.NORMAL)
            messagebox.showinfo("Success", f"Saved {len(data_to_save)} lines to\n{self.last_logfile}")
        except IOError as e:
            messagebox.showerror("Error", f"Could not save file.\nError: {e}")
            self.last_logfile = None
            
    def run_visualization_last(self):
        if not self.last_logfile:
            messagebox.showerror("Error", "No log file available to visualize.")
            return
        self.run_visualization(self.last_logfile)

    def analyze_file(self):
        filepath = filedialog.askopenfilename(
            title="Select a data file",
            initialdir=DATA_DIR,
            filetypes=(("Text files", "*.txt"), ("All files", "*.*"))
        )
        if filepath:
            self.run_visualization(filepath)

    def run_visualization(self, filepath):
        self.update_status(f"Visualizer: Launching for {os.path.basename(filepath)}")
        try:
            subprocess.Popen(["python3", VISUALIZATION_SCRIPT, "--input", filepath])
        except Exception as e:
            messagebox.showerror("Error", f"Failed to launch visualization script:\n{e}")

    def show_connection_error(self):
        messagebox.showerror("Connection Error", f"Could not connect to relay server on {RELAY_HOST}:{RELAY_PORT}")
        self.rec_status_label.config(text="Status: Idle", fg="black")
        self.start_rec_button.config(state=tk.NORMAL)
        self.stop_rec_button.config(state=tk.DISABLED)

    def update_status(self, message):
        print(message)

    def on_closing(self):
        if self.relay_process:
            self.stop_relay()
        self.master.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = MainApp(root)
    root.mainloop()
