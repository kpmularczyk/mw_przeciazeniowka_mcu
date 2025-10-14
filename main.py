import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading

class SerialReader(threading.Thread):
    def __init__(self, port, callback):
        super().__init__(daemon=True)
        self.port = port
        self.callback = callback
        self.running = True
        self.last_line = ""
        try:
            self.ser = serial.Serial(port, 9600, timeout=1)
        except Exception as e:
            self.ser = None
            messagebox.showerror("Serial Error", str(e))

    def run(self):
        while self.running and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode().strip()
                if line.startswith("CUR:"):
                    self.last_line = line
                    parts = line[4:].split(',')
                    currents = []
                    over_threshold = []
                    for part in parts:
                        part = part.strip()
                        if part.startswith('!'):
                            part = part[1:]
                            over_threshold.append(True)
                        try:
                            currents.append(float(part))
                            over_threshold.append(False)
                        except ValueError:
                            currents.append(0.0)
                    if len(currents) == 4:
                        self.callback(currents, over_threshold)
            except Exception:
                pass

    def stop(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()

class App:
    def __init__(self, root):
        self.root = root
        self.root.title("Arduino Current Monitor")

        self.serial_thread = None

        frame = ttk.Frame(root, padding=10)
        frame.pack(fill=tk.BOTH, expand=True)

        port_label = ttk.Label(frame, text="COM Port:")
        port_label.grid(row=0, column=0, sticky=tk.W)

        self.port_combo = ttk.Combobox(frame, values=self.get_ports(), state="readonly")
        self.port_combo.grid(row=0, column=1, padx=5)
        if self.port_combo['values']:
            self.port_combo.current(0)

        self.connect_btn = ttk.Button(frame, text="Connect", command=self.connect)
        self.connect_btn.grid(row=0, column=2, padx=5)

        self.disconnect_btn = ttk.Button(frame, text="Disconnect", command=self.disconnect, state=tk.DISABLED)
        self.disconnect_btn.grid(row=0, column=3, padx=5)

        self.current_fields = []
        self.threshold_fields = []
        self.set_threshold_buttons = []
        self.over_time_fields = []
        self.set_over_time_buttons = []

        for i in range(4):
            ttk.Label(frame, text=f"Channel {i} Current (A):").grid(row=i+1, column=0, sticky=tk.W)
            entry = tk.Entry(frame, width=10, state="readonly")
            entry.grid(row=i+1, column=1, padx=5, pady=2)
            self.current_fields.append(entry)

            # Threshold set field and button
            ttk.Label(frame, text="Set threshold:").grid(row=i+1, column=2, sticky=tk.E)
            threshold_entry = tk.Entry(frame, width=8)
            threshold_entry.grid(row=i+1, column=3, padx=2)
            self.threshold_fields.append(threshold_entry)

            set_btn = ttk.Button(frame, text="Set", command=lambda ch=i: self.set_threshold(ch))
            set_btn.grid(row=i+1, column=4, padx=2)
            self.set_threshold_buttons.append(set_btn)

            # Over threshold time field and button
            ttk.Label(frame, text="Over time (ms):").grid(row=i+1, column=5, sticky=tk.E)
            over_time_entry = tk.Entry(frame, width=8)
            over_time_entry.grid(row=i+1, column=6, padx=2)
            self.over_time_fields.append(over_time_entry)

            set_over_btn = ttk.Button(frame, text="Set", command=lambda ch=i: self.set_over_time(ch))
            set_over_btn.grid(row=i+1, column=7, padx=2)
            self.set_over_time_buttons.append(set_over_btn)

        # Add widgets for Over Threshold Hold (applies to all channels)
        ttk.Label(frame, text="Over threshold hold (ms):").grid(row=5, column=0, sticky=tk.W, pady=(10,0))
        self.over_hold_field = tk.Entry(frame, width=10)
        self.over_hold_field.grid(row=5, column=1, padx=5, pady=(10,0))
        self.set_over_hold_btn = ttk.Button(frame, text="Set", command=self.set_over_hold)
        self.set_over_hold_btn.grid(row=5, column=2, padx=5, pady=(10,0))

    def get_ports(self):
        return [port.device for port in serial.tools.list_ports.comports()]

    def connect(self):
        port = self.port_combo.get()
        if port:
            self.serial_thread = SerialReader(port, self.update_currents)
            self.serial_thread.start()
            self.connect_btn.config(state=tk.DISABLED)
            self.disconnect_btn.config(state=tk.NORMAL)

    def disconnect(self):
        if self.serial_thread:
            self.serial_thread.stop()
            self.serial_thread = None
        self.connect_btn.config(state=tk.NORMAL)
        self.disconnect_btn.config(state=tk.DISABLED)

    def update_currents(self, currents, over_threshold):
        for i in range(4):
            value = currents[i]
            entry = self.current_fields[i]
            entry.config(state="normal")
            entry.delete(0, tk.END)
            entry.insert(0, f"{value:.3f}")

            if over_threshold[i]:
                entry.config(bg="red")
            else:
                entry.config(bg="white")

    def set_threshold(self, channel):
        value = self.threshold_fields[channel].get()
        try:
            float_val = float(value)
            if self.serial_thread and self.serial_thread.ser and self.serial_thread.ser.is_open:
                cmd = f"C{channel}={float_val}\r\n"
                self.serial_thread.ser.write(cmd.encode())
        except ValueError:
            messagebox.showerror("Input Error", "Please enter a valid number for the threshold.")

    def set_over_time(self, channel):
        value = self.over_time_fields[channel].get()
        try:
            int_val = int(value)
            if self.serial_thread and self.serial_thread.ser and self.serial_thread.ser.is_open:
                cmd = f"T{channel}={int_val}\r\n"
                self.serial_thread.ser.write(cmd.encode())
        except ValueError:
            messagebox.showerror("Input Error", "Please enter a valid number for the over threshold time.")

    def set_over_hold(self):
        value = self.over_hold_field.get()
        try:
            int_val = int(value)
            if self.serial_thread and self.serial_thread.ser and self.serial_thread.ser.is_open:
                cmd = f"H={int_val}\r\n"
                self.serial_thread.ser.write(cmd.encode())
        except ValueError:
            messagebox.showerror("Input Error", "Please enter a valid number for the over threshold hold time.")

if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()