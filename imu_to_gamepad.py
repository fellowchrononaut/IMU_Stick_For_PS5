import re
import time
import serial
import vgamepad as vg
import math
import tkinter as tk
from tkinter import ttk

# ---------- CONFIG ----------
COM_PORT = "COM10"        # <-- change this to your ESP32 COM port
BAUD_RATE = 115200
MAX_ANGLE = 45.0         # degrees for full deflection
SAMPLE_TIMEOUT = 0.05    # seconds
UPDATE_INTERVAL_MS = 10  # GUI & gamepad update period
# ----------------------------

# Parse "Orientation: h, p, r"
ORIENT_RE = re.compile(
    r"Orientation:\s*([-0-9.]+)\s*,\s*([-0-9.]+)\s*,\s*([-0-9.]+)"
)

def map_angle_to_axis(angle, center, max_angle=MAX_ANGLE):
    """ Map (angle - center) to joystick axis range (-32767..32767). """
    delta = angle - center
    x = max(-1.0, min(1.0, delta / max_angle))
    return int(x * 32767)

class IMUControllerApp:
    def __init__(self, root):
        self.root = root
        root.title("Foot IMU → Right Stick")

        # Serial & gamepad
        print(f"Opening serial port {COM_PORT} at {BAUD_RATE}...")
        self.ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=SAMPLE_TIMEOUT)
        self.gamepad = vg.VX360Gamepad()
        print("Virtual Xbox 360 controller created.\n")

        # IMU state
        self.current_pitch = None
        self.current_roll  = None
        self.current_heading = None

        # Baseline (neutral foot forward)
        self.baseline_pitch = None
        self.baseline_roll  = None

        # Stick state
        self.stick_x = 0
        self.stick_y = 0

        # ----- GUI widgets -----
        main = ttk.Frame(root, padding=10)
        main.grid(row=0, column=0, sticky="nsew")

        row = 0
        ttk.Label(main, text="Current IMU (deg):").grid(row=row, column=0, sticky="w")
        row += 1
        self.pitch_label = ttk.Label(main, text="Pitch: ---")
        self.pitch_label.grid(row=row, column=0, sticky="w")
        row += 1
        self.roll_label = ttk.Label(main, text="Roll:  ---")
        self.roll_label.grid(row=row, column=0, sticky="w")
        row += 1
        self.heading_label = ttk.Label(main, text="Heading: ---")
        self.heading_label.grid(row=row, column=0, sticky="w")
        row += 1

        ttk.Separator(main, orient="horizontal").grid(row=row, column=0, sticky="ew", pady=5)
        row += 1

        ttk.Label(main, text="Baseline (neutral stance):").grid(row=row, column=0, sticky="w")
        row += 1
        self.base_pitch_label = ttk.Label(main, text="Baseline pitch: ---")
        self.base_pitch_label.grid(row=row, column=0, sticky="w")
        row += 1
        self.base_roll_label = ttk.Label(main, text="Baseline roll:  ---")
        self.base_roll_label.grid(row=row, column=0, sticky="w")
        row += 1

        ttk.Separator(main, orient="horizontal").grid(row=row, column=0, sticky="ew", pady=5)
        row += 1

        ttk.Label(main, text="Mapped right stick:").grid(row=row, column=0, sticky="w")
        row += 1
        self.stick_label = ttk.Label(main, text="X: 0   Y: 0")
        self.stick_label.grid(row=row, column=0, sticky="w")
        row += 1

        ttk.Separator(main, orient="horizontal").grid(row=row, column=0, sticky="ew", pady=5)
        row += 1

        self.calib_button = ttk.Button(main, text="Set Neutral (Foot Forward)",
                                       command=self.set_neutral)
        self.calib_button.grid(row=row, column=0, sticky="ew", pady=5)
        row += 1

        self.status_label = ttk.Label(main, text="Waiting for IMU data...")
        self.status_label.grid(row=row, column=0, sticky="w")
        row += 1

        # Start update loop
        self.root.after(UPDATE_INTERVAL_MS, self.update_loop)

        # Clean shutdown
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def set_neutral(self):
        """Capture current pitch/roll as the neutral baseline."""
        if self.current_pitch is not None and self.current_roll is not None:
            self.baseline_pitch = self.current_pitch
            self.baseline_roll  = self.current_roll
            self.base_pitch_label.config(
                text=f"Baseline pitch: {self.baseline_pitch:7.2f}°"
            )
            self.base_roll_label.config(
                text=f"Baseline roll:  {self.baseline_roll:7.2f}°"
            )
            self.status_label.config(text="Neutral stance set.")
            print(f"[CALIB] New baseline set: pitch={self.baseline_pitch:.2f}, "
                  f"roll={self.baseline_roll:.2f}")
        else:
            self.status_label.config(text="No IMU data yet; cannot set neutral.")

    def read_serial_line(self):
        """Read one line from serial, parse orientation if available."""
        try:
            line = self.ser.readline().decode(errors="ignore").strip()
            if not line:
                return

            m = ORIENT_RE.match(line)
            if not m:
                return

            heading_deg = float(m.group(2))
            pitch_deg   = float(m.group(3))
            roll_deg    = float(m.group(1))

            self.current_heading = heading_deg
            self.current_pitch   = pitch_deg
            self.current_roll    = roll_deg

        except Exception as e:
            # Non-fatal: keep running
            self.status_label.config(text=f"Serial error: {e}")

    def update_loop(self):
        """Periodic update: read IMU, update mapping, gamepad, and GUI."""
        # 1) Read one line (non-blocking style, done often)
        self.read_serial_line()

        # 2) If we have IMU data, update labels & mapping
        if self.current_pitch is not None and self.current_roll is not None:
            self.pitch_label.config(text=f"Pitch:   {self.current_pitch:7.2f}°")
            self.roll_label.config(text=f"Roll:    {self.current_roll:7.2f}°")
            if self.current_heading is not None:
                self.heading_label.config(text=f"Yaw:{self.current_heading:7.2f}°")

            # If no baseline yet, hint to user
            if self.baseline_pitch is None:
                self.status_label.config(
                    text="IMU OK. Click 'Set Neutral' with foot in forward position."
                )
            else:
                # 3) Map roll -> X, pitch -> Y
                self.stick_x = map_angle_to_axis(self.current_roll,  self.baseline_roll)
                self.stick_y = map_angle_to_axis(self.current_pitch, self.baseline_pitch)

                # 4) Send to gamepad
                self.gamepad.right_joystick(x_value=self.stick_x, y_value=self.stick_y)
                self.gamepad.update()

                # 5) Update GUI
                self.stick_label.config(
                    text=f"X: {self.stick_x:6d}   Y: {self.stick_y:6d}"
                )
                self.status_label.config(text="Streaming to virtual right stick...")

                # Also log to console (optional)
                # print(f"pitch={self.current_pitch:7.2f} roll={self.current_roll:7.2f} "
                #       f"-> X={self.stick_x:6d}, Y={self.stick_y:6d}")

        # 6) Schedule next update
        self.root.after(UPDATE_INTERVAL_MS, self.update_loop)

    def on_close(self):
        """Clean shutdown when GUI window is closed."""
        print("\nShutting down...")
        try:
            self.ser.close()
        except:
            pass
        self.root.destroy()

def main():
    root = tk.Tk()
    app = IMUControllerApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()
