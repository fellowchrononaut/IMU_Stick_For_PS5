import re
import serial
import tkinter as tk
from tkinter import ttk

import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from evdev import UInput, AbsInfo, ecodes as e

# ---------- CONFIG ----------
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200
MAX_ANGLE = 45.0
SAMPLE_TIMEOUT = 0.05
UPDATE_INTERVAL_MS = 10
PLOT_INTERVAL_MS = 60   # 3D redraw throttle (~16 Hz)
# ----------------------------

ORIENT_RE = re.compile(
    r"Orientation:\s*([-0-9.]+)\s*,\s*([-0-9.]+)\s*,\s*([-0-9.]+)"
)
# .ino prints "Quaternion: w, x, y, z" (scalar-first Hamilton convention,
# from bno055_read_quaternion_wxyz). Adafruit's WebSerial viewer wants xyzw —
# different consumer, different ordering. Keep w-first here to match the sketch.
QUAT_RE = re.compile(
    r"Quaternion:\s*([-0-9.]+)\s*,\s*([-0-9.]+)\s*,\s*([-0-9.]+)\s*,\s*([-0-9.]+)"
)

AXIS_MIN, AXIS_MAX = -32768, 32767

# Body-frame box for 3D viz. Half-extents (X, Y, Z) — long dimension along Y so
# the red faces (perpendicular to the X pitch axis) end up elongated.
_BOX_HALFSIZE = np.array([0.25, 0.55, 0.08])
_BOX_CORNERS = np.array([
    [-1, -1, -1], [+1, -1, -1], [+1, +1, -1], [-1, +1, -1],
    [-1, -1, +1], [+1, -1, +1], [+1, +1, +1], [-1, +1, +1],
], dtype=float) * _BOX_HALFSIZE

# Each face: (corner indices in order, fill color). Bright = positive axis end.
_BOX_FACES = [
    ([0, 3, 7, 4], "#7a2222"),  # -X
    ([1, 2, 6, 5], "#e84141"),  # +X (bright red)
    ([0, 1, 5, 4], "#1f7a1f"),  # -Y
    ([2, 3, 7, 6], "#41e84a"),  # +Y (bright green)
    ([0, 1, 2, 3], "#1f1f7a"),  # -Z
    ([4, 5, 6, 7], "#4141e8"),  # +Z (bright blue)
]


# ---------- Euler -> rotation matrix (Tait-Bryan ZYX intrinsic) ----------
def euler_to_rotmat(yaw_deg, pitch_deg, roll_deg):
    """Yaw around Z, then pitch around new Y, then roll around new X."""
    y, p, r = np.radians([yaw_deg, pitch_deg, roll_deg])
    cy, sy = np.cos(y), np.sin(y)
    cp, sp = np.cos(p), np.sin(p)
    cr, sr = np.cos(r), np.sin(r)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def rotmat_to_euler_zyx(R):
    """Extract (yaw, pitch, roll) degrees, inverse of euler_to_rotmat."""
    pitch = -np.arcsin(np.clip(R[2, 0], -1.0, 1.0))
    if abs(np.cos(pitch)) > 1e-6:
        yaw = np.arctan2(R[1, 0], R[0, 0])
        roll = np.arctan2(R[2, 1], R[2, 2])
    else:  # gimbal lock
        yaw = 0.0
        roll = np.arctan2(-R[0, 1], R[1, 1])
    return np.degrees(yaw), np.degrees(pitch), np.degrees(roll)


# ---------- quaternion helpers (Hamilton; q = [w, x, y, z]) ----------
def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_mul(qa, qb):
    w1, x1, y1, z1 = qa
    w2, x2, y2, z2 = qb
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_to_rotmat(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


# ---------- virtual gamepad ----------
def make_xbox360_uinput():
    abs_stick = AbsInfo(value=0, min=AXIS_MIN, max=AXIS_MAX, fuzz=16, flat=128, resolution=0)
    abs_trig = AbsInfo(value=0, min=0, max=255, fuzz=0, flat=0, resolution=0)
    abs_hat = AbsInfo(value=0, min=-1, max=1, fuzz=0, flat=0, resolution=0)

    cap = {
        e.EV_KEY: [
            e.BTN_A, e.BTN_B, e.BTN_X, e.BTN_Y,
            e.BTN_TL, e.BTN_TR,
            e.BTN_SELECT, e.BTN_START, e.BTN_MODE,
            e.BTN_THUMBL, e.BTN_THUMBR,
        ],
        e.EV_ABS: [
            (e.ABS_X, abs_stick), (e.ABS_Y, abs_stick),
            (e.ABS_RX, abs_stick), (e.ABS_RY, abs_stick),
            (e.ABS_Z, abs_trig), (e.ABS_RZ, abs_trig),
            (e.ABS_HAT0X, abs_hat), (e.ABS_HAT0Y, abs_hat),
        ],
    }

    return UInput(
        cap,
        name="Microsoft X-Box 360 pad",
        vendor=0x045e, product=0x028e, version=0x110,
    )


def map_angle_to_axis(angle, center, max_angle=MAX_ANGLE):
    delta = angle - center
    x = max(-1.0, min(1.0, delta / max_angle))
    return int(x * AXIS_MAX)


# ---------- app ----------
class IMUControllerApp:
    def __init__(self, root):
        self.root = root
        root.title("Foot IMU -> Gamepad (Linux/uinput)")

        print(f"Opening serial port {SERIAL_PORT} at {BAUD_RATE}...")
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=SAMPLE_TIMEOUT)

        print("Creating virtual Xbox 360 pad via /dev/uinput...")
        self.ui = make_xbox360_uinput()
        print(f"  device: {self.ui.device.path}\n")

        # IMU state (chip frame; serial order = yaw, pitch, roll)
        self.current_yaw = None
        self.current_pitch = None
        self.current_roll = None
        self.current_quat = None         # [w, x, y, z]

        # Baselines captured on "Set Neutral"
        self.baseline_yaw = None
        self.baseline_pitch = None
        self.baseline_roll = None
        self.baseline_quat_inv = None    # conjugate of pose at calibration

        self.stick_x = 0
        self.stick_y = 0
        self._new_data = False           # dirty flag for 3D redraw
        self._last_active_stick = "left"

        self._build_ui()

        self.root.after(UPDATE_INTERVAL_MS, self.update_loop)
        self.root.after(PLOT_INTERVAL_MS, self.redraw_3d)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------- UI ----------
    def _build_ui(self):
        outer = ttk.Frame(self.root, padding=10)
        outer.grid(row=0, column=0, sticky="nsew")

        left = ttk.Frame(outer)
        left.grid(row=0, column=0, sticky="nw", padx=(0, 12))
        right = ttk.Frame(outer)
        right.grid(row=0, column=1, sticky="nw")

        # ----- LEFT: status / labels / controls -----
        r = 0
        ttk.Label(left, text="Current IMU (deg):").grid(row=r, column=0, sticky="w"); r += 1
        self.yaw_label = ttk.Label(left, text="Yaw:   ---")
        self.yaw_label.grid(row=r, column=0, sticky="w"); r += 1
        self.pitch_label = ttk.Label(left, text="Pitch: ---")
        self.pitch_label.grid(row=r, column=0, sticky="w"); r += 1
        self.roll_label = ttk.Label(left, text="Roll:  ---")
        self.roll_label.grid(row=r, column=0, sticky="w"); r += 1

        ttk.Separator(left, orient="horizontal").grid(row=r, column=0, sticky="ew", pady=5); r += 1

        ttk.Label(left, text="Baseline (neutral stance):").grid(row=r, column=0, sticky="w"); r += 1
        self.base_yaw_label = ttk.Label(left, text="Baseline yaw:  ---")
        self.base_yaw_label.grid(row=r, column=0, sticky="w"); r += 1
        self.base_roll_label = ttk.Label(left, text="Baseline roll: ---")
        self.base_roll_label.grid(row=r, column=0, sticky="w"); r += 1

        ttk.Separator(left, orient="horizontal").grid(row=r, column=0, sticky="ew", pady=5); r += 1

        ttk.Label(left, text="Drive which stick?").grid(row=r, column=0, sticky="w"); r += 1
        self.stick_choice = tk.StringVar(value="left")
        ttk.Radiobutton(left, text="Left stick",  variable=self.stick_choice,
                        value="left",  command=self._on_stick_change).grid(row=r, column=0, sticky="w"); r += 1
        ttk.Radiobutton(left, text="Right stick", variable=self.stick_choice,
                        value="right", command=self._on_stick_change).grid(row=r, column=0, sticky="w"); r += 1

        ttk.Separator(left, orient="horizontal").grid(row=r, column=0, sticky="ew", pady=5); r += 1

        ttk.Label(left, text="3D viz source:").grid(row=r, column=0, sticky="w"); r += 1
        self.viz_mode = tk.StringVar(value="euler")
        ttk.Radiobutton(left, text="Euler",      variable=self.viz_mode,
                        value="euler").grid(row=r, column=0, sticky="w"); r += 1
        ttk.Radiobutton(left, text="Quaternion", variable=self.viz_mode,
                        value="quaternion").grid(row=r, column=0, sticky="w"); r += 1

        ttk.Separator(left, orient="horizontal").grid(row=r, column=0, sticky="ew", pady=5); r += 1

        ttk.Label(left, text="Gamepad source:").grid(row=r, column=0, sticky="w"); r += 1
        self.output_mode = tk.StringVar(value="euler")
        ttk.Radiobutton(left, text="Euler",      variable=self.output_mode,
                        value="euler").grid(row=r, column=0, sticky="w"); r += 1
        ttk.Radiobutton(left, text="Quaternion", variable=self.output_mode,
                        value="quaternion").grid(row=r, column=0, sticky="w"); r += 1

        ttk.Separator(left, orient="horizontal").grid(row=r, column=0, sticky="ew", pady=5); r += 1

        ttk.Label(left, text="Stick output:").grid(row=r, column=0, sticky="w"); r += 1
        self.stick_label = ttk.Label(left, text="X: 0   Y: 0")
        self.stick_label.grid(row=r, column=0, sticky="w"); r += 1

        ttk.Separator(left, orient="horizontal").grid(row=r, column=0, sticky="ew", pady=5); r += 1

        self.calib_button = ttk.Button(left, text="Set Neutral (Foot Forward)",
                                       command=self.set_neutral)
        self.calib_button.grid(row=r, column=0, sticky="ew", pady=5); r += 1

        self.status_label = ttk.Label(left, text="Waiting for IMU data...")
        self.status_label.grid(row=r, column=0, sticky="w"); r += 1

        # ----- RIGHT: 3D pose + 2D stick indicator -----
        ttk.Label(right, text="IMU orientation (relative to neutral):").grid(row=0, column=0, sticky="w")
        self.fig = Figure(figsize=(3.5, 3.5), dpi=90)
        self.ax3d = self.fig.add_subplot(111, projection="3d")
        self._init_3d_axes()
        self.canvas3d = FigureCanvasTkAgg(self.fig, master=right)
        self.canvas3d.get_tk_widget().grid(row=1, column=0, sticky="nw")

        ttk.Label(right, text="Stick position:").grid(row=2, column=0, sticky="w", pady=(8, 0))
        self.stick_canvas = tk.Canvas(right, width=200, height=200,
                                      bg="#1a1a1a", highlightthickness=1,
                                      highlightbackground="#444")
        self.stick_canvas.grid(row=3, column=0, sticky="nw")
        self._init_stick_canvas()

    def _init_3d_axes(self):
        ax = self.ax3d
        ax.set_xlim(-1.0, 1.0)
        ax.set_ylim(-1.0, 1.0)
        ax.set_zlim(-1.0, 1.0)
        ax.set_box_aspect((1, 1, 1))
        ax.set_xticks([]); ax.set_yticks([]); ax.set_zticks([])
        ax.view_init(elev=22, azim=-60)
        ax.disable_mouse_rotation()
        # World-frame reference: faint gray lines, labeled +/-
        ax.plot([-0.9, 0.9], [0, 0], [0, 0], color="#888", lw=0.5)
        ax.plot([0, 0], [-0.9, 0.9], [0, 0], color="#888", lw=0.5)
        ax.plot([0, 0], [0, 0], [-0.9, 0.9], color="#888", lw=0.5)
        ax.text(0.95, 0, 0, "X", color="#a44", fontsize=8)
        ax.text(0, 0.95, 0, "Y", color="#4a4", fontsize=8)
        ax.text(0, 0, 0.95, "Z", color="#44a", fontsize=8)
        self._body_poly = None

    def _init_stick_canvas(self):
        c = self.stick_canvas
        cx, cy, r = 100, 100, 90
        c.create_oval(cx - r, cy - r, cx + r, cy + r, outline="#666")
        c.create_line(cx, cy - r, cx, cy + r, fill="#444")
        c.create_line(cx - r, cy, cx + r, cy, fill="#444")
        self._stick_dot = c.create_oval(cx - 6, cy - 6, cx + 6, cy + 6,
                                        fill="#e44", outline="")

    # ---------- callbacks ----------
    def _on_stick_change(self):
        # zero whichever stick we *were* driving so it doesn't stick at last value
        prev = self._last_active_stick
        if prev == "left":
            self.ui.write(e.EV_ABS, e.ABS_X, 0)
            self.ui.write(e.EV_ABS, e.ABS_Y, 0)
        else:
            self.ui.write(e.EV_ABS, e.ABS_RX, 0)
            self.ui.write(e.EV_ABS, e.ABS_RY, 0)
        self.ui.syn()
        self._last_active_stick = self.stick_choice.get()

    def set_neutral(self):
        if self.current_yaw is None or self.current_roll is None:
            self.status_label.config(text="No IMU data yet; cannot set neutral.")
            return
        self.baseline_yaw = self.current_yaw
        self.baseline_pitch = self.current_pitch
        self.baseline_roll = self.current_roll
        if self.current_quat is not None:
            self.baseline_quat_inv = quat_conj(self.current_quat)
        self.base_yaw_label.config(text=f"Baseline yaw:  {self.baseline_yaw:7.2f}°")
        self.base_roll_label.config(text=f"Baseline roll: {self.baseline_roll:7.2f}°")
        self.status_label.config(text="Neutral stance set.")
        print(f"[CALIB] baseline: yaw={self.baseline_yaw:.2f}, "
              f"pitch={self.baseline_pitch:.2f}, roll={self.baseline_roll:.2f}")

    # ---------- serial ----------
    def read_serial_lines(self):
        # Drain all pending lines so 3D and stick stay in sync at high rates.
        while self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
            except Exception as ex:
                self.status_label.config(text=f"Serial error: {ex}")
                return
            if not line:
                continue
            m = ORIENT_RE.match(line)
            if m:
                self.current_yaw = float(m.group(1))
                self.current_pitch = float(m.group(2))
                self.current_roll = float(m.group(3))
                self._new_data = True
                continue
            m = QUAT_RE.match(line)
            if m:
                # .ino prints w, x, y, z in that order.
                self.current_quat = np.array([float(m.group(i)) for i in (1, 2, 3, 4)])
                self._new_data = True

    # ---------- gamepad ----------
    def send_stick(self, x, y):
        # evdev/xpad convention: positive ABS_Y = stick down. Negate so the
        # tester / game sees "stick up" when the foot pitches up.
        y_out = -y
        if self.stick_choice.get() == "left":
            self.ui.write(e.EV_ABS, e.ABS_X, x)
            self.ui.write(e.EV_ABS, e.ABS_Y, y_out)
        else:
            self.ui.write(e.EV_ABS, e.ABS_RX, x)
            self.ui.write(e.EV_ABS, e.ABS_RY, y_out)
        self.ui.syn()
        self._last_active_stick = self.stick_choice.get()

    # ---------- update loop (fast: serial + stick + 2D) ----------
    def update_loop(self):
        self.read_serial_lines()

        if self.current_yaw is not None and self.current_roll is not None:
            self.yaw_label.config(text=f"Yaw:   {self.current_yaw:7.2f}°")
            self.roll_label.config(text=f"Roll:  {self.current_roll:7.2f}°")
            if self.current_pitch is not None:
                self.pitch_label.config(text=f"Pitch: {self.current_pitch:7.2f}°")

            if self.baseline_yaw is None:
                self.status_label.config(
                    text="IMU OK. Click 'Set Neutral' with foot in forward position."
                )
            else:
                self._compute_stick()
                self.send_stick(self.stick_x, self.stick_y)
                self.stick_label.config(text=f"X: {self.stick_x:6d}   Y: {self.stick_y:6d}")
                self.status_label.config(
                    text=f"Streaming to virtual {self.stick_choice.get()} stick "
                         f"({self.output_mode.get()})..."
                )
                self._update_stick_canvas()

        self.root.after(UPDATE_INTERVAL_MS, self.update_loop)

    def _compute_stick(self):
        """Set self.stick_x / stick_y based on current output_mode."""
        if self.output_mode.get() == "quaternion" and \
           self.baseline_quat_inv is not None and self.current_quat is not None:
            q_rel = quat_mul(self.baseline_quat_inv, self.current_quat)
            R = quat_to_rotmat(q_rel)
            f_yaw, _, f_roll = rotmat_to_euler_zyx(R)
            # q_rel already encodes the delta; no sign flip on yaw (the Euler
            # path's negation existed only because the .ino's "360 - x" mangles
            # the Euler signs — the raw quaternion doesn't have that issue).
            self.stick_x = map_angle_to_axis(f_yaw, 0.0)
            self.stick_y = map_angle_to_axis(f_roll, 0.0)
        else:
            # Euler path (default). Stick X uses negated yaw delta because of the
            # 360-x flip in the .ino's Orientation print.
            self.stick_x = -map_angle_to_axis(self.current_yaw, self.baseline_yaw)
            self.stick_y = map_angle_to_axis(self.current_roll, self.baseline_roll)

    def _update_stick_canvas(self):
        cx, cy, r = 100, 100, 90
        dx = (self.stick_x / AXIS_MAX) * r
        dy = -(self.stick_y / AXIS_MAX) * r   # screen Y is inverted vs joystick Y
        px, py = cx + dx, cy + dy
        self.stick_canvas.coords(self._stick_dot, px - 6, py - 6, px + 6, py + 6)

    # ---------- 3D redraw (slower) ----------
    def redraw_3d(self):
        if self._new_data:
            self._new_data = False
            R = self._compute_viz_rotation()
            if R is not None:
                self._draw_body_axes(R)
                self.canvas3d.draw_idle()
        self.root.after(PLOT_INTERVAL_MS, self.redraw_3d)

    def _compute_viz_rotation(self):
        """Return rotation matrix for the 3D box based on current viz_mode."""
        if self.viz_mode.get() == "quaternion":
            if self.current_quat is None:
                return None
            if self.baseline_quat_inv is not None:
                q_rel = quat_mul(self.baseline_quat_inv, self.current_quat)
            else:
                q_rel = self.current_quat
            return quat_to_rotmat(q_rel)
        # Euler path. Negate pitch (Y-axis rotation) so the box rolls in the
        # same visual direction as the chip — empirically required for the
        # ".ino 360 - x" Euler sign convention. Applied to both R_cur and R_base.
        if self.current_yaw is None:
            return None
        R_cur = euler_to_rotmat(self.current_yaw, -self.current_pitch, self.current_roll)
        if self.baseline_yaw is not None:
            R_base = euler_to_rotmat(self.baseline_yaw, -self.baseline_pitch, self.baseline_roll)
            return R_base.T @ R_cur
        return R_cur

    def _draw_body_axes(self, R):
        if self._body_poly is not None:
            self._body_poly.remove()
        rotated = (R @ _BOX_CORNERS.T).T  # (8, 3)
        polys = [rotated[idx] for idx, _ in _BOX_FACES]
        colors = [c for _, c in _BOX_FACES]
        self._body_poly = Poly3DCollection(polys, facecolors=colors,
                                           edgecolors="k", linewidths=0.5)
        self.ax3d.add_collection3d(self._body_poly)

    # ---------- shutdown ----------
    def on_close(self):
        print("\nShutting down...")
        try:
            self.ui.write(e.EV_ABS, e.ABS_X, 0)
            self.ui.write(e.EV_ABS, e.ABS_Y, 0)
            self.ui.write(e.EV_ABS, e.ABS_RX, 0)
            self.ui.write(e.EV_ABS, e.ABS_RY, 0)
            self.ui.syn()
        except Exception:
            pass
        try:
            self.ui.close()
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass
        self.root.destroy()


def main():
    root = tk.Tk()
    IMUControllerApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
