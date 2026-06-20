"""
IMU Stick V1 — desktop configurator.

Connects to the V1 firmware over serial, lets the user tweak the axis mapping,
captures a neutral pose, manages multiple named profiles, persists BNO055
sensor-calibration offsets, visualises pose in 3D, and optionally drives a
virtual gamepad (Linux uinput) for testing without the PS5 controller plugged in.

Protocol mirrors the firmware. Live changes are sent immediately via SET;
"Save to ESP32" persists the current RAM config to the active profile in NVS.

Optional deps:
  - matplotlib   -> 3D pose viz panel
  - evdev        -> virtual gamepad output (Linux only)
  - pyserial     -> required for serial comms
"""

import re
import sys
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog

import serial

try:
    import numpy as np
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    HAVE_MPL = True
except Exception:
    HAVE_MPL = False

try:
    from evdev import UInput, AbsInfo, ecodes as e
    HAVE_EVDEV = True
except Exception:
    HAVE_EVDEV = False


DEFAULT_PORT = "/dev/ttyUSB0"
BAUD = 115200
SOURCES = ("heading", "pitch", "roll")
POLL_INTERVAL_MS = 30
UI_REFRESH_MS = 80
VIZ_REFRESH_MS = 60          # 3D redraw throttle

AXIS_MIN, AXIS_MAX = -32768, 32767

DATA_RE = re.compile(
    r"^D\s+h=([-0-9.]+)\s+p=([-0-9.]+)\s+r=([-0-9.]+)"
    r"\s+dx=(\d+)\s+dy=(\d+)\s+cal=(\d),(\d),(\d),(\d)"
    r"\s+baseline_set=(\d)"
)
CFG_RE = re.compile(
    r"^CFG\s+profile=(\S+)"
    r"\s+X\.src=(\w+)\s+X\.inv=(\d+)\s+X\.max=([-0-9.]+)\s+X\.dz=([-0-9.]+)"
    r"\s+Y\.src=(\w+)\s+Y\.inv=(\d+)\s+Y\.max=([-0-9.]+)\s+Y\.dz=([-0-9.]+)"
    r"\s+baseline=([-0-9.]+),([-0-9.]+),([-0-9.]+)\s+baseline_set=(\d+)"
    r"\s+bno_cal_saved=(\d+)"
)
PROFILES_RE = re.compile(r"^PROFILES\s*(.*)$")


# ---------------- Euler -> rotation matrix (ZYX intrinsic) ----------------
def euler_to_rotmat(yaw_deg, pitch_deg, roll_deg):
    if not HAVE_MPL:
        return None
    y, p, r = np.radians([yaw_deg, pitch_deg, roll_deg])
    cy, sy = np.cos(y), np.sin(y)
    cp, sp = np.cos(p), np.sin(p)
    cr, sr = np.cos(r), np.sin(r)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


# ---------------- virtual gamepad ----------------
def make_xbox360_uinput():
    abs_stick = AbsInfo(value=0, min=AXIS_MIN, max=AXIS_MAX, fuzz=16, flat=128, resolution=0)
    abs_trig = AbsInfo(value=0, min=0, max=255, fuzz=0, flat=0, resolution=0)
    abs_hat = AbsInfo(value=0, min=-1, max=1, fuzz=0, flat=0, resolution=0)
    cap = {
        e.EV_KEY: [
            e.BTN_A, e.BTN_B, e.BTN_X, e.BTN_Y, e.BTN_TL, e.BTN_TR,
            e.BTN_SELECT, e.BTN_START, e.BTN_MODE, e.BTN_THUMBL, e.BTN_THUMBR,
        ],
        e.EV_ABS: [
            (e.ABS_X, abs_stick), (e.ABS_Y, abs_stick),
            (e.ABS_RX, abs_stick), (e.ABS_RY, abs_stick),
            (e.ABS_Z, abs_trig), (e.ABS_RZ, abs_trig),
            (e.ABS_HAT0X, abs_hat), (e.ABS_HAT0Y, abs_hat),
        ],
    }
    return UInput(cap, name="IMU Stick V1 virtual pad",
                  vendor=0x045e, product=0x028e, version=0x110)


def dac_to_axis(dac):
    """DAC 0..255 -> -32768..32767, centered at 128."""
    n = (dac - 127.5) / 127.5
    n = max(-1.0, min(1.0, n))
    return int(n * AXIS_MAX)


class IMUStickApp:
    # 3D body-frame box (half-extents, long along Y so the toe-direction face is bigger)
    if HAVE_MPL:
        _BOX_HALFSIZE = np.array([0.25, 0.55, 0.08])
        _BOX_CORNERS = np.array([
            [-1, -1, -1], [+1, -1, -1], [+1, +1, -1], [-1, +1, -1],
            [-1, -1, +1], [+1, -1, +1], [+1, +1, +1], [-1, +1, +1],
        ], dtype=float) * _BOX_HALFSIZE
        _BOX_FACES = [
            ([0, 3, 7, 4], "#7a2222"), ([1, 2, 6, 5], "#e84141"),
            ([0, 1, 5, 4], "#1f7a1f"), ([2, 3, 7, 6], "#41e84a"),
            ([0, 1, 2, 3], "#1f1f7a"), ([4, 5, 6, 7], "#4141e8"),
        ]

    def __init__(self, root):
        self.root = root
        root.title("IMU Stick V1 — Configurator")

        # serial state
        self.ser = None
        self.port_var = tk.StringVar(value=DEFAULT_PORT)
        self.status_var = tk.StringVar(value="Not connected.")
        self.dirty_var = tk.StringVar(value="")
        self._line_buf = ""
        self._suppress_set = False

        # live state from D lines
        self.live = {
            "h": 0.0, "p": 0.0, "r": 0.0,
            "dx": 128, "dy": 128,
            "cal": (0, 0, 0, 0),
            "baseline_set": 0,
        }

        # config (mirror of firmware RAM, updated from CFG replies)
        self.x_src = tk.StringVar(value="heading")
        self.x_inv = tk.BooleanVar(value=False)
        self.x_max = tk.DoubleVar(value=25.0)
        self.x_dz = tk.DoubleVar(value=0.0)
        self.y_src = tk.StringVar(value="roll")
        self.y_inv = tk.BooleanVar(value=False)
        self.y_max = tk.DoubleVar(value=25.0)
        self.y_dz = tk.DoubleVar(value=0.0)
        self.baseline = (0.0, 0.0, 0.0)
        self.active_profile = tk.StringVar(value="default")
        self.bno_cal_saved = tk.BooleanVar(value=False)
        self.profiles_list = tk.StringVar(value="")     # for the combobox values

        # virtual gamepad state
        self.vp_enabled = tk.BooleanVar(value=False)
        self.vp_stick = tk.StringVar(value="left")
        self.vp_invert_y = tk.BooleanVar(value=True)   # match xpad/evdev convention
        self.vp_ui = None
        self.vp_status_var = tk.StringVar(
            value="evdev not installed" if not HAVE_EVDEV else "Disabled"
        )

        # 3D viz
        self.fig = None
        self.ax3d = None
        self._body_poly = None
        self._viz_dirty = False

        # snapshot tracking (for "unsaved changes" indicator)
        self._last_saved_snapshot = None

        self._build_ui()

        self.root.after(100, self._auto_connect)
        self.root.after(POLL_INTERVAL_MS, self._poll_serial)
        self.root.after(UI_REFRESH_MS, self._refresh_live_ui)
        if HAVE_MPL:
            self.root.after(VIZ_REFRESH_MS, self._refresh_3d)

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------------- UI layout ----------------
    def _build_ui(self):
        outer = ttk.Frame(self.root, padding=8)
        outer.grid(sticky="nsew")

        # connection bar
        conn = ttk.LabelFrame(outer, text="Connection", padding=6)
        conn.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 6))
        ttk.Label(conn, text="Port:").grid(row=0, column=0, sticky="w")
        ttk.Entry(conn, textvariable=self.port_var, width=20).grid(row=0, column=1, padx=4)
        ttk.Button(conn, text="Connect", command=self._connect).grid(row=0, column=2, padx=2)
        ttk.Button(conn, text="Disconnect", command=self._disconnect).grid(row=0, column=3, padx=2)
        ttk.Label(conn, textvariable=self.status_var, foreground="#555"
                  ).grid(row=1, column=0, columnspan=4, sticky="w", pady=(4, 0))

        # left column container
        left = ttk.Frame(outer)
        left.grid(row=1, column=0, sticky="nw", padx=(0, 8))

        # right column container
        right = ttk.Frame(outer)
        right.grid(row=1, column=1, sticky="nw")

        self._build_profile_panel(left)
        self._build_axis_panel(left, "X Axis (Ring1 / GPIO25)",
                                self.x_src, self.x_inv, self.x_max, self.x_dz, "X")
        self._build_axis_panel(left, "Y Axis (Tip / GPIO26)",
                                self.y_src, self.y_inv, self.y_max, self.y_dz, "Y")
        self._build_actions_panel(left)
        self._build_bno_panel(left)
        self._build_vpad_panel(left)

        self._build_live_panel(right)
        self._build_3d_panel(right)

        ttk.Label(outer, textvariable=self.dirty_var, foreground="#a60"
                  ).grid(row=2, column=0, columnspan=2, sticky="w", pady=(6, 0))

    # ---- profile panel ----
    def _build_profile_panel(self, parent):
        f = ttk.LabelFrame(parent, text="Profile", padding=6)
        f.grid(sticky="ew", pady=(0, 6))
        ttk.Label(f, text="Active:").grid(row=0, column=0, sticky="w")
        self.profile_cb = ttk.Combobox(f, textvariable=self.active_profile,
                                       state="readonly", width=14)
        self.profile_cb.grid(row=0, column=1, padx=4)
        self.profile_cb.bind("<<ComboboxSelected>>", self._on_profile_selected)
        ttk.Button(f, text="New...", command=self._cmd_new_profile
                   ).grid(row=1, column=0, sticky="ew", pady=(4, 0))
        ttk.Button(f, text="Delete", command=self._cmd_delete_profile
                   ).grid(row=1, column=1, sticky="ew", pady=(4, 0), padx=(4, 0))

    # ---- axis panel ----
    def _build_axis_panel(self, parent, title, src_var, inv_var, max_var, dz_var, axis_name):
        f = ttk.LabelFrame(parent, text=title, padding=6)
        f.grid(sticky="ew", pady=(0, 6))
        ttk.Label(f, text="Source:").grid(row=0, column=0, sticky="w")
        cb = ttk.Combobox(f, textvariable=src_var, values=SOURCES,
                          state="readonly", width=10)
        cb.grid(row=0, column=1, sticky="w", padx=4)
        cb.bind("<<ComboboxSelected>>", lambda ev, a=axis_name: self._on_setting_changed(a))
        ttk.Checkbutton(f, text="Invert", variable=inv_var,
                        command=lambda a=axis_name: self._on_setting_changed(a)
                        ).grid(row=0, column=2, sticky="w", padx=8)
        ttk.Label(f, text="Max angle (°):").grid(row=1, column=0, sticky="w", pady=(4, 0))
        s_max = ttk.Spinbox(f, from_=1.0, to=90.0, increment=1.0, width=8,
                            textvariable=max_var,
                            command=lambda a=axis_name: self._on_setting_changed(a))
        s_max.grid(row=1, column=1, sticky="w", padx=4, pady=(4, 0))
        s_max.bind("<FocusOut>", lambda ev, a=axis_name: self._on_setting_changed(a))
        s_max.bind("<Return>",   lambda ev, a=axis_name: self._on_setting_changed(a))
        ttk.Label(f, text="Deadzone (°):").grid(row=2, column=0, sticky="w", pady=(4, 0))
        s_dz = ttk.Spinbox(f, from_=0.0, to=30.0, increment=0.5, width=8,
                           textvariable=dz_var,
                           command=lambda a=axis_name: self._on_setting_changed(a))
        s_dz.grid(row=2, column=1, sticky="w", padx=4, pady=(4, 0))
        s_dz.bind("<FocusOut>", lambda ev, a=axis_name: self._on_setting_changed(a))
        s_dz.bind("<Return>",   lambda ev, a=axis_name: self._on_setting_changed(a))

    # ---- actions panel ----
    def _build_actions_panel(self, parent):
        f = ttk.LabelFrame(parent, text="Actions", padding=6)
        f.grid(sticky="ew", pady=(0, 6))
        ttk.Button(f, text="Set Neutral (capture pose)",
                   command=self._cmd_neutral).grid(row=0, column=0, sticky="ew", pady=2)
        ttk.Button(f, text="Save to ESP32 (persist)",
                   command=self._cmd_save).grid(row=1, column=0, sticky="ew", pady=2)
        ttk.Button(f, text="Reload from ESP32",
                   command=self._cmd_load).grid(row=2, column=0, sticky="ew", pady=2)
        ttk.Button(f, text="Reset to defaults",
                   command=self._cmd_reset).grid(row=3, column=0, sticky="ew", pady=2)

    # ---- BNO055 cal panel ----
    def _build_bno_panel(self, parent):
        f = ttk.LabelFrame(parent, text="BNO055 sensor calibration", padding=6)
        f.grid(sticky="ew", pady=(0, 6))
        self.bno_status_label = ttk.Label(f, text="Status: unknown")
        self.bno_status_label.grid(row=0, column=0, sticky="w")
        ttk.Button(f, text="Save chip cal to NVS",
                   command=self._cmd_save_bno_cal).grid(row=1, column=0, sticky="ew", pady=2)
        ttk.Button(f, text="Clear saved cal",
                   command=self._cmd_clear_bno_cal).grid(row=2, column=0, sticky="ew", pady=2)
        ttk.Label(f, text="(Save when cal reads 3,3,3,3 so the chip\n"
                          "skips re-cal on next boot.)",
                  foreground="#666").grid(row=3, column=0, sticky="w", pady=(4, 0))

    # ---- virtual gamepad panel ----
    def _build_vpad_panel(self, parent):
        f = ttk.LabelFrame(parent, text="Virtual gamepad (testing)", padding=6)
        f.grid(sticky="ew", pady=(0, 0))
        if not HAVE_EVDEV:
            ttk.Label(f, text="evdev not installed — virtual gamepad unavailable.\n"
                              "  pip install evdev",
                      foreground="#a44").grid(row=0, column=0, sticky="w")
            return
        ttk.Checkbutton(f, text="Enable virtual gamepad output",
                        variable=self.vp_enabled,
                        command=self._on_vp_enable_toggle
                        ).grid(row=0, column=0, columnspan=2, sticky="w")
        ttk.Label(f, text="Drive:").grid(row=1, column=0, sticky="w")
        ttk.Radiobutton(f, text="Left stick", variable=self.vp_stick, value="left",
                        command=self._on_vp_stick_change
                        ).grid(row=1, column=1, sticky="w")
        ttk.Radiobutton(f, text="Right stick", variable=self.vp_stick, value="right",
                        command=self._on_vp_stick_change
                        ).grid(row=2, column=1, sticky="w")
        ttk.Checkbutton(f, text="Invert Y for evdev/xpad convention",
                        variable=self.vp_invert_y
                        ).grid(row=3, column=0, columnspan=2, sticky="w")
        ttk.Label(f, textvariable=self.vp_status_var, foreground="#555"
                  ).grid(row=4, column=0, columnspan=2, sticky="w", pady=(4, 0))

    # ---- live panel ----
    def _build_live_panel(self, parent):
        f = ttk.LabelFrame(parent, text="Live IMU & DAC", padding=6)
        f.grid(sticky="nw", pady=(0, 6))
        self.lbl_h = ttk.Label(f, text="heading: ---")
        self.lbl_p = ttk.Label(f, text="pitch:   ---")
        self.lbl_r = ttk.Label(f, text="roll:    ---")
        self.lbl_dx = ttk.Label(f, text="DAC X:   ---")
        self.lbl_dy = ttk.Label(f, text="DAC Y:   ---")
        self.lbl_cal = ttk.Label(f, text="cal (sys,gyr,acc,mag): -")
        self.lbl_baseline = ttk.Label(f, text="baseline: not set")
        for i, w in enumerate([
            self.lbl_h, self.lbl_p, self.lbl_r,
            self.lbl_dx, self.lbl_dy,
            self.lbl_cal, self.lbl_baseline,
        ]):
            w.grid(row=i, column=0, sticky="w")
        self.stick_canvas = tk.Canvas(f, width=140, height=140,
                                      bg="#1a1a1a", highlightthickness=1,
                                      highlightbackground="#444")
        self.stick_canvas.grid(row=0, column=1, rowspan=7, padx=(16, 0))
        self._init_stick_canvas()

    # ---- 3D panel ----
    def _build_3d_panel(self, parent):
        f = ttk.LabelFrame(parent, text="3D pose (relative to baseline)", padding=6)
        f.grid(sticky="nw")
        if not HAVE_MPL:
            ttk.Label(f, text="matplotlib not installed — 3D viz unavailable.\n"
                              "  pip install matplotlib numpy",
                      foreground="#a44").grid(sticky="w")
            return
        self.fig = Figure(figsize=(3.6, 3.6), dpi=90)
        self.ax3d = self.fig.add_subplot(111, projection="3d")
        self._init_3d_axes()
        self.canvas3d = FigureCanvasTkAgg(self.fig, master=f)
        self.canvas3d.get_tk_widget().grid(row=0, column=0, sticky="nw")

    def _init_3d_axes(self):
        ax = self.ax3d
        ax.set_xlim(-1.0, 1.0); ax.set_ylim(-1.0, 1.0); ax.set_zlim(-1.0, 1.0)
        ax.set_box_aspect((1, 1, 1))
        ax.set_xticks([]); ax.set_yticks([]); ax.set_zticks([])
        ax.view_init(elev=22, azim=-60)
        ax.disable_mouse_rotation()
        ax.plot([-0.9, 0.9], [0, 0], [0, 0], color="#888", lw=0.5)
        ax.plot([0, 0], [-0.9, 0.9], [0, 0], color="#888", lw=0.5)
        ax.plot([0, 0], [0, 0], [-0.9, 0.9], color="#888", lw=0.5)
        ax.text(0.95, 0, 0, "X", color="#a44", fontsize=8)
        ax.text(0, 0.95, 0, "Y", color="#4a4", fontsize=8)
        ax.text(0, 0, 0.95, "Z", color="#44a", fontsize=8)

    def _init_stick_canvas(self):
        c = self.stick_canvas
        cx, cy, rad = 70, 70, 60
        c.create_oval(cx - rad, cy - rad, cx + rad, cy + rad, outline="#666")
        c.create_line(cx, cy - rad, cx, cy + rad, fill="#444")
        c.create_line(cx - rad, cy, cx + rad, cy, fill="#444")
        self._stick_dot = c.create_oval(cx - 5, cy - 5, cx + 5, cy + 5,
                                        fill="#e44", outline="")

    # ---------------- serial ----------------
    def _auto_connect(self):
        try:
            self._connect()
        except Exception:
            pass

    def _connect(self):
        if self.ser is not None:
            return
        port = self.port_var.get().strip()
        try:
            self.ser = serial.Serial(port, BAUD, timeout=0)
        except serial.SerialException as ex:
            self.status_var.set(f"Connect failed: {ex}")
            self.ser = None
            return
        self.status_var.set(f"Connected to {port}. Requesting status...")
        self._send_line("STATUS")
        self._send_line("LIST_PROFILES")

    def _disconnect(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.status_var.set("Disconnected.")

    def _send_line(self, line):
        if self.ser is None:
            self.status_var.set("Not connected — command ignored.")
            return
        try:
            self.ser.write((line + "\n").encode("ascii"))
        except Exception as ex:
            self.status_var.set(f"Serial write failed: {ex}")

    def _poll_serial(self):
        if self.ser is not None:
            try:
                if self.ser.in_waiting:
                    chunk = self.ser.read(self.ser.in_waiting).decode(
                        "ascii", errors="ignore"
                    )
                    self._line_buf += chunk
                    while "\n" in self._line_buf:
                        line, self._line_buf = self._line_buf.split("\n", 1)
                        self._handle_line(line.strip())
            except Exception as ex:
                self.status_var.set(f"Serial read failed: {ex}")
                self._disconnect()
        self.root.after(POLL_INTERVAL_MS, self._poll_serial)

    def _handle_line(self, line):
        if not line:
            return
        m = DATA_RE.match(line)
        if m:
            self.live["h"] = float(m.group(1))
            self.live["p"] = float(m.group(2))
            self.live["r"] = float(m.group(3))
            self.live["dx"] = int(m.group(4))
            self.live["dy"] = int(m.group(5))
            self.live["cal"] = (int(m.group(6)), int(m.group(7)),
                                int(m.group(8)), int(m.group(9)))
            self.live["baseline_set"] = int(m.group(10))
            self._viz_dirty = True
            self._maybe_drive_vpad()
            return
        m = CFG_RE.match(line)
        if m:
            self._apply_cfg(m)
            return
        m = PROFILES_RE.match(line)
        if m:
            raw = m.group(1).strip()
            names = [n for n in raw.split(",") if n] if raw else []
            self.profile_cb["values"] = names
            return
        if line.startswith("OK ") or line.startswith("READY") or line.startswith("ERR"):
            self.status_var.set(line)
            if line == "OK SAVED" or line.startswith("OK SAVED profile=") \
                    or line.startswith("OK SAVED_PROFILE "):
                self._last_saved_snapshot = self._snapshot()
                self._update_dirty_indicator()
                self._send_line("LIST_PROFILES")
                self._send_line("STATUS")
            elif (line.startswith("OK LOADED") or line.startswith("OK NEUTRAL")
                  or line.startswith("OK RESET") or line.startswith("OK DEFAULTS")
                  or line.startswith("OK SELECTED ")
                  or line.startswith("OK SAVED_BNO_CAL")
                  or line.startswith("OK CLEARED_BNO_CAL")):
                self._send_line("STATUS")
            elif line.startswith("OK DELETED "):
                self._send_line("LIST_PROFILES")
            return
        # boot banner, unknown lines
        self.status_var.set(line)

    def _apply_cfg(self, m):
        self._suppress_set = True
        try:
            self.active_profile.set(m.group(1))
            self.x_src.set(m.group(2))
            self.x_inv.set(m.group(3) == "1")
            self.x_max.set(float(m.group(4)))
            self.x_dz.set(float(m.group(5)))
            self.y_src.set(m.group(6))
            self.y_inv.set(m.group(7) == "1")
            self.y_max.set(float(m.group(8)))
            self.y_dz.set(float(m.group(9)))
            self.baseline = (float(m.group(10)), float(m.group(11)), float(m.group(12)))
            self.bno_cal_saved.set(m.group(14) == "1")
        finally:
            self._suppress_set = False
        self._last_saved_snapshot = self._snapshot()
        self._update_dirty_indicator()
        self.bno_status_label.config(
            text=f"Status: {'saved' if self.bno_cal_saved.get() else 'not saved'}"
        )

    # ---------------- settings change ----------------
    def _snapshot(self):
        return (
            self.x_src.get(), self.x_inv.get(),
            round(self.x_max.get(), 2), round(self.x_dz.get(), 2),
            self.y_src.get(), self.y_inv.get(),
            round(self.y_max.get(), 2), round(self.y_dz.get(), 2),
        )

    def _update_dirty_indicator(self):
        if self._last_saved_snapshot is None:
            self.dirty_var.set("")
            return
        if self._snapshot() != self._last_saved_snapshot:
            self.dirty_var.set("Unsaved changes — 'Save to ESP32' to persist.")
        else:
            self.dirty_var.set(f"In sync with profile '{self.active_profile.get()}'.")

    def _on_setting_changed(self, axis):
        if self._suppress_set:
            return
        if axis == "X":
            parts = [
                f"X.src={self.x_src.get()}",
                f"X.inv={1 if self.x_inv.get() else 0}",
                f"X.max={float(self.x_max.get()):.2f}",
                f"X.dz={float(self.x_dz.get()):.2f}",
            ]
        else:
            parts = [
                f"Y.src={self.y_src.get()}",
                f"Y.inv={1 if self.y_inv.get() else 0}",
                f"Y.max={float(self.y_max.get()):.2f}",
                f"Y.dz={float(self.y_dz.get()):.2f}",
            ]
        self._send_line("SET " + " ".join(parts))
        self._update_dirty_indicator()

    # ---------------- action buttons ----------------
    def _cmd_neutral(self):
        self._send_line("NEUTRAL")

    def _cmd_save(self):
        self._send_line("SAVE")

    def _cmd_load(self):
        self._send_line("LOAD")

    def _cmd_reset(self):
        if not messagebox.askyesno(
            "Reset to defaults",
            "Reset axis mapping to defaults?\n"
            "(RAM only — Save to ESP32 to make it permanent.)",
        ):
            return
        self._send_line("RESET")

    # ---------------- profile actions ----------------
    def _on_profile_selected(self, _ev=None):
        name = self.active_profile.get()
        self._send_line(f"SELECT_PROFILE {name}")

    def _cmd_new_profile(self):
        name = simpledialog.askstring(
            "New profile",
            "Profile name (1–12 chars, letters/digits/_-):",
            parent=self.root,
        )
        if not name:
            return
        name = name.strip()
        if not self._valid_profile_name(name):
            messagebox.showerror("Invalid name",
                                 "Use 1–12 chars: letters, digits, underscore, hyphen.")
            return
        self._send_line(f"SAVE_PROFILE {name}")

    def _cmd_delete_profile(self):
        name = self.active_profile.get()
        if not name:
            return
        if not messagebox.askyesno(
            "Delete profile",
            f"Delete profile '{name}'?\n"
            "(Cannot delete the active profile — select a different one first.)",
        ):
            return
        self._send_line(f"DELETE_PROFILE {name}")

    @staticmethod
    def _valid_profile_name(name):
        if not (1 <= len(name) <= 12):
            return False
        return all(c.isalnum() or c in "_-" for c in name)

    # ---------------- BNO cal actions ----------------
    def _cmd_save_bno_cal(self):
        if self.live["cal"][0] < 3:
            if not messagebox.askyesno(
                "Save BNO cal",
                "System calibration (first number) is not yet 3.\n"
                "Save anyway? (Better to wait until all 4 are 3.)",
            ):
                return
        self._send_line("SAVE_BNO_CAL")

    def _cmd_clear_bno_cal(self):
        if not messagebox.askyesno(
            "Clear saved BNO cal",
            "Remove the saved BNO055 calibration from ESP32 NVS?",
        ):
            return
        self._send_line("CLEAR_BNO_CAL")

    # ---------------- virtual gamepad ----------------
    def _on_vp_enable_toggle(self):
        if self.vp_enabled.get():
            try:
                self.vp_ui = make_xbox360_uinput()
                self.vp_status_var.set(f"Active: {self.vp_ui.device.path}")
            except PermissionError:
                self.vp_enabled.set(False)
                self.vp_status_var.set("Permission denied on /dev/uinput "
                                       "(see app docstring).")
            except Exception as ex:
                self.vp_enabled.set(False)
                self.vp_status_var.set(f"Init failed: {ex}")
        else:
            self._close_vpad()
            self.vp_status_var.set("Disabled")

    def _on_vp_stick_change(self):
        if self.vp_ui is None:
            return
        # zero both sticks to avoid stuck axes
        for axis in (e.ABS_X, e.ABS_Y, e.ABS_RX, e.ABS_RY):
            self.vp_ui.write(e.EV_ABS, axis, 0)
        self.vp_ui.syn()

    def _maybe_drive_vpad(self):
        if not self.vp_enabled.get() or self.vp_ui is None:
            return
        ax_x = dac_to_axis(self.live["dx"])
        ax_y = dac_to_axis(self.live["dy"])
        if self.vp_invert_y.get():
            ax_y = -ax_y
        try:
            if self.vp_stick.get() == "left":
                self.vp_ui.write(e.EV_ABS, e.ABS_X, ax_x)
                self.vp_ui.write(e.EV_ABS, e.ABS_Y, ax_y)
            else:
                self.vp_ui.write(e.EV_ABS, e.ABS_RX, ax_x)
                self.vp_ui.write(e.EV_ABS, e.ABS_RY, ax_y)
            self.vp_ui.syn()
        except Exception as ex:
            self.vp_status_var.set(f"Write failed: {ex}")

    def _close_vpad(self):
        if self.vp_ui is None:
            return
        try:
            for axis in (e.ABS_X, e.ABS_Y, e.ABS_RX, e.ABS_RY):
                self.vp_ui.write(e.EV_ABS, axis, 0)
            self.vp_ui.syn()
            self.vp_ui.close()
        except Exception:
            pass
        self.vp_ui = None

    # ---------------- UI refresh ----------------
    def _refresh_live_ui(self):
        self.lbl_h.config(text=f"heading: {self.live['h']:7.2f}°")
        self.lbl_p.config(text=f"pitch:   {self.live['p']:7.2f}°")
        self.lbl_r.config(text=f"roll:    {self.live['r']:7.2f}°")
        self.lbl_dx.config(text=f"DAC X:   {self.live['dx']:3d}")
        self.lbl_dy.config(text=f"DAC Y:   {self.live['dy']:3d}")
        c = self.live["cal"]
        self.lbl_cal.config(text=f"cal (sys,gyr,acc,mag): {c[0]},{c[1]},{c[2]},{c[3]}")
        if self.live["baseline_set"]:
            bh, bp, br = self.baseline
            self.lbl_baseline.config(text=f"baseline: h={bh:.2f} p={bp:.2f} r={br:.2f}")
        else:
            self.lbl_baseline.config(text="baseline: not set — click 'Set Neutral'")

        cx, cy, rad = 70, 70, 60
        nx = (self.live["dx"] - 127.5) / 127.5
        ny = (self.live["dy"] - 127.5) / 127.5
        px = cx + nx * rad
        py = cy - ny * rad
        self.stick_canvas.coords(self._stick_dot, px - 5, py - 5, px + 5, py + 5)

        self.root.after(UI_REFRESH_MS, self._refresh_live_ui)

    def _refresh_3d(self):
        if self._viz_dirty and self.ax3d is not None:
            self._viz_dirty = False
            # relative to baseline (yaw / pitch / roll deltas)
            if self.live["baseline_set"]:
                bh, bp, br = self.baseline
                dyaw   = self._wrap_delta(self.live["h"] - bh)
                dpitch = self.live["p"] - bp
                droll  = self.live["r"] - br
            else:
                dyaw, dpitch, droll = self.live["h"], self.live["p"], self.live["r"]
            R = euler_to_rotmat(dyaw, dpitch, droll)
            if R is not None:
                if self._body_poly is not None:
                    self._body_poly.remove()
                rotated = (R @ self._BOX_CORNERS.T).T
                polys = [rotated[idx] for idx, _ in self._BOX_FACES]
                colors = [c for _, c in self._BOX_FACES]
                self._body_poly = Poly3DCollection(polys, facecolors=colors,
                                                   edgecolors="k", linewidths=0.5)
                self.ax3d.add_collection3d(self._body_poly)
                self.canvas3d.draw_idle()
        self.root.after(VIZ_REFRESH_MS, self._refresh_3d)

    @staticmethod
    def _wrap_delta(d):
        if d > 180.0:  return d - 360.0
        if d < -180.0: return d + 360.0
        return d

    # ---------------- shutdown ----------------
    def _on_close(self):
        self._close_vpad()
        self._disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    IMUStickApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()