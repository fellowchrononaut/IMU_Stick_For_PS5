"""
IMU Stick V1 — desktop configurator.

Connects to the V1 firmware over serial, lets the user tweak the axis mapping,
captures a neutral pose, and persists everything to ESP32 NVS so the device runs
standalone once unplugged.

Protocol mirrors the firmware. Live changes are sent immediately via SET;
"Save to ESP32" flushes the current RAM config to NVS.
"""

import re
import serial
import threading
import tkinter as tk
from tkinter import ttk, messagebox

DEFAULT_PORT = "/dev/ttyUSB0"
BAUD = 115200
SOURCES = ("heading", "pitch", "roll")
POLL_INTERVAL_MS = 30
UI_REFRESH_MS = 80

DATA_RE = re.compile(
    r"^D\s+h=([-0-9.]+)\s+p=([-0-9.]+)\s+r=([-0-9.]+)"
    r"\s+dx=(\d+)\s+dy=(\d+)\s+cal=(\d),(\d),(\d),(\d)"
    r"\s+baseline_set=(\d)"
)
CFG_RE = re.compile(
    r"^CFG\s+X\.src=(\w+)\s+X\.inv=(\d+)\s+X\.max=([-0-9.]+)\s+X\.dz=([-0-9.]+)"
    r"\s+Y\.src=(\w+)\s+Y\.inv=(\d+)\s+Y\.max=([-0-9.]+)\s+Y\.dz=([-0-9.]+)"
    r"\s+baseline=([-0-9.]+),([-0-9.]+),([-0-9.]+)\s+baseline_set=(\d+)"
)


class IMUStickApp:
    def __init__(self, root):
        self.root = root
        root.title("IMU Stick V1 — Configurator")

        self.ser = None
        self.port_var = tk.StringVar(value=DEFAULT_PORT)
        self.status_var = tk.StringVar(value="Not connected.")
        self.dirty_var = tk.StringVar(value="")  # "(unsaved changes)" when applicable

        self.live = {
            "h": 0.0, "p": 0.0, "r": 0.0,
            "dx": 128, "dy": 128,
            "cal": (0, 0, 0, 0),
            "baseline_set": 0,
        }

        # Settings state (mirrors firmware RAM config). Initialized once we hear
        # back from the device via CFG.
        self.x_src = tk.StringVar(value="heading")
        self.x_inv = tk.BooleanVar(value=False)
        self.x_max = tk.DoubleVar(value=25.0)
        self.x_dz = tk.DoubleVar(value=0.0)
        self.y_src = tk.StringVar(value="roll")
        self.y_inv = tk.BooleanVar(value=False)
        self.y_max = tk.DoubleVar(value=25.0)
        self.y_dz = tk.DoubleVar(value=0.0)

        self.baseline = (0.0, 0.0, 0.0)

        # Tracks what was last persisted, so we can show "unsaved changes".
        self._last_saved_snapshot = None

        # Avoid SET being sent when we ourselves update the vars from a CFG reply.
        self._suppress_set = False

        self._line_buf = ""
        self._build_ui()

        # Try to auto-connect on launch — non-fatal if it fails.
        self.root.after(100, self._auto_connect)
        self.root.after(POLL_INTERVAL_MS, self._poll_serial)
        self.root.after(UI_REFRESH_MS, self._refresh_live_ui)

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------------- UI ----------------
    def _build_ui(self):
        outer = ttk.Frame(self.root, padding=10)
        outer.grid(sticky="nsew")

        # Top: connection
        top = ttk.LabelFrame(outer, text="Connection", padding=8)
        top.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 8))
        ttk.Label(top, text="Port:").grid(row=0, column=0, sticky="w")
        ttk.Entry(top, textvariable=self.port_var, width=22).grid(row=0, column=1, padx=4)
        ttk.Button(top, text="Connect", command=self._connect).grid(row=0, column=2, padx=2)
        ttk.Button(top, text="Disconnect", command=self._disconnect).grid(row=0, column=3, padx=2)
        ttk.Label(top, textvariable=self.status_var, foreground="#555").grid(
            row=1, column=0, columnspan=4, sticky="w", pady=(4, 0)
        )

        # Live readout
        live = ttk.LabelFrame(outer, text="Live IMU & DAC", padding=8)
        live.grid(row=1, column=0, sticky="nw", padx=(0, 8))
        self.lbl_h = ttk.Label(live, text="heading: ---")
        self.lbl_p = ttk.Label(live, text="pitch:   ---")
        self.lbl_r = ttk.Label(live, text="roll:    ---")
        self.lbl_dx = ttk.Label(live, text="DAC X:   ---")
        self.lbl_dy = ttk.Label(live, text="DAC Y:   ---")
        self.lbl_cal = ttk.Label(live, text="cal (sys,gyr,acc,mag): -")
        self.lbl_baseline = ttk.Label(live, text="baseline: not set")
        for i, w in enumerate([
            self.lbl_h, self.lbl_p, self.lbl_r,
            self.lbl_dx, self.lbl_dy,
            self.lbl_cal, self.lbl_baseline,
        ]):
            w.grid(row=i, column=0, sticky="w")

        # 2D stick indicator
        self.stick_canvas = tk.Canvas(live, width=140, height=140,
                                      bg="#1a1a1a", highlightthickness=1,
                                      highlightbackground="#444")
        self.stick_canvas.grid(row=0, column=1, rowspan=7, padx=(20, 0))
        self._init_stick_canvas()

        # Settings — X
        sx = ttk.LabelFrame(outer, text="X Axis (Ring1 / GPIO25)", padding=8)
        sx.grid(row=1, column=1, sticky="nw", pady=(0, 8))
        self._build_axis_panel(sx, self.x_src, self.x_inv, self.x_max, self.x_dz, "X")

        # Settings — Y
        sy = ttk.LabelFrame(outer, text="Y Axis (Tip / GPIO26)", padding=8)
        sy.grid(row=2, column=1, sticky="nw")
        self._build_axis_panel(sy, self.y_src, self.y_inv, self.y_max, self.y_dz, "Y")

        # Actions
        act = ttk.LabelFrame(outer, text="Actions", padding=8)
        act.grid(row=2, column=0, sticky="nsew")
        ttk.Button(act, text="Set Neutral (capture pose)",
                   command=self._cmd_neutral).grid(row=0, column=0, sticky="ew", pady=2)
        ttk.Button(act, text="Save to ESP32 (persist)",
                   command=self._cmd_save).grid(row=1, column=0, sticky="ew", pady=2)
        ttk.Button(act, text="Reload from ESP32",
                   command=self._cmd_load).grid(row=2, column=0, sticky="ew", pady=2)
        ttk.Button(act, text="Reset to defaults",
                   command=self._cmd_reset).grid(row=3, column=0, sticky="ew", pady=2)
        ttk.Label(act, textvariable=self.dirty_var, foreground="#a60").grid(
            row=4, column=0, sticky="w", pady=(8, 0)
        )

    def _build_axis_panel(self, parent, src_var, inv_var, max_var, dz_var, axis_name):
        ttk.Label(parent, text="Source:").grid(row=0, column=0, sticky="w")
        cb = ttk.Combobox(parent, textvariable=src_var, values=SOURCES,
                          state="readonly", width=10)
        cb.grid(row=0, column=1, sticky="w", padx=4)
        cb.bind("<<ComboboxSelected>>",
                lambda e, a=axis_name: self._on_setting_changed(a))

        ttk.Checkbutton(parent, text="Invert", variable=inv_var,
                        command=lambda a=axis_name: self._on_setting_changed(a)
                        ).grid(row=0, column=2, sticky="w", padx=8)

        ttk.Label(parent, text="Max angle (°):").grid(row=1, column=0, sticky="w", pady=(6, 0))
        s_max = ttk.Spinbox(parent, from_=1.0, to=90.0, increment=1.0, width=8,
                            textvariable=max_var,
                            command=lambda a=axis_name: self._on_setting_changed(a))
        s_max.grid(row=1, column=1, sticky="w", padx=4, pady=(6, 0))
        s_max.bind("<FocusOut>", lambda e, a=axis_name: self._on_setting_changed(a))
        s_max.bind("<Return>",   lambda e, a=axis_name: self._on_setting_changed(a))

        ttk.Label(parent, text="Deadzone (°):").grid(row=2, column=0, sticky="w", pady=(6, 0))
        s_dz = ttk.Spinbox(parent, from_=0.0, to=30.0, increment=0.5, width=8,
                           textvariable=dz_var,
                           command=lambda a=axis_name: self._on_setting_changed(a))
        s_dz.grid(row=2, column=1, sticky="w", padx=4, pady=(6, 0))
        s_dz.bind("<FocusOut>", lambda e, a=axis_name: self._on_setting_changed(a))
        s_dz.bind("<Return>",   lambda e, a=axis_name: self._on_setting_changed(a))

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
            pass  # user can retry manually

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
            return
        m = CFG_RE.match(line)
        if m:
            self._suppress_set = True
            try:
                self.x_src.set(m.group(1))
                self.x_inv.set(m.group(2) == "1")
                self.x_max.set(float(m.group(3)))
                self.x_dz.set(float(m.group(4)))
                self.y_src.set(m.group(5))
                self.y_inv.set(m.group(6) == "1")
                self.y_max.set(float(m.group(7)))
                self.y_dz.set(float(m.group(8)))
                self.baseline = (float(m.group(9)),
                                 float(m.group(10)),
                                 float(m.group(11)))
            finally:
                self._suppress_set = False
            self._last_saved_snapshot = self._snapshot()
            self._update_dirty_indicator()
            self.status_var.set("Config synced with ESP32.")
            return
        if line.startswith("OK ") or line.startswith("READY") or line.startswith("ERR"):
            self.status_var.set(line)
            # After OK SAVED, current state == saved state.
            if line == "OK SAVED":
                self._last_saved_snapshot = self._snapshot()
                self._update_dirty_indicator()
            # After OK LOADED / OK NEUTRAL / OK RESET / OK DEFAULTS, ask for fresh STATUS.
            if line.startswith("OK LOADED") or line.startswith("OK NEUTRAL") \
                    or line.startswith("OK RESET") or line.startswith("OK DEFAULTS"):
                self._send_line("STATUS")
            return
        # Anything else (boot banners, debug prints) — show as status briefly.
        self.status_var.set(line)

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
            self.dirty_var.set("Unsaved changes — click 'Save to ESP32' to persist.")
        else:
            self.dirty_var.set("In sync with ESP32 NVS.")

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
            "(This only affects RAM. Save to ESP32 to make it permanent.)",
        ):
            return
        self._send_line("RESET")

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
            self.lbl_baseline.config(
                text=f"baseline: h={bh:.2f} p={bp:.2f} r={br:.2f}"
            )
        else:
            self.lbl_baseline.config(text="baseline: not set — click 'Set Neutral'")

        # Stick dot position from DAC values (0..255 → -1..+1 across the canvas)
        cx, cy, rad = 70, 70, 60
        nx = (self.live["dx"] - 127.5) / 127.5
        ny = (self.live["dy"] - 127.5) / 127.5
        px = cx + nx * rad
        py = cy - ny * rad  # screen Y inverted
        self.stick_canvas.coords(self._stick_dot, px - 5, py - 5, px + 5, py + 5)

        self.root.after(UI_REFRESH_MS, self._refresh_live_ui)

    # ---------------- shutdown ----------------
    def _on_close(self):
        self._disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    IMUStickApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()