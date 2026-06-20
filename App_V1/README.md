# IMU Stick V1 — App + Firmware

Desktop configurator and ESP32 firmware for the foot-mounted IMU → PS5 Access
Controller stick adapter.

- `firmware/imu_stick_v1/` — Arduino sketch for the ESP32 (BNO055 + DAC out)
- `app/imu_stick_app.py` — Tkinter desktop configurator
- `requirements.txt` — Python dependencies

---

## Quick start (desktop side)

Tested on Ubuntu / Python 3.10+. Should work on any Python 3.8+ system with
Tkinter available.

```bash
cd App_V1
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python app/imu_stick_app.py
```

On Windows or macOS, the virtual gamepad panel will be unavailable (evdev is
Linux-only) but everything else works.

---

## Linux system permissions

Two permissions to set up once. Both persist across reboots.

### Serial port access (so the app can read `/dev/ttyUSB0` without sudo)

```bash
sudo usermod -aG dialout $USER
# log out and back in for group membership to take effect
```

### Virtual gamepad access to `/dev/uinput` (only if you want that feature)

```bash
sudo groupadd -f uinput
sudo usermod -aG uinput,input $USER
echo 'KERNEL=="uinput", MODE="0660", GROUP="uinput", OPTIONS+="static_node=uinput"' \
    | sudo tee /etc/udev/rules.d/99-uinput.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
# log out and back in
```

If you skip this step, the "Enable virtual gamepad output" checkbox will fail
with a permission error in the status line — harmless, just disabled.

---

## Firmware setup

Arduino IDE 2.x. Once-per-machine setup:

1. **ESP32 board package**:
   - `File → Preferences → Additional boards manager URLs`:
     `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
   - `Tools → Board → Boards Manager…` → install **esp32 by Espressif Systems**.

2. **BNO055 library** (the Bosch one, not Adafruit's):
   - The project lives at the same path used by the V0 sketch:
     `~/Arduino/libraries/BNO055/`. If you don't have it, install Bosch's BNO055
     example sources there. Either copy from a working machine or use Renzo
     Mischianti's port which the V0 sketch is based on.

3. **Verify the I2C address** in `~/Arduino/libraries/BNO055/BNO055.h` is set to
   `BNO055_I2C_ADDR1` (0x28). This is the default; only change if your board is
   jumpered for 0x29.

4. Open `firmware/imu_stick_v1/imu_stick_v1.ino`, select the board (e.g.
   "ESP32 Dev Module"), select the port (`/dev/ttyUSB0` on Linux), and upload.

The very first line on the serial output after upload should be
`READY imu_stick_v1` — that confirms V1 firmware is running.

---

## Workflow

1. Plug ESP32 into the PC via USB.
2. Launch `python app/imu_stick_app.py`. It auto-connects to `/dev/ttyUSB0`.
3. (One-time, per chip) Wave the IMU until the calibration line reads
   `3, 3, 3, 3`, then click **Save chip cal to NVS** under "BNO055 sensor
   calibration". Future boots will skip the cal dance.
4. Hold the foot in the desired neutral pose and click **Set Neutral
   (capture pose)**.
5. Use the X/Y axis dropdowns, "Invert" checkboxes, and Max-angle / Deadzone
   spinboxes to dial in the mapping. Changes apply live so you can watch the
   DAC react in the readout / on the 2D stick / on the 3D pose viz.
6. Click **Save to ESP32 (persist)** to write the current settings to the
   active profile in NVS.
7. Unplug USB, plug into battery + PS5 controller. ESP32 boots and runs with
   the saved profile.

### Profiles

Use the dropdown to switch between saved profiles. "New…" prompts for a name
and saves current settings under it. "Delete" removes the selected profile
(except the active one — switch first, then delete).

Profile names: 1–12 chars, letters / digits / `_` / `-`.

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| App opens but nothing populates | Port wrong, or firmware not the V1 build. The boot banner `READY imu_stick_v1` confirms V1 is running. |
| Live values stuck at all zeros | BNO055 not responding on I2C. Re-run `i2c_scanner` from the V0 folder to check. |
| `cal=0,...` and not climbing | BNO055 needs motion through several orientations. Gyro / accel / mag each calibrate via different motions; see Bosch datasheet section on calibration. |
| `DAC X/Y` won't budge | Baseline not yet captured — click **Set Neutral**. |
| Virtual gamepad checkbox shows "Permission denied" | uinput udev rule not yet applied; see Linux permissions section above. |
| 3D viz panel says "matplotlib not installed" | `pip install matplotlib numpy` in your venv. |