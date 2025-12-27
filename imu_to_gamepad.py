import re
import time
import serial
import vgamepad as vg
import math

# ---------- CONFIG ----------
COM_PORT = "COM10"        # <-- change this to your ESP32 COM port
BAUD_RATE = 115200
MAX_ANGLE = 45.0         # degrees for full stick deflection
SAMPLE_TIMEOUT = 1.0     # seconds
# ----------------------------

# Regex to parse "Orientation: h, p, r"
ORIENT_RE = re.compile(
    r"Orientation:\s*([-0-9.]+)\s*,\s*([-0-9.]+)\s*,\s*([-0-9.]+)"
)

def map_angle_to_axis(angle, center, max_angle=MAX_ANGLE):
    """
    Map angle difference (angle - center) to joystick axis (-32767..32767).
    max_angle degrees => full deflection.
    """
    delta = angle - center
    # Clamp to [-1, 1]
    x = max(-1.0, min(1.0, delta / max_angle))
    return int(x * 32767)

def main():
    print(f"Opening serial port {COM_PORT} at {BAUD_RATE}...")
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=SAMPLE_TIMEOUT)

    gamepad = vg.VX360Gamepad()
    print("Virtual Xbox 360 controller created.")

    baseline_heading = None
    baseline_pitch = None

    print("\nWaiting for first orientation sample to set neutral foot position...")
    print("Stand in your neutral stance and keep the foot still.")

    try:
        while True:
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                continue

            # Debug: show raw lines occasionally
            # print("DEBUG:", line)

            m = ORIENT_RE.match(line)
            if not m:
                continue

            heading_deg = float(m.group(1))
            pitch_deg   = float(m.group(3))
            roll_deg    = float(m.group(2))

            # Set baseline on first valid sample
            if baseline_heading is None:
                baseline_heading = heading_deg
                baseline_pitch   = pitch_deg
                print(f"Baseline set: heading={baseline_heading:.2f}, pitch={baseline_pitch:.2f}")
                print("Move your foot to test stick movement.")
                continue

            # Map:
            #   heading -> right stick X
            #   pitch   -> right stick Y
            rx = -(map_angle_to_axis(heading_deg, baseline_heading))
            ry = map_angle_to_axis(pitch_deg, baseline_pitch)
            
            print(f"Mapped angles to joystick: RX={rx}, RY={ry}")

            # Note: vgamepad expects -32768..32767
            gamepad.right_joystick(x_value=rx, y_value=ry)
            gamepad.update()

            # Optional: small delay to avoid spamming
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
