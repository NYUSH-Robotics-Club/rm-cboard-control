#!/usr/bin/env python3
"""Historical chassis-current plotter; use script/logger.py for current logs."""

import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

DEFAULT_PORT = "/dev/tty.usbmodem3088357D30341"
BAUD = 115200
BUFFER = 600

# one buffer per motor index 0..3
t = [deque(maxlen=BUFFER) for _ in range(4)]
out = [deque(maxlen=BUFFER) for _ in range(4)]
tgt = [deque(maxlen=BUFFER) for _ in range(4)]
fb  = [deque(maxlen=BUFFER) for _ in range(4)]
motor_id = ["?"] * 4
enabled = 0

def parse_motor_line(line: str):
    # expects: i=0 id=1 tgt=123.4 fb=56.7 dt=12 out=8000
    line = line.strip()
    if not line.startswith("i="):
        return None
    parts = line.replace("=", " ").split()
    if len(parts) < 12:
        return None
    try:
        idx = int(parts[1])
        mid = int(parts[3])
        _tgt = float(parts[5])
        _fb  = float(parts[7])
        _out = int(parts[11])
        return idx, mid, _tgt, _fb, _out
    except:
        return None

ser = serial.Serial(PORT, BAUD, timeout=0.1)
print(f"[OK] Opened {PORT} @ {BAUD}")

start = None
raw_print_count = 0

fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
lines_out = []
lines_tgt = []
lines_fb = []

for i in range(4):
    axes[i].grid(True)
    axes[i].set_ylabel(f"i={i}")
    lo, = axes[i].plot([], [], label="out")
    lt, = axes[i].plot([], [], linestyle="--", label="tgt")
    lf, = axes[i].plot([], [], label="fb")
    axes[i].legend(loc="upper right")
    lines_out.append(lo)
    lines_tgt.append(lt)
    lines_fb.append(lf)

axes[-1].set_xlabel("time (s)")

def update(_frame):
    global start, enabled, raw_print_count

    # read all pending lines
    while ser.in_waiting:
        line = ser.readline().decode(errors="ignore").strip()

        # show some raw lines in terminal so you KNOW it’s receiving
        if raw_print_count < 20:
            print("[RAW]", line)
            raw_print_count += 1

        if line.startswith("[CHASSIS_CUR]") and "en=" in line:
            try:
                enabled = int(line.split("en=")[-1])
            except:
                pass
            continue

        parsed = parse_motor_line(line)
        if not parsed:
            continue

        idx, mid, _tgt, _fb, _out = parsed
        if idx < 0 or idx >= 4:
            continue

        if start is None:
            import time
            start = time.time()
        import time
        tt = time.time() - start

        motor_id[idx] = str(mid)
        t[idx].append(tt)
        tgt[idx].append(_tgt)
        fb[idx].append(_fb)
        out[idx].append(_out)

    # update plots
    for i in range(4):
        if not t[i]:
            continue
        lines_out[i].set_data(t[i], out[i])
        lines_tgt[i].set_data(t[i], tgt[i])
        lines_fb[i].set_data(t[i], fb[i])

        axes[i].relim()
        axes[i].autoscale_view()

        axes[i].set_ylabel(f"i={i} id={motor_id[i]}")

    # x window last 30s
    all_last = [t[i][-1] for i in range(4) if t[i]]
    if all_last:
        xmax = max(all_last)
        xmin = max(0.0, xmax - 30.0)
        axes[-1].set_xlim(xmin, xmax)

    fig.suptitle(f"CHASSIS_CUR enabled={enabled}  (watching i=0..3)")
    return lines_out + lines_tgt + lines_fb

ani = FuncAnimation(fig, update, interval=100, blit=False)

try:
    plt.tight_layout()
    plt.show()
finally:
    ser.close()
    print("Serial closed")
