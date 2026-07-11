#!/usr/bin/env bash
# 嗅探 UWB 主站串口；USB 断线自动重开，不因 SerialException 崩掉
set -euo pipefail
PORT="${1:-/dev/ttyUSB1}"
LOG="${2:-/tmp/uwb_sniff.log}"
DUR="${3:-300}"

exec > >(tee "$LOG") 2>&1
echo "[$(date '+%H:%M:%S')] sniff $PORT for ${DUR}s → $LOG (auto-reopen)"

python3 -u - "$PORT" "$DUR" <<'PY'
import sys, time, serial

port, dur = sys.argv[1], float(sys.argv[2])
deadline = time.time() + dur
buf = ""
n_ok = 0
ser = None

def open_port():
    global ser, buf
    if ser is not None:
        try:
            ser.close()
        except Exception:
            pass
        ser = None
    buf = ""
    while time.time() < deadline:
        try:
            ser = serial.Serial(port, 115200, timeout=0.2)
            print(f"[{time.strftime('%H:%M:%S')}] open OK {port}", flush=True)
            return True
        except Exception as e:
            print(f"[{time.strftime('%H:%M:%S')}] OPEN_WAIT: {e}", flush=True)
            time.sleep(1.0)
    return False

if not open_port():
    print("never opened", flush=True)
    sys.exit(1)

while time.time() < deadline:
    try:
        chunk = ser.read(512).decode("utf-8", errors="ignore")
    except serial.SerialException as e:
        print(f"[{time.strftime('%H:%M:%S')}] DISCONNECT: {e}", flush=True)
        time.sleep(0.5)
        if not open_port():
            break
        continue
    if not chunk:
        continue
    buf += chunk
    while True:
        i = buf.find("###1.9")
        if i < 0:
            break
        j = buf.find("\n", i)
        if j < 0:
            break
        line = buf[i:j].strip()
        buf = buf[j + 1 :]
        n_ok += 1
        parts = line.split(",")
        dist = parts[5] if len(parts) > 5 else "?"
        print(f"[OK #{n_ok}] dist={dist} | {line[:120]}", flush=True)
    if len(buf) > 4000:
        buf = buf[-2000:]

print(f"done, ok_frames={n_ok}", flush=True)
if ser is not None:
    try:
        ser.close()
    except Exception:
        pass
PY
