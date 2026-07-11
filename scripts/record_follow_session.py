#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""跟随摔跤联调：多路日志落到同一目录。Ctrl+C 或写 stop 文件结束。"""
from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
from datetime import datetime

DIR = sys.argv[1] if len(sys.argv) > 1 else None
if not DIR:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    DIR = f"/home/nvidia/moon/logs/follow_debug/{stamp}"
os.makedirs(DIR, exist_ok=True)
open("/tmp/follow_debug_dir.txt", "w").write(DIR + "\n")
STOP = os.path.join(DIR, "STOP")

SETUP = None
try:
    sys.path.insert(0, "/home/nvidia/moon")
    from common.sim2real_env import source_setup_cmd

    SETUP = source_setup_cmd()
except Exception:
    SETUP = "source /home/nvidia/sim2real/install/setup.bash"


procs = []


def start(name: str, cmd: str) -> None:
    path = os.path.join(DIR, name)
    f = open(path, "a", buffering=1)
    p = subprocess.Popen(
        ["bash", "-lc", f"{SETUP} && {cmd}"],
        stdout=f,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )
    procs.append((name, p, f))
    print(f"[rec] {name} pid={p.pid}", flush=True)


def cleanup(*_a):
    open(STOP, "w").write("1\n")
    for name, p, f in procs:
        try:
            os.killpg(p.pid, signal.SIGTERM)
        except Exception:
            try:
                p.terminate()
            except Exception:
                pass
        try:
            f.close()
        except Exception:
            pass
    print("[rec] stopped", flush=True)
    sys.exit(0)


signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)

meta = open(os.path.join(DIR, "recorder_meta.txt"), "w")
meta.write(f"start={datetime.now()}\ndir={DIR}\n")
meta.flush()

start("arbiter.log", "tail -n 0 -F /home/nvidia/moon/logs/moon_arbiter_boot.log")
start(
    "rosout_filt.log",
    "tail -n 0 -F /home/nvidia/.ros/log/latest/rosout.log | "
    "grep --line-buffered -aE "
    "'switch to (standby|running)|policy name|PROTECTION|protection|Error sending|fall'",
)
start("fsm_state.log", "rostopic echo /fsm_state")
start("moon_mode.log", "rostopic echo /moon/mode")
start("cmd_vel.log", "rostopic echo /cmd_vel")
start("joy_msg.log", "rostopic echo /joy_msg")

print(f"[rec] READY dir={DIR}", flush=True)
print("[rec] 测完: touch $DIR/STOP  或 kill 本进程", flush=True)

while not os.path.exists(STOP):
    time.sleep(0.5)
    # drop heartbeats
    with open(os.path.join(DIR, "heartbeat.txt"), "w") as h:
        h.write(datetime.now().isoformat() + "\n")

cleanup()
