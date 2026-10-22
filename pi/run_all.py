#!/usr/bin/env python3
"""Run the watch receiver and the people counter together on the Pi.

Starts pi/receive.py (drive commands from the watch) and vision/people_counter.py (people
in view of the HuskyLens 2 camera) as two separate processes and prints both outputs in this
terminal, each line tagged [watch] or [camera]. The people counter runs at lower CPU priority,
so it never delays the watch's commands.

Person detection only runs while watch control is ON: the receiver shares the control state
and the people counter stops asking the HuskyLens for results whenever control is off or the
watch is lost. The people counter also tells the receiver how many people are in view, and
the receiver passes that on to the watch, which says each new count.

If the watch receiver stops, everything stops. If the people counter stops (for example, the
Pi's UART isn't turned on), the watch receiver keeps running. Ctrl+C stops both; the receiver
sends a final STOP on the way out.

Expects the repo's layout: pi/ and vision/ side by side. Both only need Python's standard library:

    python3 ~/robot/pi/run_all.py
    python3 ~/robot/pi/run_all.py --baud 9600   # extra options go to the people counter
"""

import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
RECEIVER = HERE / "receive.py"
COUNTER = HERE.parent / "vision" / "people_counter.py"
COUNTER_NICENESS = 10   # lower CPU priority for the people counter (0 is normal, 19 is lowest)
STOP_WAIT_S = 5.0       # how long each program gets to shut down after Ctrl+C

print_lock = threading.Lock()


def say(line):
    with print_lock:
        print(line, flush=True)


def start(tag, script, args=(), niceness=0):
    """Starts a script in its own process group (so Ctrl+C reaches it only through stop()) and relays its output."""
    cmd = [sys.executable, "-u", str(script), *args]
    if niceness:
        cmd = ["nice", "-n", str(niceness), *cmd]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                            start_new_session=True)

    def relay():
        for line in proc.stdout:
            if line.strip():
                say(f"[{tag}] {line.rstrip()}")

    threading.Thread(target=relay, daemon=True).start()
    return proc


def stop(*procs):
    """Interrupts each running program once, as Ctrl+C would, then waits; anything still running is killed."""
    for p in procs:
        if p.poll() is None:
            p.send_signal(signal.SIGINT)
    deadline = time.monotonic() + STOP_WAIT_S
    for p in procs:
        try:
            p.wait(timeout=max(0.0, deadline - time.monotonic()))
        except subprocess.TimeoutExpired:
            p.kill()
            p.wait()


def main():
    for path in (RECEIVER, COUNTER):
        if not path.exists():
            sys.exit(f"Can't find {path}. Copy the pi/ and vision/ folders side by side.")

    # Stopping the service (SIGTERM) shuts down the same way as Ctrl+C.
    signal.signal(signal.SIGTERM, signal.default_int_handler)

    watch = start("watch", RECEIVER)
    camera = start("camera", COUNTER, ["--watch-control", *sys.argv[1:]], COUNTER_NICENESS)
    camera_reported = False
    try:
        while watch.poll() is None:
            if camera.poll() is not None and not camera_reported:
                say(f"[run] The people counter stopped (exit code {camera.returncode}); "
                    "the watch receiver keeps running.")
                camera_reported = True
            time.sleep(0.2)
        say(f"[run] The watch receiver stopped (exit code {watch.returncode}); stopping everything.")
    except KeyboardInterrupt:
        say("[run] Stopping...")
    finally:
        stop(watch, camera)


if __name__ == "__main__":
    main()
