#!/usr/bin/env python3
"""Turn wrist tilt from the Galaxy Watch IMU app into drive commands.

Gestures, measured against the calibrated neutral pose:
    hand tilted down   -> FORWARD
    hand at neutral    -> NEUTRAL
    hand tilted up     -> STOP
    hand rolled left   -> LEFT
    hand rolled right  -> RIGHT

Tilt and roll are read independently, so a diagonal pose combines them, e.g.
tilted down and rolled left -> FORWARD+LEFT. STOP always overrides turning.

Each command also carries a speed (0..1) and a turn amount (-1 full left ..
+1 full right). Speed ramps up from just past neutral to 1 at the calibrated
full-speed pose; turn ramps up to +/-1 at the calibrated maximum left/right roll.

Tilting the hand down also shifts the roll reading a little, so turning is
calibrated twice: at neutral and at full speed. In between, the "straight" roll
and the left/right ranges are blended according to how far the hand is tilted down.

To ignore wrist jitter, gravity is smoothed (SMOOTHING_S), commands need a clear
tilt before they trigger (DEADZONE_FRACTION / MIN_ENTER_DEG), and FORWARD/LEFT/RIGHT
must be held for HOLD_S before they switch on. STOP still takes effect immediately.

Double-tap the watch screen to turn control on and off. While it is off (and
whenever the watch app opens) the command is STOP.

Usage:
    python3 laptop/watch_control.py               # start the watch app, calibrate if needed, run
    python3 laptop/watch_control.py --calibrate   # redo the calibration

The watch must be connected over adb (`adb devices`) for the script to start the
app; otherwise open "IMU Stream" on the watch yourself and pass --no-launch.
"""

import argparse
import json
import math
import re
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

DEFAULT_PORT = 5005
PACKAGE = "com.sofwerx.imuwatch"
CALIBRATION_FILE = Path(__file__).with_name("calibration.json")
CALIBRATION_VERSION = 3   # bump when the poses change so old files are redone

SIGNAL_TIMEOUT_S = 0.5    # no packets for this long -> STOP
SETTLE_S = 0.5            # pause after Enter before sampling a calibration pose
SAMPLE_S = 1.5            # how long each calibration pose is averaged
MIN_POSE_DEG = 15.0       # calibration poses must be at least this far from neutral
STOP_FRACTION = 0.5       # STOP triggers halfway to the calibrated full-stop tilt
DEADZONE_FRACTION = 0.25  # FORWARD/LEFT/RIGHT trigger this far toward their calibrated pose
MIN_ENTER_DEG = 10.0      # ...but never closer to neutral than this
EXIT_RATIO = 0.7          # once active, a command holds until tilt drops below 70% of its trigger
HOLD_S = 0.15             # FORWARD/LEFT/RIGHT must stay past their trigger this long to switch on
SMOOTHING_S = 0.1         # time constant of the low-pass filter on gravity
OUTPUT_STEP = 0.05        # resend when speed or turn moves by at least this much
LOOP_INTERVAL_S = 0.05


def send_command(cmd, speed, turn):
    """Called whenever the command changes or speed/turn move by OUTPUT_STEP. Hook the robot in here.

    cmd is "STOP", "NEUTRAL", "FORWARD", "LEFT", "RIGHT", "FORWARD+LEFT" or "FORWARD+RIGHT".
    speed is 0..1 (non-zero only with FORWARD); turn is -1 (full left) .. +1 (full right),
    non-zero only with LEFT/RIGHT. Both are 0 for STOP and NEUTRAL.
    """
    print(f"\r\033[K{time.strftime('%H:%M:%S')}  {cmd:<13}  speed {speed:4.0%}  turn {turn:+5.0%}")


# --- vector helpers -------------------------------------------------------

def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def sub(a, b):
    return [x - y for x, y in zip(a, b)]


def scale(a, k):
    return [x * k for x in a]


def norm(a):
    return math.sqrt(dot(a, a))


def unit(a):
    n = norm(a)
    if n < 1e-9:
        raise ValueError("zero-length vector")
    return [x / n for x in a]


def mean(vectors):
    return [sum(c) / len(vectors) for c in zip(*vectors)]


# --- receiving ------------------------------------------------------------

class Receiver(threading.Thread):
    """Reads IMU packets in the background and keeps the latest gravity vector and on/off switch."""

    def __init__(self, port):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.2)
        self.lock = threading.Lock()
        self.gravity = None       # low-pass filtered
        self.sample_t = None      # watch timestamp (s) of the last sample
        self.control_on = False
        self.last_time = 0.0
        self.count = 0
        self._recording = None

    def run(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            try:
                pkt = json.loads(data)
                g = [float(x) for x in (pkt.get("grav") or pkt["acc"])[:3]]
                on = pkt.get("on", True) is True  # older watch builds have no switch
                t = float(pkt.get("t", time.monotonic() * 1000)) / 1000
            except (ValueError, KeyError, TypeError):
                continue
            with self.lock:
                self._smooth(g, t)
                self.control_on = on
                self.last_time = time.monotonic()
                self.count += 1
                if self._recording is not None:
                    self._recording.append(g)

    def _smooth(self, g, t):
        """Exponential low-pass using the watch's sample timestamps, so Wi-Fi bursts don't skew it."""
        dt = t - self.sample_t if self.sample_t is not None else None
        self.sample_t = t
        if self.gravity is None or dt is None or not 0 < dt < SIGNAL_TIMEOUT_S:
            self.gravity = g   # first sample, gap, or watch app restarted
            return
        a = 1 - math.exp(-dt / SMOOTHING_S)
        self.gravity = [old + a * (new - old) for old, new in zip(self.gravity, g)]

    def latest(self):
        with self.lock:
            return self.gravity, self.control_on, self.last_time

    def record(self, seconds):
        with self.lock:
            self._recording = []
        time.sleep(seconds)
        with self.lock:
            samples, self._recording = self._recording, None
        return samples


def wait_for_stream(rx):
    if rx.count:
        return
    print("Waiting for IMU data from the watch (keep the IMU Stream app open)...")
    while not rx.count:
        time.sleep(0.1)
    print("Receiving data.")


# --- starting the watch app -----------------------------------------------

def adb(*args):
    return subprocess.run(["adb", *args], capture_output=True, text=True, timeout=15)


def launch_watch_app(port, host=None):
    """Start the watch app over adb, pointed at this laptop. Returns True on success."""
    try:
        if host is None:
            out = adb("shell", "ip", "-4", "-o", "addr", "show", "wlan0").stdout
            m = re.search(r"inet (\d+\.\d+\.\d+\.\d+)", out)
            if not m:
                print("Couldn't reach the watch over adb (check `adb devices`).")
                return False
            # Use whichever local address routes to the watch.
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect((m.group(1), 9))
                host = s.getsockname()[0]
        r = adb("shell", "am", "start", "-n", f"{PACKAGE}/.MainActivity",
                "--es", "host", host, "--ei", "port", str(port))
    except FileNotFoundError:
        print("adb not found; open the IMU Stream app on the watch manually.")
        return False
    except subprocess.TimeoutExpired:
        print("adb timed out talking to the watch.")
        return False
    if r.returncode != 0 or "Error" in r.stdout + r.stderr:
        print(f"Couldn't start the watch app:\n{(r.stdout + r.stderr).strip()}")
        return False
    print(f"Started IMU Stream on the watch, sending to {host}:{port}")
    return True


# --- calibration ----------------------------------------------------------

class CalibrationError(Exception):
    pass


POSES = [
    ("neutral", "Hold your hand where you want NEUTRAL"),
    ("down", "Tilt your hand DOWN to where you want FULL SPEED"),
    ("up", "Tilt your hand UP to where you want FULL STOP"),
    ("left", "Turn your hand as far LEFT as you can"),
    ("right", "Turn your hand as far RIGHT as you can"),
    ("down_left", "Tilt DOWN to FULL SPEED and turn as far LEFT as you can"),
    ("down_right", "Tilt DOWN to FULL SPEED and turn as far RIGHT as you can"),
]


def record_poses(rx):
    print("\nCalibration: hold each pose with the watch hand and press Enter with the other.")
    poses = {}
    for key, prompt in POSES:
        while True:
            input(f"\n{prompt}, then press Enter and hold still...")
            time.sleep(SETTLE_S)
            samples = rx.record(SAMPLE_S)
            if len(samples) >= 10:
                poses[key] = unit(mean(samples))
                print("  got it")
                break
            print(f"  only {len(samples)} samples arrived; is the watch app still open? Try again.")
    return poses


def tilt_angles(cal, g):
    """Pitch (+ = down) and roll (+ = right) in degrees relative to the neutral pose."""
    g = unit(g)
    z = dot(g, cal["neutral"])
    pitch = math.degrees(math.atan2(dot(g, cal["pitch_axis"]), z))
    roll = math.degrees(math.atan2(dot(g, cal["roll_axis"]), z))
    return pitch, roll


def build_calibration(poses):
    """Derive pitch/roll axes from the recorded gravity directions.

    Gravity in the watch frame moves one way when the hand tilts down/up and another
    way when it rolls left/right, so the poses give both axes regardless of which
    wrist the watch is on or how it is rotated.
    """
    n = poses["neutral"]

    def tangent(v):
        return sub(v, scale(n, dot(v, n)))

    min_sep = 2 * math.sin(math.radians(MIN_POSE_DEG / 2))
    pitch_raw = sub(tangent(poses["down"]), tangent(poses["up"]))
    if norm(pitch_raw) < min_sep:
        raise CalibrationError("the down and up poses were almost the same; tilt further.")
    pitch_axis = unit(pitch_raw)

    roll_raw = sub(tangent(poses["right"]), tangent(poses["left"]))
    if norm(roll_raw) < min_sep:
        raise CalibrationError("the left and right poses were almost the same; roll further.")
    roll_axis = sub(roll_raw, scale(pitch_axis, dot(roll_raw, pitch_axis)))
    if norm(roll_axis) < 0.5 * norm(roll_raw):
        raise CalibrationError("rolling moved the watch the same way as tilting; "
                               "keep the hand level while rolling.")
    roll_axis = unit(roll_axis)

    cal = {"version": CALIBRATION_VERSION, "neutral": n, "pitch_axis": pitch_axis, "roll_axis": roll_axis}
    ranges = {
        "FORWARD": tilt_angles(cal, poses["down"])[0],
        "STOP": -tilt_angles(cal, poses["up"])[0],
        "RIGHT": tilt_angles(cal, poses["right"])[1],
        "LEFT": -tilt_angles(cal, poses["left"])[1],
    }
    for cmd, deg in ranges.items():
        if deg < MIN_POSE_DEG:
            raise CalibrationError(f"the {cmd} pose was only {deg:.0f}° from neutral; "
                                   f"use at least {MIN_POSE_DEG:.0f}°.")
    cal["range_deg"] = ranges

    # Roll measured at full speed: where "straight" sits and how far each way the hand turns.
    center = tilt_angles(cal, poses["down"])[1]
    full_speed = {
        "center": center,
        "RIGHT": tilt_angles(cal, poses["down_right"])[1] - center,
        "LEFT": center - tilt_angles(cal, poses["down_left"])[1],
    }
    for cmd in ("LEFT", "RIGHT"):
        if full_speed[cmd] < MIN_POSE_DEG:
            raise CalibrationError(f"the full-speed {cmd} pose was only {full_speed[cmd]:.0f}° "
                                   f"from full-speed straight; use at least {MIN_POSE_DEG:.0f}°.")
    cal["full_speed_roll_deg"] = full_speed
    return cal


def level_roll(cal, pitch, roll):
    """Roll relative to "straight" at this tilt, plus the LEFT/RIGHT ranges at this tilt.

    Blends linearly from the neutral calibration (pitch 0) to the full-speed one.
    """
    f = min(1.0, max(0.0, pitch / cal["range_deg"]["FORWARD"]))
    full = cal["full_speed_roll_deg"]
    ranges = {cmd: (1 - f) * cal["range_deg"][cmd] + f * full[cmd] for cmd in ("LEFT", "RIGHT")}
    return roll - f * full["center"], ranges


def calibrate(rx):
    while True:
        try:
            cal = build_calibration(record_poses(rx))
            break
        except CalibrationError as e:
            print(f"\nCalibration failed: {e} Starting over.")
    CALIBRATION_FILE.write_text(json.dumps(cal, indent=2))
    ranges = ", ".join(f"{cmd} {deg:.0f}°" for cmd, deg in cal["range_deg"].items())
    full = cal["full_speed_roll_deg"]
    print(f"\nCalibrated ({ranges}; at full speed: straight {full['center']:+.0f}°, "
          f"LEFT {full['LEFT']:.0f}°, RIGHT {full['RIGHT']:.0f}°). Saved to {CALIBRATION_FILE}")
    return cal


def load_calibration():
    try:
        cal = json.loads(CALIBRATION_FILE.read_text())
        if cal.get("version") == CALIBRATION_VERSION:
            return cal
        print("Saved calibration is from an older version; recalibrating.")
    except (OSError, ValueError):
        pass
    return None


# --- gesture classification -----------------------------------------------

class GestureClassifier:
    """Classifies pitch and roll separately so FORWARD and LEFT/RIGHT can combine."""

    AXES = {"pitch": ("FORWARD", "STOP"), "roll": ("RIGHT", "LEFT")}

    def __init__(self, cal):
        self.base_ranges = cal["range_deg"]
        self.reset()

    def _set_ranges(self, ranges):
        self.ranges = ranges
        self.thresholds = {
            cmd: max(MIN_ENTER_DEG, (STOP_FRACTION if cmd == "STOP" else DEADZONE_FRACTION) * deg)
            for cmd, deg in ranges.items()}

    def _amount(self, cmd, angle):
        """0 where cmd releases, rising to 1 at its calibrated pose."""
        start = EXIT_RATIO * self.thresholds[cmd]
        return min(1.0, max(0.0, (abs(angle) - start) / (self.ranges[cmd] - start)))

    def _update_axis(self, axis, angle, now):
        pos, neg = self.AXES[axis]
        score = {pos: angle / self.thresholds[pos], neg: -angle / self.thresholds[neg]}
        active = self.active[axis]
        if active and score[active] < EXIT_RATIO:
            self.active[axis] = active = None
        best = max(score, key=score.get)
        if score[best] < 1 or best == active:
            self.pending[axis] = (None, 0.0)
            return
        # A new command has to hold past its trigger for HOLD_S, except STOP.
        if self.pending[axis][0] != best:
            self.pending[axis] = (best, now)
        if best == "STOP" or now - self.pending[axis][1] >= HOLD_S:
            self.active[axis] = best
            self.pending[axis] = (None, 0.0)

    def update(self, pitch, roll, roll_ranges, now):
        """Takes roll and LEFT/RIGHT ranges from level_roll. Returns (cmd, speed, turn)."""
        self._set_ranges({**self.base_ranges, **roll_ranges})
        self._update_axis("pitch", pitch, now)
        self._update_axis("roll", roll, now)
        if self.active["pitch"] == "STOP":
            # Stop wins over everything else.
            return "STOP", 0.0, 0.0
        speed = self._amount("FORWARD", pitch) if self.active["pitch"] == "FORWARD" else 0.0
        turn = 0.0
        if self.active["roll"] == "RIGHT":
            turn = self._amount("RIGHT", roll)
        elif self.active["roll"] == "LEFT":
            turn = -self._amount("LEFT", roll)
        parts = [c for c in (self.active["pitch"], self.active["roll"]) if c]
        return "+".join(parts) or "NEUTRAL", speed, turn

    def reset(self):
        self.active = {"pitch": None, "roll": None}
        self.pending = {"pitch": (None, 0.0), "roll": (None, 0.0)}  # (cmd, since) waiting out HOLD_S


def output_changed(new, old):
    """True once a value has moved a full OUTPUT_STEP, or has just reached an end of its range."""
    return abs(new - old) >= OUTPUT_STEP or (new != old and new in (-1.0, 0.0, 1.0))


def run(rx, cal):
    clf = GestureClassifier(cal)
    print("\nRunning. Double-tap the watch to turn control on/off.\n"
          "Tilt down = FORWARD, up = STOP, roll = LEFT/RIGHT (combine for FORWARD+LEFT). Ctrl+C to quit.\n")
    current = (None, 0.0, 0.0)
    try:
        while True:
            g, on, t = rx.latest()
            if g is None or time.monotonic() - t > SIGNAL_TIMEOUT_S:
                out, status = ("STOP", 0.0, 0.0), "no signal from watch"
                clf.reset()
            elif not on:
                out, status = ("STOP", 0.0, 0.0), "control OFF (double-tap watch to turn on)"
                clf.reset()
            else:
                pitch, roll = tilt_angles(cal, g)
                roll, roll_ranges = level_roll(cal, pitch, roll)
                out = clf.update(pitch, roll, roll_ranges, time.monotonic())
                status = f"tilt {pitch:+6.1f}°  roll {roll:+6.1f}°"
            cmd, speed, turn = out
            if (cmd != current[0] or output_changed(speed, current[1])
                    or output_changed(turn, current[2])):
                current = (cmd, round(speed, 2), round(turn, 2))
                send_command(*current)
            sys.stdout.write(f"\r\033[K{status}  [{cmd}  speed {speed:4.0%}  turn {turn:+5.0%}]")
            sys.stdout.flush()
            time.sleep(LOOP_INTERVAL_S)
    except KeyboardInterrupt:
        print()
        if current != ("STOP", 0.0, 0.0):
            send_command("STOP", 0.0, 0.0)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--port", type=int, default=DEFAULT_PORT, help="UDP port to listen on")
    ap.add_argument("--host", help="laptop IP the watch should send to (default: auto-detect)")
    ap.add_argument("--no-launch", action="store_true", help="don't start the watch app over adb")
    ap.add_argument("--calibrate", action="store_true", help="redo the calibration")
    args = ap.parse_args()

    try:
        rx = Receiver(args.port)
    except OSError as e:
        sys.exit(f"Can't listen on UDP port {args.port}: {e}")
    rx.start()

    if not args.no_launch:
        launch_watch_app(args.port, args.host)

    try:
        wait_for_stream(rx)
        cal = None if args.calibrate else load_calibration()
        if cal is None:
            cal = calibrate(rx)
        else:
            print(f"Using saved calibration from {CALIBRATION_FILE} (--calibrate to redo).")
    except (KeyboardInterrupt, EOFError):
        print()
        return
    run(rx, cal)


if __name__ == "__main__":
    main()
