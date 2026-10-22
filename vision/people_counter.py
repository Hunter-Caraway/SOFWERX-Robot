#!/usr/bin/env python3
"""Count the people in view of a HuskyLens 2 AI camera wired to the Pi's serial port.

The HuskyLens 2 runs the person detection itself, on its own chip, with its Object Recognition
algorithm, which finds 80 kinds of everyday things: each "person" it finds is one person. It finds
bodies, not faces, so people facing away still count. This program switches the HuskyLens to that
algorithm and asks it for its results over the Pi's UART about 33 times a second. It needs only
Python's standard library.

Wiring, on the Pi 5's 40-pin header:
    T (green) -> pin 10 (GPIO15, the Pi's RX)
    R (blue)  -> pin 8  (GPIO14, the Pi's TX)
    - (black) -> pin 6  (GND)
    + (red)   -> pin 1 or 17 (3.3 V), which keeps its signals at the 3.3 V the Pi's pins take
On the HuskyLens 2, set System Settings > Protocol Type to Serial 115200 (the light by its port
turns green). On the Pi, turn on the UART on GPIO 14/15 (dtparam=uart0=on in
/boot/firmware/config.txt, then reboot); it is then /dev/ttyAMA0.

The per-result count is smoothed before it is reported. A higher count has to hold for
RISE_S, which filters out one-off false detections, and a lower count has to hold for
FALL_S, so someone briefly hidden or missed for a moment keeps being counted. The trade-off
is that the count drops FALL_S after someone really leaves.

Prints a line whenever the smoothed count changes. on_count is the hook for anything that
should react to the count.

With --watch-control, detection only runs while watch control is ON, as shared by
pi/receive.py on this Pi (pi/run_all.py turns this on). While it's off, or the receiver
isn't running, the counter stops asking the HuskyLens for results. It also tells the receiver
the count (UDP port PEOPLE_PORT on 127.0.0.1, as JSON: {"people": 2}, or {"people": null} while
there's none) whenever it changes and every PEOPLE_SHARE_INTERVAL_S, and the receiver passes it
on to the watch, which says each new count: "All clear!", "Person ahead!", "2 people ahead!".

While the watch is in follow mode, it also works out how to drive after a person (follower.py: at first, whoever
is nearest the middle of the picture) from each result, and sends that to the receiver (UDP port FOLLOW_PORT on
127.0.0.1, as JSON: {"cmd": "RIGHT", "speed": 0, "turn": 0.3, "target": true, "until": null}: target is whether it
has someone to follow, and until, for a timed turn on the spot while calibrating, when to stop (on the Pi's
time.monotonic() clock), else null, as other turns last only while it keeps asking). The
receiver drives with it in place of the watch's wrist tilt. What it measures as it goes of how the robot's turns move
the picture (how fast, how far the picture lags behind, and how long the robot keeps turning after a turn ends)
carries over from one time in follow mode to the next, and to the next run (in CALIBRATION_FILE).

Usage:
    python3 people_counter.py                       # HuskyLens on /dev/ttyAMA0 at 115200 baud
    python3 people_counter.py --baud 9600           # if its Protocol Type is Serial 9600
    python3 people_counter.py --port /dev/ttyUSB0   # a different serial port
    python3 people_counter.py --watch-control       # detect only while watch control is ON
    python3 people_counter.py --recalibrate         # follow mode measures turning afresh (camera or wheels changed)
"""

import argparse
import json
import os
import select
import socket
import struct
import sys
import termios
import time
import tty
from collections import deque

from follower import Box, Follower

SERIAL_PORT = "/dev/ttyAMA0"  # the Pi 5's UART on GPIO 14/15 (dtparam=uart0=on)
BAUD = 115200
RISE_S = 0.5                # a higher count must hold this long before it is reported
FALL_S = 0.5                # a lower count must hold this long (bridges people missed for a moment)
POLL_S = 0.03               # ask for results this often; the HuskyLens has new ones several times a second
REPLY_TIMEOUT_S = 1.0       # how long to wait for an answer to one request
MISSED_REPLIES = 3          # this many unanswered requests in a row and the HuskyLens counts as gone
SWITCH_TIMEOUT_S = 10.0     # switching algorithm can take a while, as it loads the model
RETRY_S = 2.0               # while the HuskyLens doesn't answer, how often to try again
HINT_AFTER_S = 10.0         # once it has been answering, how long it may be gone before saying what to check
CONTROL_PORT = 5006         # where pi/receive.py shares whether watch control is on
CONTROL_TIMEOUT_S = 1.5     # no word from the receiver for this long counts as off (it repeats every 0.5 s)
PEOPLE_PORT = 5007          # where pi/receive.py hears the count, to pass on to the watch
PEOPLE_SHARE_INTERVAL_S = 0.5  # repeat the count this often, so the receiver can tell when this program has stopped
PAUSED_POLL_S = 0.2         # while paused, how often to check whether control is back on
FOLLOW_PORT = 5008          # where pi/receive.py hears how to drive in follow mode
CALIBRATION_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "follow_calibration.json")
                            # what follow mode measured of how the robot's turns move the picture, for next time

# HuskyLens 2 protocol (DFRobot's HuskyLens2_Protocol.md). Every message, both ways, is one frame:
# 0x55 0xAA, command, algorithm, data length, data, checksum (the low byte of the sum of all bytes before it).
CMD_KNOCK = 0x00            # handshake
CMD_GET_RESULT = 0x01       # answered with RETURN_INFO, then one frame per result
CMD_SET_ALGORITHM = 0x0A
RETURN_ARGS = 0x1A          # data: number of int arguments, then 0 for success
RETURN_INFO = 0x1B          # data: max ID, RFU, then little-endian u16s: results, learned, blocks, blocks learned
RETURN_BLOCKS = 0x1C        # one result: ID, algorithm, then little-endian int16s: x and y of its middle, width,
                            # height; then the length of its name and the name. x and y run 0..640 and 0..480.
ALGORITHM_OBJECTS = 2       # Object Recognition: one result per thing found, of 80 kinds, each with its name
PERSON = "person"           # Object Recognition's name for a person


def on_count(count):
    """Called when the smoothed count of people changes, and with None when there is no count
    (detection paused, or the HuskyLens stopped answering)."""
    if count is not None:
        log(f"People in frame: {count}")


def log(msg):
    print(f"{time.strftime('%H:%M:%S')}  {msg}", flush=True)


def load_calibration(path):
    """What follow mode measured last time, (rate, lag, overrun) for a Follower, or None."""
    try:
        with open(path) as f:
            saved = json.load(f)
        rate, lag, overrun = float(saved["rate"]), float(saved["lag"]), float(saved["overrun"])
    except (OSError, ValueError, KeyError, TypeError):
        return None
    return (rate, lag, overrun) if rate and 0 < lag < 2 and 0 <= overrun < 1 else None


def save_calibration(path, calibration):
    """Keeps what follow mode measured, for next time. Writes a new file and swaps it in, so a power cut can't leave
    half of one."""
    rate, lag, overrun = calibration
    try:
        with open(path + ".new", "w") as f:
            json.dump({"rate": round(rate, 1), "lag": round(lag, 3), "overrun": round(overrun, 3)}, f)
        os.replace(path + ".new", path)
    except OSError as e:
        log(f"Couldn't save follow mode's calibration to {path}: {e}")


# --- HuskyLens 2 ----------------------------------------------------------------

def frame(command, algorithm=0, data=b""):
    """One protocol frame, ready to send."""
    head = bytes([0x55, 0xAA, command, algorithm, len(data)]) + bytes(data)
    return head + bytes([sum(head) & 0xFF])


class HuskyLens:
    """A HuskyLens 2 on a serial port."""

    def __init__(self, port, baud):
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
        tty.setraw(self.fd)
        attrs = termios.tcgetattr(self.fd)
        # 3 wires: no modem or flow control lines
        attrs[2] = (attrs[2] | termios.CLOCAL | termios.CREAD) & ~termios.CRTSCTS
        attrs[4] = attrs[5] = getattr(termios, f"B{baud}")  # input and output speed
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        self.buf = bytearray()

    def close(self):
        os.close(self.fd)

    def knock(self):
        """Handshake. True if it answered."""
        reply = self._ask(CMD_KNOCK, data=bytes([1]) + bytes(9))  # 1: a "large-memory" controller
        return reply is not None and reply[1] == 0

    def set_algorithm(self, algorithm):
        """Switches its algorithm. True once it says it has."""
        reply = self._ask(CMD_SET_ALGORITHM, data=bytes([algorithm]) + bytes(9), timeout=SWITCH_TIMEOUT_S)
        return reply is not None and reply[1] == 0

    def results(self, algorithm):
        """What the algorithm sees right now (for Object Recognition, one result per thing found), as (name, Box)
        pairs with the name in lower case, or None if no answer."""
        info = self._ask(CMD_GET_RESULT, algorithm, expect=RETURN_INFO)
        if info is None:
            return None
        count = struct.unpack_from("<H", info, 2)[0]
        # Each result follows as a frame of its own; read them so they don't get in the way of the next answer.
        results = []
        deadline = time.monotonic() + REPLY_TIMEOUT_S
        for _ in range(count):
            reply = self._read_frame(deadline)
            if reply is None:
                break
            data = reply[2]
            if reply[0] == RETURN_BLOCKS and len(data) >= 11:
                name = data[11:11 + data[10]].decode("utf-8", errors="replace").lower()
                results.append((name, Box(*struct.unpack_from("<4h", data, 2))))
        return results

    def _ask(self, command, algorithm=0, data=b"", expect=RETURN_ARGS, timeout=REPLY_TIMEOUT_S):
        """Sends a command and returns the data of the answer, or None if there's none in time."""
        termios.tcflush(self.fd, termios.TCIFLUSH)  # anything still unread belongs to an earlier exchange
        self.buf.clear()
        os.write(self.fd, frame(command, algorithm, data))
        deadline = time.monotonic() + timeout
        while (reply := self._read_frame(deadline)) is not None:
            if reply[0] == expect and len(reply[2]) >= 10:
                return reply[2]
        return None

    def _read_frame(self, deadline):
        """The next intact frame as (command, algorithm, data), or None if none arrives by the deadline."""
        while True:
            start = self.buf.find(b"\x55\xaa")
            if start < 0:
                del self.buf[:-1]  # keep a last 0x55, which may start a frame
            else:
                del self.buf[:start]
                if len(self.buf) >= 5 and len(self.buf) >= 6 + self.buf[4]:
                    end = 5 + self.buf[4]
                    head, checksum = bytes(self.buf[:end]), self.buf[end]
                    if sum(head) & 0xFF == checksum:
                        del self.buf[:end + 1]
                        return head[2], head[3], head[5:]
                    del self.buf[:2]  # not a real frame: look for the next start
                    continue
            left = deadline - time.monotonic()
            if left <= 0 or not select.select([self.fd], [], [], left)[0]:
                return None
            self.buf += os.read(self.fd, 4096)


def connect(lens, port, baud, hint_after_s=0.0):
    """Waits until the HuskyLens answers, then switches it to Object Recognition. If that takes longer than
    hint_after_s, says once what to check."""
    start, hinted = time.monotonic(), False
    while not (lens.knock() and lens.set_algorithm(ALGORITHM_OBJECTS)):
        if not hinted and time.monotonic() - start >= hint_after_s:
            log(f"No answer from the HuskyLens 2 on {port} at {baud} baud. Check that its T wire goes to pin 10 and R "
                f"to pin 8, and that its Protocol Type is Serial {baud} (green light by the port). Still trying...")
            hinted = True
        time.sleep(RETRY_S)


# --- counting ---------------------------------------------------------------

class CountFilter:
    """Smooths the per-result count: a rise must hold for rise_s, a drop for fall_s."""

    def __init__(self, rise_s, fall_s):
        self.rise_s, self.fall_s = rise_s, fall_s
        self.history = deque()  # (time, count); the oldest entry is the one in effect fall_s ago
        self.count = None       # the smoothed count, None until rise_s of history

    def _counts_since(self, start):
        """Every count in effect from start until now, or None if the history doesn't reach back that far."""
        if self.history[0][0] > start:
            return None
        before = [c for t, c in self.history if t <= start][-1:]  # the one in effect at start
        return before + [c for t, c in self.history if t > start]

    def update(self, t, count):
        """Adds this result's count. Returns the smoothed count when it changes, else None."""
        self.history.append((t, count))
        while len(self.history) > 1 and self.history[1][0] <= t - self.fall_s:
            self.history.popleft()

        new = self.count
        rise = self._counts_since(t - self.rise_s)
        if rise is not None and (new is None or min(rise) > new):
            new = min(rise)  # at least this many people for rise_s
        fall = self._counts_since(t - self.fall_s)
        if fall is not None and new is not None and max(fall) < new:
            new = max(fall)  # at most this many people for fall_s
        if new == self.count:
            return None
        self.count = new
        return new


# --- watch ------------------------------------------------------------------

class WatchControl:
    """Follows whether watch control is on, and in follow mode, as pi/receive.py shares it on this Pi."""

    def __init__(self, port, timeout_s=CONTROL_TIMEOUT_S):
        self.timeout_s = timeout_s
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", port))
        self.sock.setblocking(False)
        self.state = "off"
        self.heard_at = None

    def mode(self):
        """"on", or "follow" in follow mode, while the receiver says so; "off" if it says off or has gone quiet."""
        while True:
            try:
                msg = self.sock.recv(64).strip()
            except BlockingIOError:
                break
            if msg in (b"on", b"off", b"follow"):
                self.state, self.heard_at = msg.decode(), time.monotonic()
        if self.heard_at is None or time.monotonic() - self.heard_at >= self.timeout_s:
            return "off"
        return self.state

    def is_on(self):
        """True while the receiver says control is on; False if it says off or has gone quiet."""
        return self.mode() != "off"


class PeopleShare:
    """Tells pi/receive.py on this Pi the count, whenever it changes and every PEOPLE_SHARE_INTERVAL_S."""

    def __init__(self, port):
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.shared, self.shared_at = None, None

    def share(self, count):
        """count is the smoothed count, or None while there's none (detection paused, or no answer from the
        HuskyLens)."""
        msg = json.dumps({"people": count}, separators=(",", ":")).encode()
        now = time.monotonic()
        if msg == self.shared and self.shared_at is not None and now - self.shared_at < PEOPLE_SHARE_INTERVAL_S:
            return
        try:
            self.sock.sendto(msg, ("127.0.0.1", self.port))
        except OSError:
            pass  # nobody listening is fine
        self.shared, self.shared_at = msg, now


class FollowShare:
    """Tells pi/receive.py on this Pi how to drive in follow mode, after every result."""

    def __init__(self, port):
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def share(self, drive, target, until=None):
        """drive is (cmd, speed, turn); target is whether there's someone to follow; until is when to stop turning on
        the spot (time.monotonic()), if it is."""
        cmd, speed, turn = drive
        msg = json.dumps({"cmd": cmd, "speed": speed, "turn": turn, "target": target, "until": until},
                         separators=(",", ":"))
        try:
            self.sock.sendto(msg.encode(), ("127.0.0.1", self.port))
        except OSError:
            pass  # nobody listening is fine


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--port", default=SERIAL_PORT, help=f"serial port the HuskyLens is on (default {SERIAL_PORT})")
    ap.add_argument("--baud", type=int, default=BAUD, choices=(9600, 115200),
                    help=f"its Protocol Type setting: Serial 9600 or Serial 115200 (default {BAUD})")
    ap.add_argument("--watch-control", type=int, nargs="?", const=CONTROL_PORT, metavar="PORT",
                    help="detect only while watch control is ON, as shared by pi/receive.py on this Pi "
                         f"(default port {CONTROL_PORT})")
    ap.add_argument("--recalibrate", action="store_true",
                    help="in follow mode, measure afresh how the robot's turns move the picture, rather than start "
                         "from what it measured last time")
    args = ap.parse_args()

    try:
        lens = HuskyLens(args.port, args.baud)
    except FileNotFoundError:
        sys.exit(f"No serial port {args.port}. Turn on the Pi's UART on GPIO 14/15: add dtparam=uart0=on to "
                 "/boot/firmware/config.txt and reboot.")
    except PermissionError:
        sys.exit(f"Not allowed to open {args.port}. Add this user to the dialout group "
                 "(sudo usermod -aG dialout $USER), then log in again.")

    control = share = follow = None
    if args.watch_control:
        try:
            control = WatchControl(args.watch_control)
        except OSError as e:
            sys.exit(f"Can't listen for watch control on port {args.watch_control}: {e}")
        share = PeopleShare(PEOPLE_PORT)
        follow = FollowShare(FOLLOW_PORT)

    log(f"Counting people with the HuskyLens 2 on {args.port} at {args.baud} baud. Ctrl+C to quit.")
    if control:
        log("Detecting only while watch control is ON.")
    try:
        connect(lens, args.port, args.baud)
        log("HuskyLens 2 connected, running Object Recognition.")
        smoother = CountFilter(RISE_S, FALL_S)
        missed = 0
        paused = None
        following = False  # in follow mode
        target = False     # following someone
        follower = None
        calibration = None if args.recalibrate else load_calibration(CALIBRATION_FILE)
        if calibration:
            log("Follow mode starts from last time's calibration: the picture moves {:.0f} px/s turning, {:.2f} s "
                "behind, and keeps moving {:.2f} s after".format(abs(calibration[0]), *calibration[1:]))
        while True:
            mode = control.mode() if control else "on"
            if mode == "off":
                if paused is not True:
                    log("Detection paused: watch control is OFF")
                    on_count(None)
                    smoother = CountFilter(RISE_S, FALL_S)  # count afresh once control is back on
                    paused, following = True, False
                share.share(None)
                time.sleep(PAUSED_POLL_S)
                continue
            if paused:
                log("Detection resumed: watch control is ON")
            paused = False
            if (mode == "follow") != following:
                following, target = mode == "follow", False
                log("Follow mode ON: following the person nearest the middle" if following else "Follow mode OFF")
                follower = Follower(*(calibration or ()), log=lambda msg: log(f"Follow: {msg}"))  # locks on afresh
            if share:
                share.share(smoother.count)

            asked = time.monotonic()
            found = lens.results(ALGORITHM_OBJECTS)
            if found is None:
                missed += 1  # it can miss a few just after switching algorithm, while the model loads
                if missed >= MISSED_REPLIES:
                    log("The HuskyLens 2 stopped answering.")
                    on_count(None)
                    if share:
                        share.share(None)
                    connect(lens, args.port, args.baud, HINT_AFTER_S)
                    log("HuskyLens 2 answering again.")
                    smoother, missed = CountFilter(RISE_S, FALL_S), 0
                continue
            missed = 0
            people = [box for name, box in found if name == PERSON]
            now = time.monotonic()
            changed = smoother.update(now, len(people))
            if changed is not None:
                on_count(changed)
            if following:
                drive, found_target, until = follower.update(now, people)
                if follower.calibration() not in (None, calibration):
                    calibration = follower.calibration()
                    save_calibration(CALIBRATION_FILE, calibration)
                if found_target != target:
                    log("Following someone" if found_target else "Lost them")
                    target = found_target
                follow.share(drive, target, until)
            time.sleep(max(0.0, POLL_S - (time.monotonic() - asked)))
    except KeyboardInterrupt:
        pass
    finally:
        lens.close()


if __name__ == "__main__":
    main()
