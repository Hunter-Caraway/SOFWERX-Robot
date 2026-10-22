#!/usr/bin/env python3
"""Drive the robot with the commands the watch sends to this Pi, and print them.

The IMU Stream watch app works out a command from your wrist tilt and sends it about
50 times a second over UDP:

    hand level              -> STOP
    tilted down / up        -> FORWARD / REVERSE   (with speed 0..100%)
    rolled left / right     -> LEFT / RIGHT        (with turn -100..+100%)
    tilted and rolled       -> FORWARD-LEFT, FORWARD-RIGHT, REVERSE-LEFT, REVERSE-RIGHT

The watch also sends its gear, as the top speed (0..1): every wrist command's speed is scaled
by it, so tilting all the way drives at that speed. In follow mode (held down on the watch), it
drives after the nearest person the camera sees instead of by wrist tilt, as the people
counter works out (UDP port FOLLOW_PORT on 127.0.0.1), and stops if that goes quiet. Follow
mode has its own speeds, unscaled by the gear, as it measures how fast the robot turns. Its
turns on the spot come with a time to stop, which this program keeps to.

This prints a line and sets the motors (through motors.py) whenever the command changes
or speed/turn move by 5%. The command is STOP while control is off on the watch
(double-tap to switch it) and whenever no packets have arrived for 0.5 s. Ctrl+C, or
stopping it with kill, turns the motors off. If it can't reach the motors, it says why
and only prints the commands.

It also tells other programs on this Pi whether watch control is on, by sending "on",
"follow" (on, in follow mode) or "off" to UDP port CONTROL_PORT on 127.0.0.1 whenever that
changes and every CONTROL_SHARE_INTERVAL_S. The people counter uses it to detect only while
control is on, and to work out how to follow someone only in follow mode.

In return, the people counter tells this program how many people are in view (UDP port
PEOPLE_PORT on 127.0.0.1). This program passes that on to the watch, which says each new count:
{"people": 2, "target": null}, or {"people": null, ...} while there's no count (detection paused, or
no word from the people counter). In follow mode, target says whether the robot has someone to follow
(true or false); otherwise it's null. It goes back to the address and port the watch's packets come
from, whenever it changes and every PEOPLE_SEND_INTERVAL_S.

Usage:
    python3 receive.py              # drive the motors and print commands
    python3 receive.py --no-motors  # only print commands
"""

import argparse
import json
import signal
import socket
import sys
import time

from motors import I2C_ADDRESS, I2C_BUS, Motors, explain

DEFAULT_PORT = 5005
SIGNAL_TIMEOUT_S = 0.5    # no packets for this long -> STOP
OUTPUT_STEP = 0.05        # resend when speed or turn moves by at least this much
COMMANDS = {"STOP", "FORWARD", "REVERSE", "LEFT", "RIGHT",
            "FORWARD-LEFT", "FORWARD-RIGHT", "REVERSE-LEFT", "REVERSE-RIGHT"}
STOP = ("STOP", 0.0, 0.0)
REVERSED = True                 # drive toward the end motors.py calls the rear (see wheel_speeds); False to undo
CONTROL_PORT = 5006             # where this Pi's other programs hear whether watch control is on
CONTROL_SHARE_INTERVAL_S = 0.5  # repeat it this often, so they can tell when this program has stopped
PEOPLE_PORT = 5007              # where the people counter on this Pi gives the count
PEOPLE_TIMEOUT_S = 1.5          # no word from the people counter for this long: no count (it repeats every 0.5 s)
PEOPLE_SEND_INTERVAL_S = 0.5    # repeat the count to the watch this often, in case a packet is lost
FOLLOW_PORT = 5008              # where the people counter says how to drive in follow mode
FOLLOW_TIMEOUT_S = 0.5          # no word from it for this long in follow mode: STOP (it sends after every result)

motors = None       # the robot's Motors, once main() has reached them
motor_error = None  # set while the motors haven't taken the latest command; main() keeps retrying


def send_command(cmd, speed, turn):
    """Prints the command and sets the motors. Called whenever the command changes or speed/turn move by OUTPUT_STEP.

    cmd is one of COMMANDS. speed is 0..1 and non-zero only with FORWARD or REVERSE (the
    direction is in cmd); turn is -1 (full left) .. +1 (full right), non-zero only with
    LEFT or RIGHT. Both are 0 for STOP.
    """
    line = f"{cmd:<13}"
    if speed:
        line += f"  speed {speed:4.0%}"
    if turn:
        line += f"  turn {turn:+5.0%}"
    log(line.rstrip())
    drive_motors(cmd, speed, turn)


def wheel_speeds(cmd, speed, turn):
    """Left and right wheel speeds for a command, -1 (full reverse) .. +1 (full forward).

    LEFT and RIGHT on their own spin the robot on the spot. FORWARD and REVERSE drive both
    sides at speed, and with a turn (FORWARD-LEFT etc.) the inside of the turn slows, down
    to a standstill at full turn, so the robot curves.

    With REVERSED, the robot's front is the end motors.py calls its rear. Its left side is then
    the motors' right side and each wheel's forward is the motors' backward, so FORWARD and
    REVERSE swap while every turn still goes the way you steer.
    """
    if cmd in ("LEFT", "RIGHT"):
        left, right = turn, -turn
    else:
        v = speed if cmd.startswith("FORWARD") else -speed if cmd.startswith("REVERSE") else 0.0
        left, right = v * (1 + min(turn, 0.0)), v * (1 - max(turn, 0.0))
    return (-right, -left) if REVERSED else (left, right)


def drive_motors(cmd, speed, turn):
    """Sets the motors for a command. If they don't answer, says so once and leaves motor_error set."""
    global motor_error
    if motors is None:
        return
    try:
        motors.drive(*wheel_speeds(cmd, speed, turn))
    except OSError as e:
        if motor_error is None:
            log(f"Can't set the motors ({e}). Retrying...")
        motor_error = e
        return
    if motor_error is not None:
        log("Motors answering again")
        motor_error = None


def log(msg):
    print(f"{time.strftime('%H:%M:%S')}  {msg}", flush=True)


def local_ip():
    """Best guess at the address the watch should send to."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 9))  # picks the outgoing interface; nothing is sent
            return s.getsockname()[0]
    except OSError:
        return "0.0.0.0"


def output_changed(new, old):
    """True once a value has moved a full OUTPUT_STEP, or has just reached an end of its range."""
    return abs(new - old) >= OUTPUT_STEP or (new != old and new in (-1.0, 0.0, 1.0))


def parse(data):
    """Returns (on, follow, top, (cmd, speed, turn)) from a watch packet: follow is whether it's in follow mode,
    and top its gear's top speed, 0..1 (1 from watch apps without gears). Raises ValueError/KeyError/TypeError."""
    pkt = json.loads(data)
    on = pkt.get("on") is True
    follow = pkt.get("follow") is True
    top = min(1.0, max(0.0, float(pkt.get("top", 1.0))))
    cmd = pkt["cmd"]
    speed = min(1.0, max(0.0, float(pkt["speed"])))
    turn = min(1.0, max(-1.0, float(pkt["turn"])))
    if not on or cmd not in COMMANDS:
        return on, follow, top, STOP
    return on, follow, top, (cmd, round(speed, 2), round(turn, 2))


def parse_follow(data):
    """The people counter's follow-mode command, as ((cmd, speed, turn), target, until): target is whether it has
    someone to follow, and until when to stop (time.monotonic()), or None. Raises ValueError/KeyError/TypeError."""
    msg = json.loads(data)
    cmd, target, until = msg["cmd"], msg["target"], msg.get("until")
    if cmd not in COMMANDS or type(target) is not bool:
        raise ValueError("not a follow command")
    speed = min(1.0, max(0.0, float(msg["speed"])))
    turn = min(1.0, max(-1.0, float(msg["turn"])))
    return (cmd, round(speed, 2), round(turn, 2)), target, None if until is None else float(until)


def geared(drive, top):
    """The command with its speed scaled down to the top speed (0..1). Spinning on the spot, the turn is scaled
    instead; with FORWARD or REVERSE the turn stays, so the robot curves the same, only slower."""
    cmd, speed, turn = drive
    if cmd in ("LEFT", "RIGHT"):
        return cmd, speed, round(turn * top, 2)
    return cmd, round(speed * top, 2), turn


def parse_people(data):
    """The people counter's count, as {"people": count}, count None while it has none. Raises ValueError."""
    status = json.loads(data)
    if not isinstance(status, dict) or "people" not in status:
        raise ValueError("not a people count")
    people = status["people"]
    if people is not None and (type(people) is not int or people < 0):
        raise ValueError("not a people count")
    return {"people": people}


def main():
    global motors
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--port", type=int, default=DEFAULT_PORT, help="UDP port to listen on")
    ap.add_argument("--no-motors", action="store_true", help="only print commands; don't drive the motors")
    args = ap.parse_args()
    signal.signal(signal.SIGTERM, signal.default_int_handler)  # stopping it with kill also stops the motors

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("0.0.0.0", args.port))
    except OSError as e:
        sys.exit(f"Can't listen on UDP port {args.port}: {e}")
    sock.settimeout(0.1)
    people_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        people_sock.bind(("127.0.0.1", PEOPLE_PORT))
        people_sock.setblocking(False)
    except OSError as e:
        log(f"Can't hear the people counter on port {PEOPLE_PORT} ({e}); the watch won't get the people count")
        people_sock = None
    follow_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        follow_sock.bind(("127.0.0.1", FOLLOW_PORT))
        follow_sock.setblocking(False)
    except OSError as e:
        log(f"Can't hear the people counter on port {FOLLOW_PORT} ({e}); follow mode won't move the robot")
        follow_sock = None
    if not args.no_motors:
        try:
            motors = Motors()
            log(f"Motors ready (PCA9685 at 0x{I2C_ADDRESS:02x} on I2C bus {I2C_BUS})")
        except OSError as e:
            log(f"Motors OFF, only printing commands: {explain(e)}")
    log(f"Listening on {local_ip()}:{args.port}. Open IMU Stream on the watch. Ctrl+C to quit.")

    current = STOP
    send_command(*current)
    sender = None         # watch address while packets are arriving
    reply_to = None       # (address, port) of the watch's latest packet
    control_on = None
    following = False     # the watch is in follow mode
    top = None            # the watch's top speed
    last_rx = 0.0
    warned = set()        # senders already told their packets have no command

    def output(new):
        nonlocal current
        if (new[0] != current[0] or output_changed(new[1], current[1])
                or output_changed(new[2], current[2])):
            current = new
            send_command(*current)

    share = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    shared, shared_at = None, 0.0

    def share_control(force=False):
        """Tells this Pi's other programs whether watch control is on, and in follow mode (off while the watch is
        lost)."""
        nonlocal shared, shared_at
        msg = b"off" if sender is None or control_on is not True else b"follow" if following else b"on"
        now = time.monotonic()
        if force or msg != shared or now - shared_at >= CONTROL_SHARE_INTERVAL_S:
            try:
                share.sendto(msg, ("127.0.0.1", CONTROL_PORT))
            except OSError:
                pass  # nobody listening is fine
            shared, shared_at = msg, now

    follow_drive, follow_target, follow_until, follow_at = STOP, None, None, None  # its latest follow-mode command

    def hear_follow():
        """Takes in the people counter's follow-mode commands."""
        nonlocal follow_drive, follow_target, follow_until, follow_at
        while follow_sock is not None:
            try:
                follow_drive, follow_target, follow_until = parse_follow(follow_sock.recv(512))
                follow_at = time.monotonic()
            except BlockingIOError:
                break
            except (ValueError, KeyError, TypeError):
                pass

    def follow_fresh():
        return follow_at is not None and time.monotonic() - follow_at < FOLLOW_TIMEOUT_S

    people, people_at = None, None  # the people counter's latest count, and when it said so
    told, told_at = None, None      # what the watch was last told, and when

    def relay_people():
        """Passes the people counter's count on to the watch."""
        nonlocal people, people_at, told, told_at
        while people_sock is not None:
            try:
                people, people_at = parse_people(people_sock.recv(512)), time.monotonic()
            except BlockingIOError:
                break
            except ValueError:
                pass
        now = time.monotonic()
        status = people if people_at is not None and now - people_at < PEOPLE_TIMEOUT_S else {"people": None}
        following_now = sender is not None and control_on is True and following
        status = dict(status, target=follow_target if following_now and follow_fresh() else None)
        if sender is None or reply_to is None:
            return
        if status != told or told_at is None or now - told_at >= PEOPLE_SEND_INTERVAL_S:
            try:
                sock.sendto(json.dumps(status).encode(), reply_to)
            except OSError:
                pass  # the watch will hear the next one
            told, told_at = status, now

    try:
        while True:
            share_control()
            hear_follow()
            relay_people()
            if motor_error is not None:
                drive_motors(*current)
            try:
                data, src = sock.recvfrom(2048)
                addr = src[0]
            except socket.timeout:
                if sender is not None and time.monotonic() - last_rx > SIGNAL_TIMEOUT_S:
                    log(f"Lost the watch (no packets for {SIGNAL_TIMEOUT_S} s)")
                    sender = control_on = None
                    output(STOP)
                continue

            try:
                on, follow, new_top, new = parse(data)
            except (ValueError, KeyError, TypeError):
                if addr not in warned:
                    log(f"Ignoring packets from {addr} without a drive command (reinstall the watch app?)")
                    warned.add(addr)
                continue

            if sender is None:
                log(f"Watch connected ({addr})")
                sender = addr
            reply_to = src
            last_rx = time.monotonic()
            if on != control_on:
                log(f"Control {'ON' if on else 'OFF (double-tap the watch to turn on)'}")
                control_on = on
            if follow != following:
                log("Follow mode ON: following the nearest person the camera sees" if follow else "Follow mode OFF")
                following = follow
            if new_top != top:
                log(f"Top speed {new_top:.0%}")
                top = new_top
            if on and following:
                going = follow_fresh() and (follow_until is None or time.monotonic() < follow_until)
                output(follow_drive if going else STOP)  # its own speeds: the gear is for the wrist
            else:
                output(geared(new, top))
    except KeyboardInterrupt:
        print()
        if current != STOP:
            send_command(*STOP)
        sender = None
        share_control(force=True)  # "off", so the people counter pauses straight away
    finally:
        if motors is not None:
            try:
                motors.close()
            except OSError as e:
                log(f"Couldn't turn the motors off ({e}). Switch off the motor power!")


if __name__ == "__main__":
    main()
