#!/usr/bin/env python3
"""Drive the robot's four motors: two L298N motor drivers controlled by a PCA9685 PWM board on I2C.

The right motors are on one L298N, wired to PCA9685 outputs 0-3, and the left motors on the
other, on outputs 4-7 (each L298N's IN1, IN2, IN3, IN4 in order), as listed in LEFT_MOTORS and
RIGHT_MOTORS. Each motor has two IN pins: one is pulsed PWM_HZ times a second to set its power,
and which one sets its direction. Both low brakes it. The ENA/ENB jumpers stay on, so the L298Ns
are always enabled. It only needs Python's standard library: it talks to the PCA9685 through
Linux's I2C device file.

Wiring, PCA9685 to the Pi 5's 40-pin header:
    SDA -> pin 3 (GPIO2)
    SCL -> pin 5 (GPIO3)
    VCC -> pin 1 or 17 (3.3 V), which keeps its I2C signals at the 3.3 V the Pi's pins take
    GND -> pin 9 (GND)
The L298Ns' GND also goes to the PCA9685's GND. The PCA9685's V+ terminal is only for servos and
isn't needed. On the Pi, turn on I2C on GPIO 2/3 once (dtparam=i2c_arm=on in
/boot/firmware/config.txt, then reboot); it is then /dev/i2c-1.

Run this file on its own to check the wiring. It spins each motor forward, then backward, one at
a time, and says which one should be turning. Lift the wheels off the ground first.

    python3 motors.py
    python3 motors.py --speed 0.8

pi/receive.py drives the motors from the watch's commands. The PCA9685 has no timeout of its own:
if a program driving it is killed outright (kill -9), the motors keep going until something
starts it again, which turns every output off first.
"""

import argparse
import fcntl
import os
import signal
import sys
import time

I2C_BUS = 1             # /dev/i2c-1, on GPIO 2/3 (dtparam=i2c_arm=on)
I2C_ADDRESS = 0x40      # the PCA9685's address with none of its address jumpers soldered
PWM_HZ = 1000           # how often the IN pins are pulsed
MIN_DUTY = 0.3          # the least power that turns the wheels; any speed above 0 gets at least this
MAX_DUTY = 1.0          # power at full speed; lower it to cap the robot's top speed

# The PCA9685 outputs wired to each motor's two L298N IN pins: (forward, backward). Which of the two
# turns a wheel forward depends on how the motor is connected, so running this file shows what each
# entry does. A motor turning the wrong way: swap its two outputs.
LEFT_MOTORS = {
    "front left": (6, 7),    # left L298N, IN3/IN4 -> OUT3/OUT4
    "rear left": (4, 5),     # left L298N, IN1/IN2 -> OUT1/OUT2
}
RIGHT_MOTORS = {
    "front right": (2, 3),   # right L298N, IN3/IN4 -> OUT3/OUT4
    "rear right": (0, 1),    # right L298N, IN1/IN2 -> OUT1/OUT2
}

TEST_S = 1.0            # how long the motor test runs each motor each way

I2C_SLAVE = 0x0703      # ioctl that picks the device to talk to on the bus
OSC_HZ = 25_000_000     # the PCA9685's internal oscillator
MODE1, MODE2, LED0_ON_L, ALL_LED_OFF_H, PRESCALE = 0x00, 0x01, 0x06, 0xFD, 0xFE
SLEEP, AUTO_INCREMENT, OUTDRV = 0x10, 0x20, 0x04
FULL = 0x10             # bit in an output's ON_H / OFF_H register: always on / always off


class Busy(OSError):
    """Another program is driving the motors."""


class PCA9685:
    """The PCA9685's 16 PWM outputs. Every output starts off.

    Only one program at a time can have it. Raises Busy if another program does, and OSError if it
    can't be reached.
    """

    def __init__(self, bus=I2C_BUS, address=I2C_ADDRESS, pwm_hz=PWM_HZ):
        self.fd = os.open(f"/dev/i2c-{bus}", os.O_RDWR)
        try:
            try:
                fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)  # released when this program exits, however
            except BlockingIOError:
                raise Busy(f"another program has /dev/i2c-{bus}") from None
            fcntl.ioctl(self.fd, I2C_SLAVE, address)
            self._start(pwm_hz)
        except OSError:
            os.close(self.fd)
            raise

    def _start(self, pwm_hz):
        self.all_off()                              # first, in case a killed program left the motors going
        self._write(MODE1, SLEEP | AUTO_INCREMENT)  # the PWM frequency can only be set while asleep
        self._write(PRESCALE, round(OSC_HZ / (4096 * pwm_hz)) - 1)
        self._write(MODE2, OUTDRV)                  # push-pull outputs, which drive the L298N's inputs directly
        self._write(MODE1, AUTO_INCREMENT)          # wake up
        time.sleep(0.001)                           # the oscillator takes 500 us to start

    def set(self, channel, level):
        """Sets an output from 0 (off) to 1 (always on)."""
        if level >= 1:
            regs = (0, FULL, 0, 0)
        elif level <= 0:
            regs = (0, 0, 0, FULL)
        else:
            off = min(4095, max(1, round(level * 4096)))  # on at count 0, off at this count of 4096
            regs = (0, 0, off & 0xFF, off >> 8)
        self._write(LED0_ON_L + 4 * channel, *regs)

    def all_off(self):
        self._write(ALL_LED_OFF_H, FULL)

    def close(self):
        """Turns every output off and lets go of the I2C bus."""
        try:
            self.all_off()
        finally:
            os.close(self.fd)

    def _write(self, register, *data):
        os.write(self.fd, bytes([register, *data]))


def duty(speed):
    """Power (0..1) for a motor speed (-1..1): 0 stays 0, anything else is scaled into MIN_DUTY..MAX_DUTY."""
    speed = min(abs(speed), 1.0)
    return MIN_DUTY + (MAX_DUTY - MIN_DUTY) * speed if speed else 0.0


class Motors:
    """The robot's motors. drive() sets each side; close() turns everything off."""

    def __init__(self, pwm=None):
        self.pwm = pwm or PCA9685()
        self.motors = {**LEFT_MOTORS, **RIGHT_MOTORS}
        self.levels = {}  # output -> the level it was last set to

    def drive(self, left, right):
        """Speed of each side, -1 (full reverse) .. +1 (full forward); 0 brakes.

        Raises OSError if the PCA9685 doesn't answer. Calling again sets whatever didn't get set.
        """
        for name in LEFT_MOTORS:
            self.set_motor(name, left)
        for name in RIGHT_MOTORS:
            self.set_motor(name, right)

    def set_motor(self, name, speed):
        """One motor's speed, -1 .. +1; 0 brakes it."""
        forward, backward = self.motors[name]
        if speed < 0:
            forward, backward = backward, forward
        self._set(backward, 0.0)  # first, so both IN pins are never on together
        self._set(forward, duty(speed))

    def _set(self, channel, level):
        """Writes an output only when its level changes."""
        if self.levels.get(channel) != level:
            self.levels.pop(channel, None)  # unknown until the write goes through
            self.pwm.set(channel, level)
            self.levels[channel] = level

    def close(self):
        """Turns every output off, which brakes the motors, and lets go of the I2C bus. Raises OSError if that fails."""
        self.pwm.close()


def explain(error):
    """What to check when the PCA9685 can't be reached."""
    if isinstance(error, FileNotFoundError):
        return (f"there's no I2C bus /dev/i2c-{I2C_BUS}. Turn on I2C on the Pi: "
                "echo dtparam=i2c_arm=on | sudo tee -a /boot/firmware/config.txt, then sudo reboot")
    if isinstance(error, Busy):
        return ("another program is driving them. If it's the robot service, stop it first: "
                "sudo systemctl stop robot")
    if isinstance(error, PermissionError):
        return (f"no access to /dev/i2c-{I2C_BUS}. Add yourself to the i2c group: sudo usermod -aG i2c $USER, "
                "then log in again")
    return (f"no answer from the PCA9685 at 0x{I2C_ADDRESS:02x} on I2C bus {I2C_BUS} ({error}). Check its "
            "wiring and power: SDA to pin 3, SCL to pin 5, VCC to 3.3 V, GND to GND")


def main():
    ap = argparse.ArgumentParser(description="Spin each motor forward, then backward, to check the wiring.")
    ap.add_argument("--speed", type=float, default=0.5, help="how fast, 0..1 (default 0.5)")
    args = ap.parse_args()
    signal.signal(signal.SIGTERM, signal.default_int_handler)  # stopping it with kill also turns the motors off

    try:
        motors = Motors()
    except OSError as e:
        sys.exit(f"Can't drive the motors: {explain(e)}")
    print(f"Spinning each motor at {args.speed:.0%}, forward then backward, {TEST_S:g} s each way. "
          "Ctrl+C stops.", flush=True)
    try:
        for name, (forward, backward) in motors.motors.items():
            for direction, speed in (("forward", args.speed), ("backward", -args.speed)):
                print(f"{name:<11} {direction:<9} (outputs: forward {forward}, backward {backward})", flush=True)
                motors.set_motor(name, speed)
                time.sleep(TEST_S)
                motors.set_motor(name, 0)
                time.sleep(TEST_S / 2)
        print("Each motor should have turned the wheel it's named after, forward and then backward. Fix any "
              "that didn't in LEFT_MOTORS / RIGHT_MOTORS at the top of motors.py.")
    except KeyboardInterrupt:
        print()
    finally:
        motors.close()


if __name__ == "__main__":
    main()
