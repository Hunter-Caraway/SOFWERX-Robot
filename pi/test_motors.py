"""Tests for motors.py and the motor side of receive.py: the PCA9685's registers, each motor's L298N
inputs and how the watch's commands turn into wheel speeds, with a fake PCA9685 (no hardware needed).

    python3 -m unittest pi/test_motors.py
"""

import io
import sys
import unittest
from contextlib import redirect_stdout
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import receive  # noqa: E402
from motors import (ALL_LED_OFF_H, FULL, LED0_ON_L, LEFT_MOTORS, MIN_DUTY, MODE1, PRESCALE, PWM_HZ,  # noqa: E402
                    RIGHT_MOTORS, SLEEP, Busy, Motors, PCA9685, duty, explain)

FRONT_LEFT, REAR_RIGHT = LEFT_MOTORS["front left"], RIGHT_MOTORS["rear right"]  # (forward, backward) outputs


class RecordingChip(PCA9685):
    """A PCA9685 that records the bytes it would send instead of opening the I2C bus."""

    def __init__(self):
        self.sent = []
        self._start(PWM_HZ)

    def _write(self, register, *data):
        self.sent.append((register, *data))


class PCA9685Test(unittest.TestCase):

    def setUp(self):
        self.chip = RecordingChip()

    def test_turns_everything_off_before_anything_else(self):
        self.assertEqual((ALL_LED_OFF_H, FULL), self.chip.sent[0])

    def test_sets_the_frequency_while_asleep(self):
        sent = self.chip.sent
        sleep = next(i for i, (reg, *data) in enumerate(sent) if reg == MODE1 and data[0] & SLEEP)
        self.assertIn((PRESCALE, 5), sent[sleep:])   # 25 MHz / (4096 * 6) = 1017 Hz
        self.assertFalse(sent[-1][1] & SLEEP)         # and wakes up again

    def test_output_levels(self):
        self.chip.sent.clear()
        self.chip.set(3, 0.5)
        self.chip.set(3, 1.0)
        self.chip.set(3, 0.0)
        reg = LED0_ON_L + 4 * 3
        self.assertEqual([(reg, 0, 0, 0x00, 0x08),   # off at count 2048 of 4096
                          (reg, 0, FULL, 0, 0),       # always on
                          (reg, 0, 0, 0, FULL)],      # always off
                         self.chip.sent)


class FakePWM:
    """Stands in for the PCA9685: remembers each output's level, and can be made to stop answering."""

    def __init__(self):
        self.levels = {}
        self.writes = 0
        self.answering = True

    def set(self, channel, level):
        if not self.answering:
            raise OSError(121, "Remote I/O error")
        self.levels[channel] = level
        self.writes += 1

    def close(self):
        self.levels = dict.fromkeys(self.levels, 0.0)


class MotorsTest(unittest.TestCase):

    def setUp(self):
        self.pwm = FakePWM()
        self.motors = Motors(self.pwm)

    def outputs(self, motor):
        return [self.pwm.levels.get(ch) for ch in motor]

    def test_uses_all_eight_outputs_once(self):
        outputs = [ch for motor in (*LEFT_MOTORS.values(), *RIGHT_MOTORS.values()) for ch in motor]
        self.assertEqual(list(range(8)), sorted(outputs))

    def test_forward(self):
        self.motors.drive(0.5, 0.5)
        self.assertEqual([duty(0.5), 0.0], self.outputs(FRONT_LEFT))
        self.assertEqual([duty(0.5), 0.0], self.outputs(REAR_RIGHT))

    def test_reverse(self):
        self.motors.drive(-0.5, -0.5)
        self.assertEqual([0.0, duty(0.5)], self.outputs(FRONT_LEFT))

    def test_each_side_separately(self):
        self.motors.drive(-1.0, 1.0)
        for motor in LEFT_MOTORS.values():
            self.assertEqual([0.0, 1.0], self.outputs(motor))
        for motor in RIGHT_MOTORS.values():
            self.assertEqual([1.0, 0.0], self.outputs(motor))

    def test_stop_brakes(self):
        self.motors.drive(0.5, 0.5)
        self.motors.drive(0.0, 0.0)
        self.assertEqual(dict.fromkeys(range(8), 0.0), self.pwm.levels)  # both IN pins low

    def test_any_speed_gets_enough_power_to_move(self):
        self.assertEqual(0.0, duty(0.0))
        self.assertAlmostEqual(MIN_DUTY, duty(0.001), places=2)
        self.assertEqual(1.0, duty(-1.0))

    def test_only_writes_what_changed(self):
        self.motors.drive(0.5, 0.5)
        writes = self.pwm.writes
        self.motors.drive(0.5, 0.5)
        self.assertEqual(writes, self.pwm.writes)

    def test_retries_what_did_not_get_set(self):
        self.motors.drive(0.5, 0.5)
        self.pwm.answering = False
        with self.assertRaises(OSError):
            self.motors.drive(0.0, 0.0)
        self.pwm.answering = True
        self.motors.drive(0.0, 0.0)
        self.assertEqual(dict.fromkeys(range(8), 0.0), self.pwm.levels)


class ExplainTest(unittest.TestCase):

    def test_tells_another_program_from_a_bus_problem(self):
        self.assertIn("another program", explain(Busy("another program has /dev/i2c-1")))
        lost = BlockingIOError(11, "Resource temporarily unavailable")  # what the I2C bus gives when it's in trouble
        self.assertIn("no answer from the PCA9685", explain(lost))


class WheelSpeedsTest(unittest.TestCase):

    def setUp(self):
        reversed_before = receive.REVERSED
        receive.REVERSED = False
        self.addCleanup(setattr, receive, "REVERSED", reversed_before)

    def check(self, cmd, speed, turn, left, right):
        got = receive.wheel_speeds(cmd, speed, turn)
        self.assertAlmostEqual(left, got[0], msg=f"{cmd} left")
        self.assertAlmostEqual(right, got[1], msg=f"{cmd} right")

    def test_straight(self):
        self.check("STOP", 0, 0, 0, 0)
        self.check("FORWARD", 0.6, 0, 0.6, 0.6)
        self.check("REVERSE", 0.6, 0, -0.6, -0.6)

    def test_spins_on_the_spot(self):
        self.check("LEFT", 0, -1, -1, 1)
        self.check("RIGHT", 0, 0.5, 0.5, -0.5)

    def test_curves_by_slowing_the_inside(self):
        self.check("FORWARD-LEFT", 1, -0.4, 0.6, 1)
        self.check("FORWARD-RIGHT", 0.5, 1, 0.5, 0)
        self.check("REVERSE-LEFT", 1, -0.4, -0.6, -1)
        self.check("REVERSE-RIGHT", 1, 0.4, -1, -0.6)

    def test_no_speed_no_movement(self):
        self.check("FORWARD-LEFT", 0, -1, 0, 0)

    def test_reversed_swaps_forward_and_reverse_but_not_turns(self):
        receive.REVERSED = True
        self.check("FORWARD", 0.6, 0, -0.6, -0.6)
        self.check("REVERSE", 0.6, 0, 0.6, 0.6)
        self.check("LEFT", 0, -1, -1, 1)               # still spins the same way
        self.check("FORWARD-LEFT", 1, -0.4, -1, -0.6)  # the motors' right side is now the inside of the turn
        self.check("REVERSE-RIGHT", 1, 0.4, 0.6, 1)
        self.check("STOP", 0, 0, 0, 0)


class ReceiverMotorsTest(unittest.TestCase):

    def setUp(self):
        self.pwm = FakePWM()
        receive.motors, receive.motor_error = Motors(self.pwm), None
        self.out = io.StringIO()

    def tearDown(self):
        receive.motors, receive.motor_error = None, None

    def send(self, *command):
        with redirect_stdout(self.out):
            receive.send_command(*command)

    def test_prints_and_drives(self):
        self.send("FORWARD", 0.5, 0.0)
        self.assertIn("FORWARD        speed  50%", self.out.getvalue())
        pin = FRONT_LEFT[1] if receive.REVERSED else FRONT_LEFT[0]  # reversed, FORWARD runs it backward
        self.assertEqual(duty(0.5), self.pwm.levels[pin])

    def test_says_once_when_the_motors_stop_answering_and_when_they_are_back(self):
        self.send("FORWARD", 0.5, 0.0)
        self.pwm.answering = False
        self.send("STOP", 0.0, 0.0)
        with redirect_stdout(self.out):
            receive.drive_motors(*receive.STOP)  # main() retries like this
        self.assertIsNotNone(receive.motor_error)
        self.assertEqual(1, self.out.getvalue().count("Can't set the motors"))
        self.pwm.answering = True
        with redirect_stdout(self.out):
            receive.drive_motors(*receive.STOP)
        self.assertIsNone(receive.motor_error)
        self.assertIn("Motors answering again", self.out.getvalue())
        self.assertEqual(dict.fromkeys(range(8), 0.0), self.pwm.levels)  # braking


if __name__ == "__main__":
    unittest.main()
