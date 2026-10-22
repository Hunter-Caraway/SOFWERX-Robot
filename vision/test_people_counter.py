"""Tests for people_counter.py: the count smoothing, watch control, sharing the count and follow-mode commands, and
the HuskyLens 2 protocol, with a fake HuskyLens on a pseudo-terminal (no hardware needed).

    python3 -m unittest vision/test_people_counter.py
"""

import json
import os
import socket
import struct
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from follower import Box  # noqa: E402
from people_counter import (ALGORITHM_OBJECTS, CMD_GET_RESULT, CMD_KNOCK, CMD_SET_ALGORITHM,  # noqa: E402
                            FALL_S, PEOPLE_SHARE_INTERVAL_S, RETURN_ARGS, RETURN_BLOCKS, RETURN_INFO, RISE_S,
                            CountFilter, FollowShare, HuskyLens, PeopleShare, WatchControl, connect, frame,
                            load_calibration, save_calibration)

FPS = 15  # new results per second from the HuskyLens
FRAME = 1 / FPS


class CountFilterTest(unittest.TestCase):

    def setUp(self):
        self.filter = CountFilter(RISE_S, FALL_S)
        self.t = 0.0
        self.reported = []

    def feed(self, seconds, count):
        """Feeds the same per-result count for a while. Returns the smoothed count afterwards."""
        for _ in range(round(seconds * FPS)):
            changed = self.filter.update(self.t, count)
            if changed is not None:
                self.reported.append(changed)
            self.t += FRAME
        return self.filter.count

    def test_first_count_after_rise_time(self):
        self.feed(RISE_S - FRAME, 2)
        self.assertIsNone(self.filter.count)
        self.assertEqual(2, self.feed(3 * FRAME, 2))

    def test_brief_extra_detection_is_ignored(self):
        self.feed(2, 2)
        self.feed(RISE_S - FRAME, 3)  # not quite RISE_S
        self.assertEqual(2, self.feed(2, 2))
        self.assertEqual([2], self.reported)

    def test_new_person_counted_after_rise_time(self):
        self.feed(2, 2)
        self.assertEqual(3, self.feed(RISE_S + 2 * FRAME, 3))

    def test_briefly_missed_person_keeps_counting(self):
        self.feed(2, 3)
        self.feed(FALL_S - FRAME, 2)  # not quite FALL_S
        self.assertEqual(3, self.feed(1, 3))
        self.assertEqual([3], self.reported)

    def test_flickering_detection_keeps_counting(self):
        self.feed(2, 3)
        for _ in range(10):  # the third person is found only some of the time
            self.feed(FALL_S / 2, 2)
            self.feed(FRAME, 3)
        self.assertEqual(3, self.filter.count)

    def test_person_leaving_drops_after_fall_time(self):
        self.feed(2, 3)
        self.assertEqual(3, self.feed(FALL_S - FRAME, 2))
        self.assertEqual(2, self.feed(3 * FRAME, 2))

    def test_drop_goes_to_the_highest_count_in_the_window(self):
        self.feed(2, 4)
        self.feed(FALL_S / 2, 2)
        self.feed(FALL_S / 2 + 2 * FRAME, 1)
        self.assertEqual(2, self.filter.count)  # 2 was seen within the last FALL_S
        self.assertEqual(1, self.feed(FALL_S, 1))


class FakeHuskyLens:
    """Answers like a HuskyLens 2 on the other end of a pseudo-terminal."""

    def __init__(self):
        self.master, self.slave = os.openpty()
        self.port = os.ttyname(self.slave)
        self.people = 0
        self.objects = []         # what else Object Recognition finds
        self.box = (320, 240, 100, 300)  # where each result is: x and y of its middle, width, height
        self.algorithm = None     # the algorithm it's running
        self.silent = False
        self.noise = b""  # sent before each answer
        self.received = []
        self.running = True
        self.thread = threading.Thread(target=self._serve, daemon=True)
        self.thread.start()

    def close(self):
        self.running = False
        self.thread.join(timeout=1)
        os.close(self.master)
        os.close(self.slave)

    def _serve(self):
        import select
        buf = bytearray()
        while self.running:
            if not select.select([self.master], [], [], 0.05)[0]:
                continue
            buf += os.read(self.master, 1024)
            while len(buf) >= 6 and len(buf) >= 6 + buf[4]:
                n = 6 + buf[4]
                command, algorithm, data = buf[2], buf[3], bytes(buf[5:n - 1])
                assert buf[:2] == b"\x55\xaa" and sum(buf[:n - 1]) & 0xFF == buf[n - 1], buf[:n].hex()
                del buf[:n]
                self.received.append(command)
                if not self.silent:
                    os.write(self.master, self.noise + self._answer(command, algorithm, data))

    def _answer(self, command, algorithm, data):
        ok = frame(RETURN_ARGS, 0, bytes([0, 0]) + bytes(8))
        if command == CMD_KNOCK:
            return ok
        if command == CMD_SET_ALGORITHM:
            self.algorithm = data[0]
            return ok
        if command == CMD_GET_RESULT:
            names = ["Person"] * self.people + self.objects if algorithm == self.algorithm == ALGORITHM_OBJECTS else []
            n = len(names)
            info = frame(RETURN_INFO, algorithm, bytes([0, 0]) + struct.pack("<4H", n, 0, n, 0))
            return info + b"".join(
                frame(RETURN_BLOCKS, algorithm, bytes([0, algorithm]) + struct.pack("<4h", *self.box)
                      + bytes([len(name)]) + name.encode() + bytes([0]))
                for name in names)
        raise AssertionError(f"unexpected command {command:#x}")


class HuskyLensTest(unittest.TestCase):

    def setUp(self):
        self.fake = FakeHuskyLens()
        self.lens = HuskyLens(self.fake.port, 115200)

    def tearDown(self):
        self.lens.close()
        self.fake.close()

    def test_frame_checksum_is_the_low_byte_of_the_sum(self):
        self.assertEqual(bytes([0x55, 0xAA, 0x01, 0x02, 0x00, 0x02]), frame(CMD_GET_RESULT, ALGORITHM_OBJECTS))

    def test_finds_the_people_it_sees(self):
        self.assertTrue(self.lens.knock())
        self.assertTrue(self.lens.set_algorithm(ALGORITHM_OBJECTS))
        self.assertEqual(ALGORITHM_OBJECTS, self.fake.algorithm)
        self.fake.objects = ["Chair"]
        for people in (0, 3, 1, 0):
            self.fake.people = people
            self.assertEqual(["person"] * people + ["chair"],
                             [name for name, _ in self.lens.results(ALGORITHM_OBJECTS)])

    def test_finds_where_they_are(self):
        self.lens.set_algorithm(ALGORITHM_OBJECTS)
        self.fake.people = 1
        self.fake.box = (50, 300, 120, 360)
        self.assertEqual([("person", Box(50, 300, 120, 360))], self.lens.results(ALGORITHM_OBJECTS))

    def test_connects_to_object_recognition(self):
        connect(self.lens, self.fake.port, 115200)
        self.assertEqual(ALGORITHM_OBJECTS, self.fake.algorithm)
        self.assertEqual([CMD_KNOCK, CMD_SET_ALGORITHM], self.fake.received)

    def test_skips_noise_and_damaged_frames(self):
        self.lens.set_algorithm(ALGORITHM_OBJECTS)
        self.fake.people = 2
        damaged = bytearray(frame(RETURN_INFO, ALGORITHM_OBJECTS, bytes([0, 0]) + struct.pack("<4H", 9, 0, 9, 0)))
        damaged[-1] ^= 0xFF
        self.fake.noise = b"\x00\x55\x12\x55" + bytes(damaged)
        for _ in range(2):
            self.assertEqual(["person", "person"], [name for name, _ in self.lens.results(ALGORITHM_OBJECTS)])

    def test_no_answer_gives_none(self):
        self.fake.silent = True
        start = time.monotonic()
        self.assertFalse(self.lens.knock())
        self.assertIsNone(self.lens.results(ALGORITHM_OBJECTS))
        self.assertLess(time.monotonic() - start, 3)


class PeopleShareTest(unittest.TestCase):

    def setUp(self):
        self.inbox = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.inbox.bind(("127.0.0.1", 0))  # any free port
        self.inbox.settimeout(0.2)
        self.people = PeopleShare(self.inbox.getsockname()[1])

    def tearDown(self):
        self.people.sock.close()
        self.inbox.close()

    def heard(self):
        """Every message waiting in the receiver's inbox."""
        msgs = []
        try:
            while True:
                msgs.append(json.loads(self.inbox.recv(512)))
        except socket.timeout:
            return msgs

    def test_changes_go_at_once(self):
        for count in (None, 0, 0, 2, None):
            self.people.share(count)
        self.assertEqual([{"people": None}, {"people": 0}, {"people": 2}, {"people": None}], self.heard())

    def test_repeats_the_count(self):
        self.people.share(1)
        time.sleep(PEOPLE_SHARE_INTERVAL_S + 0.05)
        self.people.share(1)
        self.assertEqual([{"people": 1}] * 2, self.heard())


class FollowShareTest(unittest.TestCase):

    def test_sends_every_command(self):
        inbox = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        inbox.bind(("127.0.0.1", 0))  # any free port
        inbox.settimeout(0.2)
        follow = FollowShare(inbox.getsockname()[1])
        try:
            follow.share(("FORWARD-LEFT", 0.5, -0.3), True)
            follow.share(("RIGHT", 0.0, 0.3), True, 812.25)
            self.assertEqual({"cmd": "FORWARD-LEFT", "speed": 0.5, "turn": -0.3, "target": True, "until": None},
                             json.loads(inbox.recv(512)))
            self.assertEqual({"cmd": "RIGHT", "speed": 0.0, "turn": 0.3, "target": True, "until": 812.25},
                             json.loads(inbox.recv(512)))
        finally:
            follow.sock.close()
            inbox.close()


class CalibrationFileTest(unittest.TestCase):

    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.path = os.path.join(self.folder.name, "follow_calibration.json")

    def tearDown(self):
        self.folder.cleanup()

    def test_keeps_what_follow_mode_measured(self):
        save_calibration(self.path, (-1196.4, 0.412, 0.031))
        self.assertEqual((-1196.4, 0.412, 0.031), load_calibration(self.path))

    def test_starts_afresh_without_a_good_file(self):
        self.assertIsNone(load_calibration(self.path))  # none yet
        for text in ("not json", '{"rate": 1200}', '{"rate": 0, "lag": 0.4, "overrun": 0}'):
            with open(self.path, "w") as f:
                f.write(text)
            self.assertIsNone(load_calibration(self.path), text)


class WatchControlTest(unittest.TestCase):

    def setUp(self):
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.bind(("127.0.0.1", 0))  # any free port
        self.port = probe.getsockname()[1]
        probe.close()
        self.control = WatchControl(self.port, timeout_s=0.3)
        self.out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def tearDown(self):
        self.control.sock.close()
        self.out.close()

    def say(self, msg):
        self.out.sendto(msg, ("127.0.0.1", self.port))
        time.sleep(0.02)

    def test_off_until_told_on(self):
        self.assertFalse(self.control.is_on())
        self.say(b"off")
        self.assertFalse(self.control.is_on())
        self.say(b"on")
        self.assertTrue(self.control.is_on())
        self.say(b"off")
        self.assertFalse(self.control.is_on())

    def test_follow_mode_is_on(self):
        self.say(b"follow")
        self.assertEqual("follow", self.control.mode())
        self.assertTrue(self.control.is_on())
        self.say(b"on")
        self.assertEqual("on", self.control.mode())
        self.say(b"off")
        self.assertEqual("off", self.control.mode())

    def test_off_when_the_receiver_goes_quiet(self):
        self.say(b"on")
        self.assertTrue(self.control.is_on())
        time.sleep(0.4)
        self.assertFalse(self.control.is_on())
        self.say(b"on")
        self.assertTrue(self.control.is_on())


if __name__ == "__main__":
    unittest.main()
