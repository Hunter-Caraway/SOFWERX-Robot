"""Tests for the packets receive.py reads: the watch's drive commands, and the people
counter's count and follow-mode commands; and for the gears (no hardware needed).

    python3 -m unittest pi/test_receive.py
"""

import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from receive import STOP, geared, parse, parse_follow, parse_people  # noqa: E402


def packet(**fields):
    pkt = {"on": True, "cmd": "FORWARD", "speed": 0.5, "turn": 0.0}
    pkt.update(fields)
    return json.dumps(pkt).encode()


class ParseTest(unittest.TestCase):

    def test_reads_the_command(self):
        self.assertEqual((True, False, 0.65, ("FORWARD", 0.5, 0.0)), parse(packet(top=0.65)))
        self.assertEqual((True, True, 1.0, ("FORWARD", 0.5, 0.0)), parse(packet(follow=True, top=1)))

    def test_older_apps_have_no_gears(self):
        self.assertEqual((True, False, 1.0, ("FORWARD", 0.5, 0.0)), parse(packet()))
        self.assertEqual((True, False, 1.0, ("FORWARD", 0.5, 0.0)), parse(packet(door=2, grav=[0, 0, 9.8])))

    def test_off_is_stop(self):
        self.assertEqual((False, True, 0.35, STOP), parse(packet(on=False, follow=True, top=0.35)))


class GearedTest(unittest.TestCase):

    def test_scales_the_speed(self):
        self.assertEqual(("FORWARD", 0.35, 0.0), geared(("FORWARD", 1.0, 0.0), 0.35))
        self.assertEqual(("REVERSE-LEFT", 0.3, -0.4), geared(("REVERSE-LEFT", 0.6, -0.4), 0.5))  # curves the same
        self.assertEqual(("FORWARD", 0.8, 0.0), geared(("FORWARD", 0.8, 0.0), 1.0))

    def test_scales_spinning_on_the_spot(self):
        self.assertEqual(("RIGHT", 0.0, 0.33), geared(("RIGHT", 0.0, 0.5), 0.65))
        self.assertEqual(("LEFT", 0.0, -0.35), geared(("LEFT", 0.0, -1.0), 0.35))
        self.assertEqual(STOP, geared(STOP, 0.35))


class ParseFollowTest(unittest.TestCase):

    def test_reads_the_command(self):
        self.assertEqual((("FORWARD-LEFT", 0.5, -0.3), True, None),
                         parse_follow(b'{"cmd":"FORWARD-LEFT","speed":0.5,"turn":-0.3,"target":true,"until":null}'))
        self.assertEqual((("RIGHT", 0.0, 0.3), True, 812.25),
                         parse_follow(b'{"cmd":"RIGHT","speed":0,"turn":0.3,"target":true,"until":812.25}'))
        self.assertEqual((STOP, False, None), parse_follow(b'{"cmd":"STOP","speed":0,"turn":0,"target":false}'))

    def test_rejects_anything_else(self):
        for data in (b"on", b'{"people": 2}', b'{"cmd":"JUMP","speed":0,"turn":0,"target":true}',
                     b'{"cmd":"STOP","speed":0,"turn":0,"target":null}', b'{"cmd":"STOP","speed":0,"turn":0}',
                     b'{"cmd":"RIGHT","speed":0,"turn":0.3,"target":true,"until":"soon"}'):
            with self.assertRaises((ValueError, KeyError, TypeError)):
                parse_follow(data)


class ParsePeopleTest(unittest.TestCase):

    def test_reads_the_count(self):
        self.assertEqual({"people": 3}, parse_people(b'{"people":3}'))
        self.assertEqual({"people": 0}, parse_people(b'{"people": 0}'))
        self.assertEqual({"people": None}, parse_people(b'{"people":null}'))

    def test_rejects_anything_else(self):
        for data in (b"2", b"none", b'{"room": "inside"}', b'{"people": -1}', b'{"people": 1.5}',
                     b'{"people": true}', b'{"people": "2"}', b"\xff"):
            with self.assertRaises(ValueError):
                parse_people(data)


if __name__ == "__main__":
    unittest.main()
