"""Tests for follower.py: how follow-me mode tracks people, whom it follows, and how it steers, including in a simulated
room with people walking about and a robot whose camera lags behind it and misses people (no hardware needed).

    python3 -m unittest vision/test_follower.py
"""

import bisect
import math
import random
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from follower import (CENTER, FRAME_H, FRAME_W, KEEP, LOST_S, MAX_SPEED, MIDDLE_PX, SLOW_SPIN,  # noqa: E402
                      SPIN, STOP, UNPAIRED, Axis, Box, Follower, Tracker, assign, overlap)

FULL_TURN = 3300                      # pixels of the picture in a whole turn: about 640 for every 70 degrees
PX_PER_RAD = FULL_TURN / (2 * math.pi)
PERSON_W, PERSON_H = 0.5, 1.7         # a person's width (arms and all) and height, in metres
TRACK_M = 0.3                         # how far apart the robot's wheels are, in effect (they skid as it turns)


def locked_on(logs):
    return any(line.startswith("Locked on") for line in logs)


# --- where people in the simulated room are at time t, in metres, or None while they're out of it ---

def standing(x, y):
    return lambda t: (x, y)


def walking(*waypoints, speed=0.7, start=0.0):
    """Stands at the first waypoint until start, then walks from one to the next at speed (m/s), and stays at the
    last."""
    def at(t):
        left = max(0.0, t - start) * speed
        for (x0, y0), (x1, y1) in zip(waypoints, waypoints[1:]):
            leg = math.hypot(x1 - x0, y1 - y0)
            if left <= leg:
                k = left / leg if leg else 1.0
                return x0 + k * (x1 - x0), y0 + k * (y1 - y0)
            left -= leg
        return waypoints[-1]
    return at


def away(path, start, end=math.inf):
    """Someone out of the room, or hidden, from start until end."""
    return lambda t: None if start <= t < end else path(t)


class Room:
    """A simulated robot in a room with people in it, and a HuskyLens on the robot whose results lag behind.

    The robot starts at (0, 0) facing along +y. Turning on the spot at SPIN moves the picture rate px a second, and it
    keeps turning for coast seconds after each turn ends; driving at speed 1 goes 1 m/s, and its wheels are track
    metres apart, in effect. The HuskyLens has new results fps times a second, each latency seconds old, and finds each
    person in view with probability detect, except while blind (a list of (start, end) times), or while the picture
    moves faster than blur px a second as the robot turns, when it finds nobody. Boxes jitter a little, and are cut off
    at the edges of the picture.
    """

    def __init__(self, *paths, rate=1200.0, latency=0.3, fps=9, detect=1.0, blind=(), blur=math.inf, mirrored=False,
                 rate_guess=None, lag_guess=None, seed=1, track=TRACK_M, coast=0.0):
        self.paths = list(paths)
        self.rate, self.latency, self.fps, self.detect, self.blind = rate, latency, fps, detect, blind
        self.blur = blur
        self.mirrored, self.seed, self.track, self.coast = mirrored, seed, track, coast
        self.spin = None          # (until, direction) of the latest turn on the spot, for coasting after it
        self.times, self.poses = [0.0], [(0.0, 0.0, math.pi / 2)]  # (x, y, heading), heading anticlockwise from +x
        self.t = 0.0
        self.frames = {}
        self.logs = []
        self.follower = Follower(rate_guess, lag_guess, log=self.logs.append)
        self.drives = []          # (time, drive) of each update
        self.turns = []           # (direction, until) of each turn on the spot, in order
        self.views = []           # (time, [x in the picture now of each person, or None if behind]) of each update

    def pose_at(self, t):
        return self.poses[max(0, bisect.bisect_right(self.times, t) - 1)]

    def where(self, i, t, pose=None):
        """Where person i is across a picture taken at time t, from the robot's pose then (None if out of the room or
        behind the robot), and how far away they are."""
        spot = self.paths[i](t)
        if spot is None:
            return None, None
        x, y, heading = pose or self.pose_at(t)
        dx, dy = spot[0] - x, spot[1] - y
        angle = (math.atan2(dy, dx) - heading + math.pi) % (2 * math.pi) - math.pi  # anticlockwise: to the left
        if abs(angle) >= math.pi / 2:
            return None, math.hypot(dx, dy)
        px = angle * PX_PER_RAD
        return (CENTER + px if self.mirrored else CENTER - px), math.hypot(dx, dy)

    def frame(self, t):
        """The HuskyLens's latest result at time t: the same list until it has a new one."""
        n = math.floor((t - self.latency) * self.fps)
        if n < 0:
            return []
        if n not in self.frames:
            taken = n / self.fps
            boxes = []
            if not any(start <= taken < end for start, end in self.blind) and not self._blurred(taken):
                for i in range(len(self.paths)):
                    box = self._box(i, taken, random.Random(hash((self.seed, n, i))))
                    if box is not None:
                        boxes.append(box)
            self.frames[n] = boxes
        return self.frames[n]

    def _blurred(self, taken):
        """Whether the robot was turning fast enough when a picture was taken to blur it."""
        before, now = self.pose_at(taken - 0.06)[2], self.pose_at(taken)[2]
        turned = abs((now - before + math.pi) % (2 * math.pi) - math.pi)
        return turned / 0.06 * PX_PER_RAD > self.blur

    def _box(self, i, taken, luck):
        x, distance = self.where(i, taken)
        if x is None or distance < 0.2 or luck.random() >= self.detect:
            return None
        w = PX_PER_RAD * PERSON_W / distance * (1 + luck.gauss(0, 0.05))
        h = min(FRAME_H, PX_PER_RAD * PERSON_H / distance)  # up close, it sees only their legs
        x += luck.gauss(0, 4)
        left, right = max(0.0, x - w / 2), min(FRAME_W, x + w / 2)
        if right - left < 0.4 * w:
            return None  # mostly out of the picture
        return Box(round((left + right) / 2), round(FRAME_H - h / 2), round(right - left), round(h))

    def run(self, seconds, dt=0.03):
        end = self.t + seconds
        while self.t < end:
            t = self.t
            drive, _, until = self.follower.update(t, self.frame(t))
            self.drives.append((t, drive))
            self.views.append((t, [self.where(i, t)[0] for i in range(len(self.paths))]))
            if drive[0] in ("LEFT", "RIGHT") and (not self.turns or self.turns[-1][1] != until):
                self.turns.append((drive[0], until))
            self.t = t + dt
            self.times.append(self.t)
            self.poses.append(self._moved(drive, until, t, dt))

    def _moved(self, drive, until, t, dt):
        """The robot's pose dt after t, driving as pi/receive.py does: turns on the spot stop at until, and with a turn
        while driving forward, the inside of the turn slows."""
        cmd, speed, turn = drive
        x, y, heading = self.poses[-1]
        if cmd in ("LEFT", "RIGHT"):
            spun = dt if until is None else max(0.0, min(dt, until - t))
            heading -= spun * self.rate / PX_PER_RAD * turn / SPIN  # turning right is clockwise
            self.spin = (t + dt if until is None else until, turn / SPIN)
        elif self.spin is not None and t < self.spin[0] + self.coast:
            coasting = max(0.0, min(t + dt, self.spin[0] + self.coast) - max(t, self.spin[0]))
            heading -= coasting * self.rate / PX_PER_RAD * self.spin[1]
        if cmd.startswith("FORWARD"):
            left, right = speed * (1 + min(turn, 0.0)), speed * (1 - max(turn, 0.0))
            heading += (right - left) / self.track * dt
            x += (left + right) / 2 * math.cos(heading) * dt
            y += (left + right) / 2 * math.sin(heading) * dt
        return x, y, heading

    def offset(self, i=0):
        """Where person i is in the picture now, from the middle (None if behind the robot)."""
        x, _ = self.where(i, self.t)
        return None if x is None else x - CENTER

    def distance(self, i=0, t=None):
        t = self.t if t is None else t
        x, y, _ = self.pose_at(t)
        spot = self.paths[i](t)
        return math.hypot(spot[0] - x, spot[1] - y)

    def in_view(self, i=0, since=0.0):
        """The share of the time since then that person i was in the picture."""
        xs = [xs[i] for t, xs in self.views if t >= since]
        return sum(x is not None and 0 <= x <= FRAME_W for x in xs) / len(xs)

    def reversals(self):
        """How many times it turned on the spot back the other way."""
        directions = [d for d, _ in self.turns]
        return sum(a != b for a, b in zip(directions, directions[1:]))

    def following(self, i):
        """Whether its target's latest box is person i's."""
        target = self.follower.target
        if target is None:
            return False
        taken = math.floor((target.seen - self.latency) * self.fps) / self.fps  # when that result's picture was
        x, _ = self.where(i, taken)
        return x is not None and abs(x - target.box.x) < 40


AT_KEEP = round(KEEP * FRAME_W)  # how wide someone looks at the distance it follows at


def person(x, w=80, y=None):
    """A box for someone with the middle of it at x, w wide and three times as tall."""
    return Box(round(x), 240 if y is None else y, w, 3 * w)


# --- tracking ---

class TrackingTest(unittest.TestCase):

    def test_works_out_how_fast_something_moves(self):
        axis = Axis(100.0, 15.0, 300.0)
        for i in range(1, 30):
            axis.predict(0.1, 800.0)
            axis.correct(100.0 + 20.0 * i, 15.0)  # 200 px/s
        self.assertAlmostEqual(200, axis.rate, delta=40)

    def test_pairs_boxes_up_with_the_least_misfit_overall(self):
        # The nearest pair (track 1, box 0) would leave box 1 with nobody; pairing each with the other does better.
        self.assertEqual({0: 0, 1: 1}, assign({(0, 0): 4.0, (1, 0): 1.0, (1, 1): 4.0}, 2))
        self.assertEqual({}, assign({}, 3))
        self.assertGreater(UNPAIRED, 8.0)

    def test_boxes_go_to_whoever_expected_them(self):
        tracker, t = Tracker(), 0.0
        for i in range(5):
            tracker.predict(t)
            tracker.correct(t, [person(200 + 10 * i), person(420 - 10 * i)])
            t += 0.1
        a, b = tracker.tracks
        tracker.predict(t)
        tracker.correct(t, [person(370), person(250)])  # listed the other way round
        self.assertEqual((250, 370), (a.box.x, b.box.x))
        self.assertEqual(2, len(tracker.tracks))

    def test_someone_missed_for_a_while_keeps_their_track(self):
        tracker, t = Tracker(), 0.0
        for _ in range(5):
            tracker.predict(t)
            tracker.correct(t, [person(300)])
            t += 0.1
        number = tracker.tracks[0].number
        t += 0.8  # missed
        tracker.predict(t)
        tracker.correct(t, [person(330)])
        self.assertEqual([number], [track.number for track in tracker.tracks])

    def test_someone_unseen_cannot_take_a_box_far_off(self):
        tracker = Tracker()
        tracker.predict(0.0)
        tracker.correct(0.0, [person(100)])
        tracker.predict(3.0)
        tracker.correct(3.0, [person(560)])
        self.assertEqual(2, len(tracker.tracks))

    def test_skips_boxes_of_people_in_front_of_each_other(self):
        self.assertAlmostEqual(1.0, overlap(person(300), person(300)))
        self.assertEqual(0.0, overlap(person(100), person(400)))
        tracker = Tracker()
        tracker.predict(0.0)
        tracker.correct(0.0, [person(300), person(310), person(500)])
        self.assertEqual([500], [track.box.x for track in tracker.tracks])

    def test_forgets_people_gone_a_while(self):
        tracker = Tracker()
        for i in range(4):
            tracker.predict(i * 0.1)
            tracker.correct(i * 0.1, [person(300 + i)])
        tracker.correct(0.4, [person(600)])  # someone seen once
        tracker.forget(1.0)
        self.assertEqual([303], [track.box.x for track in tracker.tracks])
        tracker.forget(3.0, keep=tracker.tracks[0])
        self.assertEqual(1, len(tracker.tracks))
        tracker.forget(3.0)
        self.assertEqual([], tracker.tracks)


# --- whom it follows, and the commands it gives (results given directly) ---

class FollowerTest(unittest.TestCase):

    def setUp(self):
        self.follower = Follower(rate=1200.0, lag=0.3)
        self.t = 0.0
        self.n = 0

    def see(self, *people, seconds=0.11):
        """A new result, with these people in it (nudged a pixel either way, so each result differs)."""
        self.t += seconds
        self.n += 1
        boxes = [box._replace(x=box.x + self.n % 2) for box in people]
        drive, following, _ = self.follower.update(self.t, boxes)
        return drive, following

    def lock_on(self, w=AT_KEEP):
        for _ in range(4):
            self.see(person(CENTER, w))
        self.assertIsNotNone(self.follower.keep)

    def test_picks_the_person_nearest_the_middle(self):
        for _ in range(4):
            drive, following = self.see(person(CENTER + 10, AT_KEEP), person(60, 120))  # someone else, off to the side
        self.assertTrue(following)
        self.assertAlmostEqual(CENTER + 10, self.follower.target.box.x, delta=1)
        self.assertEqual(STOP, drive)

    def test_waits_until_someone_has_been_seen_a_few_times(self):
        self.assertFalse(self.see(person(CENTER))[1])
        self.assertFalse(self.see(person(CENTER))[1])
        self.assertTrue(self.see(person(CENTER))[1])

    def test_closes_in_faster_the_farther_they_are(self):
        self.lock_on()
        for _ in range(3):
            near = self.see(person(CENTER, round(AT_KEEP * 0.85)))[0]
        self.assertEqual("FORWARD", near[0])
        self.assertLess(near[1], MAX_SPEED)
        for _ in range(6):
            far = self.see(person(CENTER, 30))[0]
        self.assertEqual(("FORWARD", MAX_SPEED, 0.0), far)

    def test_curves_toward_them(self):
        self.lock_on()
        for _ in range(4):
            cmd, speed, turn = self.see(person(CENTER - 100, 120))[0]
        self.assertEqual("FORWARD-LEFT", cmd)
        self.assertGreater(speed, 0)
        self.assertLess(turn, 0)

    def test_never_reverses(self):
        self.lock_on()
        for _ in range(4):
            drive = self.see(person(CENTER, round(AT_KEEP * 1.5)))[0]  # nearer than it follows at
        self.assertEqual(STOP, drive)

    def test_follows_at_the_same_distance_whatever_the_distance_at_lock_on(self):
        for w in (500, 20):  # right up close, far off
            self.setUp()
            for _ in range(4):
                self.see(person(CENTER, w))
            self.assertEqual(KEEP, self.follower.keep)

    def test_turns_on_the_spot_toward_someone_near_but_off_the_middle(self):
        self.lock_on()
        for _ in range(3):
            drive = self.see(person(CENTER + 250, AT_KEEP))[0]
            if drive[0] != "STOP":
                break
        self.assertEqual("RIGHT", drive[0])

    def test_sticks_with_them_when_someone_bigger_comes_along(self):
        self.lock_on(80)
        for _ in range(5):
            self.see(person(CENTER + 10, 80), person(80, 300))
        self.assertAlmostEqual(CENTER + 10, self.follower.target.box.x, delta=1)

    def test_does_not_take_someone_else_while_they_are_missed(self):
        self.lock_on(80)
        for _ in range(3):
            self.see(person(CENTER, 80), person(520, 80))  # someone else to the right
        for _ in range(int(LOST_S / 0.11) + 5):
            self.see(person(520, 80))  # the HuskyLens misses them, but still sees the other person
        self.assertFalse(any(self.follower.target is not None and abs(self.follower.target.box.x - 520) < 5
                             for _ in [0]))
        self.assertNotIn("Found them again", " ".join(self.follower_logs()))

    def follower_logs(self):
        logs = []
        self.follower.log = logs.append
        return logs

    def test_waits_where_they_vanished(self):
        self.lock_on()
        for _ in range(int(LOST_S / 0.11) - 1):
            drive, following = self.see()
            self.assertEqual((STOP, True), (drive, following))  # (it doesn't drive on blind, so near them)
        self.assertEqual(STOP, self.see(person(CENTER + 15, AT_KEEP))[0])  # there they are again
        self.assertTrue(self.follower.visible)

    def test_lets_go_after_waiting_in_vain(self):
        self.lock_on()
        self.see(seconds=LOST_S + 0.1)
        drive, following = self.see()
        self.assertFalse(following)
        self.assertEqual(("RIGHT", 0.0, SLOW_SPIN), drive)  # then turns slowly to look for them


# --- in the simulated room ---

class CalibratingTest(unittest.TestCase):

    def test_turns_toward_someone_without_swinging_past(self):
        room = Room(standing(-0.7, 2.0), rate=600)
        room.run(8)
        self.assertLessEqual(abs(room.offset()), MIDDLE_PX + 10)
        self.assertEqual(0, room.reversals())
        self.assertTrue(locked_on(room.logs))

    def test_measures_how_fast_the_picture_moves(self):
        room = Room(standing(0.9, 2.0), rate=450)
        room.run(8)
        self.assertAlmostEqual(450, abs(room.follower.rate), delta=120)

    def test_settles_on_someone_when_it_turns_faster_than_it_guessed(self):
        room = Room(standing(-0.6, 2.0), rate=2500)
        room.run(10)
        self.assertLessEqual(abs(room.offset()), MIDDLE_PX + 10)
        self.assertLessEqual(room.reversals(), 3)

    def test_turns_the_other_way_with_a_mirrored_picture(self):
        import follower
        follower.MIRRORED = True
        try:
            room = Room(standing(-0.5, 2.0), mirrored=True)
            room.run(8)
        finally:
            follower.MIRRORED = False
        self.assertLessEqual(abs(room.offset()), MIDDLE_PX + 10)
        self.assertEqual(0, room.reversals())

    def test_looks_around_for_someone_out_of_view(self):
        room = Room(standing(2.0, -0.5))  # off to the right, behind
        room.run(15)
        self.assertEqual("RIGHT", room.turns[0][0])
        self.assertTrue(locked_on(room.logs))
        self.assertLessEqual(abs(room.offset()), 2 * MIDDLE_PX)

    def test_what_it_measured_carries_over(self):
        room = Room(standing(0.8, 2.0), rate=450, latency=0.25)
        room.run(8)
        calibration = room.follower.calibration()
        again = Follower(*calibration)
        self.assertEqual(calibration, again.calibration())
        self.assertAlmostEqual(0.3, room.follower.lag, delta=0.1)  # the camera's lag, and half a result's time
        self.assertIsNone(Follower().calibration())

    def test_works_out_how_long_it_keeps_turning_after_a_turn(self):
        follower = Follower()
        follower.spins = [(0.05, 1200 * 0.13), (0.13, 1200 * 0.21), (0.08, 1200 * 0.16)]  # (seconds, px)
        rate, overrun = follower._fit()
        self.assertAlmostEqual(1200, rate, delta=1)
        self.assertAlmostEqual(0.08, overrun, delta=0.001)
        follower.spins = [(0.05, 156), (0.05, 160)]  # all the same length: it can't tell
        self.assertIsNone(follower._fit())

    def test_settles_on_someone_who_keeps_turning_after_each_turn(self):
        room = Room(standing(-0.7, 2.0), rate=900, latency=0.3, coast=0.08)
        room.run(20)
        self.assertLessEqual(abs(room.offset()), MIDDLE_PX + 30)
        self.assertLessEqual(room.reversals(), 3)

    def test_learns_how_long_it_keeps_turning_from_turns_while_following(self):
        # From a calibration that missed it: short turns toward them show it, so it stops nudging either side of them.
        room = Room(standing(0.5, 2.0), rate=1800, latency=0.3, coast=0.08, rate_guess=2070, lag_guess=0.4)
        room.run(20)
        self.assertAlmostEqual(0.08, room.follower.overrun, delta=0.04)
        spins = [t > 12 and drive[0] in ("LEFT", "RIGHT") for t, drive in room.drives]
        self.assertLessEqual(sum(1 for before, now in zip(spins, spins[1:]) if now and not before), 3)


class FollowingTest(unittest.TestCase):

    def test_follows_closely_as_they_walk_away(self):
        room = Room(walking((0, 1.2), (0, 5.0), start=3.0), rate_guess=1200, lag_guess=0.35)
        room.run(3)
        self.assertTrue(locked_on(room.logs))
        start = room.distance()
        room.run(11)  # they walk off, and stop
        self.assertAlmostEqual(start, room.distance(), delta=0.25 * start)
        self.assertLess(max(room.distance(t=t) for t, _ in room.drives), 2.2 * start)

    def test_keeps_up_as_they_walk_around_a_corner(self):
        room = Room(walking((0, 1.2), (0, 3.0), (2.5, 3.5), (3.0, 6.0), start=3.0), rate_guess=1200, lag_guess=0.35)
        room.run(20)
        self.assertGreater(room.in_view(since=3.0), 0.9)
        self.assertLess(room.distance(), 2.0)
        self.assertTrue(room.following(0))

    def test_does_not_run_into_them_when_they_stop(self):
        room = Room(walking((0, 1.0), (0, 3.0), start=2.0, speed=0.9), rate_guess=1200, lag_guess=0.35)
        room.run(10)
        self.assertGreater(min(room.distance(t=t) for t, _ in room.drives), 0.6)

    def test_holds_steady_in_front_of_someone_standing_still(self):
        for rate, latency, guess in [(1200, 0.15, None), (1800, 0.15, None), (600, 0.45, None), (1200, 0.3, 1400)]:
            room = Room(standing(-0.8, 2.0), rate=rate, latency=latency, rate_guess=guess,
                        lag_guess=guess and latency + 0.1)
            room.run(20)
            late = [t for t, drive in room.drives if t > 10 and drive[0] in ("LEFT", "RIGHT")]
            self.assertLessEqual(room.reversals(), 3, (rate, latency))
            self.assertEqual([], late, (rate, latency))  # (it used to swing either side of them for good)
            self.assertLessEqual(abs(room.offset()), MIDDLE_PX + 30, (rate, latency))

    def test_never_reverses(self):
        room = Room(walking((0, 2.5), (0, 0.8), start=3.0), rate_guess=1200, lag_guess=0.35)
        room.run(10)
        self.assertFalse(any(drive[0].startswith("REVERSE") for _, drive in room.drives))

    def test_sticks_with_them_through_a_flaky_camera(self):
        for seed in range(4):
            room = Room(walking((0, 1.3), (0.5, 3.0), (-0.5, 5.0), start=3.0), detect=0.5,
                        blind=[(5.0, 5.6), (7.0, 7.4)], rate_guess=1200, lag_guess=0.35, seed=seed)
            room.run(16)
            self.assertGreater(room.in_view(since=3.0), 0.85, seed)
            self.assertLess(room.distance(), 2.0, seed)

    def test_waits_when_the_camera_misses_them_up_close(self):
        room = Room(standing(0.1, 0.9), blind=[(4.0, 5.5)], rate_guess=1200, lag_guess=0.35)
        room.run(4.2)
        turns = len(room.turns)
        room.run(1.5)
        self.assertEqual(turns, len(room.turns))
        room.run(2)
        self.assertTrue(room.following(0))

    def test_keeps_them_in_view_as_they_step_aside_up_close(self):
        for rate, latency in [(900, 0.15), (1200, 0.3)]:
            room = Room(walking((0, 0.75), (0.5, 0.75), start=3.0, speed=0.6), rate=rate, latency=latency,
                        rate_guess=rate * 1.15, lag_guess=latency + 0.1)
            room.run(8)
            self.assertGreater(room.in_view(since=3.0), 0.95, rate)
            self.assertTrue(room.following(0), rate)
            self.assertLessEqual(abs(room.offset()), MIDDLE_PX + 30, rate)
            self.assertGreater(min(room.distance(t=t) for t, _ in room.drives[100:]), 0.6, rate)

    def test_finds_them_past_the_edge_they_left_by_though_turning_fast_blurs_the_picture(self):
        # On the robot, the HuskyLens made nobody out while it turned at SPIN, and it spun round and round past them.
        for path in [((0, 0.8), (-1.2, 0.4), (-1.0, -0.6)), ((0, 0.8), (-1.2, 0.0))]:  # off to the left, and round
            room = Room(walking(*path, start=3.0, speed=1.2), rate=790, latency=0.25, blur=500, rate_guess=790,
                        lag_guess=0.25)
            room.run(16)
            self.assertTrue(room.following(0), path)
            self.assertLess(abs(room.follower.turned * room.follower.rate), 3300, path)  # (less than once round)

    def test_turns_after_them_when_they_dash_out_of_the_picture(self):
        for speed in (1.5, 2.5, 3.0):  # running off to the left, faster than it can turn
            room = Room(walking((0, 1.5), (-3.0, 1.5), start=3.0, speed=speed), latency=0.45, rate_guess=1200,
                        lag_guess=0.5)
            room.run(12)
            turns = [drive[0] for t, drive in room.drives if t > 3.0 and drive[0] in ("LEFT", "RIGHT")]
            self.assertEqual("LEFT", turns[0], speed)
            self.assertTrue(room.following(0), speed)
            self.assertLessEqual(abs(room.offset()), 2 * MIDDLE_PX, speed)


class WhoToFollowTest(unittest.TestCase):

    def test_picks_the_person_in_front_not_the_nearest(self):
        room = Room(standing(0.1, 2.0), standing(-1.0, 1.0), rate_guess=1200, lag_guess=0.35)
        room.run(3)
        self.assertTrue(room.following(0))

    def test_never_switches_to_someone_else_while_the_camera_misses_them(self):
        for seed in range(4):
            room = Room(walking((0, 1.5), (0.3, 4.0), start=3.0), standing(1.2, 1.6), detect=0.5,
                        rate_guess=1200, lag_guess=0.35, seed=seed)
            room.run(14)
            self.assertTrue(room.following(0), seed)
            self.assertLess(room.distance(0), 2.0, seed)

    def test_keeps_to_them_when_someone_walks_between(self):
        room = Room(walking((0, 1.5), (0, 3.5), start=3.0, speed=0.5),
                    walking((-2.0, 1.4), (2.0, 1.6), start=4.0, speed=1.0), rate_guess=1200, lag_guess=0.35)
        room.run(14)
        self.assertTrue(room.following(0))
        self.assertLess(room.distance(0), 2.0)

    def test_keeps_to_them_when_someone_walks_between_through_a_flaky_camera(self):
        for seed in range(4):  # the camera misses them as the other person comes in, and as they hide them
            room = Room(walking((0, 1.5), (0, 3.5), start=3.0, speed=0.5),
                        walking((-2.0, 1.4), (2.0, 1.6), start=4.0, speed=1.0), detect=0.6, rate_guess=1200,
                        lag_guess=0.35, seed=seed)
            room.run(14)
            self.assertTrue(room.following(0), seed)

    def test_finds_them_again_where_they_left_the_picture(self):
        room = Room(walking((0, 1.5), (-2.5, 2.0), start=3.0, speed=1.2), rate_guess=1200, lag_guess=0.35)
        room.run(12)
        self.assertTrue(room.following(0))
        self.assertLessEqual(abs(room.offset()), 120)

    def test_does_not_take_someone_who_was_with_them_after_losing_them(self):
        room = Room(away(standing(0, 1.5), 5.0), standing(1.0, 1.5), rate_guess=1200, lag_guess=0.35)
        room.run(15)  # they vanish for good; the other person stays
        self.assertFalse(room.following(1))
        self.assertFalse(any(line.startswith("Found them again") for line in room.logs))

    def test_takes_them_back_when_they_come_back(self):
        room = Room(away(standing(0, 1.5), 5.0, 9.0), rate_guess=1200, lag_guess=0.35)
        room.run(14)
        self.assertTrue(room.following(0))


if __name__ == "__main__":
    unittest.main()
