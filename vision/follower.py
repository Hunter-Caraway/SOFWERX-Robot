"""Follow-me: steers the robot after a person the HuskyLens 2 sees.

people_counter.py gives it the people in each of the HuskyLens's results, as boxes, and passes the drive
command it works out on to pi/receive.py, which drives with it while the watch is in follow mode.

It works the way person-following robots generally do: it tracks everyone in view, picks whom to follow, tells
them apart from everyone else, and steers to keep them in the middle of the picture at a set distance (the README's
Follow me section lists the papers this follows).

- Tracking: it keeps track of everyone in view from result to result (tracking by detection). Each person has a
  Kalman filter that guesses where their box will be next, from where it was, how it was moving, and how far the
  robot's own turns have moved the picture. Each new box goes to whoever was expected nearest it, keeping the total
  misfit least (global nearest neighbour), and a box far from everyone expected starts a new track. So someone the
  HuskyLens misses for a moment keeps their place, and nobody else takes it. Boxes of people in front of each other
  overlap and are unreliable, so they're skipped until the people part. A result that comes in while the picture
  is showing a turn counts for less, as it can't tell how much of the turn it shows, and for nothing about how
  people are moving (counting it in full made the robot swing either side of someone standing still).
- Picking whom to follow: once someone has been seen in a few results, it picks the person nearest the middle of the
  picture. From then on it follows their track, however many other people come and go. Anyone seen at the same time
  as them, once it has locked on, is someone else, and isn't taken for them while it keeps track of them.
- Calibrating, before it locks on: it turns toward the person it picked a step at a time, stopping after each step
  until the picture has caught up and settled (the HuskyLens's results lag behind the robot by lag). If they stand
  still, how far they moved across the picture tells it how fast the picture moves as it turns (rate), and how long
  it took to start moving, the lag. Turns of different lengths tell how long the robot keeps turning after a turn
  ends, as it winds down (overrun). Once locked on, it goes on learning the rate and overrun from turns toward someone
  holding still before and after (people walking about would throw it off): a short turn tells mostly the overrun,
  and a long one the rate. people_counter.py keeps what it measured for next time.
- Locking on: once they're in the middle, it follows them at the distance their box looks KEEP wide: about 1-2 feet.
  Width tells distance up close too, when the HuskyLens sees only their legs, where height and overall size don't.
- Following (visual servoing): while they're farther than that, it drives forward, faster the farther they are,
  curving toward them to keep them in the middle. It never reverses. As the picture lags behind the robot, it steers
  by where they will be by the time the picture catches up: where the filter has them, moved on by how they're
  moving and by turns the picture doesn't show yet. Off the middle with no need to drive, it turns on the spot
  toward them, for as long as it keeps asking rather than in steps, slowing as they near the middle, and stops where,
  allowing for the overrun, they'll end up in it. It doesn't start a turn that even at its shortest would swing them
  past the middle, and nearly facing someone who isn't going far, it first sees where its last turn left them.
- Losing them: if they vanish from the middle of the picture (the HuskyLens misses people, most of all close up), it
  faces where it expects them and waits until LOST_S. If they left by an edge, or someone new turned up off to one
  side just as they vanished, with nobody else about (them, moving too fast to keep track of), it turns that way for
  up to LEFT_S: fast until where they left is in the middle, then slowly, as the HuskyLens can't make people out while
  it turns fast. Anyone new who turns up past that edge is them. Someone turning up near where it expected them is
  them again, unless they were seen alongside them, or look a good deal nearer (someone walking in front of them).
  After that it lets go and turns slowly toward where it last saw them, no more than once round, and takes someone
  as them again only once they've been seen in REID_HITS results in a row, looking about as far away as they were,
  and weren't seen alongside them, or where someone seen alongside them stood.
"""

import math
from collections import namedtuple

FRAME_W, FRAME_H = 640, 480  # the HuskyLens 2's picture, in the units of its results
CENTER = FRAME_W / 2
FOV_DEG = 70            # how wide the HuskyLens 2 sees, roughly (degrees across the picture)

# Turning on the spot, and calibrating it
SPIN = 0.3              # turns on the spot at this (0..1, of full turn), and calibrates by it
SLOW_SPIN = 0.15        # ...or as slowly as this, looking for someone or nearly facing them (if the robot stalls
                        # turning this slowly, raise it); the picture moves in proportion, as motors.py's power does
RATE_GUESS = 1200.0     # how far the picture moves (px per second turning at SPIN) until it has measured it (as
                        # measured on the robot)
RATE_MIN, RATE_MAX = 100.0, 5000.0
RATE_STEP = 2.0         # each measurement after the first changes the rate by at most this factor, as people move
                        # about too
LAG_GUESS = 0.45        # how long (s) the picture takes to show a turn, until it has measured it (as measured)
LAG_MIN, LAG_MAX = 0.1, 0.8
OVERRUN_MAX_S = 0.15    # the robot may keep turning up to this long after a turn on the spot ends, in effect, as it
                        # winds down (it works out how long from turns of different lengths)
AIM = 0.8               # each turn aims this share of the way to the middle, so it falls short rather than past
TURN_MIN_S, TURN_MAX_S = 0.05, 0.4  # how long each turn on the spot lasts
STEADY_S = 0.2          # while calibrating, it stops after each turn for the lag and this long; someone who moved no
STEADY_PX = 10          # more than this across the picture meanwhile held still
REFINE_S = 0.4          # locked on before it has measured anything, it learns only from turns toward someone who held
                        # still this long before and after, as people walking about would throw it off (once it has,
                        # STEADY_S will do, as it then learns from turns while following, each a little at a time)
SETTLE_MAX_S = 1.2      # ...and while the picture hasn't moved and settled since a turn toward someone holding still,
                        # it waits up to this
MIN_SHIFT_PX = 10       # a turn that moves them less than this is too small to measure from
SEARCH_LIMIT_S = 60.0   # stops turning to look after this long, and waits for someone to come into view
LOOK_AFTER_S = 1.0      # at the start, waits this long for the HuskyLens to see someone before it looks around
MIRRORED = False        # True if the HuskyLens's picture is the mirror image, so the robot turns away from people

# Tracking people from result to result
NOISE_PX = 15.0         # how far a box's middle strays from where the person is, from one result to the next
NOISE_WIDTH = 0.1       # ...and its width, on a log scale...
JUMP_WIDTH = 0.3        # ...though it can jump by more than that (as legs and arms come and go), which it allows for in
                        # telling whose box is whose
WANDER_PX = 800.0       # how fast someone's speed across the picture can change (px/s^2), walking or turning
WANDER_WIDTH = 1.5      # ...and how fast their width's rate of change can (log scale, per s^2), walking nearer or away
FADE_S = 1.5            # a track's speed fades over this long, as someone unseen for a while has likely stopped
NEW_SPEED_PX = 300.0    # how fast (px/s) someone new might be moving across the picture
NEW_SPEED_WIDTH = 1.0   # ...and growing or shrinking (log scale, per s)
TURN_DOUBT = 0.25       # the robot's turns move the picture this share more or less than it works out, give or take...
DOUBT_S = 0.1           # ...and show up to this long sooner or later than the lag says, so it can't tell how much of a
                        # turn a result shows while one is showing; until it has measured the lag and rate, they could
                        # move it any amount, any time up to LAG_MAX after them
GATE = 9.2              # a box farther than this from where a track expected it (squared standard deviations: 99%
                        # for two measures) isn't theirs...
DRIFT_PX = 100.0        # ...however long they've been out of sight, counting them as having strayed no more than this
DRIFT_WIDTH = 0.5       # ...and this in width (log scale), as unseen people could otherwise take anyone's box
UNPAIRED = GATE + 5.0   # what leaving a box unpaired costs: more than pairing it with any track it's within the gate of
REJOIN_GATE = 4 * GATE  # someone new this near where it expected the person it lost is them
OVERLAP = 0.5           # boxes overlapping another this much (intersection over union) are skipped
CONFIRM_HITS = 3        # a new track counts as someone once seen in this many results...
TENTATIVE_S = 0.5       # ...and is dropped if unseen for this long before that
FORGET_S = 1.5          # anyone else unseen for this long is forgotten

# Following
MIDDLE_PX = 40          # someone within this of the middle of the picture needs no turning on the spot
AHEAD_PX = 160          # turning on the spot, it turns at SPIN toward someone this far off the middle, slower nearer
KEEP = 0.55             # it follows at the distance someone looks this wide (share of the picture's width): about
                        # 1-2 feet behind them (walking, it hangs back a little farther)
CLOSE_ENOUGH = 0.08     # standing, it drives on once they're this share farther than the distance it keeps
SPEED_GAIN = 1.5        # speed per share they're farther than that, plus...
CATCH_UP = 0.8          # ...this per share-second they've been farther lately (so it keeps up with someone walking)...
CATCH_UP_S = 2.0        # ...counting back about this long...
MAX_SPEED = 0.8         # ...up to this (0..1), and half that with them AHEAD_PX off the middle...
BRAKE = 1.5             # ...and no more than this per share they're farther, so it slows down as it gets there
STEER = 1.0             # while driving, it steers to move them toward the middle at this many times their offset per
                        # second...
CURVE = 1.5             # ...reckoning that driving at speed 1 with turn 1 turns it this many times as fast as turning
                        # on the spot at SPIN
COAST_S = 0.3           # keeps driving this long when they drop out of view, as the HuskyLens misses people for a
                        # moment (unless they were near, as it may be the robot that hid them)
EDGE_PX = 120           # expected this near an edge of the picture, or past it: they left by it, so it turns that way
LOST_S = 2.0            # gone from the middle of the picture this long: lets go of them
LEFT_S = 6.0            # gone by an edge this long: lets go of them
REID_S = 15.0           # once it has let go, it looks for them this long, turning slowly toward where it last saw
                        # them, but no more than one turn round (as it would then see the people it passed again)...
REID_HITS = 5           # ...and takes someone as them once seen in this many results in a row...
REID_WIDTH = 2.0        # ...looking no more than this many times wider or narrower than they did, and not where
OTHER_PX = 100          # someone seen alongside them was (within this, across the picture as it would be had the
                        # robot not turned), looking about as wide, as that's likely the same someone else
JUMP_S = 0.25           # someone new who turns up this soon after they vanish (in the next result or so) is likely
ALONE_S = 3.0           # them, moving too fast to keep track of, unless it saw anyone else alongside them this lately

STOP = ("STOP", 0.0, 0.0)
Box = namedtuple("Box", "x y w h")  # where a result is in the picture: its middle, width and height


def width(box):
    """How wide a box is, as a share of the picture's width: twice as wide at half the distance."""
    return max(box.w, 1) / FRAME_W


def clipped(box):
    """Whether a box is cut off by the left or right edge of the picture, so the person is wider than it: -1 by the
    left, +1 by the right, else 0."""
    return -1 if box.x - box.w / 2 <= 2 else 1 if box.x + box.w / 2 >= FRAME_W - 2 else 0


def overlap(a, b):
    """How much two boxes overlap: the area they share over the area they cover together."""
    w = min(a.x + a.w / 2, b.x + b.w / 2) - max(a.x - a.w / 2, b.x - b.w / 2)
    h = min(a.y + a.h / 2, b.y + b.h / 2) - max(a.y - a.h / 2, b.y - b.h / 2)
    if w <= 0 or h <= 0:
        return 0.0
    shared = w * h
    return shared / max(1.0, a.w * a.h + b.w * b.h - shared)


class Axis:
    """A Kalman filter for one measure of a person that changes at a rate that can drift: its value, its rate of
    change, and how unsure of them it is (their variances p and p_rate, and covariance p_both). doubted is how much of
    p is down to known changes it was unsure of, since the latest measurement."""

    def __init__(self, value, noise, rate_noise):
        self.value, self.rate = value, 0.0
        self.p, self.p_both, self.p_rate = noise ** 2, 0.0, rate_noise ** 2
        self.doubted = 0.0

    def predict(self, dt, wander, shift=0.0, doubt=0.0):
        """Moves on dt seconds. wander is how fast the rate can change (per s^2); shift is a known change (the robot
        turning moves everyone across the picture), give or take doubt."""
        fade = math.exp(-dt / FADE_S)
        q = wander ** 2
        self.value += self.rate * dt + shift
        self.p += 2 * dt * self.p_both + dt * dt * self.p_rate + q * dt ** 4 / 4 + doubt ** 2
        self.doubted += doubt ** 2
        self.p_both = fade * (self.p_both + dt * self.p_rate) + q * dt ** 3 / 2
        self.p_rate = fade * fade * self.p_rate + q * dt * dt
        self.rate *= fade

    def misfit(self, measured, noise, drift):
        """How far a measurement is from what it expected, squared, in standard deviations, counting the value as having
        drifted no more than drift, besides known changes. Also returns how unsure it was (log scale, 0 for as sure as
        a measurement)."""
        s = min(self.p, drift ** 2 + self.doubted) + noise ** 2
        return (measured - self.value) ** 2 / s, math.log(s / noise ** 2)

    def correct(self, measured, noise, rate_too=True):
        """Takes in a measurement; with rate_too false, for the value alone, as it tells nothing of the rate."""
        s = self.p + noise ** 2
        gain, rate_gain = self.p / s, (self.p_both / s if rate_too else 0.0)
        error = measured - self.value
        self.value += gain * error
        self.rate += rate_gain * error
        self.p, self.p_both, self.p_rate = ((1 - gain) * self.p, (1 - gain) * self.p_both,
                                            self.p_rate - rate_gain * self.p_both)  # (the same either way)
        self.doubted = 0.0


class Track:
    """Someone being tracked: where the middle of their box is across the picture (x) and how wide it is (size, on a
    log scale), each with a Kalman filter."""

    def __init__(self, number, t, box):
        self.number = number
        self.x = Axis(box.x, NOISE_PX, NEW_SPEED_PX)
        self.size = Axis(math.log(width(box)), NOISE_WIDTH, NEW_SPEED_WIDTH)
        self.box, self.born, self.seen = box, t, t  # their latest box, and when they were first and last seen
        self.hits = 1           # how many results they've been in
        self.streak = 1         # ...in a row, up to the latest
        self.other = False      # seen alongside the person being followed, so not them
        self.near = None        # first seen while the person being followed was out of sight: how near it has been to
                                # where it expected them (misfit), at nearest
        self.bearing = None     # where they were last seen, across the picture as it would be had the robot not turned
        self.first_bearing = None  # ...and first seen

    @property
    def confirmed(self):
        return self.hits >= CONFIRM_HITS

    def width(self, ahead=0.0):
        """How wide their box is, or will be ahead seconds from now, as a share of the picture's width."""
        return math.exp(self.size.value + self.size.rate * ahead)

    def misfit(self, box, unsure=0.0, anywhere=False):
        """How far a box is from where it expected theirs (squared standard deviations), and how unsure it was where to
        expect it (log scale). unsure is how much less than usual the box's middle tells where they are (px), as the
        robot's turns move the picture. With anywhere, they could be anywhere across it, so only the width counts."""
        x, x_unsure = (0.0, 0.0) if anywhere else self.x.misfit(box.x, math.hypot(NOISE_PX, unsure), DRIFT_PX)
        size, size_unsure = self.size.misfit(math.log(width(box)), JUMP_WIDTH, DRIFT_WIDTH)
        return x + (0.0 if self.cut_short(box) else size), x_unsure + size_unsure

    def cut_short(self, box):
        """Whether a box is cut off by the edge of the picture, and narrower than it expected theirs: it tells only that
        they're at least that wide. (Taking it in shrank the track as they left the picture, so that it went on
        shrinking out of sight, until it knew them by a tenth of their width.)"""
        return clipped(box) != 0 and math.log(width(box)) < self.size.value

    def predict(self, dt, shift, doubt):
        self.x.predict(dt, WANDER_PX, shift, doubt)
        self.size.predict(dt, WANDER_WIDTH)

    def correct(self, t, box, unsure=0.0):
        # A box the robot's turns may have moved any which way tells where they are, roughly, but not how they move.
        self.x.correct(box.x, math.hypot(NOISE_PX, unsure), rate_too=unsure <= NOISE_PX)
        if not self.cut_short(box):
            self.size.correct(math.log(width(box)), NOISE_WIDTH)
        else:
            self.size.rate = 0.0  # it can't tell how their width is changing, so it holds it (rather than let it drift)
        self.box, self.seen = box, t
        self.hits += 1
        self.streak += 1


def assign(costs, boxes):
    """Pairs boxes with tracks, each at most once, keeping the total cost least (global nearest neighbour). costs maps
    (track, box) indexes to cost, for the pairs within the gate; a box left unpaired costs more than any of them.
    Returns {box: track}."""
    if boxes > 6:  # a crowd: too many ways to pair them up, so the nearest pairs go first
        pairs, used = {}, set()
        for (track, box), _ in sorted(costs.items(), key=lambda item: item[1]):
            if box not in pairs and track not in used:
                pairs[box] = track
                used.add(track)
        return pairs
    best_total, best = math.inf, {}

    def search(box, used, total, pairs):
        nonlocal best_total, best
        if total >= best_total:
            return
        if box == boxes:
            best_total, best = total, pairs
            return
        for (track, b), cost in costs.items():
            if b == box and track not in used:
                search(box + 1, used | {track}, total + cost, {**pairs, box: track})
        search(box + 1, used, total + UNPAIRED, pairs)

    search(0, frozenset(), 0.0, {})
    return best


class Tracker:
    """Everyone in view, tracked from result to result."""

    def __init__(self):
        self.tracks = []
        self.numbered = 0  # tracks started so far
        self.t = None      # the time the tracks are at

    def predict(self, t, shift=0.0, doubt=0.0):
        """Moves every track on to time t. shift is how far the robot's turns have moved the picture meanwhile, give or
        take doubt."""
        if self.t is not None:
            for track in self.tracks:
                track.predict(t - self.t, shift, doubt)
        self.t = t

    def correct(self, t, boxes, unsure=0.0):
        """Takes in a new result's boxes: each goes to the track that expected it nearest, or starts a new one. unsure
        is how much less than usual their middles tell where people are (px)."""
        boxes = [box for i, box in enumerate(boxes)
                 if all(overlap(box, other) <= OVERLAP for j, other in enumerate(boxes) if j != i)]
        costs = {}
        for i, track in enumerate(self.tracks):
            for j, box in enumerate(boxes):
                misfit, doubt = track.misfit(box, unsure)
                if misfit < GATE:  # those it was surer of come first, and someone known before someone new
                    costs[i, j] = misfit + doubt / 2 + (0.0 if track.confirmed else 1.0)
        pairs = assign(costs, len(boxes))
        new = []
        for j, box in enumerate(boxes):
            if j in pairs:
                self.tracks[pairs[j]].correct(t, box, unsure)
            else:
                self.numbered += 1
                new.append(Track(self.numbered, t, box))
        for track in self.tracks:
            if track.seen != t:
                track.streak = 0
        self.tracks += new

    def forget(self, t, keep=None):
        """Drops the tracks of anyone gone for a while, except keep. Returns those it dropped."""
        gone = [track for track in self.tracks if track is not keep
                and t - track.seen >= (FORGET_S if track.confirmed else TENTATIVE_S)]
        self.tracks = [track for track in self.tracks if track not in gone]
        return gone


class Follower:
    """Picks the person to follow from each result, and how to drive after them.

    rate is how far the picture moves (px per second turning on the spot at SPIN; negative if it's mirrored), lag how
    long the picture takes to show a turn, and overrun how long the robot keeps turning after a turn ends, as measured
    by an earlier Follower (its calibration()), or None to start from RATE_GUESS and LAG_GUESS. log is called with a
    line for each step.
    """

    def __init__(self, rate=None, lag=None, overrun=None, log=None):
        self.rate = rate or (-RATE_GUESS if MIRRORED else RATE_GUESS)
        self.measured = rate is not None
        self.lag = lag or LAG_GUESS
        self.overrun = overrun or 0.0
        self.log = log or (lambda msg: None)
        self.tracker = Tracker()
        self.people = None       # the latest result's boxes, to tell a new result from the same one asked for again
        self.started = None      # the time of the first update
        self.t = None            # ...and of the latest
        self.dt = 0.0            # ...and the time since the one before
        self.target = None       # the track of the person being followed
        self.visible = False     # they were in the latest result
        self.keep = None         # the width to keep them at, once locked on
        self.last_width = None   # how wide they looked when it let go of them, to know them again by
        self.side = 1            # the side they were last seen on: -1 left, +1 right
        self.drive = STOP
        self.turn = None         # the latest turn on the spot: (direction, seconds, their x when it began or None,
                                 # whether they held still then, when it began)
        self.turn_until = 0.0
        self.spinning = 0.0      # the turn on the spot under way, for as long as it keeps asking (not a timed turn):
                                 # how fast, as a share of turning right at SPIN (-: left)
        self.spun_until = 0.0    # ...and when the latest such turn ended
        self.sweep = None        # such a turn toward someone holding still, to learn from: (their x, how far it had
                                 # turned then), and once it ends, (x, how far it turned, how fast at the end, as a
                                 # share of SPIN)
        self.moves = []          # (start, end, turn) of the robot's recent turns, for how they move the picture: turn
                                 # is how fast it turned, as a share of turning right on the spot at SPIN (-: left)
        self.turned = 0.0        # ...and how far it has turned in all (in seconds of turning right at SPIN)
        self.others = []         # (bearing, width, cut off: -1 left, +1 right, 0 not) of the latest people seen
                                 # alongside them, whose tracks it has since forgotten
        self.lately = 0.0        # share-seconds they've been farther than the distance it keeps, lately
        self.stopped_at = None   # when the latest turn ended, while waiting for the picture to catch up
        self.spins = []          # (seconds, px) of the latest turns it measured, for the rate and overrun
        self.sightings = []      # (time, x) of the target over the last couple of seconds
        self.waiting = False     # waiting where they vanished, for them to show up again
        self.searching = False   # turning toward the edge they left by...
        self.exit = 0.0          # ...where they were last thought to be, then (px, as if the robot had never turned)
        self.exit_turned = 0.0   # ...and how far the robot had turned, then
        self.stray = None        # where someone new last turned up just as they vanished, who may be them dashing off
        self.others_at = -math.inf  # when it last saw anyone else alongside them
        self.left = False        # they left by an edge, when it let go of them (rather than vanishing)
        self.looking_since = None  # when it started looking for someone, while it is, and how far it had turned then
        self.looked_from = 0.0
        self.gave_up = False       # it has stopped turning to look

    def calibration(self):
        """What it has measured of how the robot's turns move the picture, to start the next Follower from: (rate,
        lag, overrun), or None if it hasn't yet."""
        return (self.rate, self.lag, self.overrun) if self.measured else None

    def update(self, t, people):
        """Takes the people (boxes) in a result, at time t (time.monotonic()). Returns (drive, following, until): the
        drive command; whether it has someone to follow; and while turning on the spot, when to stop, else None."""
        people = list(people)
        new = people != self.people  # the HuskyLens is asked more often than it has new results
        self.people = people
        if self.t is None:
            self.started = t
            self.tracker.predict(t)
        else:
            cmd, speed, turn = self.drive
            if cmd.startswith("FORWARD") and turn:
                self._move(self.t, t, CURVE * speed * turn)  # it has curved since the last update
            elif self.spinning:
                self._move(self.t, t, self.spinning)  # ...or turned on the spot
            self.moves = [move for move in self.moves if move[1] + LAG_MAX > t - 1.0]
            self.tracker.predict(t, self._shift(self.t, t) if self.measured else 0.0, self._doubt(self.t, t))
        self.dt = t - (self.t if self.t is not None else t)
        self.t = t
        if new:
            self.tracker.correct(t, people, self._unsure(t))
            self._identify(t)
        for track in self.tracker.forget(t, keep=self.target):
            # Where someone else stood (not someone passing by, who has gone on, maybe to where they now are).
            if track.other and track.bearing is not None and abs(track.bearing - track.first_bearing) < OTHER_PX:
                self.others = (self.others + [(track.bearing, track.width(), clipped(track.box))])[-10:]
        self.drive = self._decide(t)
        cmd, _, turn = self.drive
        spinning = turn / SPIN if cmd in ("LEFT", "RIGHT") and t >= self.turn_until else 0.0
        if self.spinning and not spinning:
            if self.sweep is not None:
                before, turned = self.sweep
                self.sweep = (before, self.turned - turned, abs(self.spinning))
            self._move(t, t + self.overrun, self.spinning)  # it winds down
            self.spun_until = t
        self.spinning = spinning
        return self.drive, self.target is not None, self.turn_until if t < self.turn_until else None

    # --- who to follow ---

    def _identify(self, t):
        """After a new result: finds the person being followed in it, or picks someone to follow."""
        if self.target is not None and self.target.seen != t:
            unsure = self._unsure(t)
            for track in self.tracker.tracks:
                if track.born > self.target.seen and track.seen == t:
                    # Gone by an edge, they could have gone on any distance past it.
                    beyond = (self.searching and self._alone(t)
                              and (self._bearing(track.box.x, t) - self.exit) * self.side > -DRIFT_PX)
                    misfit = self.target.misfit(track.box, unsure, anywhere=beyond)[0]
                    track.near = misfit if track.near is None else min(track.near, misfit)
            self._rejoin(t)
        for track in self.tracker.tracks:
            if track.seen == t:
                track.bearing = self._bearing(track.box.x, t)
                if track.first_bearing is None:
                    track.first_bearing = track.bearing
                if self.keep is not None and not track.other and track is not self.target and self._was_other(track):
                    track.other = True  # where someone seen alongside them was: likely them again
                if track.other:
                    self.others_at = t
        if self.target is None:
            self._choose(t)
        self.visible = self.target is not None and self.target.seen == t
        if not self.visible:
            if self.target is not None and self._alone(t):
                for track in self.tracker.tracks:
                    if (track.seen == t and self.target.seen < track.born <= self.target.seen + JUMP_S
                            and not track.other and not self._in_front(track)):
                        self.stray = track.box.x
            return
        self.stray = None
        box = self.target.box
        if abs(box.x - CENTER) > MIDDLE_PX:
            self.side = 1 if box.x > CENTER else -1
        self.sightings = [(when, x) for when, x in self.sightings if when > t - 2.0] + [(t, box.x)]
        if self.keep is not None:
            for track in self.tracker.tracks:
                if track is not self.target and track.seen == t:
                    track.other = True  # in the same picture as them, so someone else
                    self.others_at = t

    def _choose(self, t):
        """Nobody to follow: before locking on, picks the person nearest the middle; after, someone who looks like the
        person it lost."""
        seen = [track for track in self.tracker.tracks if track.seen == t and track.confirmed and not track.other]
        if self.keep is None:
            if seen:
                self.target = min(seen, key=lambda track: abs(track.box.x - CENTER))
                self.sightings = []
                self.log(f"Picked the person nearest the middle, at x={self.target.box.x} ({len(seen)} in view)")
            return
        if self.gave_up:
            return  # it has stopped looking for them
        alike = [track for track in seen if track.streak >= REID_HITS
                 and abs(math.log(track.width() / self.last_width)) <= math.log(REID_WIDTH)]
        if alike:
            self.target = min(alike, key=lambda track: abs(math.log(track.width() / self.last_width)))
            self.sightings = []
            self.log(f"Found them again, at x={self.target.box.x}")

    def _rejoin(self, t):
        """While they're out of sight: someone new who has turned up near where it expected them (when first seen, or
        since, as it grows less sure where they are) is them, once seen twice in a row, as the HuskyLens can put a box
        where the track didn't expect it (a turn moving the picture more than it thought, the box jumping from their
        whole body to their legs, or them dashing off)."""
        near = [track for track in self.tracker.tracks
                if track.seen == t and track.near is not None and track.near < REJOIN_GATE and not track.other
                and track.streak >= 2 and not self._in_front(track)]
        if near:
            lost, self.target = self.target, min(near, key=lambda track: track.near)
            self.tracker.tracks.remove(lost)  # one track for them
            self.log(f"Picked them up again, at x={self.target.box.x}")

    def _in_front(self, track):
        """Whether someone new looks a good deal wider than the person it lost did: nearer, so likely someone else who
        walked in front of them (and hid them) rather than them."""
        return track.width() > self.target.width() * math.exp(JUMP_WIDTH)

    def _alone(self, t):
        """Whether it has seen nobody else alongside them lately, so that someone new turning up after they vanish
        is likely them, moving faster than it could keep track of."""
        return t - self.others_at > ALONE_S

    def _was_other(self, track):
        """Whether a track is likely someone seen alongside the person being followed, whose track it forgot: about as
        wide, where they were. (Someone cut off by the edge of the picture may have been wider, and farther out.)"""
        for bearing, width, cut in self.others:
            away = (track.bearing - bearing) * (cut or 1)  # how far out from where they were (or just how far)
            slack = OTHER_PX + track.box.w / 2
            wider = math.log(track.width() / width)
            if -slack < away < slack + (track.box.w if cut else 0.0) and (
                    abs(wider) <= math.log(REID_WIDTH) or (cut and wider > 0)):
                return True
        return False

    def _bearing(self, x, t):
        """Where something at x across the picture in a result at time t would be had the robot never turned (px)."""
        unshown = self._turned(math.inf, self.lag) - self._turned(t, self.lag)
        return x + self.rate * (self.turned - unshown)  # turning right moves the picture left, unless it's mirrored

    def _move(self, start, end, turn):
        """Notes a turn of the robot's, from start to end, at turn (a share of turning right on the spot at SPIN)."""
        self.moves.append((start, end, turn))
        self.turned += turn * (end - start)

    def _ahead(self, t):
        """Where they'll be in the picture once it catches up with now: where the track has them, moved on by how
        they're moving and by turns it doesn't show yet (once it has measured how turns move it)."""
        pending = self._shift(t, math.inf) if self.measured else 0.0
        return self.target.x.value + self.target.x.rate * self.lag + pending

    def _shift(self, t0, t1):
        """How far the robot turning moves the picture from t0 to t1, as the results show it, lag behind."""
        turned = self._turned(t1, self.lag) - self._turned(t0, self.lag)
        return -turned * self.rate  # turning right moves the picture left, unless it's mirrored

    def _turned(self, t, lag):
        """How far the robot had turned, as of a result at time t, if the picture is lag behind it: in seconds of
        turning right on the spot at SPIN, from a second or so ago (the same for any t, so it cancels out)."""
        return sum(turn * max(0.0, min(end - start, t - lag - start)) for start, end, turn in self.moves)

    def _unsure(self, t):
        """How unsure it is how much of the robot's turns a result at time t shows (px, about a standard deviation), as
        the lag could be DOUBT_S longer or shorter than it thinks. Until it has measured the lag and rate, it counts on
        turns moving the picture by no particular amount, so a result up to LAG_MAX after one could show any of it, or
        even twice as much (and after that, it is unsure where everyone is: see _doubt)."""
        if self.measured:
            return abs((self._turned(t, self.lag - DOUBT_S) - self._turned(t, self.lag + DOUBT_S)) * self.rate) / 2
        return sum(abs(turn * self.rate) * (min(end - start, max(0.0, t - LAG_MIN - start)) -
                                            min(end - start, max(0.0, t - LAG_MAX - start)))
                   for start, end, turn in self.moves)

    def _doubt(self, t0, t1):
        """How much the robot turning from t0 to t1 makes it unsure where everyone is in the picture (px, a standard
        deviation): a share of how far the turns move it, spread over when the picture could show them."""
        variance = 0.0
        for start, end, turn in self.moves:
            moved = abs(turn * self.rate) * (end - start)
            if self.measured:
                begins, ends, share = start + self.lag - DOUBT_S, end + self.lag + DOUBT_S, TURN_DOUBT
            else:
                begins, ends, share = start, end + LAG_MAX, 1.0
            during = min(ends, t1) - max(begins, t0)
            if during > 0:
                variance += (share * moved) ** 2 * during / (ends - begins)
        return math.sqrt(variance)

    # --- how to drive ---

    def _decide(self, t):
        if t < self.turn_until:
            return self.drive  # finishing a turn
        if self.turn is not None:
            if self._calibrating():  # waits for the picture to catch up, and learns from it
                if self.stopped_at is None:
                    self.stopped_at = t
                if not self._settled(t):
                    return STOP
                self._learn(t)
            self.turn = self.stopped_at = None
        if self.sweep is not None and len(self.sweep) == 3:
            self._learn_sweep(t)
        if self.target is None:
            return self._look(t)
        if self.looking_since is not None:
            self.log("Found someone")
            self.looking_since, self.gave_up = None, False
        if not self.visible:
            return self._missing(t)
        self.waiting = self.searching = False
        return self._approach(t) if self.keep is None else self._follow(t)

    def _approach(self, t):
        """Before locking on: turns toward them until they're in the middle, then locks on."""
        box = self.target.box
        offset = box.x - CENTER
        if not self.measured and t - self.sightings[0][0] < STEADY_S + 0.05 and abs(offset) > MIDDLE_PX:
            return STOP  # watches them for a moment first, to see whether they hold still, to calibrate by
        if self.measured:
            turning = self._center(t, self._ahead(t) - CENTER)
            if turning:
                return turning
        elif abs(offset) > MIDDLE_PX and self._worth_turning(offset):
            return self._turn(t, offset, AIM * abs(offset), box.x)  # a step at a time, to calibrate by
        self.keep = KEEP
        self.log(f"Locked on: following at width {self.keep:.2f} (they look {self.target.width():.2f})")
        return STOP

    def _follow(self, t):
        """Locked on and in view: keeps them in the middle, at the distance it keeps."""
        offset = self._ahead(t) - CENTER
        # How fast they're moving across the picture themselves, once it can tell that from how the robot turns.
        moving = self.target.x.rate if self.measured else 0.0
        farther = self.keep / self.target.width(self.lag) - 1  # distance goes as 1 / width
        if clipped(self.target.box):
            farther = min(farther, 0.0)  # cut off by the edge of the picture: nearer than it looks
        closing = farther > (0.0 if self.drive[0].startswith("FORWARD") else CLOSE_ENOUGH)  # once going, it closes in
        if not self.measured and (abs(offset) > AHEAD_PX or (abs(offset) > MIDDLE_PX and not closing)):
            # Until it has measured how turns move the picture, it turns a step at a time, and learns from each.
            if self._worth_turning(offset):
                self.lately = 0.0
                return self._turn(t, offset, AIM * abs(offset), self.target.box.x)
        if not closing:
            self.lately = 0.0
            return (self._center(t, offset) if self.measured else None) or STOP  # still only once they are
        # Speed: proportional to how much farther they are, plus how much farther they've been lately (a leaky
        # integral), so it keeps up with someone walking instead of trailing behind; slower while they're off the
        # middle.
        self.lately += (farther - self.lately / CATCH_UP_S) * self.dt
        speed = min(MAX_SPEED, SPEED_GAIN * farther + CATCH_UP * self.lately, BRAKE * farther)
        speed = round(speed * (1 - 0.5 * min(1.0, abs(offset) / AHEAD_PX)), 2)
        # Steering: turns to move them toward the middle at STEER times their offset per second, as well as keeping up
        # with them moving across. Curving turns the robot faster the faster it drives, so the turn is that over how
        # fast it would turn it.
        turn = (STEER * offset + moving) / (CURVE * self.rate * max(speed, 0.1))
        turn = round(max(-1.0, min(1.0, turn)), 2)
        side = "-RIGHT" if turn > 0 else "-LEFT" if turn < 0 else ""
        return "FORWARD" + side, speed, turn

    def _missing(self, t):
        """They aren't in the latest result."""
        gone = t - self.target.seen
        if gone < COAST_S:
            near = self.keep is not None and self.target.width() > self.keep / (1 + CLOSE_ENOUGH)
            return self.drive if self.drive[0].startswith("FORWARD") and not near else STOP  # missed for a moment
        x = self._ahead(t)
        if self.stray is not None and abs(self.stray - self.target.box.x) > MIDDLE_PX:
            # Someone new turned up off to one side since: likely them, moving too fast to keep track of, so by the
            # next result they'd have gone as far again.
            x = 2 * self.stray - self.target.box.x
        if self.searching or abs(x - CENTER) >= CENTER - EDGE_PX:  # they left by an edge
            if not self.searching:
                self.side = 1 if x > CENTER else -1
                self.log(f"They left by the {'right' if self.side > 0 else 'left'} edge: turning that way")
                self.searching, self.waiting = True, False
                self.exit, self.exit_turned = x + self.rate * self.turned, self.turned
            if gone >= LEFT_S or self._round_since(self.exit_turned):
                return self._let_go(t)
            # Fast until where they left is in the middle, as they're likely just past it, then slowly, as the HuskyLens
            # can't make people out while the robot turns fast (on the robot, it spun round and round past them).
            unturned = (self.exit - self.rate * self.turned - CENTER) * self.side  # how far that is from the middle
            return self._spin(self.side, SPIN if unturned > 0 else SLOW_SPIN)
        if gone >= LOST_S:
            return self._let_go(t)
        if not self.waiting:
            self.log(f"Lost sight of them at x={x:.0f}, not at an edge: facing where they were, waiting for them")
            self.waiting, self.searching = True, False
        return (self._center(t, x - CENTER) if self.measured else None) or STOP

    def _let_go(self, t):
        self.log(f"Let go of them after {t - self.target.seen:.1f} s out of sight")
        self.last_width = self.target.width()
        self.left = self.searching
        self.target, self.visible, self.stray = None, False, None
        self.waiting = self.searching = False
        return self._look(t)

    def _look(self, t):
        """Nobody to follow: turns slowly toward the side someone was last seen on, until SEARCH_LIMIT_S (or when it's
        looking for the person it lost, REID_S, or once round, counting any turn after them when they left by an
        edge)."""
        if self.looking_since is None:
            self.looking_since, self.looked_from = t, (self.exit_turned if self.left else self.turned)
            self.log(f"Looking for {'them' if self.keep else 'someone'} to the {'right' if self.side > 0 else 'left'}")
        round_once = self._round_since(self.looked_from)
        if t - self.looking_since >= (REID_S if self.keep else SEARCH_LIMIT_S) or (self.keep and round_once):
            if not self.gave_up:
                self.log("Stopped looking" + (": hold the watch screen to follow someone afresh" if self.keep else
                                              ": waiting for someone to come into view"))
                self.gave_up = True
            return STOP
        if t - self.started < LOOK_AFTER_S or t < self.turn_until:
            return STOP
        if any(t - track.seen < COAST_S and not track.other for track in self.tracker.tracks):
            return STOP  # someone new is in view: it sees whether they stay
        return self._spin(self.side, SLOW_SPIN)

    def _round_since(self, turned):
        """Whether the robot has turned once round since it had turned that far (as _move counts it)."""
        return abs((self.turned - turned) * self.rate) >= FRAME_W * 360 / FOV_DEG

    def _calibrating(self):
        """Whether it stops after each turn on the spot until the picture has caught up, to learn from it: always
        before it locks on, and after that until it has measured the rate and lag once."""
        return self.keep is None or not self.measured

    def _center(self, t, offset):
        """Turns on the spot toward someone offset px off the middle of the picture (where it will show them once it
        catches up), slower as they near it, so as to stop with them in the middle. None once they are, or will be."""
        final = offset - self.spinning * self.rate * self.overrun  # where they'll end up if it stops now
        toward = self.spinning * self.rate  # the side it's bringing nearer the middle: -left, +right, 0 if not turning
        if toward and (final * toward <= 0 or abs(final) <= MIDDLE_PX / 4):
            return None  # it has turned far enough: they'll be in the middle once the picture catches up
        turn = max(SLOW_SPIN, min(SPIN, SPIN * abs(final) / AHEAD_PX))
        if not toward:
            if abs(final) <= MIDDLE_PX:
                return None
            waiting = self._shown_at() + STEADY_S - t
            if abs(final) < AHEAD_PX and waiting > 0 and abs(self.target.x.rate) * waiting < MIDDLE_PX:
                return None  # nearly facing them, who aren't going far: first it sees where the latest turn left them
            if abs(self.rate) * turn / SPIN * (self.dt + self.overrun) > abs(final) + MIDDLE_PX / 2:
                return None  # even the shortest turn would swing them past the middle, farther than they are now
        drive = self._spin(final, turn)
        if not toward and self._still(t, STEADY_S):
            self.sweep = (self.sightings[-1][1], self.turned)  # (after _spin, which forgets any earlier one)
        return drive

    def _spin(self, toward, turn):
        """Turns on the spot at turn (0..1), bringing the side toward (-: left, +: right) nearer the middle, for as long
        as it keeps asking."""
        direction = 1 if toward * self.rate > 0 else -1  # turning right moves the picture left, unless mirrored
        if not self.spinning:
            self.sweep = None  # a new turn
        return ("RIGHT" if direction > 0 else "LEFT"), 0.0, round(direction * turn, 2)

    def _shown_at(self):
        """When the picture shows all of the latest turn on the spot, as far as it knows."""
        if self.spinning:
            return math.inf
        return max(self.turn_until, self.spun_until) + self.overrun + self.lag

    def _worth_turning(self, offset):
        """Whether turning on the spot toward someone offset px off the middle of the picture brings them into the
        middle, rather than past it: the shortest turn swings them no more than MIDDLE_PX / 2 past it."""
        return abs(self.rate) * (TURN_MIN_S + self.overrun) <= abs(offset) + MIDDLE_PX / 2

    def _turn(self, t, toward, px, x):
        """Starts a turn on the spot that moves the picture px, bringing the side toward (-: left, +: right) nearer
        the middle. x is where the target is, if in view."""
        direction = 1 if toward * self.rate > 0 else -1  # turning right moves the picture left, unless mirrored
        seconds = min(TURN_MAX_S, max(TURN_MIN_S, px / abs(self.rate) - self.overrun))
        self.turn = (direction, seconds, x, x is not None and self._still(t), t)
        self.turn_until = t + seconds
        self._move(t, t + seconds + self.overrun, direction)
        return ("RIGHT" if direction > 0 else "LEFT"), 0.0, direction * SPIN

    # --- calibrating ---

    def _still(self, t, steady_s=None):
        """Whether the target has held still in the picture long enough to measure a turn by (or steady_s), since it
        caught up with any turn."""
        start = t - (steady_s or self._steady_s())
        if start < self._shown_at():
            return False  # the picture may not have caught up with the latest turn
        xs = [x for when, x in self.sightings if when >= start]
        seen_from_start = any(start - 0.15 <= when <= start + 0.15 for when, _ in self.sightings)
        return len(xs) >= 2 and seen_from_start and max(xs) - min(xs) <= STEADY_PX

    def _steady_s(self):
        """How long someone must hold still for it to measure a turn by them."""
        return STEADY_S if self.keep is None else REFINE_S

    def _since(self, t):
        """The target's x in each sighting since time t."""
        return [x for when, x in self.sightings if when >= t]

    def _settled(self, t):
        """Whether the picture has caught up with the turn: the lag and long enough to see someone hold still have
        passed, and if they were holding still, they've moved in the picture, and held still again since (or
        SETTLE_MAX_S has passed)."""
        waited = t - self.stopped_at
        if waited >= SETTLE_MAX_S:
            return True
        if t < self._shown_at() + self._steady_s():
            return False
        _, _, before, still, began = self.turn
        if still and self.visible:
            if all(abs(x - before) < MIN_SHIFT_PX for x in self._since(began)):
                return False  # the picture hasn't shown the turn yet: the lag is longer than it thought
            if not self._still(t):
                return False  # ...or is still showing it
        return True

    def _learn(self, t):
        """After a turn toward someone who held still: how far they moved across the picture gives the rate (and from
        turns of different lengths, the overrun), and how soon, the lag. They must have held still after it too."""
        direction, seconds, before, still, began = self.turn
        if not still or not self.sightings:
            return
        lasted = seconds + self.overrun
        calibrating = self._calibrating()
        if not self.visible:  # if they left the picture, it turned at least as fast as their last sighting shows
            last = self.sightings[-1][1]
            rate = -(last - before) / (direction * lasted)
            at_edge = abs(last - CENTER) >= CENTER - EDGE_PX
            if not self.measured and at_edge and rate * self.rate > 0 and abs(rate) > abs(self.rate):
                self.rate = math.copysign(min(RATE_MAX, abs(rate)), rate)
                self.log(f"Turned {seconds:.2f} s and they left the picture: it moves at least {abs(rate):.0f} px/s")
            return
        if not self._still(t):
            if calibrating:
                self.log(f"Turned {seconds:.2f} s; they were moving, so it measured nothing")
            return
        moved = self.sightings[-1][1] - before
        if abs(moved) < MIN_SHIFT_PX:
            if calibrating:
                self.rate *= 0.7  # it hardly turned: turn longer next time
                self.log(f"Turned {seconds:.2f} s, they hardly moved: turning longer next time")
            return
        rate = -moved / (direction * lasted)
        if rate * self.rate < 0:
            self.log(f"Turned {seconds:.2f} s, they moved {moved:+.0f} px, away from the middle: they must have moved")
            return
        # The lag: when the picture showed the turn under way (a share of the way, so a little drift doesn't count).
        first = next(when for when, x in self.sightings if when >= began and abs(x - before) >= 0.3 * abs(moved))
        lag = first - began - 0.3 * lasted  # (the picture was a share of the way when the turn was)
        self.spins = (self.spins + [(seconds, abs(moved))])[-6:]
        fit = self._fit()
        if fit:
            new, self.overrun = fit
        else:
            new = abs(rate) if not self.measured else (abs(self.rate) + abs(rate)) / 2
        if self.measured:  # (after the first, which may be far from the guess)
            new = min(abs(self.rate) * RATE_STEP, max(abs(self.rate) / RATE_STEP, new))
        self.rate = math.copysign(min(RATE_MAX, max(RATE_MIN, new)), rate)
        self.lag = min(LAG_MAX, max(LAG_MIN, lag if not self.measured else (self.lag + lag) / 2))
        self.measured = True
        self.log(f"Turned {seconds:.2f} s, they moved {moved:+.0f} px after {lag:.2f} s: the picture moves "
                 f"{abs(self.rate):.0f} px/s, {self.lag:.2f} s behind, and keeps moving {self.overrun:.2f} s after")

    def _learn_sweep(self, t):
        """After a turn toward someone holding still, once locked on: how far they moved across the picture, once it
        caught up and they held still again, gives the rate and overrun, as for a turn while calibrating. Turning
        slower moves the picture slower, in proportion, so the turn counts as one at SPIN of the same length."""
        before, turned, speed = self.sweep
        if t < self._shown_at() + STEADY_S:
            return
        if not self.visible or t > self._shown_at() + SETTLE_MAX_S:
            self.sweep = None  # they moved, or left
            return
        if not self._still(t, STEADY_S):
            return
        self.sweep = None
        moved = self.sightings[-1][1] - before
        if abs(moved) < MIN_SHIFT_PX or -moved / turned * self.rate < 0:
            return  # too small to tell by, or they moved
        seconds, px = abs(turned) / speed, abs(moved) / speed
        self.spins = (self.spins + [(seconds, px)])[-6:]
        fit = self._fit()
        if fit:
            new, self.overrun = fit
        elif seconds >= 2 * OVERRUN_MAX_S:  # turns of much the same length: a long one tells mostly the rate...
            new = (abs(self.rate) + px / (seconds + self.overrun)) / 2
        else:  # ...and a short one, the overrun
            new = abs(self.rate)
            self.overrun = min(OVERRUN_MAX_S, max(0.0, (self.overrun + px / new - seconds) / 2))
        new = min(abs(self.rate) * RATE_STEP, max(abs(self.rate) / RATE_STEP, new))
        self.rate = math.copysign(min(RATE_MAX, max(RATE_MIN, new)), self.rate)
        self.log(f"Turned {abs(turned) / speed:.2f} s at {speed * SPIN:.2f}, they moved {moved:+.0f} px: the picture "
                 f"moves {abs(self.rate):.0f} px/s, and keeps moving {self.overrun:.2f} s after")

    def _fit(self):
        """The rate and overrun that fit the turns it measured best (px = rate * (seconds + overrun)), if they were
        of different lengths, else None."""
        n = len(self.spins)
        seconds = sum(s for s, _ in self.spins) / n
        px = sum(p for _, p in self.spins) / n
        spread = sum((s - seconds) ** 2 for s, _ in self.spins)
        if n < 2 or spread < n * 0.03 ** 2:
            return None
        rate = sum((s - seconds) * (p - px) for s, p in self.spins) / spread
        if rate <= 0:
            return None
        overrun = min(OVERRUN_MAX_S, max(0.0, px / rate - seconds))
        return px / (seconds + overrun), overrun
