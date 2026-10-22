# SOFWERX Robot

Drive a robot with wrist gestures from a smartwatch, or have it follow you. A Wear OS app on a Galaxy Watch reads
its motion sensors, turns wrist tilt and roll into drive commands (forward / reverse / stop / left / right, with
proportional speed and turn), and sends them over Wi-Fi to the robot's Raspberry Pi 5 (`Pi5.local`). There,
`pi/receive.py` prints each command and drives the robot's four motors through a PCA9685 PWM board and two L298N
motor drivers. A HuskyLens 2 AI camera on the robot counts the people in view, which the watch calls out, and in
follow mode the robot finds you, locks on and follows you about.

## What it does

| Gesture (relative to your calibrated poses) | Command |
|---|---|
| Hand level | `STOP` |
| Hand tilted down | `FORWARD`. Speed ramps from 0 to 100% as you tilt toward your full-speed pose |
| Hand tilted up | `REVERSE`. Speed ramps from 0 to 100% as you tilt toward your full-reverse pose |
| Hand rolled left / right | `LEFT` / `RIGHT`. Turn ramps from 0 to ±100% |
| Tilted and rolled | `FORWARD-LEFT`, `FORWARD-RIGHT`, `REVERSE-LEFT`, `REVERSE-RIGHT` |

- **Calibrate or start:** when the app opens it asks **Calibrate?**. Tap **Start** to use your saved
  calibration, or **Calibrate** to record 7 poses (level, full forward, full reverse, max left/right, and max
  left/right at full forward). For each pose, hold it and tap the screen with your other hand. The watch waits
  1 s for the tap's jolt to settle, records for 1.5 s, and buzzes. The axes come from the gravity direction in
  each pose, so it works on either wrist with any watch orientation.
- **Double-tap the watch** to turn control on and off. The screen goes green for ON (showing the current
  command) and red for OFF, and the watch buzzes once for on and twice for off. Control always starts OFF.
- **Speed gears:** turn the watch's bezel to change gear: **Slow** (35% of full speed), **Medium** (65%) or
  **Fast** (100%). Each click clockwise goes up a gear, and each click back goes down one. The watch ticks, and once
  the bezel stops BMO says the gear. Tilting all the way drives at the gear's top speed. The gear is kept between
  runs, and starts at Medium. Follow mode has its own speeds.
- **Follow me:** hold a finger on the watch's screen to switch follow mode on (three short buzzes) or off (one long
  buzz). With control ON, the robot then turns to look for you, locks on, and follows you; wrist tilt does nothing.
  BMO says "Looking for you!", "Following you!" or "Lost you!" as that changes. Double-tap still turns control off,
  which stops it. See [Follow me](#follow-me).
- **Fail-safe:** the command is `STOP` whenever control is off, and the Pi also stops when no packets have
  arrived for 0.5 s (watch out of range, app closed, Wi-Fi dropped). Bringing your hand back to level stops at
  once. Quitting the receiver (Ctrl+C or `kill`) turns the motors off.
- **Jitter rejection:** gravity is low-pass filtered, commands have dead zones and hysteresis, and a command
  must be held for 150 ms before it switches on.
- **People callouts:** the robot's camera counts the people in view, and each time the count changes the watch
  says it at full volume, in a voice like BMO's from Adventure Time: "All clear!", "Person ahead!", "2 people
  ahead!". This needs the people counter running on the Pi ([Run everything on the Pi](#run-everything-on-the-pi)).

### How it works

```
Galaxy Watch (watch-app)                                      Raspberry Pi (pi/receive.py)
  gravity @ 50 Hz → smooth → pitch/roll vs. calibration
  → gesture classifier → cmd, speed, turn  ──UDP:5005 JSON──►  STOP on OFF / lost signal
  double-tap ON/OFF, bezel gear, hold for follow mode           (follow mode: cmd from vision/follower.py)
                                                                → speed scaled by the gear
                                                                → send_command(cmd, speed, turn)
                                                                → PCA9685 (I2C) → 2× L298N → 4 motors
  speaker, BMO: "2 people ahead!"      ◄──UDP {"people":2}──  count from vision/people_counter.py
```

The gesture logic is in [`Gestures.kt`](watch-app/app/src/main/kotlin/com/sofwerx/imuwatch/Gestures.kt). Each
packet is one small JSON message:
`{"on":true,"follow":false,"top":0.65,"cmd":"FORWARD-LEFT","speed":0.60,"turn":-0.40}`. `top` is the gear's top
speed, and `follow` is follow mode.

### Driving the motors

`send_command(cmd, speed, turn)` in [`pi/receive.py`](pi/receive.py) prints each command and sets the motors.
It's called whenever the command changes or speed/turn move by 5%. When driving by wrist, the speed has already been
scaled by the gear (`geared` in `receive.py`): with `FORWARD` or `REVERSE` the speed is scaled, and spinning on the
spot the turn is.

- `cmd` is one of `STOP`, `FORWARD`, `REVERSE`, `LEFT`, `RIGHT`, `FORWARD-LEFT`, `FORWARD-RIGHT`,
  `REVERSE-LEFT`, `REVERSE-RIGHT`
- `speed` runs from 0 to 1 (the direction is in `cmd`)
- `turn` runs from -1 (full left) to +1 (full right)

The two left motors always run together, and so do the two right ones (`wheel_speeds` in `receive.py`):

| Command | Left motors | Right motors |
|---|---|---|
| `STOP` | brake | brake |
| `FORWARD` | forward at `speed` | forward at `speed` |
| `REVERSE` | backward at `speed` | backward at `speed` |
| `LEFT` | backward at `turn` | forward at `turn` |
| `RIGHT` | forward at `turn` | backward at `turn` |
| `FORWARD-LEFT` | forward, slowing as `turn` grows, stopped at full turn | forward at `speed` |
| `FORWARD-RIGHT` | forward at `speed` | forward, slowing as `turn` grows, stopped at full turn |

`LEFT` and `RIGHT` on their own spin the robot on the spot; with `FORWARD` it curves. `REVERSE-LEFT` and
`REVERSE-RIGHT` work like `FORWARD-LEFT` and `FORWARD-RIGHT`, backward.

[`pi/motors.py`](pi/motors.py) talks to the PCA9685 over I2C, with only Python's standard library. Each motor has
two L298N IN pins wired to the PCA9685: one is pulsed to set the motor's power, and which one sets its direction.
Any speed above 0 gets at least `MIN_DUTY` (30%) power, since less won't turn the wheels; `MAX_DUTY` caps the top
speed. If the receiver can't reach the PCA9685, it says why and carries on, printing the commands only.
`--no-motors` does the same on purpose.

## Run it yourself

### What you need

- A Wear OS 3+ watch (Android API 30+; built and tested on a Samsung Galaxy Watch)
- A Raspberry Pi on the **same Wi-Fi network** as the watch, with Python 3.8+ (standard library only; nothing to
  `pip install`)
- On the robot: a PCA9685 PWM board on the Pi's I2C pins, and two L298N motor drivers (left motors on one,
  right motors on the other) wired to it
- For the people counter and follow mode: a DFRobot HuskyLens 2 on the Pi's serial pins
  ([wiring](#wire-it-up))
- A laptop with `adb` (Android platform-tools) to install the watch app
- To build the watch app: JDK 17+ and the Android SDK (API 36). Android Studio installs both.

### 1. Connect the watch over adb

On the watch, go to **Settings → About watch → Software** and tap **Software version** 5 times to enable
developer options. Then turn on **Developer options → ADB debugging** and **Wireless debugging**, and pair from
the laptop:

```bash
adb pair <watch-ip>:<pairing-port>      # code shown on the watch under "Pair new device"
adb connect <watch-ip>:<port>
adb devices                             # the watch should be listed
```

### 2. Build and install the watch app

```bash
cd watch-app
echo "sdk.dir=$HOME/Android/Sdk" > local.properties   # path to your Android SDK
./gradlew installDebug                                # builds and installs "IMU Stream" on the watch
./gradlew testDebugUnitTest                           # optional: tests for the gesture logic, gears and callouts
```

The watch speaks in a voice made to sound like BMO from Adventure Time. BMO's own voice isn't available to
text-to-speech, so it's one of Google's women's English voices, raised in pitch and sped up a little (`BMO_PITCH`
and `BMO_RATE` in `Announcer.kt`). Without a woman's voice it raises the watch's default voice the same way, and
the screen says `no woman's voice; BMO sounds deeper`. It turns the watch's media volume up to full while it speaks,
and puts it back afterwards.

You can also open `watch-app/` in Android Studio and run it.

### 3. Wire up the motors

The PCA9685 goes on the Pi 5's I2C pins, which are separate from the HuskyLens's UART pins:

| PCA9685 | Pi 5 |
|---|---|
| SDA | pin 3 (GPIO2) |
| SCL | pin 5 (GPIO3) |
| VCC | pin 1 or 17 (3.3 V), whichever the HuskyLens isn't using |
| GND | pin 9 (GND) |

Powering the PCA9685 from 3.3 V keeps its I2C signals at the 3.3 V the Pi's pins take, and the L298Ns read its
3.3 V outputs fine. Its V+ terminal is only for servos and isn't needed. The L298Ns' GND also goes to the
PCA9685's GND.

The right motors are on one L298N and the left motors on the other. Each L298N's IN1-IN4 go to the PCA9685, and
its ENA/ENB jumpers stay on:

| PCA9685 output | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
|---|---|---|---|---|---|---|---|---|
| L298N pin | right IN1 | right IN2 | right IN3 | right IN4 | left IN1 | left IN2 | left IN3 | left IN4 |
| Motor | rear right | rear right | front right | front right | rear left | rear left | front left | front left |

`LEFT_MOTORS` and `RIGHT_MOTORS` at the top of [`pi/motors.py`](pi/motors.py) list each motor's two outputs.

Turn on I2C on the Pi once, and reboot. It is then `/dev/i2c-1`:

```bash
echo dtparam=i2c_arm=on | sudo tee -a /boot/firmware/config.txt
sudo reboot
```

Then check the wiring with the motor test. **Lift the wheels off the ground first.** It spins each motor forward,
then backward, one at a time, and prints which one should be turning:

```bash
ssh sofwerx@Pi5.local mkdir -p robot && scp -r pi sofwerx@Pi5.local:robot/   # from the laptop
python3 ~/robot/pi/motors.py                                                 # on the Pi
python3 ~/robot/pi/motors.py --speed 0.8                                     # faster
python3 -m unittest pi/test_motors.py      # tests, in the repo folder on the laptop (no hardware needed)
```

- **The wrong wheel turns:** fix that motor's outputs in `LEFT_MOTORS` / `RIGHT_MOTORS`.
- **A wheel turns the wrong way:** swap that motor's forward and backward outputs.
- **The wheels hum but don't turn at low speeds:** raise `MIN_DUTY` in `motors.py`.
- **`another program is driving them`:** only one program can drive the motors at a time. Stop the receiver,
  or the robot service if it's installed ([Start it on boot](#start-it-on-boot)).

### 4. Run the receiver on the Pi

Copy the `pi/` folder to the Pi and run the receiver there. To run it together with the people counter, see
[Run everything on the Pi](#run-everything-on-the-pi).

```bash
ssh sofwerx@Pi5.local mkdir -p robot && scp -r pi sofwerx@Pi5.local:robot/   # from the laptop
python3 ~/robot/pi/receive.py                                                # on the Pi
```

Then open **IMU Stream** on the watch, calibrate or start, and double-tap to turn control on. The watch sends to
`Pi5.local`, falling back to the Pi's last known IP (`PI_FALLBACK_IP` in `MainActivity.kt`) if it can't
resolve the name. On the Pi you should see:

```
10:35:01  Motors ready (PCA9685 at 0x40 on I2C bus 1)
10:35:02  Watch connected (10.4.2.171)
10:35:02  Control ON
10:35:03  FORWARD        speed  50%
10:35:03  FORWARD-LEFT   speed 100%  turn  -40%
10:35:03  STOP
10:35:03  REVERSE        speed  60%
```

`--no-motors` only prints the commands. If the first line says `Motors OFF` instead of `Motors ready`, it goes on
to say what to check.

**Troubleshooting:** check that the watch screen (after Start) shows `Wi-Fi: up` and a non-zero Hz. If it does
but the Pi only says `Listening…`, check whether the Pi has a firewall up (`sudo ufw status`), and if so run
`sudo ufw allow 5005/udp`.

## People counter (HuskyLens 2)

`vision/people_counter.py` is a separate program that counts the people in view of a DFRobot HuskyLens 2 AI camera
wired to the Pi's serial port. It prints a line whenever the count changes:

```
12:43:26  People in frame: 4
12:43:30  People in frame: 3
```

The HuskyLens 2 does the detecting itself, on its own chip, with its **Object Recognition** algorithm, which finds
80 kinds of everyday things. Each "person" it finds is one person. It finds bodies, not faces, so people facing away
still count. The program switches the HuskyLens to that algorithm when it starts, then asks it for its results about
33 times a second. It only needs Python's standard library.

The per-result count is smoothed before it's reported (`RISE_S` and `FALL_S` in `people_counter.py`):

- **A higher count must hold for 0.5 s.** That filters out one-off false detections.
- **A lower count must hold for 0.5 s.** Someone the HuskyLens misses for a moment keeps being counted, and when
  someone really leaves, the count drops 0.5 s later.

**Limits:** someone hidden or turned so their body can't be made out for longer than 0.5 s drops out of the count
until they're seen again. Object Recognition has to find a body, so someone who is only barely in view may not be
counted. A longer `FALL_S` bridges longer gaps, at the cost of a slower drop when people leave.

### Wire it up

The HuskyLens 2's 4-pin Gravity port goes to the Pi 5's 40-pin header:

| HuskyLens 2 | Pi 5 |
|---|---|
| T (green) | pin 10 (GPIO15, RX) |
| R (blue) | pin 8 (GPIO14, TX) |
| − (black) | pin 6 (GND) |
| + (red) | pin 1 or 17 (3.3 V) |

Each side's transmit goes to the other's receive, so T goes to the Pi's RX and R to its TX. Powering it from
3.3 V keeps its signals at the 3.3 V the Pi's pins take. On the HuskyLens 2, set **System Settings → Protocol
Type → Serial 115200**. The light by its port turns green; the setting can take a few seconds to apply.

### Set it up on the Pi

Turn on the UART on GPIO 14/15 once, and reboot. On a Pi 5 it is then `/dev/ttyAMA0`:

```bash
echo dtparam=uart0=on | sudo tee -a /boot/firmware/config.txt
sudo reboot
```

From the laptop, copy the program over, then run it on the Pi:

```bash
scp vision/people_counter.py vision/follower.py sofwerx@Pi5.local:robot/vision/   # from the laptop
python3 ~/robot/vision/people_counter.py
```

Options:

```bash
python3 ~/robot/vision/people_counter.py --baud 9600           # if its Protocol Type is Serial 9600
python3 ~/robot/vision/people_counter.py --port /dev/ttyUSB0   # a different serial port
python3 ~/robot/vision/people_counter.py --watch-control       # see below
python3 ~/robot/vision/people_counter.py --recalibrate         # follow mode measures turning afresh
python3 -m unittest vision/test_people_counter.py              # tests, with a fake HuskyLens (no hardware needed)
python3 -m unittest vision/test_follower.py                    # tests for follow mode, in a simulated room
```

`--watch-control` detects only while watch control is ON, and sends the count to the watch; `run_all.py` runs it
that way. `--recalibrate` does the same as deleting `~/robot/vision/follow_calibration.json`.

If the HuskyLens doesn't answer, the program says so once, with what to check, and keeps trying. It also picks up
again on its own if the HuskyLens restarts. The HuskyLens's own screen shows what it detects, with a box around each
person, which is the quickest way to see why the count is what it is.

`on_count(count)` in `people_counter.py` is the hook for anything that should react to the count.

## Follow me

In follow mode, [`vision/follower.py`](vision/follower.py) works out how to drive after one person from each of
the HuskyLens's results. The people counter sends that to the receiver (UDP port 5008 on 127.0.0.1), which drives
with it in place of the watch's tilt. Follow mode has its own speeds, so the gear doesn't change it.

It works the way published person-following robots do (see [where this comes from](#where-this-comes-from)): it
tracks everyone in view, picks one person, tells them apart from everyone else, and steers to keep them in the middle
of the picture at a set distance, allowing for the camera's lag.

- **Looking:** with nobody in view, it turns slowly and steadily on the spot (`SLOW_SPIN`) toward the side it last
  saw someone on (right to begin with). BMO says "Looking for you!". It gives up turning after 60 s
  (`SEARCH_LIMIT_S`) and waits for someone to come into view.
- **Tracking everyone:** each person in view gets a track: a Kalman filter that predicts where their box will be in
  the next result, from where it was, how it was moving, and how far the robot's own turns have moved the picture.
  Each new box goes to the track that expected it nearest, keeping the total misfit least (global nearest neighbour),
  and a box far from every track starts a new one. So someone the HuskyLens misses for a moment keeps their place,
  and nobody else takes it. Boxes of people standing in front of each other overlap and are unreliable, so they're
  skipped until the people part. A result that arrives while the picture is still showing a turn counts for less,
  since the camera's lag isn't known to the frame.
- **Picking whom to follow:** once someone has been seen in 3 results in a row, it picks the person **nearest the
  middle of the picture** (not the biggest), and follows their track from then on, however many people come and go.
  Anyone seen at the same time as them is someone else, and is never taken for them.
- **Calibrating as it locks on:** stand still while it turns toward you, a step at a time. Each step is a short turn,
  then a stop until the picture has caught up and settled (the HuskyLens's results lag about a quarter of a second
  behind the robot).
  How far you moved across the picture shows how fast the picture moves as it turns, which sizes the next step, and
  how long the picture took to move shows the lag. Turns of different lengths also show how long the robot keeps
  turning after a turn ends. After it locks on, it keeps learning from its turns toward you while you stand still
  (not while you walk, which would throw it off). What it measured is kept in `vision/follow_calibration.json` on
  the Pi, so it starts from there next time, even after a restart. Run the people counter with `--recalibrate` once
  to measure afresh after moving the camera or changing the wheels.
- **Locking on:** once you're in the middle, it locks on, and BMO says "Following you!". It follows about 1-2 feet
  behind you, judging the distance by how wide your box looks (`KEEP`: width still works up close, when the camera
  sees only your legs). As you walk it hangs back a little farther, as it needs room to stop when you do.
- **Following:** walk away, so you look narrower, and it drives forward, faster the farther you are, plus a little
  more the longer you've been farther than it keeps you (so it keeps up instead of trailing behind), up to
  `MAX_SPEED` (80%). It brakes as it closes in, and never reverses. As it drives it curves to bring you back to the
  middle. Since the picture lags behind, it steers by where you'll be once the picture catches up: where the filter
  has you, moved on by how you're moving and by the turns the picture doesn't show yet. Close up and off the middle,
  it turns on the spot toward you in one smooth turn, slowing as you near the middle, and stops so that you end up in
  it, allowing for how long it keeps turning after it stops. So it keeps moving while you do, and stops when you do.
- **Losing them:** if you vanish from the middle of the picture, the HuskyLens has most likely missed you for a
  moment (it misses people most up close), so it keeps going for 0.3 s (`COAST_S`) unless it's already close, then
  turns to face where it expects you and waits for up to 2 s (`LOST_S`). If you left by an edge, or dashed off faster
  than it could see, it turns that way for up to 6 s (`LEFT_S`): fast until where you left is in the middle, then
  slowly, since the camera can't make people out while the robot turns fast. Someone turning up near where it
  expected you is you again, unless they were seen alongside you, or look a good deal nearer (someone walking in front
  of you). After that BMO says "Lost you!" and it looks for you for up to 15 s (`REID_S`), turning slowly toward
  where it last saw you, no more than once round all told. It takes someone as you again only once they've been seen
  5 times in a row, looking about as far away as you did, and weren't seen alongside you, or where someone seen
  alongside you stood. Otherwise hold the screen to start afresh.
- **Fail-safe:** it stops if the people counter goes quiet for 0.5 s, if the watch is lost, or if control goes OFF.
  Turns on the spot last only as long as the people counter keeps asking for them, except while calibrating, when
  each comes with a time to stop, which the receiver keeps to by itself.

The people counter logs each step (`journalctl -u robot -f`), for example `Follow: Turned 0.13 s, they moved +159 px
after 0.23 s: the picture moves 1211 px/s, 0.27 s behind, and keeps moving 0.00 s after`, which shows what it has
measured.

**Try it with the wheels off the ground first.** Stand in front of the camera, turn control ON and hold the screen
for follow mode, standing still off to one side. It should turn toward you (in a few steps the first time, as it
calibrates) and lock on, then follow you as you move. If it turns away from you every time, set `MIRRORED = True` in
`follower.py`. The tuning constants are at the top of `follower.py`: `KEEP` (how close it follows: the share of the
picture's width you fill, so bigger is closer), `SPIN` and `SLOW_SPIN` (how fast it turns on the spot, and how slowly
when nearly facing you or looking for you: raise `SLOW_SPIN` if the robot stalls turning), `MIDDLE_PX` and `AHEAD_PX`
(how far off the middle someone can be before it turns on the spot, and before it turns at full speed), and the
speeds.

`python3 -m unittest vision/test_follower.py` also runs it in a simulated room: a robot whose camera lags behind it,
misses people, and can't make them out while the robot turns fast, with people standing, walking away, round
corners, stepping aside, passing between, and dashing off.

### Where this comes from

- Islam, Hong and Sattar, [Person Following by Autonomous Robots: A Categorical Overview](https://arxiv.org/pdf/1803.08202)
  (the survey: detection, tracking, identifying the target, and control)
- Ye et al., [Following Closely: A Robust Monocular Person Following System for Mobile Robot](https://arxiv.org/pdf/2204.10540)
  (following at close range with a single camera, by box width, where height fails)
- Koide, Miura and Menegatti, Monocular Person Tracking and Identification with On-line Deep Feature Selection for
  Person Following Robots ([paper](https://www.researchgate.net/publication/336871285_Monocular_Person_Tracking_and_Identification_with_On-line_Deep_Feature_Selection_for_Person_Following_Robots),
  [code](https://github.com/koide3/monocular_person_following)) (a Kalman-filter track per person, and re-identifying
  the target after losing them)
- [A person-following method based on monocular camera for quadruped robots](https://www.sciencedirect.com/science/article/pii/S2667379722000225)
- [Robot Person Following Under Partial Occlusion](https://www.alphaxiv.org/abs/2302.02121)
- [Robot Person Following in Uniform Crowd Environment](https://arxiv.org/pdf/2205.10553)
- [Human Following for Wheeled Robot with Monocular Pan-tilt Camera](https://arxiv.org/pdf/1909.06087)
- [Towards a Generic Diver-Following Algorithm](https://arxiv.org/pdf/1809.06849)
- [A Cost-Effective Person-Following System for Assistive Unmanned Vehicles with Deep Learning at the Edge](https://arxiv.org/pdf/2008.13474)
- The HuskyLens 2: [DFRobot wiki](https://wiki.dfrobot.com/sen0638/), [Object Tracking](https://wiki.dfrobot.com/sen0638/docs/22614),
  [DFRobot_HuskylensV2](https://github.com/DFRobot/DFRobot_HuskylensV2) (its protocol),
  [pyhuskylens](https://github.com/AntonsMindstorms/pyhuskylens). Its own Object Tracking follows one object you
  draw a box round, but running it alongside Object Recognition (its multi-algorithm mode) was unreliable when
  tried on the robot, so follow mode tracks people itself.

## Run everything on the Pi

`pi/run_all.py` runs the watch receiver and the people counter together, with both outputs in one terminal:

```
[watch] 14:16:47  FORWARD        speed  60%
[camera] 14:16:47  People in frame: 2
[camera] 14:17:02  People in frame: 0
```

- **Detection only runs while watch control is ON.** When you double-tap OFF, or the watch is lost, the people
  counter stops asking the HuskyLens for results and prints `Detection paused: watch control is OFF`. It picks up
  again as soon as control is back ON. The receiver shares the control state with the people counter
  on the Pi itself (UDP port 5006 on 127.0.0.1), whenever it changes and every 0.5 s. If the receiver stops
  sending, the counter treats that as OFF.
- **The watch says each new count.** The people counter tells the receiver the count (UDP port 5007 on
  127.0.0.1) whenever it changes and every 0.5 s, and the receiver sends it on to the watch: `{"people": 2}`, or
  `{"people": null}` while there's no count (control OFF, or no word from the people counter). The watch says each
  new count once, however often it hears it: "All clear!", "Person ahead!", "2 people ahead!". If the count changes
  again while it's still speaking, it says only the latest, once it's done.
- **In follow mode, the people counter also says how to drive.** The receiver tells it follow mode is on (`follow`
  on port 5006, in place of `on`), and it sends a drive command after each result (port 5008), which the receiver
  drives with. The receiver passes on to the watch whether there's someone to follow: `{"people": 1, "target":
  true}`.
- **They run as separate programs.** The people counter runs at lower CPU priority, so it never delays the
  watch's commands.
- **If the people counter stops** (for example, the Pi's UART isn't turned on), the watch receiver keeps running.
- **If the watch receiver stops,** everything stops.
- **Ctrl+C** stops both, and the receiver sends a final `STOP` and turns the motors off.

From the laptop, copy both folders into `~/robot` on the Pi, keeping them side by side:

```bash
ssh sofwerx@Pi5.local mkdir -p robot && scp -r pi vision sofwerx@Pi5.local:robot/
```

Turn on the Pi's UART once ([Set it up on the Pi](#set-it-up-on-the-pi)), then on the Pi:

```bash
python3 ~/robot/pi/run_all.py
python3 ~/robot/pi/run_all.py --baud 9600   # options after run_all.py go to the people counter
```

If `Pi5.local` doesn't resolve from the laptop, use the Pi's IP address instead (`hostname -I` on the Pi).

### Start it on boot

[`pi/robot.service`](pi/robot.service) runs `run_all.py` whenever the Pi starts, and starts it again 2 s after it
stops for any reason. Install it once, on the Pi:

```bash
sudo cp ~/robot/pi/robot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now robot     # start it now and on every boot
journalctl -u robot -f                # its output, live
```

The robot still only moves once you double-tap control ON on the watch. Stopping the service stops it the same
way Ctrl+C does, with a final `STOP` and the motors off.

While it runs, it has the watch's port, the HuskyLens and the motors. To run `receive.py`, `run_all.py` or the
motor test by hand, stop it first:

```bash
sudo systemctl stop robot             # until the next boot, or: sudo systemctl start robot
sudo systemctl disable --now robot    # stop starting it on boot
```

After copying new code to the Pi, `sudo systemctl restart robot` picks it up.

## Layout

| Path | What it is |
|---|---|
| `watch-app/` | Wear OS app (Kotlin, no dependencies beyond the Android SDK): calibration, gesture classification, double-tap on/off switch, bezel gears, follow mode, sends drive commands over UDP, says the people count in a BMO voice |
| `pi/receive.py` | Runs on the Pi: drives the motors with the watch's commands and prints them |
| `pi/motors.py` | Runs on the Pi: the PCA9685 and L298N motor drivers, and a motor test for checking the wiring |
| `pi/robot.service` | Starts `run_all.py` when the Pi boots |
| `pi/run_all.py` | Runs on the Pi: starts the watch receiver and the people counter together |
| `vision/people_counter.py` | Separate program: counts people in view of the HuskyLens 2 camera, and tells the watch the count |
| `vision/follower.py` | Follow mode: picks the person to follow and steers after them, for `people_counter.py` |
| `pi/test_*.py`, `vision/test_*.py` | Tests for the Pi side, no hardware needed: `python3 -m unittest discover -s pi` (and `-s vision`) |
| `watch-app/app/src/test/` | Tests for the watch's gestures, gears and callouts: `./gradlew testDebugUnitTest` |
