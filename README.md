# SOFWERX Robot

Drive a robot with wrist gestures from a smartwatch. A Wear OS app on a Galaxy Watch streams its motion sensors
over Wi-Fi to a laptop. A Python script on the laptop turns wrist tilt and roll into drive commands
(forward / stop / left / right, with proportional speed and turn).

## What it does

| Gesture (relative to your calibrated neutral pose) | Command |
|---|---|
| Hand tilted down | `FORWARD`. Speed ramps from 0 to 100% as you tilt toward your full-speed pose |
| Hand level | `NEUTRAL` |
| Hand tilted up | `STOP`. Takes effect immediately and overrides everything else |
| Hand rolled left / right | `LEFT` / `RIGHT`. Turn ramps from 0 to ±100% |
| Tilted down and rolled | `FORWARD+LEFT` / `FORWARD+RIGHT` |

- **Double-tap the watch** to turn control on and off. The screen goes green for ON and red for OFF, and the
  watch buzzes once for on and twice for off. Control always starts OFF when the app opens.
- **Fail-safe:** the output is `STOP` whenever control is off or no packets have arrived for 0.5 s (watch out of
  range, app closed, Wi-Fi dropped).
- **Per-user calibration:** you hold 7 poses once (neutral, full speed, full stop, max left/right, and max
  left/right while at full speed). The script derives the pitch and roll axes from the gravity vectors, so it
  works on either wrist with any watch orientation. Turning is calibrated at both neutral and full speed,
  because tilting your hand down shifts the roll reading.
- **Jitter rejection:** gravity is low-pass filtered, commands have dead zones and hysteresis, and
  FORWARD/LEFT/RIGHT must be held for 150 ms before they switch on.

### How it works

```
Galaxy Watch (watch-app)                         Laptop (laptop/watch_control.py)
  gravity, accel, gyro, rotation @ 50 Hz  ──UDP:5005 JSON──►  smooth → pitch/roll vs. calibration
  double-tap ON/OFF switch                                     → gesture classifier → send_command(cmd, speed, turn)
```

Each packet looks like this:
`{"seq":1,"t":123,"on":true,"grav":[x,y,z],"acc":[x,y,z],"gyr":[x,y,z],"rot":[x,y,z,w]}`

### Hooking up the robot

The robot interface is `send_command(cmd, speed, turn)` in
[`laptop/watch_control.py`](laptop/watch_control.py). For now it just prints the command. Replace its body with
whatever drives your robot (serial, ROS, HTTP, etc.). It's called whenever the command changes or speed/turn
move by 5%:

- `cmd` is one of `STOP`, `NEUTRAL`, `FORWARD`, `LEFT`, `RIGHT`, `FORWARD+LEFT`, `FORWARD+RIGHT`
- `speed` runs from 0 to 1
- `turn` runs from -1 (full left) to +1 (full right)

## Run it yourself

### What you need

- A Wear OS 3+ watch (Android API 30+; built and tested on a Samsung Galaxy Watch)
- A laptop on the **same Wi-Fi network** as the watch, with Python 3.8+ (standard library only; nothing to `pip install`)
- `adb` (Android platform-tools)
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
```

You can also open `watch-app/` in Android Studio and run it.

### 3. Run the controller on the laptop

```bash
python3 laptop/watch_control.py
```

The script:

1. starts **IMU Stream** on the watch over adb, already pointed at the laptop's IP;
2. walks you through calibration the first time (hold each pose with the watch hand, press Enter with the
   other), and saves it to `laptop/calibration.json`;
3. prints live tilt and roll plus the current command. Double-tap the watch to turn control on.

Options:

```bash
python3 laptop/watch_control.py --calibrate   # redo the calibration
python3 laptop/watch_control.py --no-launch   # open IMU Stream on the watch yourself (no adb needed)
python3 laptop/watch_control.py --host 192.168.1.20 --port 5005   # override the auto-detected IP / port
```

If you use `--no-launch`, the watch app needs to know where to send its data. The address is remembered after
the first launch, or you can set it once:

```bash
adb shell am start -n com.sofwerx.imuwatch/.MainActivity --es host <laptop-ip> --ei port 5005
```

**Troubleshooting:**

- If the script waits forever for data, check that the laptop's firewall allows UDP port 5005 in. On Fedora:
  `sudo firewall-cmd --add-port=5005/udp`.
- Also check that the watch screen shows `Wi-Fi: up` and a non-zero Hz.

## Layout

| Path | What it is |
|---|---|
| `watch-app/` | Wear OS app (Kotlin, no dependencies beyond the Android SDK): streams sensors over UDP, double-tap on/off switch |
| `laptop/watch_control.py` | Calibration, gesture classification, and the `send_command` hook for the robot |
