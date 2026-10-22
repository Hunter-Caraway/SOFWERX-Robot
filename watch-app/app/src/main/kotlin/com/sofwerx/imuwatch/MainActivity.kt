package com.sofwerx.imuwatch

import android.app.Activity
import android.graphics.Color
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.net.ConnectivityManager
import android.net.Network
import android.net.NetworkCapabilities
import android.net.NetworkRequest
import android.os.Bundle
import android.os.Handler
import android.os.HandlerThread
import android.os.Build
import android.os.Looper
import android.os.SystemClock
import android.os.VibrationEffect
import android.os.Vibrator
import android.os.VibratorManager
import android.util.TypedValue
import android.view.GestureDetector
import android.view.Gravity
import android.view.InputDevice
import android.view.MotionEvent
import android.view.View
import android.view.WindowManager
import android.widget.Button
import android.widget.LinearLayout
import android.widget.TextView
import org.json.JSONArray
import org.json.JSONException
import org.json.JSONObject
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.Inet4Address
import java.net.InetAddress
import java.util.Locale
import kotlin.math.roundToInt

/**
 * Turns wrist tilt into drive commands (see Gestures.kt) and streams them to the robot's
 * Raspberry Pi (PI_HOST) as JSON over UDP while the app is on screen.
 *
 * On opening, the app asks whether to calibrate. Calibrating walks through each Pose (tap the
 * screen with the other hand to record it) and saves the result; Start reuses the saved one.
 *
 * Once running, double-tapping the screen turns control on and off (the watch buzzes once for
 * on, twice for off). Control starts off each time the app comes to the foreground. Packets keep
 * streaming while it is off, with cmd STOP, so the receiver can tell "off" from "lost".
 *
 * Turning the bezel changes gear (see Gears.kt), which sets the robot's top speed. Holding a finger
 * on the screen switches follow mode on and off (three short buzzes on, one long one off): the
 * robot then follows the nearest person its camera sees, and wrist tilt does nothing. Double-tap
 * still turns control off, which stops it. Follow mode is off each time the app comes to the
 * foreground.
 *
 * Packet (one per gravity sample, ~50 Hz):
 *   {"on":true,"follow":false,"top":0.65,"cmd":"FORWARD-LEFT","speed":0.60,"turn":-0.40}
 * on is the control switch, follow is follow mode and top is the gear's top speed (0..1). cmd is
 * STOP, FORWARD, REVERSE, LEFT, RIGHT, FORWARD-LEFT, FORWARD-RIGHT, REVERSE-LEFT or REVERSE-RIGHT;
 * speed is 0..1 and turn is -1 (full left) .. +1 (full right), before the Pi scales them by top.
 *
 * The Pi sends back how many people its camera sees, and the watch says each new count on its
 * speaker at full volume: "All clear!", "Person ahead!", "2 people ahead!" (see Announcer.kt). In
 * follow mode it also says whether the robot has found someone to follow.
 */
class MainActivity : Activity(), SensorEventListener {

    private enum class Mode { ASK, CALIBRATING, RUNNING }

    private lateinit var sensorManager: SensorManager
    private lateinit var connectivity: ConnectivityManager
    private lateinit var vibrator: Vibrator
    private lateinit var rootView: LinearLayout
    private lateinit var titleView: TextView
    private lateinit var detailView: TextView
    private lateinit var statusView: TextView
    private lateinit var askButtons: LinearLayout
    private lateinit var startButton: Button
    private lateinit var announcer: Announcer

    private val mainHandler = Handler(Looper.getMainLooper())
    private var sensorThread: HandlerThread? = null

    // Main thread only.
    private var mode = Mode.ASK
    private var savedCalibration: Calibration? = null
    private var step = 0  // index into Pose.entries while calibrating
    private val poses = HashMap<Pose, DoubleArray>()
    private var sampling = false
    private var calibrationFailed = false
    private var rate = 0L
    private var rateSince = 0L
    private var sentAtRate = 0L
    private var people: Int? = null  // the Pi's latest people count
    private var following: Boolean? = null  // in follow mode, whether the robot has someone to follow
    private val rotary = RotaryClicks()
    private var saidGear = Gear.MEDIUM  // the gear BMO said last

    // Written only on the sensor thread.
    private val gravityFilter = LowPass(SMOOTHING_S)

    private val recordLock = Any()
    private var recording: MutableList<DoubleArray>? = null  // guarded by recordLock

    @Volatile private var classifier: GestureClassifier? = null  // set while running
    @Volatile private var drive = Drive.STOP
    @Volatile private var socket: DatagramSocket? = null
    @Volatile private var target: InetAddress? = null
    @Volatile private var sent = 0L
    @Volatile private var lastError: String? = null
    @Volatile private var controlOn = false
    @Volatile private var follow = false
    @Volatile private var gear = Gear.MEDIUM
    @Volatile private var listener: Thread? = null  // the thread hearing the Pi's replies

    // Wear OS prefers Bluetooth through the phone and powers Wi-Fi down when idle;
    // requesting a Wi-Fi network keeps it up while we stream.
    private val networkCallback = object : ConnectivityManager.NetworkCallback() {
        override fun onAvailable(network: Network) {
            closeSocket()
            try {
                val s = DatagramSocket()
                network.bindSocket(s)
                socket = s
                lastError = null
            } catch (e: Exception) {
                lastError = e.message
            }
            resolveTarget(network)
        }

        override fun onLost(network: Network) {
            closeSocket()
        }
    }

    private val uiTick = object : Runnable {
        override fun run() {
            updateStatus()
            mainHandler.postDelayed(this, UI_INTERVAL_MS)
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)

        sensorManager = getSystemService(SensorManager::class.java)
        connectivity = getSystemService(ConnectivityManager::class.java)

        vibrator = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            getSystemService(VibratorManager::class.java).defaultVibrator
        } else {
            @Suppress("DEPRECATION")
            getSystemService(Vibrator::class.java)
        }

        titleView = label()
        detailView = label()
        statusView = label()
        val calibrateButton = Button(this).apply {
            text = "Calibrate"
            setOnClickListener { startCalibration() }
        }
        startButton = Button(this).apply {
            text = "Start"
            setOnClickListener { savedCalibration?.let { startRunning(it) } }
        }
        askButtons = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            gravity = Gravity.CENTER
            val margin = (4 * resources.displayMetrics.density).toInt()
            for (b in listOf(startButton, calibrateButton)) {
                addView(b, LinearLayout.LayoutParams(
                    LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT,
                ).apply { topMargin = margin })
            }
        }
        rootView = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            gravity = Gravity.CENTER
            val pad = (resources.displayMetrics.widthPixels * 0.12f).toInt()
            setPadding(pad, pad, pad, pad)
            addView(titleView)
            addView(detailView)
            addView(statusView)
            addView(askButtons)
        }
        val taps = GestureDetector(this, object : GestureDetector.SimpleOnGestureListener() {
            override fun onDown(e: MotionEvent) = true
            override fun onSingleTapConfirmed(e: MotionEvent): Boolean {
                if (mode == Mode.CALIBRATING) onCalibrationTap()
                return true
            }
            override fun onDoubleTap(e: MotionEvent): Boolean {
                if (mode == Mode.RUNNING) setControl(!controlOn)
                return true
            }
            override fun onLongPress(e: MotionEvent) {
                if (mode == Mode.RUNNING) setFollow(!follow)
            }
        })
        rootView.setOnTouchListener { _, e -> taps.onTouchEvent(e) }
        setContentView(rootView)

        announcer = Announcer(this)
        gear = loadGear()
        saidGear = gear
        savedCalibration = loadCalibration()
        showAsk()
    }

    override fun onResume() {
        super.onResume()
        startSensors()
        val request = NetworkRequest.Builder()
            .addTransportType(NetworkCapabilities.TRANSPORT_WIFI)
            .build()
        connectivity.requestNetwork(request, networkCallback)
        startListening()
        rateSince = SystemClock.uptimeMillis()
        sentAtRate = sent
        mainHandler.post(uiTick)
    }

    override fun onPause() {
        // Never come back to the foreground already driving.
        controlOn = false
        follow = false
        following = null
        showControl()
        mainHandler.removeCallbacksAndMessages(null)  // the UI tick and any pose being recorded
        if (sampling) {
            sampling = false
            stopRecording()
            showStep("Tap to record this pose again")
        }
        connectivity.unregisterNetworkCallback(networkCallback)
        stopSensors()
        listener = null
        people = null
        closeSocket()  // also ends the listening thread
        super.onPause()
    }

    override fun onDestroy() {
        announcer.shutdown()
        super.onDestroy()
    }

    /**
     * Looks the Pi up on the Wi-Fi network (blocking). The receiver listens on IPv4 only. If the
     * watch can't resolve the Pi's .local name, falls back to the Pi's last known IP.
     */
    private fun resolveTarget(network: Network) {
        target = try {
            network.getAllByName(PI_HOST).firstOrNull { it is Inet4Address }
        } catch (e: Exception) {
            null
        } ?: InetAddress.getByName(PI_FALLBACK_IP)
    }

    // --- screens ----------------------------------------------------------

    private fun label() = TextView(this).apply {
        gravity = Gravity.CENTER
        setTextColor(Color.WHITE)
    }

    /** Shows text at the given size, or hides the view when text is null. */
    private fun TextView.show(text: CharSequence?, sp: Float) {
        this.text = text
        setTextSize(TypedValue.COMPLEX_UNIT_SP, sp)
        visibility = if (text == null) View.GONE else View.VISIBLE
    }

    private fun showAsk() {
        mode = Mode.ASK
        rootView.setBackgroundColor(COLOR_SETUP)
        val saved = savedCalibration != null
        titleView.show("Calibrate?", 28f)
        detailView.show(if (saved) "Start uses your saved calibration" else "No saved calibration yet", 14f)
        statusView.show(null, 12f)
        askButtons.visibility = View.VISIBLE
        startButton.visibility = if (saved) View.VISIBLE else View.GONE
    }

    private fun startCalibration() {
        classifier = null
        controlOn = false
        mode = Mode.CALIBRATING
        step = 0
        poses.clear()
        calibrationFailed = false
        rootView.setBackgroundColor(COLOR_SETUP)
        askButtons.visibility = View.GONE
        showStep(null)
    }

    private fun showStep(note: String?) {
        titleView.show("Step ${step + 1} of ${Pose.entries.size}", 14f)
        detailView.show(Pose.entries[step].prompt, 16f)
        statusView.show(note ?: "Hold it, then tap the screen with your other hand", 12f)
    }

    private fun onCalibrationTap() {
        if (sampling) return
        if (calibrationFailed) {
            startCalibration()
            return
        }
        sampling = true
        statusView.text = "Hold still…"
        // Wait for the tap's jolt to settle before recording.
        mainHandler.postDelayed({
            synchronized(recordLock) { recording = ArrayList() }
            mainHandler.postDelayed({ finishPose() }, SAMPLE_MS)
        }, SETTLE_MS)
    }

    private fun stopRecording(): List<DoubleArray> = synchronized(recordLock) {
        val samples = recording.orEmpty()
        recording = null
        samples
    }

    private fun finishPose() {
        sampling = false
        val samples = stopRecording()
        if (samples.size < MIN_POSE_SAMPLES) {
            showStep("Only ${samples.size} readings came in. Tap to try again.")
            return
        }
        poses[Pose.entries[step]] = direction(samples)
        step++
        if (step < Pose.entries.size) {
            buzz(60)
            showStep(null)
            return
        }
        try {
            val cal = buildCalibration(poses)
            saveCalibration(poses)
            savedCalibration = cal
            buzz(250)
            startRunning(cal)
        } catch (e: CalibrationException) {
            calibrationFailed = true
            vibrator.vibrate(VibrationEffect.createWaveform(longArrayOf(0, 80, 120, 80, 120, 80), -1))
            titleView.show("Calibration failed", 16f)
            detailView.show(e.message, 14f)
            statusView.show("Tap to start over", 12f)
        }
    }

    private fun startRunning(cal: Calibration) {
        classifier = GestureClassifier(cal)
        controlOn = false
        mode = Mode.RUNNING
        askButtons.visibility = View.GONE
        showControl()
        updateStatus()
    }

    private fun setControl(on: Boolean) {
        controlOn = on
        showControl()
        updateStatus()
        val effect = if (on) {
            VibrationEffect.createOneShot(250, VibrationEffect.DEFAULT_AMPLITUDE)
        } else {
            VibrationEffect.createWaveform(longArrayOf(0, 80, 120, 80), -1)
        }
        vibrator.vibrate(effect)
    }

    /** Follow mode on or off. */
    private fun setFollow(on: Boolean) {
        follow = on
        following = null
        announcer.onTarget(null)
        updateStatus()
        val effect = if (on) {
            VibrationEffect.createWaveform(longArrayOf(0, 60, 80, 60, 80, 60), -1)
        } else {
            VibrationEffect.createOneShot(400, VibrationEffect.DEFAULT_AMPLITUDE)
        }
        vibrator.vibrate(effect)
    }

    /** The bezel changes gear while running. */
    override fun dispatchGenericMotionEvent(ev: MotionEvent): Boolean {
        if (mode != Mode.RUNNING || ev.action != MotionEvent.ACTION_SCROLL ||
            !ev.isFromSource(InputDevice.SOURCE_ROTARY_ENCODER)
        ) {
            return super.dispatchGenericMotionEvent(ev)
        }
        // Turning clockwise reads negative (Wear OS scrolls down for it).
        val clicks = rotary.add(-ev.getAxisValue(MotionEvent.AXIS_SCROLL))
        if (clicks != 0) shiftGear(clicks)
        return true
    }

    private fun shiftGear(clicks: Int) {
        val next = gear.shift(clicks)
        if (next == gear) return
        gear = next
        getSharedPreferences(SETTINGS, MODE_PRIVATE).edit().putString(SETTINGS_GEAR, next.name).apply()
        buzz(30)
        updateStatus()
        // Say the gear once the bezel stops turning, not every click on the way.
        mainHandler.removeCallbacks(sayGear)
        mainHandler.postDelayed(sayGear, GEAR_SAY_DELAY_MS)
    }

    private val sayGear = Runnable {
        if (gear != saidGear) {
            announcer.say("${gear.label}!")
            saidGear = gear
        }
    }

    private fun loadGear(): Gear {
        val saved = getSharedPreferences(SETTINGS, MODE_PRIVATE).getString(SETTINGS_GEAR, null)
        return Gear.entries.firstOrNull { it.name == saved } ?: Gear.MEDIUM
    }

    private fun showControl() {
        if (mode != Mode.RUNNING) return
        rootView.setBackgroundColor(if (controlOn) COLOR_ON else COLOR_OFF)
        titleView.show(if (controlOn) "ON" else "OFF", 40f)
    }

    private fun buzz(ms: Long) {
        vibrator.vibrate(VibrationEffect.createOneShot(ms, VibrationEffect.DEFAULT_AMPLITUDE))
    }

    // --- calibration storage ----------------------------------------------

    /** Stores the recorded poses; the calibration is rebuilt from them on load. */
    private fun saveCalibration(poses: Map<Pose, DoubleArray>) {
        val json = JSONObject()
        for ((pose, v) in poses) json.put(pose.name, JSONArray(v.toList()))
        getSharedPreferences(PREFS, MODE_PRIVATE).edit().putString(PREFS_POSES, json.toString()).apply()
    }

    private fun loadCalibration(): Calibration? {
        val saved = getSharedPreferences(PREFS, MODE_PRIVATE).getString(PREFS_POSES, null) ?: return null
        return try {
            val json = JSONObject(saved)
            buildCalibration(Pose.entries.associateWith { pose ->
                val a = json.getJSONArray(pose.name)
                DoubleArray(3) { a.getDouble(it) }
            })
        } catch (e: JSONException) {
            null
        } catch (e: CalibrationException) {
            null
        }
    }

    // --- sensors and network ----------------------------------------------

    private fun startSensors() {
        val thread = HandlerThread("imu").also { it.start() }
        sensorThread = thread
        val sensor = sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY)
            ?: sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)  // gravity plus the hand's own motion
        sensor?.let { sensorManager.registerListener(this, it, SAMPLE_PERIOD_US, Handler(thread.looper)) }
    }

    private fun stopSensors() {
        sensorManager.unregisterListener(this)
        sensorThread?.quitSafely()
        sensorThread = null
    }

    private fun closeSocket() {
        socket?.close()
        socket = null
    }

    /** Hears the Pi's replies (the people count, and how following goes) on the socket the commands go out on. */
    private fun startListening() {
        val thread = Thread({
            val buf = ByteArray(512)
            while (listener === Thread.currentThread()) {
                val s = socket
                if (s == null) {
                    SystemClock.sleep(100)
                    continue
                }
                val packet = DatagramPacket(buf, buf.size)
                try {
                    s.receive(packet)
                } catch (e: Exception) {
                    SystemClock.sleep(100)  // the socket closed; Wi-Fi coming back opens a new one
                    continue
                }
                val status = parseStatus(String(packet.data, 0, packet.length, Charsets.UTF_8)) ?: continue
                mainHandler.post { onStatus(status) }
            }
        }, "replies")
        listener = thread  // before it starts, or it would stop straight away
        thread.start()
    }

    private fun onStatus(status: PiStatus) {
        if (listener == null) return
        people = status.people
        announcer.onPeople(status.people)
        following = if (follow) status.target else null  // the Pi may not have heard yet that follow mode is off
        announcer.onTarget(following)
    }

    override fun onSensorChanged(event: SensorEvent) {
        val v = event.values
        synchronized(recordLock) { recording?.add(DoubleArray(3) { v[it].toDouble() }) }
        val g = gravityFilter.update(v, event.timestamp)
        val clf = classifier
        val d = if (controlOn && clf != null) {
            clf.update(g, event.timestamp / 1e9)
        } else {
            clf?.reset()
            Drive.STOP
        }
        drive = d
        send(d)
    }

    override fun onAccuracyChanged(sensor: Sensor, accuracy: Int) {}

    private fun send(d: Drive) {
        val s = socket ?: return
        val addr = target ?: return
        val msg = String.format(
            Locale.US, "{\"on\":%b,\"follow\":%b,\"top\":%.2f,\"cmd\":\"%s\",\"speed\":%.2f,\"turn\":%.2f}",
            controlOn, follow, gear.top, d.cmd, d.speed, d.turn,
        ).toByteArray()
        try {
            s.send(DatagramPacket(msg, msg.size, addr, PI_PORT))
            sent++
        } catch (e: Exception) {
            lastError = e.message
        }
    }

    private fun updateStatus() {
        val now = SystemClock.uptimeMillis()
        if (now - rateSince >= 1000) {
            val total = sent
            rate = (total - sentAtRate) * 1000 / (now - rateSince)
            sentAtRate = total
            rateSince = now
        }
        if (mode != Mode.RUNNING) return

        val d = drive
        detailView.show(if (follow) "FOLLOW" else if (controlOn) d.cmd else null, 20f)
        statusView.show(buildString {
            if (controlOn && follow) {
                appendLine(describeTarget(following))
            } else if (controlOn) {
                appendLine(String.format(  // the speed after the gear
                    Locale.US, "speed %d%% · turn %+d%%",
                    (d.speed * gear.top * 100).roundToInt(), (d.turn * 100).roundToInt(),
                ))
            }
            people?.let { appendLine(describePeople(it)) }
            appendLine("gear: ${gear.label} (${(gear.top * 100).roundToInt()}%)")
            appendLine("double-tap: ${if (controlOn) "off" else "on"} · hold: ${if (follow) "wrist" else "follow"}")
            appendLine("→ $PI_HOST:$PI_PORT")
            target?.hostAddress?.let { appendLine("($it)") }
            append(if (socket != null) "Wi-Fi: up · $rate Hz" else "Wi-Fi: connecting…")
            lastError?.let { append("\n$it") }
            announcer.problem?.let { append("\n$it") }
        }, 12f)
    }

    companion object {
        // The Raspberry Pi on the robot. PI_FALLBACK_IP is used if the watch can't resolve
        // the .local name; update it if the Pi's address changes.
        const val PI_HOST = "Pi5.local"
        const val PI_FALLBACK_IP = "10.4.3.229"
        const val PI_PORT = 5005
        const val SAMPLE_PERIOD_US = 20_000 // 50 Hz
        const val SMOOTHING_S = 0.1         // time constant of the low-pass filter on gravity
        const val UI_INTERVAL_MS = 100L
        const val SETTLE_MS = 1000L         // pause after the tap before recording a pose
        const val SAMPLE_MS = 1500L         // how long each pose is averaged
        const val MIN_POSE_SAMPLES = 10
        const val PREFS = "calibration"
        const val PREFS_POSES = "poses"
        const val SETTINGS = "settings"
        const val SETTINGS_GEAR = "gear"
        const val GEAR_SAY_DELAY_MS = 700L  // BMO says the gear once the bezel has been still this long
        val COLOR_ON = Color.rgb(0, 110, 40)
        val COLOR_OFF = Color.rgb(120, 0, 0)
        val COLOR_SETUP = Color.rgb(20, 40, 90)
    }
}
