package com.sofwerx.imuwatch

import android.app.Activity
import android.content.Context
import android.content.Intent
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
import android.os.VibrationEffect
import android.os.Vibrator
import android.os.VibratorManager
import android.util.TypedValue
import android.view.GestureDetector
import android.view.Gravity
import android.view.MotionEvent
import android.view.WindowManager
import android.widget.LinearLayout
import android.widget.TextView
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.util.Locale
import kotlin.math.max
import kotlin.math.sqrt

/**
 * Streams IMU readings to the laptop as JSON over UDP while the app is on screen.
 *
 * The laptop address is passed as launch extras and remembered for later launches:
 *   adb shell am start -n com.sofwerx.imuwatch/.MainActivity --es host 10.4.2.186 --ei port 5005
 * laptop/watch_control.py does this automatically.
 *
 * Double-tapping the screen turns control on and off (the watch buzzes once for on,
 * twice for off). Control starts off each time the app comes to the foreground, and
 * readings keep streaming while it is off so the laptop can tell "off" from "lost".
 *
 * Packet (one per gravity sample, ~50 Hz):
 *   {"seq":1,"t":123,"on":true,"grav":[x,y,z],"acc":[x,y,z],"gyr":[x,y,z],"rot":[x,y,z,w]}
 * on is the control switch, grav/acc are m/s^2, gyr is rad/s, rot is the game rotation
 * vector quaternion, t is the sensor timestamp in ms, all in the watch's sensor frame.
 */
class MainActivity : Activity(), SensorEventListener {

    private lateinit var sensorManager: SensorManager
    private lateinit var connectivity: ConnectivityManager
    private lateinit var vibrator: Vibrator
    private lateinit var rootView: LinearLayout
    private lateinit var stateView: TextView
    private lateinit var statusView: TextView

    private val mainHandler = Handler(Looper.getMainLooper())
    private var sensorThread: HandlerThread? = null
    private var hasGravitySensor = false

    // Written only on the sensor thread.
    private val acc = FloatArray(3)
    private val gyr = FloatArray(3)
    private val grav = FloatArray(3)
    private val rot = FloatArray(4)
    private var seq = 0L

    @Volatile private var socket: DatagramSocket? = null
    @Volatile private var target: InetAddress? = null
    @Volatile private var host = ""
    @Volatile private var port = DEFAULT_PORT
    @Volatile private var sent = 0L
    @Volatile private var lastError: String? = null
    @Volatile private var controlOn = false
    private var sentAtLastTick = 0L

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
        }

        override fun onLost(network: Network) {
            closeSocket()
        }
    }

    private val statusTick = object : Runnable {
        override fun run() {
            updateStatus()
            mainHandler.postDelayed(this, STATUS_INTERVAL_MS)
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

        stateView = TextView(this).apply {
            gravity = Gravity.CENTER
            setTextSize(TypedValue.COMPLEX_UNIT_SP, 40f)
            setTextColor(Color.WHITE)
        }
        statusView = TextView(this).apply {
            gravity = Gravity.CENTER
            setTextSize(TypedValue.COMPLEX_UNIT_SP, 12f)
            setTextColor(Color.WHITE)
        }
        rootView = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            gravity = Gravity.CENTER
            val pad = (resources.displayMetrics.widthPixels * 0.12f).toInt()
            setPadding(pad, pad, pad, pad)
            addView(stateView)
            addView(statusView)
        }
        val taps = GestureDetector(this, object : GestureDetector.SimpleOnGestureListener() {
            override fun onDown(e: MotionEvent) = true
            override fun onDoubleTap(e: MotionEvent): Boolean {
                setControl(!controlOn)
                return true
            }
        })
        rootView.setOnTouchListener { _, e -> taps.onTouchEvent(e) }
        setContentView(rootView)
        showControl()

        loadTarget(intent)
    }

    override fun onNewIntent(intent: Intent) {
        super.onNewIntent(intent)
        setIntent(intent)
        loadTarget(intent)
    }

    override fun onResume() {
        super.onResume()
        startSensors()
        val request = NetworkRequest.Builder()
            .addTransportType(NetworkCapabilities.TRANSPORT_WIFI)
            .build()
        connectivity.requestNetwork(request, networkCallback)
        mainHandler.post(statusTick)
    }

    override fun onPause() {
        // Never come back to the foreground already driving.
        controlOn = false
        showControl()
        mainHandler.removeCallbacks(statusTick)
        connectivity.unregisterNetworkCallback(networkCallback)
        stopSensors()
        closeSocket()
        super.onPause()
    }

    private fun loadTarget(intent: Intent?) {
        val prefs = getSharedPreferences("target", Context.MODE_PRIVATE)
        val edit = prefs.edit()
        intent?.getStringExtra("host")?.let { edit.putString("host", it) }
        if (intent?.hasExtra("port") == true) edit.putInt("port", intent.getIntExtra("port", DEFAULT_PORT))
        edit.apply()

        host = prefs.getString("host", "") ?: ""
        port = prefs.getInt("port", DEFAULT_PORT)
        target = try {
            if (host.isEmpty()) null else InetAddress.getByName(host)
        } catch (e: Exception) {
            lastError = "bad host: $host"
            null
        }
    }

    private fun setControl(on: Boolean) {
        controlOn = on
        showControl()
        val effect = if (on) {
            VibrationEffect.createOneShot(250, VibrationEffect.DEFAULT_AMPLITUDE)
        } else {
            VibrationEffect.createWaveform(longArrayOf(0, 80, 120, 80), -1)
        }
        vibrator.vibrate(effect)
    }

    private fun showControl() {
        stateView.text = if (controlOn) "ON" else "OFF"
        rootView.setBackgroundColor(if (controlOn) COLOR_ON else COLOR_OFF)
    }

    private fun startSensors() {
        val thread = HandlerThread("imu").also { it.start() }
        sensorThread = thread
        val handler = Handler(thread.looper)

        hasGravitySensor = sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) != null
        for (type in SENSOR_TYPES) {
            val sensor = sensorManager.getDefaultSensor(type) ?: continue
            sensorManager.registerListener(this, sensor, SAMPLE_PERIOD_US, handler)
        }
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

    override fun onSensorChanged(event: SensorEvent) {
        val v = event.values
        when (event.sensor.type) {
            Sensor.TYPE_GRAVITY -> v.copyInto(grav, 0, 0, 3)
            Sensor.TYPE_GYROSCOPE -> v.copyInto(gyr, 0, 0, 3)
            Sensor.TYPE_ACCELEROMETER -> {
                v.copyInto(acc, 0, 0, 3)
                if (!hasGravitySensor) v.copyInto(grav, 0, 0, 3)
            }
            Sensor.TYPE_GAME_ROTATION_VECTOR -> {
                v.copyInto(rot, 0, 0, 3)
                rot[3] = if (v.size > 3) v[3] else sqrt(max(0f, 1f - v[0] * v[0] - v[1] * v[1] - v[2] * v[2]))
            }
        }

        val trigger = if (hasGravitySensor) Sensor.TYPE_GRAVITY else Sensor.TYPE_ACCELEROMETER
        if (event.sensor.type == trigger) send(event.timestamp / 1_000_000)
    }

    override fun onAccuracyChanged(sensor: Sensor, accuracy: Int) {}

    private fun send(timeMs: Long) {
        val s = socket ?: return
        val addr = target ?: return
        val msg = String.format(
            Locale.US,
            "{\"seq\":%d,\"t\":%d,\"on\":%b,\"grav\":[%.4f,%.4f,%.4f],\"acc\":[%.4f,%.4f,%.4f]," +
                "\"gyr\":[%.4f,%.4f,%.4f],\"rot\":[%.5f,%.5f,%.5f,%.5f]}",
            seq++, timeMs, controlOn,
            grav[0], grav[1], grav[2],
            acc[0], acc[1], acc[2],
            gyr[0], gyr[1], gyr[2],
            rot[0], rot[1], rot[2], rot[3],
        ).toByteArray()
        try {
            s.send(DatagramPacket(msg, msg.size, addr, port))
            sent++
        } catch (e: Exception) {
            lastError = e.message
        }
    }

    private fun updateStatus() {
        val total = sent
        val rate = (total - sentAtLastTick) * 1000 / STATUS_INTERVAL_MS
        sentAtLastTick = total

        statusView.text = buildString {
            appendLine("double-tap to turn ${if (controlOn) "off" else "on"}")
            if (host.isEmpty()) {
                appendLine("No laptop set")
                append("Run watch_control.py")
                return@buildString
            }
            appendLine("→ $host:$port")
            appendLine(if (socket != null) "Wi-Fi: up" else "Wi-Fi: connecting…")
            append("$rate Hz · $total sent")
            lastError?.let { append("\n$it") }
        }
    }

    companion object {
        const val DEFAULT_PORT = 5005
        const val SAMPLE_PERIOD_US = 20_000 // 50 Hz
        const val STATUS_INTERVAL_MS = 500L
        val COLOR_ON = Color.rgb(0, 110, 40)
        val COLOR_OFF = Color.rgb(120, 0, 0)
        val SENSOR_TYPES = listOf(
            Sensor.TYPE_GRAVITY,
            Sensor.TYPE_ACCELEROMETER,
            Sensor.TYPE_GYROSCOPE,
            Sensor.TYPE_GAME_ROTATION_VECTOR,
        )
    }
}
