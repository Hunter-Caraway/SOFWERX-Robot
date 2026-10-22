package com.sofwerx.imuwatch

import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.exp
import kotlin.math.max
import kotlin.math.roundToInt
import kotlin.math.sin
import kotlin.math.sqrt

/*
 * Wrist tilt -> drive commands, measured against the user's calibrated poses:
 *   hand level          -> STOP
 *   tilted down / up    -> FORWARD / REVERSE, speed ramping up to 1 at the calibrated full-speed pose
 *   rolled left / right -> LEFT / RIGHT, turn ramping up to -1 / +1 at the calibrated maximum roll
 * Tilt and roll are read independently, so they combine, e.g. FORWARD-LEFT or REVERSE-RIGHT.
 *
 * To ignore wrist jitter, gravity is smoothed (LowPass), commands need a clear tilt before they
 * trigger (DEADZONE_FRACTION / MIN_ENTER_DEG) and must be held for HOLD_S before they switch on.
 * Going back to level stops immediately.
 */

private const val MIN_POSE_DEG = 15.0       // calibration poses must be at least this far from level
private const val DEADZONE_FRACTION = 0.25  // commands trigger this far toward their calibrated pose
private const val MIN_ENTER_DEG = 10.0      // ...but never closer to level than this
private const val EXIT_RATIO = 0.7          // once active, a command holds until tilt drops below 70% of its trigger
private const val HOLD_S = 0.15             // a command must stay past its trigger this long to switch on
private const val RESET_GAP_S = 0.5         // LowPass starts over after a gap this long between samples

/** What the robot should do. speed is 0..1 (the direction is in cmd); turn is -1 (full left) .. +1 (full right). */
data class Drive(val cmd: String, val speed: Double, val turn: Double) {
    companion object {
        val STOP = Drive("STOP", 0.0, 0.0)
    }
}

/** The calibration poses, in the order the watch asks for them. */
enum class Pose(val prompt: String) {
    LEVEL("Hold your hand level, where you want STOP"),
    FORWARD("Tilt your hand DOWN to full speed FORWARD"),
    REVERSE("Tilt your hand UP to full speed REVERSE"),
    LEFT("Hand level, roll it as far LEFT as you can"),
    RIGHT("Hand level, roll it as far RIGHT as you can"),
    FORWARD_LEFT("Tilt DOWN to full speed, then roll as far LEFT as you can"),
    FORWARD_RIGHT("Tilt DOWN to full speed, then roll as far RIGHT as you can"),
}

class CalibrationException(message: String) : Exception(message)

/** Pitch (+ = down) and roll (+ = right) in degrees from the level pose. */
data class Tilt(val pitch: Double, val roll: Double)

/** Roll relative to "straight" at the current pitch, and how far LEFT/RIGHT reach full turn there. */
class Turn(val roll: Double, val leftDeg: Double, val rightDeg: Double)

/**
 * The user's calibrated axes and ranges. Gravity directions are unit vectors in the watch's
 * sensor frame; angles are in degrees.
 */
class Calibration(
    val neutral: DoubleArray,
    val pitchAxis: DoubleArray,
    val rollAxis: DoubleArray,
    val forwardDeg: Double,
    val reverseDeg: Double,
    val leftDeg: Double,
    val rightDeg: Double,
    // Tilting the hand shifts the roll reading, so "straight" is also measured at full speed
    // both ways, and the turn ranges at full forward speed.
    val forwardCenterDeg: Double,
    val forwardLeftDeg: Double,
    val forwardRightDeg: Double,
    val reverseCenterDeg: Double,
) {
    fun tilt(gravity: DoubleArray) = tiltAngles(neutral, pitchAxis, rollAxis, gravity)

    /**
     * Blends linearly from the level calibration to the full-speed one as the hand tilts. Reverse
     * only shifts "straight" and keeps the level turn ranges, so it needs no extra poses.
     */
    fun turnAt(t: Tilt): Turn {
        if (t.pitch >= 0) {
            val f = (t.pitch / forwardDeg).coerceIn(0.0, 1.0)
            return Turn(
                t.roll - f * forwardCenterDeg,
                leftDeg + f * (forwardLeftDeg - leftDeg),
                rightDeg + f * (forwardRightDeg - rightDeg),
            )
        }
        val f = (-t.pitch / reverseDeg).coerceIn(0.0, 1.0)
        return Turn(t.roll - f * reverseCenterDeg, leftDeg, rightDeg)
    }
}

/**
 * Derives the pitch and roll axes from the gravity direction recorded in each pose (unit vectors).
 *
 * Gravity in the watch frame moves one way when the hand tilts and another way when it rolls,
 * so the poses give both axes whichever wrist the watch is on and however it is rotated.
 */
fun buildCalibration(poses: Map<Pose, DoubleArray>): Calibration {
    fun pose(p: Pose) = poses.getValue(p)
    val n = pose(Pose.LEVEL)
    fun tangent(v: DoubleArray) = v - n * dot(v, n)

    val minSep = 2 * sin(Math.toRadians(MIN_POSE_DEG / 2))
    val pitchRaw = tangent(pose(Pose.FORWARD)) - tangent(pose(Pose.REVERSE))
    if (norm(pitchRaw) < minSep) {
        throw CalibrationException("The DOWN and UP poses were almost the same. Tilt further.")
    }
    val pitchAxis = unit(pitchRaw)

    val rollRaw = tangent(pose(Pose.RIGHT)) - tangent(pose(Pose.LEFT))
    if (norm(rollRaw) < minSep) {
        throw CalibrationException("The LEFT and RIGHT poses were almost the same. Roll further.")
    }
    val rollPerp = rollRaw - pitchAxis * dot(rollRaw, pitchAxis)
    if (norm(rollPerp) < 0.5 * norm(rollRaw)) {
        throw CalibrationException(
            "Rolling moved the watch the same way as tilting. Keep your hand level while rolling.",
        )
    }
    val rollAxis = unit(rollPerp)

    fun angles(p: Pose) = tiltAngles(n, pitchAxis, rollAxis, pose(p))
    fun atLeast(deg: Double, name: String, from: String = "level"): Double {
        if (deg < MIN_POSE_DEG) {
            throw CalibrationException(
                "The $name pose was only ${deg.roundToInt()}° from $from. Use at least ${MIN_POSE_DEG.roundToInt()}°.")
        }
        return deg
    }

    val forwardCenter = angles(Pose.FORWARD).roll
    return Calibration(
        neutral = n,
        pitchAxis = pitchAxis,
        rollAxis = rollAxis,
        forwardDeg = atLeast(angles(Pose.FORWARD).pitch, "FORWARD"),
        reverseDeg = atLeast(-angles(Pose.REVERSE).pitch, "REVERSE"),
        leftDeg = atLeast(-angles(Pose.LEFT).roll, "LEFT"),
        rightDeg = atLeast(angles(Pose.RIGHT).roll, "RIGHT"),
        forwardCenterDeg = forwardCenter,
        forwardLeftDeg = atLeast(
            forwardCenter - angles(Pose.FORWARD_LEFT).roll, "full-speed LEFT", "full-speed straight",
        ),
        forwardRightDeg = atLeast(
            angles(Pose.FORWARD_RIGHT).roll - forwardCenter, "full-speed RIGHT", "full-speed straight",
        ),
        reverseCenterDeg = angles(Pose.REVERSE).roll,
    )
}

/** Average direction of a pose's gravity samples, as a unit vector. */
fun direction(samples: List<DoubleArray>): DoubleArray =
    unit(DoubleArray(3) { i -> samples.sumOf { it[i] } / samples.size })

/** Classifies pitch and roll separately so FORWARD/REVERSE and LEFT/RIGHT can combine. */
class GestureClassifier(private val cal: Calibration) {

    /** One axis: [pos] is the command for positive angles, [neg] for negative ones. */
    private class Axis(val pos: String, val neg: String) {
        var active: String? = null
        var pending: String? = null  // past its trigger, waiting out HOLD_S
        var pendingSince = 0.0
    }

    private val pitch = Axis("FORWARD", "REVERSE")
    private val roll = Axis("RIGHT", "LEFT")
    private var ranges = emptyMap<String, Double>()  // angle of each command's calibrated pose at the current tilt

    private fun threshold(cmd: String) = max(MIN_ENTER_DEG, DEADZONE_FRACTION * ranges.getValue(cmd))

    /** 0 where cmd releases, rising to 1 at its calibrated pose. */
    private fun amount(cmd: String, angle: Double): Double {
        val start = EXIT_RATIO * threshold(cmd)
        return ((abs(angle) - start) / (ranges.getValue(cmd) - start)).coerceIn(0.0, 1.0)
    }

    private fun update(axis: Axis, angle: Double, now: Double) {
        val score = mapOf(axis.pos to angle / threshold(axis.pos), axis.neg to -angle / threshold(axis.neg))
        axis.active?.let { if (score.getValue(it) < EXIT_RATIO) axis.active = null }
        val best = if (score.getValue(axis.pos) >= score.getValue(axis.neg)) axis.pos else axis.neg
        if (score.getValue(best) < 1 || best == axis.active) {
            axis.pending = null
            return
        }
        if (axis.pending != best) {
            axis.pending = best
            axis.pendingSince = now
        }
        if (now - axis.pendingSince >= HOLD_S) {
            axis.active = best
            axis.pending = null
        }
    }

    /** Takes smoothed gravity and a timestamp in seconds. */
    fun update(gravity: DoubleArray, now: Double): Drive {
        val tilt = cal.tilt(gravity)
        val turn = cal.turnAt(tilt)
        ranges = mapOf(
            "FORWARD" to cal.forwardDeg, "REVERSE" to cal.reverseDeg,
            "LEFT" to turn.leftDeg, "RIGHT" to turn.rightDeg,
        )
        update(pitch, tilt.pitch, now)
        update(roll, turn.roll, now)

        val speed = pitch.active?.let { amount(it, tilt.pitch) } ?: 0.0
        val turnAmount = when (roll.active) {
            "RIGHT" -> amount("RIGHT", turn.roll)
            "LEFT" -> -amount("LEFT", turn.roll)
            else -> 0.0
        }
        val cmd = listOfNotNull(pitch.active, roll.active).joinToString("-").ifEmpty { "STOP" }
        return Drive(cmd, speed, turnAmount)
    }

    fun reset() {
        for (axis in listOf(pitch, roll)) {
            axis.active = null
            axis.pending = null
        }
    }
}

/** Exponential low-pass on gravity, timed by the sensor timestamps so uneven delivery doesn't skew it. */
class LowPass(private val timeConstantS: Double) {
    private val value = DoubleArray(3)
    private var lastNs = -1L

    fun update(v: FloatArray, timeNs: Long): DoubleArray {
        val dt = (timeNs - lastNs) / 1e9
        val a = if (lastNs < 0 || dt <= 0 || dt > RESET_GAP_S) 1.0 else 1 - exp(-dt / timeConstantS)
        lastNs = timeNs
        for (i in 0..2) value[i] += a * (v[i] - value[i])
        return value
    }
}

// --- vector helpers -------------------------------------------------------

private fun tiltAngles(neutral: DoubleArray, pitchAxis: DoubleArray, rollAxis: DoubleArray, g: DoubleArray): Tilt {
    val u = unit(g)
    val z = dot(u, neutral)
    return Tilt(Math.toDegrees(atan2(dot(u, pitchAxis), z)), Math.toDegrees(atan2(dot(u, rollAxis), z)))
}

private fun dot(a: DoubleArray, b: DoubleArray) = a.indices.sumOf { a[it] * b[it] }

private fun norm(a: DoubleArray) = sqrt(dot(a, a))

private fun unit(a: DoubleArray): DoubleArray {
    val n = norm(a)
    return if (n < 1e-9) a.copyOf() else a * (1 / n)
}

private operator fun DoubleArray.minus(b: DoubleArray) = DoubleArray(size) { this[it] - b[it] }

private operator fun DoubleArray.times(k: Double) = DoubleArray(size) { this[it] * k }
