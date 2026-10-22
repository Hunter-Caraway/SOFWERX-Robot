package com.sofwerx.imuwatch

import org.junit.Assert.assertEquals
import org.junit.Assert.assertThrows
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.sqrt
import kotlin.math.tan

class GesturesTest {

    /** Gravity direction in a frame where level is +z, tilting down moves it toward +x and rolling right toward +y. */
    private fun dir(pitchDeg: Double, rollDeg: Double): DoubleArray {
        val v = doubleArrayOf(tan(Math.toRadians(pitchDeg)), tan(Math.toRadians(rollDeg)), 1.0)
        val n = sqrt(v.sumOf { it * it })
        return DoubleArray(3) { v[it] / n }
    }

    private fun gravity(pitchDeg: Double, rollDeg: Double) = dir(pitchDeg, rollDeg).map { it * 9.81 }.toDoubleArray()

    private val poses = mapOf(
        Pose.LEVEL to dir(0.0, 0.0),
        Pose.FORWARD to dir(40.0, 0.0),
        Pose.REVERSE to dir(-35.0, 0.0),
        Pose.LEFT to dir(0.0, -45.0),
        Pose.RIGHT to dir(0.0, 50.0),
        Pose.FORWARD_LEFT to dir(40.0, -30.0),
        Pose.FORWARD_RIGHT to dir(40.0, 55.0),
    )

    @Test
    fun calibrationRecoversPoseAngles() {
        val cal = buildCalibration(poses)
        assertEquals(40.0, cal.forwardDeg, 1e-6)
        assertEquals(35.0, cal.reverseDeg, 1e-6)
        assertEquals(45.0, cal.leftDeg, 1e-6)
        assertEquals(50.0, cal.rightDeg, 1e-6)
        assertEquals(0.0, cal.forwardCenterDeg, 1e-6)
        assertEquals(30.0, cal.forwardLeftDeg, 1e-6)
        assertEquals(55.0, cal.forwardRightDeg, 1e-6)
        assertEquals(0.0, cal.reverseCenterDeg, 1e-6)
    }

    @Test
    fun calibrationRejectsSmallPoses() {
        val e = assertThrows(CalibrationException::class.java) {
            buildCalibration(poses + (Pose.FORWARD to dir(8.0, 0.0)))
        }
        assertTrue(e.message!!, e.message!!.startsWith("The FORWARD pose was only 8°"))
        assertThrows(CalibrationException::class.java) {
            buildCalibration(poses + (Pose.RIGHT to dir(0.0, -40.0)))
        }
    }

    @Test
    fun commands() {
        val clf = GestureClassifier(buildCalibration(poses))
        var t = 0.0
        fun hold(pitchDeg: Double, rollDeg: Double, seconds: Double = 0.3): Drive {
            var d = Drive.STOP
            val end = t + seconds
            while (t < end) {
                d = clf.update(gravity(pitchDeg, rollDeg), t)
                t += 0.02
            }
            return d
        }
        fun assertDrive(cmd: String, speed: Double, turn: Double, d: Drive) {
            assertEquals(cmd, d.cmd)
            assertEquals("speed of $cmd", speed, d.speed, 1e-6)
            assertEquals("turn of $cmd", turn, d.turn, 1e-6)
        }

        assertDrive("STOP", 0.0, 0.0, hold(0.0, 0.0))
        assertDrive("STOP", 0.0, 0.0, hold(5.0, -5.0))                  // inside the dead zone
        assertEquals("STOP", hold(40.0, 0.0, seconds = 0.1).cmd)       // not held long enough yet
        assertDrive("FORWARD", 1.0, 0.0, hold(40.0, 0.0))
        assertDrive("FORWARD", (20 - 7.0) / (40 - 7.0), 0.0, hold(20.0, 0.0))
        assertDrive("STOP", 0.0, 0.0, clf.update(gravity(0.0, 0.0), t))  // level stops at once
        assertDrive("REVERSE", 1.0, 0.0, hold(-35.0, 0.0))
        hold(0.0, 0.0)
        assertDrive("LEFT", 0.0, -1.0, hold(0.0, -45.0))
        hold(0.0, 0.0)
        assertDrive("FORWARD-RIGHT", 1.0, 1.0, hold(40.0, 55.0))
        assertDrive("REVERSE-LEFT", 1.0, -1.0, hold(-35.0, -45.0))

        clf.reset()
        assertEquals("STOP", clf.update(gravity(0.0, 0.0), t).cmd)
    }
}
