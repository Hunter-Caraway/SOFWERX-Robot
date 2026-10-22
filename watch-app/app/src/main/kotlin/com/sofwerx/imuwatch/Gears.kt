package com.sofwerx.imuwatch

import kotlin.math.sign

/*
 * Speed gears, picked with the watch's rotating bezel: each click clockwise goes up a gear, and each
 * click back goes down one. The gear sets the robot's top speed. The Pi scales every command by it,
 * so tilting all the way drives at the gear's top speed, and less tilt drives slower.
 */

/** How fast the robot may go: top is its top speed, as a share of full speed. */
enum class Gear(val label: String, val top: Double) {
    SLOW("Slow", 0.35),
    MEDIUM("Medium", 0.65),
    FAST("Fast", 1.0);

    /** The gear this many clicks up (down for negative clicks), stopping at the lowest and highest. */
    fun shift(clicks: Int) = entries[(ordinal + clicks).coerceIn(0, entries.size - 1)]
}

/**
 * Adds up rotary input (a bezel or crown) into whole clicks, clockwise positive. A Galaxy Watch bezel
 * gives 1 per click; a crown gives smaller steps, which add up.
 */
class RotaryClicks {
    private var turned = 0f  // rotation not yet counted as a click

    /** Takes one rotary event's rotation. Returns the clicks it completes, negative counterclockwise. */
    fun add(rotation: Float): Int {
        if (turned != 0f && rotation.sign != turned.sign) turned = 0f  // turning back: count afresh
        turned += rotation
        val clicks = turned.toInt()
        turned -= clicks
        return clicks
    }
}
