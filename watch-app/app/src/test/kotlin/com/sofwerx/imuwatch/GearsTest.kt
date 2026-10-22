package com.sofwerx.imuwatch

import org.junit.Assert.assertEquals
import org.junit.Test

class GearsTest {

    @Test
    fun shiftsUpAndDownWithinTheGears() {
        assertEquals(Gear.FAST, Gear.MEDIUM.shift(1))
        assertEquals(Gear.SLOW, Gear.MEDIUM.shift(-1))
        assertEquals(Gear.FAST, Gear.FAST.shift(1))
        assertEquals(Gear.SLOW, Gear.FAST.shift(-5))
        assertEquals(Gear.MEDIUM, Gear.MEDIUM.shift(0))
    }

    @Test
    fun topSpeedRisesWithEachGear() {
        assertEquals(listOf(Gear.SLOW, Gear.MEDIUM, Gear.FAST), Gear.entries.sortedBy { it.top })
        assertEquals(1.0, Gear.FAST.top, 0.0)
    }

    @Test
    fun countsEachBezelClick() {
        val r = RotaryClicks()
        assertEquals(1, r.add(1f))
        assertEquals(1, r.add(1f))
        assertEquals(-1, r.add(-1f))
        assertEquals(2, r.add(2f))
    }

    @Test
    fun addsUpSmallCrownSteps() {
        val r = RotaryClicks()
        assertEquals(0, r.add(0.4f))
        assertEquals(0, r.add(0.4f))
        assertEquals(1, r.add(0.4f))
        assertEquals(0, r.add(-0.4f))  // turning back starts the count afresh
        assertEquals(0, r.add(-0.4f))
        assertEquals(-1, r.add(-0.4f))
    }
}
