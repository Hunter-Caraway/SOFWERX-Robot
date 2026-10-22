package com.sofwerx.imuwatch

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Test

class AnnouncerTest {

    @Test
    fun parsesThePeopleCount() {
        assertEquals(PiStatus(2), parseStatus("""{"people": 2}"""))
        assertEquals(PiStatus(0), parseStatus("""{"people":0}"""))
        assertEquals(PiStatus(null), parseStatus("""{"people": null}"""))
        assertNull(parseStatus("""{"room": "inside"}"""))
        assertNull(parseStatus("""{"people": "2"}"""))
    }

    @Test
    fun parsesWhetherItHasSomeoneToFollow() {
        assertEquals(PiStatus(1, true), parseStatus("""{"people": 1, "target": true}"""))
        assertEquals(PiStatus(0, false), parseStatus("""{"people":0,"target":false}"""))
        assertEquals(PiStatus(null, null), parseStatus("""{"people": null, "target": null}"""))
    }

    @Test
    fun saysHowFollowingGoes() {
        val f = FollowCallouts()
        assertNull(f.update(null))
        assertEquals("Looking for you!", f.update(false))
        assertNull(f.update(false))  // the Pi repeats it
        assertEquals("Following you!", f.update(true))
        assertNull(f.update(true))
        assertEquals("Lost you!", f.update(false))
        assertEquals("Following you!", f.update(true))
        assertNull(f.update(null))  // follow mode off
        assertEquals("Following you!", f.update(true))
    }

    @Test
    fun showsHowFollowingGoes() {
        assertEquals("following you", describeTarget(true))
        assertEquals("looking for you", describeTarget(false))
        assertEquals("waiting for the camera", describeTarget(null))
    }

    @Test
    fun showsTheCount() {
        assertEquals("all clear", describePeople(0))
        assertEquals("1 person ahead", describePeople(1))
        assertEquals("3 people ahead", describePeople(3))
    }

    @Test
    fun saysAllClearOrPeopleAhead() {
        assertEquals("All clear!", phrase(0))
        assertEquals("Person ahead!", phrase(1))
        assertEquals("2 people ahead!", phrase(2))
    }

    @Test
    fun saysEachNewCountOnce() {
        val c = Callouts()
        assertNull(c.update(null))
        assertEquals(0, c.update(0))
        assertNull(c.update(0))  // the Pi repeats it
        assertNull(c.done())
        assertNull(c.update(0))
        assertEquals(1, c.update(1))
        assertNull(c.done())
        assertEquals(2, c.update(2))
    }

    @Test
    fun saysOnlyTheLatestCountOnceDoneSpeaking() {
        val c = Callouts()
        assertEquals(1, c.update(1))
        assertNull(c.update(2))  // still saying "Person ahead!"
        assertNull(c.update(3))
        assertEquals(3, c.done())
        assertNull(c.update(3))
        assertNull(c.done())
    }

    @Test
    fun saysNothingMoreIfTheCountComesBackWhileSpeaking() {
        val c = Callouts()
        assertEquals(1, c.update(1))
        assertNull(c.update(2))
        assertNull(c.update(1))  // back to what's being said
        assertNull(c.done())
    }

    @Test
    fun saysTheCountAgainAfterAPause() {
        val c = Callouts()
        assertEquals(2, c.update(2))
        assertNull(c.done())
        assertNull(c.update(null))  // control off: no count
        assertEquals(2, c.update(2))
    }

    @Test
    fun turnsTheVolumeUpWhileSpeaking() {
        var volume = 4
        val full = FullVolume({ volume }, { volume = it }, { 15 })
        full.start("a")
        assertEquals(15, volume)
        full.start("b")  // queued behind the first
        full.done("a")
        assertEquals(15, volume)
        full.done("b")
        assertEquals(4, volume)  // back where it was
        full.done("b")  // heard twice
        assertEquals(4, volume)
        full.start("c")
        full.restore()  // the app closed mid-sentence
        assertEquals(4, volume)
    }

    @Test
    fun picksAWomansVoiceForBmo() {
        fun v(name: String, language: String = "en", network: Boolean = false, installed: Boolean = true) =
            VoiceInfo(name, language, network, installed)
        val google = listOf(v("en-us-x-iom-local"), v("en-us-x-iog-local"), v("en-us-x-tpc-local"),
            v("en-us-x-sfg-network", network = true), v("de-de-x-dea-local", "de"))
        assertEquals("en-us-x-tpc-local", pickBmoVoice(google))
        assertEquals("en-us-x-sfg-local", pickBmoVoice(google + v("en-us-x-sfg-local")))
        assertNull(pickBmoVoice(listOf(v("en-us-x-sfg-local", installed = false), v("en-us-x-iom-local"))))
        assertEquals("English female 2", pickBmoVoice(listOf(v("English male 1"), v("English female 2"))))
    }
}
