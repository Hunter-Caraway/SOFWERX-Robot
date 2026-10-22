package com.sofwerx.imuwatch

import android.content.Context
import android.media.AudioAttributes
import android.media.AudioManager
import android.os.Handler
import android.os.Looper
import android.speech.tts.TextToSpeech
import android.speech.tts.UtteranceProgressListener
import android.util.Log
import java.util.Locale

/*
 * Says how many people the robot's camera sees, on the watch's speaker, in a voice made to sound
 * like BMO from Adventure Time: high and quick. Each time the count changes it says "All clear!",
 * "Person ahead!" or "2 people ahead!". The Pi sends the count back to the watch:
 *   {"people":2}      2 people in view
 *   {"people":null}   no count: detection paused while control is off, or no people counter running
 * The Pi repeats the count every half second, and each new count is said once. If the count changes
 * again while BMO is still speaking, only the latest is said, once BMO is done.
 *
 * In follow mode the Pi also says whether the robot has someone to follow ("target":true or false;
 * null otherwise), and BMO says "Looking for you!", "Following you!" or "Lost you!" as that changes.
 * Anything else, such as the gear, is said with say().
 *
 * BMO's own voice isn't available to text-to-speech. The closest is a woman's voice raised in pitch
 * (BMO_PITCH) and sped up a little (BMO_RATE). It speaks at full media volume, and puts the volume
 * back once it's done.
 */

private const val TAG = "Announcer"
private const val GOOGLE_TTS = "com.google.android.tts"
private const val PEOPLE_ID = "people-"  // starts the ID of each people count said
const val BMO_PITCH = 1.7f  // 1 is the voice's normal pitch
const val BMO_RATE = 1.15f  // 1 is its normal speed

private val PEOPLE = Regex(""""people"\s*:\s*(null|\d+)""")
private val TARGET = Regex(""""target"\s*:\s*(true|false|null)""")

/**
 * What the Pi says: people is the count, null while it has none; target is whether the robot has
 * someone to follow, null when it isn't following.
 */
data class PiStatus(val people: Int?, val target: Boolean? = null)

/** The Pi's status in a packet from it, or null if the packet doesn't carry a people count. */
fun parseStatus(text: String): PiStatus? {
    val people = PEOPLE.find(text)?.groupValues?.get(1) ?: return null
    val target = TARGET.find(text)?.groupValues?.get(1)?.toBooleanStrictOrNull()
    return PiStatus(people.toIntOrNull(), target)
}

/** The count, for the screen: "all clear", "1 person ahead", "3 people ahead". */
fun describePeople(people: Int) = when (people) {
    0 -> "all clear"
    1 -> "1 person ahead"
    else -> "$people people ahead"
}

/** What the watch says when this many people are in view. */
fun phrase(people: Int) = when (people) {
    0 -> "All clear!"
    1 -> "Person ahead!"
    else -> "$people people ahead!"
}

/** How following is going, for the screen. */
fun describeTarget(target: Boolean?) = when (target) {
    true -> "following you"
    false -> "looking for you"
    null -> "waiting for the camera"
}

/** Picks what to say as following goes: "Looking for you!", "Following you!", "Lost you!". */
class FollowCallouts {
    private var target: Boolean? = null

    /** Whether the robot has someone to follow (null: not following). Returns what to say now, or null. */
    fun update(target: Boolean?): String? {
        val was = this.target
        this.target = target
        if (target == was) return null
        return when (target) {
            true -> "Following you!"
            false -> if (was == true) "Lost you!" else "Looking for you!"
            null -> null
        }
    }
}

/**
 * Picks which counts to say: each new count once, however many times the Pi repeats it. A count
 * that comes while BMO is still speaking waits until it's done, and only the latest of those is said.
 */
class Callouts {
    private var said: Int? = null  // the count said last, or being said; null once the count has gone
    private var speaking = false
    private var next: Int? = null  // a newer count, to say once BMO is done

    /** The latest count from the Pi (null: none). Returns the count to say now, or null. */
    fun update(count: Int?): Int? {
        if (count == null) {
            said = null  // so the count is said again when it comes back
            next = null
            return null
        }
        if (speaking) {
            next = if (count != said) count else null
            return null
        }
        if (count == said) return null
        said = count
        speaking = true
        return count
    }

    /** BMO has finished speaking. Returns the count to say next, or null. */
    fun done(): Int? {
        speaking = false
        val n = next ?: return null
        next = null
        said = n
        speaking = true
        return n
    }
}

/**
 * Turns a volume up to full while something plays, and back to where it was once everything playing
 * has finished. get and set read and change the volume; max is full volume.
 */
class FullVolume(private val get: () -> Int, private val set: (Int) -> Unit, private val max: () -> Int) {
    private var before: Int? = null  // the volume to go back to, while something plays
    private val playing = HashSet<String>()

    fun start(id: String) {
        if (before == null) {
            before = get()
            set(max())
        }
        playing.add(id)
    }

    fun done(id: String) {
        if (playing.remove(id) && playing.isEmpty()) restore()
    }

    /** Puts the volume back now, whatever is still playing. */
    fun restore() {
        playing.clear()
        before?.let(set)
        before = null
    }
}

/** What pickBmoVoice needs to know about a text-to-speech voice. */
data class VoiceInfo(val name: String, val language: String, val needsNetwork: Boolean, val installed: Boolean)

// Google's women's English voices, best first: raised in pitch, they come closest to BMO.
private val HIGH_VOICES = listOf("en-us-x-sfg", "en-us-x-tpc", "en-us-x-iog", "en-us-x-iob", "en-us-x-tpf",
    "en-gb-x-gba", "en-gb-x-gbc", "en-gb-x-gbg")

/** The name of a woman's English voice that works offline, or null if there's none. */
fun pickBmoVoice(voices: List<VoiceInfo>): String? {
    val usable = voices.filter { it.installed && !it.needsNetwork && it.language.startsWith("en") }
    for (prefix in HIGH_VOICES) {
        usable.firstOrNull { it.name.lowercase().startsWith(prefix) }?.let { return it.name }
    }
    return usable.firstOrNull { "female" in it.name.lowercase() }?.name
}

/**
 * Says each new people count, how following goes, and anything else it's told, in BMO's voice, at full volume. Main
 * thread only.
 */
class Announcer(context: Context) : TextToSpeech.OnInitListener {
    private val handler = Handler(Looper.getMainLooper())
    private val calls = Callouts()
    private val following = FollowCallouts()
    private val audio = context.getSystemService(AudioManager::class.java)
    private val volume = FullVolume(
        { audio.getStreamVolume(AudioManager.STREAM_MUSIC) },
        {
            try {
                audio.setStreamVolume(AudioManager.STREAM_MUSIC, it, 0)
            } catch (e: SecurityException) {
                Log.w(TAG, "can't change the media volume", e)
            }
        },
        { audio.getStreamMaxVolume(AudioManager.STREAM_MUSIC) },
    )
    // GOOGLE_TTS, or the watch's default engine if Google's is missing
    private val tts = TextToSpeech(context, this, GOOGLE_TTS).apply {
        setOnUtteranceProgressListener(object : UtteranceProgressListener() {  // called on another thread
            override fun onStart(utteranceId: String?) {}
            override fun onDone(utteranceId: String?) = finished(utteranceId)
            @Deprecated("Deprecated in Java")
            override fun onError(utteranceId: String?) = finished(utteranceId)
            override fun onStop(utteranceId: String?, interrupted: Boolean) = finished(utteranceId)
        })
    }
    private var ready = false
    private var utterances = 0L  // numbers each thing said

    /** Why the watch can't speak, or doesn't sound like BMO, for the screen; null when all is well. */
    @Volatile var problem: String? = null
        private set

    override fun onInit(status: Int) {
        // Choosing the voice here, off the main thread: the engine can take a while to list its voices.
        val ok = status == TextToSpeech.SUCCESS && chooseVoice()
        handler.post {
            ready = ok
            if (!ok) problem = "no text-to-speech on this watch"
        }
    }

    /** The latest people count from the Pi (null: none). */
    fun onPeople(count: Int?) {
        if (!ready) return  // the Pi repeats the count, so it's heard again once text-to-speech is ready
        calls.update(count)?.let { sayPeople(it) }
    }

    /** Whether the robot has someone to follow, from the Pi (null: not following). */
    fun onTarget(target: Boolean?) {
        if (!ready) return  // the Pi repeats it too
        following.update(target)?.let { say(it) }
    }

    /** Says something once, after anything already being said. */
    fun say(text: String) {
        if (ready) speak(text, "say-${++utterances}")
    }

    fun shutdown() {
        handler.removeCallbacksAndMessages(null)
        tts.shutdown()
        volume.restore()
    }

    private fun finished(utteranceId: String?) {
        if (utteranceId != null) handler.post { done(utteranceId) }
    }

    /**
     * One thing said: after a count, says the next count, if one came in meanwhile, before letting the volume back
     * down.
     */
    private fun done(utteranceId: String) {
        if (utteranceId.startsWith(PEOPLE_ID)) calls.done()?.let { sayPeople(it) }
        volume.done(utteranceId)
    }

    /**
     * Sets BMO's voice: a woman's voice if there is one, raised and sped up. False if the engine can't speak
     * English.
     */
    private fun chooseVoice(): Boolean {
        val voices = tts.voices.orEmpty().toList()
        val name = pickBmoVoice(voices.map {
            VoiceInfo(it.name, it.locale.language, it.isNetworkConnectionRequired,
                TextToSpeech.Engine.KEY_FEATURE_NOT_INSTALLED !in it.features)
        })
        val voice = voices.firstOrNull { it.name == name }
        if (voice == null || tts.setVoice(voice) != TextToSpeech.SUCCESS) {
            if (tts.setLanguage(Locale.US) < TextToSpeech.LANG_AVAILABLE) return false
            problem = "no woman's voice; BMO sounds deeper"
        }
        tts.setPitch(BMO_PITCH)
        tts.setSpeechRate(BMO_RATE)
        tts.setAudioAttributes(AudioAttributes.Builder()
            .setUsage(AudioAttributes.USAGE_MEDIA)  // media volume (turned up to full), which silent mode doesn't mute
            .setContentType(AudioAttributes.CONTENT_TYPE_SPEECH)
            .build())
        return true
    }

    private fun sayPeople(people: Int) = speak(phrase(people), "$PEOPLE_ID${++utterances}")

    private fun speak(text: String, id: String) {
        volume.start(id)
        if (tts.speak(text, TextToSpeech.QUEUE_ADD, null, id) != TextToSpeech.SUCCESS) done(id)
    }
}
