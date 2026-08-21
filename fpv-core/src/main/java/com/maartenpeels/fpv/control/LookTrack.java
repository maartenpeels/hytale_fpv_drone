package com.maartenpeels.fpv.control;

/**
 * The last absolute look orientation seen from a pilot, and how long ago — the memory a delta needs.
 *
 * <p>{@link PilotInputMapper} derives pitch and roll stick deflection from how much the pilot's look
 * *changed*, not from where it points (the reasoning is in {@code docs/plans/17.md}). A difference
 * needs something to difference against, and this is it.
 *
 * <h2>Why the elapsed time is part of the memory</h2>
 *
 * A look delta is only meaningful divided by the interval it was measured over, and that interval is
 * <em>not</em> the tick length. {@code lookOrientation} is nullable and the client omits it when
 * nothing moved, so look samples arrive at the client's frame rate while {@code map} is called at the
 * tick rate. Dividing a delta that spans four ticks by one tick's {@code dt} over-reports the rate
 * four-fold — and the error grows exactly where decision 3's escape hatch lives, since raising the
 * world to 240 TPS multiplies it by TPS/clientFPS. {@link #secondsSinceSample} is what lets the
 * mapper divide by the real interval, so a 60 FPS client on a 240 TPS world gets a steady half-stick
 * instead of a full-stick square wave.
 *
 * <p>{@link #UNSET} is not "looking straight ahead" — it is "no previous sample". The two have to be
 * distinguishable, because treating a missing previous angle as zero would make the first packet
 * after launch read as a full-deflection flick from an angle the pilot never chose. An absent track
 * canonicalises its angles to zero so that every "nothing seen yet" compares equal.
 *
 * <p>A value, so a pilot's input history can be reset, copied or replayed without a mutable object
 * quietly remembering a timeline that no longer happened. Reset it on launch.
 *
 * @param yaw the last absolute look yaw in radians; forced to zero unless {@code present}
 * @param pitch the last absolute look pitch in radians; forced to zero unless {@code present}
 * @param secondsSinceSample seconds elapsed since those angles were sampled; forced to zero unless
 *     {@code present}
 * @param present whether a previous sample exists at all
 */
public record LookTrack(double yaw, double pitch, double secondsSinceSample, boolean present) {

    /** No look seen yet. The state a freshly launched drone's pilot starts in. */
    public static final LookTrack UNSET = new LookTrack(0.0, 0.0, 0.0, false);

    public LookTrack {
        if (present) {
            requireFinite(yaw, "yaw");
            requireFinite(pitch, "pitch");
            requireFinite(secondsSinceSample, "secondsSinceSample");
            if (secondsSinceSample < 0.0) {
                throw new IllegalArgumentException(
                        "secondsSinceSample must not be negative but was " + secondsSinceSample);
            }
        } else {
            yaw = 0.0;
            pitch = 0.0;
            secondsSinceSample = 0.0;
        }
    }

    /** A track sitting at a look sampled just now. */
    public static LookTrack at(double yaw, double pitch) {
        return new LookTrack(yaw, pitch, 0.0, true);
    }

    /**
     * The same angles, {@code seconds} further from when they were sampled — what a tick that brought
     * no new look orientation does to the memory.
     *
     * <p>An absent track stays absent: there is no sample for the clock to run from.
     */
    public LookTrack aged(double seconds) {
        if (!Double.isFinite(seconds) || seconds < 0.0) {
            throw new IllegalArgumentException(
                    "seconds must be finite and non-negative but was " + seconds);
        }
        if (!this.present) {
            return this;
        }
        return new LookTrack(this.yaw, this.pitch, this.secondsSinceSample + seconds, true);
    }

    private static void requireFinite(double value, String name) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(
                    name + " must be finite on a present track but was " + value);
        }
    }
}
