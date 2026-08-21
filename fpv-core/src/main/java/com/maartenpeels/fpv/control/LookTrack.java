package com.maartenpeels.fpv.control;

/**
 * The last absolute look orientation seen from a pilot — the memory a delta needs.
 *
 * <p>{@link PilotInputMapper} derives pitch and roll stick deflection from how much the pilot's look
 * *changed*, not from where it points (the reasoning is in {@code docs/plans/17.md}). A difference
 * needs something to difference against, and this is it.
 *
 * <p>{@link #UNSET} is not "looking straight ahead" — it is "no previous sample". The two have to be
 * distinguishable, because treating a missing previous angle as zero would make the first packet
 * after launch read as a full-deflection flick from an angle the pilot never chose.
 *
 * <p>A value, so a pilot's input history can be reset, copied or replayed without a mutable object
 * quietly remembering a timeline that no longer happened. Reset it on launch.
 *
 * @param yaw the last absolute look yaw in radians; meaningless unless {@code present}
 * @param pitch the last absolute look pitch in radians; meaningless unless {@code present}
 * @param present whether a previous sample exists at all
 */
public record LookTrack(double yaw, double pitch, boolean present) {

    /** No look seen yet. The state a freshly launched drone's pilot starts in. */
    public static final LookTrack UNSET = new LookTrack(0.0, 0.0, false);

    public LookTrack {
        if (present && !(Double.isFinite(yaw) && Double.isFinite(pitch))) {
            throw new IllegalArgumentException(
                    "a present look track must be finite but was yaw=" + yaw + " pitch=" + pitch);
        }
    }

    /** A track sitting at a known absolute look. */
    public static LookTrack at(double yaw, double pitch) {
        return new LookTrack(yaw, pitch, true);
    }
}
