package com.maartenpeels.fpv.control;

/**
 * Normalised pilot control axes — the boundary contract between the plugin and the flight
 * model (CLAUDE.md decision 1).
 *
 * <p>The plugin layer is responsible for turning {@code ClientMovement.wishMovement} and
 * {@code lookOrientation} into one of these. Nothing downstream of this type knows that
 * Hytale exists, which axis came from which packet field, or whether the pilot is on a
 * keyboard or (from Hytale update 5) a real transmitter.
 *
 * <p>Ranges follow transmitter convention rather than a symmetric-everything convention,
 * because a quadcopter's throttle is unidirectional:
 * <ul>
 *   <li>{@code throttle} — {@code 0..1}, 0 being motors-idle and 1 being full stick.</li>
 *   <li>{@code roll}, {@code pitch}, {@code yaw} — {@code -1..1}, 0 being centred.</li>
 * </ul>
 *
 * <p>Sign convention, viewed from behind the aircraft: positive {@code roll} banks right,
 * positive {@code pitch} pitches nose <strong>down</strong>, positive {@code yaw} rotates
 * nose right.
 *
 * <p>Pitch is the one that trips people up. This follows transmitter convention, not
 * aerospace body-frame convention: pushing the pitch stick <em>forward</em> is positive, and
 * pitches the nose down so the quad accelerates forward. Anyone arriving from an aerospace
 * background will expect positive pitch to mean nose up — it does not here. Do not "correct"
 * it. When mapping onto Hytale's {@code Direction.pitch}, verify that field's sign against the
 * decompiled sources rather than assuming it agrees.
 *
 * <p>These are <em>stick positions</em>, not rates. Converting stick deflection into a
 * demanded angular rate is the job of the rate/expo curve, not of this type — so that the
 * same input can be flown under different rate profiles.
 *
 * <p>Instances are validated on construction. Use {@link #clamped} at the packet boundary,
 * where values arrive untrusted, and the canonical constructor everywhere else, where an
 * out-of-range value means a bug rather than a noisy client.
 */
public record ControlInput(float throttle, float roll, float pitch, float yaw) {

    /** Sticks centred, throttle closed. The state a disarmed or freshly spawned drone is in. */
    public static final ControlInput NEUTRAL = new ControlInput(0f, 0f, 0f, 0f);

    public ControlInput {
        throttle = requireInRange(throttle, 0f, 1f, "throttle");
        roll = requireInRange(roll, -1f, 1f, "roll");
        pitch = requireInRange(pitch, -1f, 1f, "pitch");
        yaw = requireInRange(yaw, -1f, 1f, "yaw");
    }

    /**
     * Builds an input from untrusted values, clamping each axis into range instead of
     * throwing. Intended for the packet adapter: a client sending a slightly out-of-range or
     * non-finite {@code wishMovement} should be corrected, not crash the tick.
     *
     * <p>Non-finite values collapse to the neutral position for that axis, since there is no
     * meaningful clamp for NaN.
     */
    public static ControlInput clamped(float throttle, float roll, float pitch, float yaw) {
        return new ControlInput(
                clamp(throttle, 0f, 1f),
                clamp(roll, -1f, 1f),
                clamp(pitch, -1f, 1f),
                clamp(yaw, -1f, 1f));
    }

    /** True when all three attitude sticks are centred, regardless of throttle. */
    public boolean sticksCentred() {
        return roll == 0f && pitch == 0f && yaw == 0f;
    }

    private static float clamp(float value, float min, float max) {
        if (Float.isNaN(value)) {
            return min < 0f ? 0f : min;
        }
        return Math.clamp(value, min, max);
    }

    private static float requireInRange(float value, float min, float max, String axis) {
        if (!Float.isFinite(value) || value < min || value > max) {
            throw new IllegalArgumentException(
                    axis + " must be within [" + min + ", " + max + "] but was " + value);
        }
        return value;
    }
}
