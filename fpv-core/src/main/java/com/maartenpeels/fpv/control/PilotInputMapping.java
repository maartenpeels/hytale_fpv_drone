package com.maartenpeels.fpv.control;

/**
 * What counts as full stick deflection — the three constants {@link PilotInputMapper} needs to turn
 * raw client numbers into a normalised {@code -1..1} axis.
 *
 * <p>All three are, honestly, guesses that only flying can settle, which is exactly why they are
 * values rather than literals in the mapper. Getting one wrong is not a crash: it is a throttle that
 * is either off or full, or a drone that ignores the mouse.
 *
 * <h2>{@code wishFullScale}</h2>
 *
 * The magnitude Hytale's {@code wishMovement} reaches at full single-axis deflection. It cannot be
 * read out of the decompiled server: nothing there produces the vector, and the one place that
 * consumes it ({@code KnockbackPredictionSystems}) treats it interchangeably with a world-space
 * position delta, which suggests it may carry a speed rather than a unit direction. Defaults to
 * {@code 1.0} — the assumption that it is normalised — and #24 is where that gets corrected by
 * flying.
 *
 * <h2>The two look rates</h2>
 *
 * The angular rate of the pilot's *view* that corresponds to a fully deflected stick, in rad/s. Roll
 * rides the look-yaw channel and pitch the look-pitch channel, and they get separate constants
 * because mouse X and Y sensitivities differ on real clients.
 *
 * <p>{@link #DEFAULT} uses 2π rad/s — one full revolution per second of view rotation to reach full
 * stick. Too low and any glance saturates the axis; too high and the drone feels dead. This is also
 * where decision 11's calibration wizard will eventually write to.
 *
 * <p>Per-pilot, so this does <em>not</em> belong in {@code FpvConfig} — see that class' javadoc.
 * {@link #DEFAULT} stands in until {@code PilotProfile} exists.
 *
 * @param wishFullScale magnitude of {@code wishMovement} at full deflection; must be finite and
 *     positive
 * @param rollLookRateFullScale look-yaw rate in rad/s that means full roll stick; must be finite and
 *     positive
 * @param pitchLookRateFullScale look-pitch rate in rad/s that means full pitch stick; must be finite
 *     and positive
 */
public record PilotInputMapping(
        double wishFullScale, double rollLookRateFullScale, double pitchLookRateFullScale) {

    /** Unit wish vector, and one revolution per second of view rotation for full stick. */
    public static final PilotInputMapping DEFAULT =
            new PilotInputMapping(1.0, 2.0 * Math.PI, 2.0 * Math.PI);

    public PilotInputMapping {
        requirePositive(wishFullScale, "wishFullScale");
        requirePositive(rollLookRateFullScale, "rollLookRateFullScale");
        requirePositive(pitchLookRateFullScale, "pitchLookRateFullScale");
    }

    private static void requirePositive(double value, String name) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(
                    name + " must be finite and positive but was " + value);
        }
    }
}
