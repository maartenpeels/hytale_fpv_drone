package com.maartenpeels.fpv.flight;

import com.maartenpeels.fpv.control.ControlInput;
import java.util.function.UnaryOperator;

/**
 * A complete set of rates: one {@link RateCurve} per axis, and the map from a whole
 * {@link ControlInput} to the {@link BodyRates} it demands.
 *
 * <p>Per axis rather than one shared curve, because yaw is not roll. A quad yaws on prop reaction
 * rather than differential lift, so it turns far more slowly, and pilots set yaw rates lower still
 * because yaw is used to aim rather than to throw the aircraft around. Keeping the curves separate
 * makes that a number rather than a special case.
 *
 * <p>This is the pilot's tune, not the airframe — the same line {@link RatePidGains} draws.
 * {@link QuadParameters} is server-wide; #5 persists one of these per player. That is why full-stick
 * rate lives here and not in the airframe record: it is the first number a pilot changes.
 */
public record RateProfile(RateCurve roll, RateCurve pitch, RateCurve yaw) {

    /**
     * Betaflight's literal default centre sensitivity. Against #13's 800 °/s endpoint this is a
     * 1:4 ratio, where stock Betaflight's 200:670 is nearer 1:3.35 — so this default is slightly
     * softer around centre than a stock quad, a consequence of inheriting the higher endpoint.
     */
    public static final double DEFAULT_ROLL_PITCH_CENTRE_SENSITIVITY = Math.toRadians(200.0);

    /** Inherited unchanged from #13, where it was chosen deliberately and #24 judges it. */
    public static final double DEFAULT_ROLL_PITCH_MAX_RATE = Math.toRadians(800.0);

    /** Scaled to hold roll and pitch's centre-to-max ratio rather than picked separately. */
    public static final double DEFAULT_YAW_CENTRE_SENSITIVITY = Math.toRadians(100.0);

    /** Also #13's, and half of roll and pitch, as it is on most real tunes. */
    public static final double DEFAULT_YAW_MAX_RATE = Math.toRadians(400.0);

    /** A plausible racing tune. #24 is where it gets judged. */
    public static final RateProfile DEFAULT =
            new RateProfile(
                    new RateCurve(
                            DEFAULT_ROLL_PITCH_CENTRE_SENSITIVITY,
                            DEFAULT_ROLL_PITCH_MAX_RATE,
                            RateCurve.DEFAULT_EXPO),
                    new RateCurve(
                            DEFAULT_ROLL_PITCH_CENTRE_SENSITIVITY,
                            DEFAULT_ROLL_PITCH_MAX_RATE,
                            RateCurve.DEFAULT_EXPO),
                    new RateCurve(
                            DEFAULT_YAW_CENTRE_SENSITIVITY,
                            DEFAULT_YAW_MAX_RATE,
                            RateCurve.DEFAULT_EXPO));

    public RateProfile {
        requirePresent(roll, "roll");
        requirePresent(pitch, "pitch");
        requirePresent(yaw, "yaw");
    }

    /** The same curve on all three axes. Only sensible in tests, where the axes are symmetric. */
    public static RateProfile uniform(RateCurve curve) {
        return new RateProfile(curve, curve, curve);
    }

    /**
     * The rates {@code input}'s sticks are asking for.
     *
     * <p>Each stick axis drives its own axis and no other. That sounds too obvious to state, but a
     * transposition here is invisible in every unit test that does not look for it and shows up in
     * flight as pitch rolling the aircraft, so {@code RateProfileTest} looks for it.
     */
    public BodyRates demand(ControlInput input) {
        if (input == null) {
            throw new IllegalArgumentException("input must not be null");
        }
        return new BodyRates(
                this.roll.rateFor(input.roll()),
                this.pitch.rateFor(input.pitch()),
                this.yaw.rateFor(input.yaw()));
    }

    /** Applies {@code change} to all three axes — for asking "what does this tune do at expo 0?". */
    public RateProfile onEveryAxis(UnaryOperator<RateCurve> change) {
        return new RateProfile(
                change.apply(this.roll), change.apply(this.pitch), change.apply(this.yaw));
    }

    /** The rates at full stick on every axis — the bound {@link RatePid} is tuned against. */
    public BodyRates maxRates() {
        return new BodyRates(this.roll.maxRate(), this.pitch.maxRate(), this.yaw.maxRate());
    }

    private static void requirePresent(RateCurve curve, String axis) {
        if (curve == null) {
            throw new IllegalArgumentException(axis + " curve must not be null");
        }
    }
}
