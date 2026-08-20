package com.maartenpeels.fpv.flight;

/**
 * One axis' worth of stick-to-rate mapping — the curve that turns a stick position into the
 * angular rate the pilot is asking for.
 *
 * <p>{@link com.maartenpeels.fpv.control.ControlInput} carries stick <em>positions</em> on purpose,
 * so that the same input flies differently under different tunes. This is where that happens.
 *
 * <h2>The curve</h2>
 *
 * Betaflight's "Actual Rates", with {@code x} the stick position in {@code [-1, 1]}:
 *
 * <pre>
 * rate(x) = C·x + (M − C) · sign(x) · ( e·|x|⁶ + (1−e)·|x|² )
 * </pre>
 *
 * where {@code C} is {@link #centreSensitivity}, {@code M} is {@link #maxRate} and {@code e} is
 * {@link #expo}. Betaflight writes the shaped term as {@code |x| · (x⁵·e + x·(1−e))}, which hides a
 * sixth power behind an odd fifth power and a separate absolute value; it is the same function.
 *
 * <p>Read it as the centre-sensitivity line {@code C·x}, plus the remaining {@code M − C} of
 * authority handed out according to the shaped weight — which is 0 at centre and 1 at full stick.
 *
 * <p>The family was chosen because both of its parameters are things a pilot can read off the
 * curve, which is what makes #5's tuning UI honest:
 *
 * <ul>
 *   <li>{@code M} is the rate at full stick, <em>exactly</em> — the shaped term is exactly 1 at
 *       {@code |x| = 1} for every expo, and {@link #rateFor} is arranged so that survives into the
 *       result.
 *   <li>{@code C} is the slope at centre, because the shaped term's own slope is 0 there.
 * </ul>
 *
 * <h2>What expo actually does</h2>
 *
 * <strong>Expo does not change the slope at centre.</strong> That is {@code C}, for every value of
 * {@code e}. What expo does is push the remaining {@code M − C} of authority further out toward
 * full stick: {@code ∂/∂e [e|x|⁶ + (1−e)|x|²]} is negative for every {@code 0 < |x| < 1}, so
 * raising expo lowers the demanded rate at every partial deflection while pinning both endpoints.
 * Pilots feel that as "softer around centre", which is the sense in which it reduces sensitivity —
 * but anyone reading it as a slope change will think the implementation is broken, so
 * {@code RateCurveTest} pins both halves.
 *
 * <p>Also: at {@code e = 0} the curve is still not a straight line unless {@code C = M}. The extra
 * authority arrives quadratically. An independent centre sensitivity and a pinned endpoint cannot
 * both hold along a straight line, so this is inherent rather than a defaulting mistake — and
 * {@code C = M} recovers the linear map #13 shipped, to within a few ULP, which is the feel-neutral
 * configuration to compare against.
 *
 * <h2>Units</h2>
 *
 * Radians per second, like {@link BodyRates} and everything else in this module. Pilots and every
 * published tune think in degrees per second, so {@link #fromDegrees} exists and
 * {@link RateProfile}'s defaults are written that way.
 *
 * <p>Immutable, so a retune (#5) is a new curve rather than mutated state, and one instance serves
 * every pilot flying the same tune.
 */
public record RateCurve(double centreSensitivity, double maxRate, double expo) {

    /** Betaflight's default. Enough curvature to be felt without making centre stick feel dead. */
    public static final double DEFAULT_EXPO = 0.54;

    public RateCurve {
        requirePositive(centreSensitivity, "centreSensitivity");
        requirePositive(maxRate, "maxRate");
        if (!Double.isFinite(expo) || expo < 0.0 || expo > 1.0) {
            throw new IllegalArgumentException(
                    "expo must be within [0, 1] but was " + expo);
        }
        if (maxRate < centreSensitivity) {
            throw new IllegalArgumentException(
                    "maxRate must be at least centreSensitivity, or the curve stops being"
                            + " monotonic, but was "
                            + maxRate
                            + " against "
                            + centreSensitivity);
        }
    }

    /**
     * The same curve with its two rates given in degrees per second, which is how every published
     * tune and every pilot states them.
     */
    public static RateCurve fromDegrees(
            double centreSensitivity, double maxRate, double expo) {
        return new RateCurve(
                Math.toRadians(centreSensitivity), Math.toRadians(maxRate), expo);
    }

    /**
     * The angular rate {@code stick} is asking for, in radians per second.
     *
     * <p>Throws outside {@code [-1, 1]} rather than clamping, following
     * {@link com.maartenpeels.fpv.control.ControlInput}: the only legitimate source of a stick
     * position is already validated, so an out-of-range one is a bug rather than a noisy client.
     * Clamping would also quietly break the guarantee that the result never exceeds
     * {@link #maxRate}.
     */
    public double rateFor(double stick) {
        if (!Double.isFinite(stick) || stick < -1.0 || stick > 1.0) {
            throw new IllegalArgumentException("stick must be within [-1, 1] but was " + stick);
        }

        double squared = stick * stick;
        double sixth = squared * squared * squared;
        double shaped = Math.copySign(this.expo * sixth + (1.0 - this.expo) * squared, stick);

        // Grouped as C(x − s) + Ms rather than the algebraically identical Cx + (M − C)s, because
        // the latter reaches full stick via C + (M − C) and lands one ULP short of maxRate for
        // about 5 % of whole-degree tunes. This grouping multiplies maxRate by an s that is exactly
        // 1 there, so the endpoint is exact for every tune. copySign rather than a signum multiply
        // keeps the whole thing exactly odd, zeros included.
        return this.centreSensitivity * (stick - shaped) + this.maxRate * shaped;
    }

    /** The same rates with different curvature — the contrast case for what expo is worth. */
    public RateCurve withExpo(double expo) {
        return new RateCurve(this.centreSensitivity, this.maxRate, expo);
    }

    /**
     * The same endpoint and curvature, with centre sensitivity raised to it — #13's linear
     * stick-to-rate map, recovered to within a few ULP (see the class javadoc on why not exactly).
     * The reference tune, not a useful one.
     */
    public RateCurve asLinear() {
        return new RateCurve(this.maxRate, this.maxRate, this.expo);
    }

    private static void requirePositive(double value, String name) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(
                    name + " must be finite and positive but was " + value);
        }
    }
}
