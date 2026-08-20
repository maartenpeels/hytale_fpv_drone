package com.maartenpeels.fpv.flight;

/**
 * One axis' worth of rate-PID tuning.
 *
 * <p>Gains map a rate error in radians per second onto a <em>normalised torque demand</em> in
 * {@code [-1, 1]} — {@link MotorMixer}'s units, not radians per second². So the airframe's angular
 * authority is baked into the numbers: the same gains on a twitchier frame produce a twitchier
 * drone. That is how a real flight controller's gains work too, and it is deliberate — the
 * controller has to own its own output clamp in order to know it is saturated, which is what makes
 * {@code integralLimit} meaningful (see {@link RatePid}).
 *
 * <p>Units, per axis:
 *
 * <ul>
 *   <li>{@code proportional} — torque per rad/s of error.
 *   <li>{@code integral} — torque per rad/s of error, per second of it persisting.
 *   <li>{@code derivative} — torque per rad/s² of measured rate change.
 *   <li>{@code integralLimit} — a ceiling on the accumulated integral term, as a fraction of full
 *       output authority. {@code 0} disables the integrator's memory entirely; {@code 1} lets it
 *       command full torque on its own.
 * </ul>
 *
 * <p>Zero is allowed for {@code proportional}, {@code integral} and {@code derivative}, because
 * P-only and PI-only are tunes a pilot legitimately arrives at. Negative is not: it inverts the
 * feedback loop, which is a bug rather than a taste.
 *
 * <p>#5 persists these per pilot. Nothing here holds state — the accumulator lives in
 * {@link PidState}, so the same gains can be shared by every pilot flying the same tune.
 */
public record PidGains(
        double proportional, double integral, double derivative, double integralLimit) {

    /**
     * A third of output authority. Betaflight's ballpark, and enough to absorb the drag and mixer
     * losses the integral term exists for without letting a stalled axis store a full-torque
     * surprise.
     */
    public static final double DEFAULT_INTEGRAL_LIMIT = 0.3;

    public PidGains {
        requireNonNegative(proportional, "proportional");
        requireNonNegative(integral, "integral");
        requireNonNegative(derivative, "derivative");
        requireNonNegative(integralLimit, "integralLimit");
        if (integralLimit > 1.0) {
            throw new IllegalArgumentException(
                    "integralLimit is a fraction of full output authority and must not exceed 1 but"
                            + " was "
                            + integralLimit);
        }
    }

    /**
     * Gains with {@link #DEFAULT_INTEGRAL_LIMIT}.
     *
     * <p>Expressed as integral and derivative <em>times</em> rather than raw gains, because that is
     * the form in which the numbers mean something: {@code integralTime} is roughly how long a
     * standing error takes to be absorbed, {@code derivativeTime} how far ahead the damping term
     * looks. Both in seconds.
     */
    public static PidGains fromTimes(
            double proportional, double integralTime, double derivativeTime) {
        if (!Double.isFinite(integralTime) || integralTime <= 0.0) {
            throw new IllegalArgumentException(
                    "integralTime must be finite and positive but was " + integralTime);
        }
        return new PidGains(
                proportional,
                proportional / integralTime,
                proportional * derivativeTime,
                DEFAULT_INTEGRAL_LIMIT);
    }

    /** The same tuning with the integral term removed — the contrast case for steady-state error. */
    public PidGains withoutIntegral() {
        return new PidGains(this.proportional, 0.0, this.derivative, this.integralLimit);
    }

    /** The same tuning with the derivative term removed — the contrast case for derivative kick. */
    public PidGains withoutDerivative() {
        return new PidGains(this.proportional, this.integral, 0.0, this.integralLimit);
    }

    public PidGains withProportional(double proportional) {
        return new PidGains(proportional, this.integral, this.derivative, this.integralLimit);
    }

    public PidGains withIntegralLimit(double integralLimit) {
        return new PidGains(this.proportional, this.integral, this.derivative, integralLimit);
    }

    private static void requireNonNegative(double value, String name) {
        if (!Double.isFinite(value) || value < 0.0) {
            throw new IllegalArgumentException(
                    name + " must be finite and non-negative but was " + value);
        }
    }
}
