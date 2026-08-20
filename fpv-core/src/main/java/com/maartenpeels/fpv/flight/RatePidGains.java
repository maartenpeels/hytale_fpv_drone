package com.maartenpeels.fpv.flight;

import java.util.function.UnaryOperator;

/**
 * A complete rate-loop tune: {@link PidGains} for each of the three axes.
 *
 * <p>Per axis rather than one shared set, because a quad's yaw axis is nothing like its roll and
 * pitch axes — yaw torque comes from prop reaction rather than differential lift, so it is several
 * times weaker and needs correspondingly higher gains to reach the same rate. Keeping them separate
 * makes that fall out of the numbers instead of needing a special case anywhere in {@link RatePid}.
 *
 * <p>This is the pilot's tune, not the airframe: {@link QuadParameters} is server-wide, while #5
 * persists one of these per player. {@link #DEFAULT} is therefore calibrated <em>for</em>
 * {@link QuadParameters#DEFAULT} — a substantially different airframe wants a different tune.
 */
public record RatePidGains(PidGains roll, PidGains pitch, PidGains yaw) {

    /**
     * The time constant the default proportional gains are derived from: a gain of
     * {@code 1 / (this · authority)} asks for the torque that would close the whole rate error in
     * this many seconds.
     *
     * <p>#13's placeholder rate tracker was exactly that expression, so reusing it here makes the
     * arrival of a real rate loop feel-neutral — the drone pulls toward a demanded rate exactly as
     * hard as it did before, and merely gains the ability to arrive.
     */
    public static final double DEFAULT_RATE_TIME_CONSTANT = 0.03;

    /**
     * How long a standing rate error takes to be absorbed by the integral term, in seconds.
     *
     * <p>Chosen by measurement rather than by feel: shortening it to 0.15 s more than doubles
     * overshoot (5.3 % against 2.3 %) and nearly trebles the rate the drone swings <em>past</em>
     * centre when the stick is released, for no gain in rise time — both are bounce-back, which is
     * the thing pilots complain about. Lengthening it to 0.5 s keeps trimming overshoot but starts
     * leaving a measurable steady-state error and slows disturbance recovery.
     */
    public static final double DEFAULT_INTEGRAL_TIME = 0.3;

    /**
     * Derivative lookahead, in seconds — <strong>zero by default</strong>, which deserves its
     * reasons written down.
     *
     * <p>Derivative action earns its place against a plant that <em>lags</em>: motor spin-up, prop
     * inertia, frame flex, gyro filtering. #13's model has none of those. Torque becomes angular
     * acceleration within the same step, and the airframe's own {@code angularDrag} is already a
     * rate-proportional damping term, so the rate loop is first order and a PI controller closes it
     * about as well as it can be closed.
     *
     * <p>Measured, on the default airframe at the default substep: every non-zero derivative time
     * tried made every metric worse — more overshoot, a deeper swing past centre on stick release,
     * and slower recovery from a kick. It is also the first term to misbehave as {@code dt} grows,
     * since it divides by it.
     *
     * <p>The term itself is fully implemented and tested, because it is a gain #5 hands to pilots,
     * and because the moment #13's model grows any lag of its own — motor response, a smoothed
     * throttle — it stops being useless.
     */
    public static final double DEFAULT_DERIVATIVE_TIME = 0.0;

    public static final double DEFAULT_ROLL_PITCH_PROPORTIONAL =
            1.0 / (DEFAULT_RATE_TIME_CONSTANT * QuadParameters.DEFAULT_ROLL_PITCH_AUTHORITY);

    public static final double DEFAULT_YAW_PROPORTIONAL =
            1.0 / (DEFAULT_RATE_TIME_CONSTANT * QuadParameters.DEFAULT_YAW_AUTHORITY);

    /** A plausible racing tune for {@link QuadParameters#DEFAULT}. #24 is where it gets judged. */
    public static final RatePidGains DEFAULT =
            new RatePidGains(
                    PidGains.fromTimes(
                            DEFAULT_ROLL_PITCH_PROPORTIONAL,
                            DEFAULT_INTEGRAL_TIME,
                            DEFAULT_DERIVATIVE_TIME),
                    PidGains.fromTimes(
                            DEFAULT_ROLL_PITCH_PROPORTIONAL,
                            DEFAULT_INTEGRAL_TIME,
                            DEFAULT_DERIVATIVE_TIME),
                    PidGains.fromTimes(
                            DEFAULT_YAW_PROPORTIONAL,
                            DEFAULT_INTEGRAL_TIME,
                            DEFAULT_DERIVATIVE_TIME));

    public RatePidGains {
        requirePresent(roll, "roll");
        requirePresent(pitch, "pitch");
        requirePresent(yaw, "yaw");
    }

    /** The same tuning on all three axes. Only sensible in tests, where the axes are symmetric. */
    public static RatePidGains uniform(PidGains gains) {
        return new RatePidGains(gains, gains, gains);
    }

    /** Applies {@code change} to all three axes — for asking "what does this tune do without I?". */
    public RatePidGains onEveryAxis(UnaryOperator<PidGains> change) {
        return new RatePidGains(
                change.apply(this.roll), change.apply(this.pitch), change.apply(this.yaw));
    }

    private static void requirePresent(PidGains gains, String axis) {
        if (gains == null) {
            throw new IllegalArgumentException(axis + " gains must not be null");
        }
    }
}
