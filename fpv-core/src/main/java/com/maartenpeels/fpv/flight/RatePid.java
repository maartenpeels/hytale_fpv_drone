package com.maartenpeels.fpv.flight;

/**
 * The rate loop: demanded body rates in, normalised torque demand out.
 *
 * <p>This is the piece that makes the drone <em>hold</em> a rate rather than merely lean toward one.
 * #13 shipped a proportional-only placeholder in its place, and a proportional tracker cannot null
 * an error it is fighting continuously — every rad/s of drag, and every rad/s the mixer gives up to
 * saturation, shows up as a permanent gap between what the pilot asked for and what the drone does.
 * The integral term is what closes that gap.
 *
 * <h2>Derivative on measurement, not on error</h2>
 *
 * {@code d(error)/dt} contains {@code d(setpoint)/dt}, and a stick snap is a setpoint <em>step</em>:
 * at 4 ms and an 800 °/s throw, differentiating it produces a torque spike of thousands of times
 * full authority for exactly one step. Real flight controllers derive on the gyro signal for this
 * reason, and so does this one — the D term is {@code −kd · Δrate / dt}, and the setpoint never
 * enters it.
 *
 * <p>The cost is that the controller has to remember the last <em>measured</em> rate, which makes
 * {@link PidState#at} rather than {@link PidState#ZERO} the right way to start a controller for a
 * drone that is already rotating.
 *
 * <h2>Windup, handled twice</h2>
 *
 * The case that exposes it is a quad held against a wall at full stick: the demanded rate is
 * unreachable, so the error never shrinks, and a naive accumulator grows without limit until the
 * drone comes free and snaps. Two independent guards, because they fail differently:
 *
 * <ol>
 *   <li><b>The accumulator is clamped</b> to {@code ±integralLimit}. This bounds the worst case
 *       absolutely — however long the quad is pinned, it can only ever store that much stale
 *       correction.
 *   <li><b>Accumulation freezes while the output is saturated and the error is pushing it further
 *       out.</b> Without this, an axis sitting at full torque keeps integrating an error it is
 *       already doing everything it can about, which buys nothing and costs authority later. An
 *       error that reverses sign always unfreezes it, so the loop recovers on the step the drone
 *       comes free rather than waiting for the accumulator to bleed down.
 * </ol>
 *
 * <p>Both matter: the clamp alone leaves a pinned axis sitting at its ceiling, and the freeze alone
 * would let a slowly-converging axis accumulate an unbounded amount before it ever saturates.
 *
 * <h2>Output space</h2>
 *
 * The result is a {@link TorqueDemand} in {@code [-1, 1]}, the mixer's units — deliberately not an
 * angular acceleration. Saturation is this class' whole subject, and a controller that let something
 * downstream do its clamping could not see that it had saturated, which would make both guards
 * above impossible to implement here.
 *
 * <p>Immutable and stateless; the memory is the {@link RatePidState} passed in and returned. One
 * instance serves every pilot flying the same tune.
 */
public final class RatePid {

    private final RatePidGains gains;

    public RatePid(RatePidGains gains) {
        if (gains == null) {
            throw new IllegalArgumentException("gains must not be null");
        }
        this.gains = gains;
    }

    public RatePidGains gains() {
        return this.gains;
    }

    /**
     * Answers the torque that closes the gap between {@code demanded} and {@code measured}.
     *
     * @param state the controller's memory from the previous step
     * @param demanded the rates the pilot's sticks are asking for, rad/s
     * @param measured the rates the drone is actually turning at, rad/s
     * @param dt seconds since the last update; must be finite and positive
     */
    public RatePidUpdate update(
            RatePidState state, BodyRates demanded, BodyRates measured, double dt) {
        requirePresent(state, "state");
        requireFiniteRates(demanded, "demanded");
        requireFiniteRates(measured, "measured");
        if (!Double.isFinite(dt) || dt <= 0.0) {
            throw new IllegalArgumentException("dt must be finite and positive but was " + dt);
        }

        AxisUpdate roll =
                axis(this.gains.roll(), state.roll(), demanded.roll(), measured.roll(), dt);
        AxisUpdate pitch =
                axis(this.gains.pitch(), state.pitch(), demanded.pitch(), measured.pitch(), dt);
        AxisUpdate yaw = axis(this.gains.yaw(), state.yaw(), demanded.yaw(), measured.yaw(), dt);

        return new RatePidUpdate(
                new TorqueDemand(roll.torque, pitch.torque, yaw.torque),
                new RatePidState(roll.state, pitch.state, yaw.state));
    }

    private static AxisUpdate axis(
            PidGains gains, PidState state, double demanded, double measured, double dt) {
        double error = demanded - measured;
        double proportional = gains.proportional() * error;
        double derivative = -gains.derivative() * (measured - state.lastRate()) / dt;

        double candidate =
                Math.clamp(
                        state.integral() + gains.integral() * error * dt,
                        -gains.integralLimit(),
                        gains.integralLimit());

        double integral =
                pushesFurtherIntoSaturation(proportional + candidate + derivative, error)
                        ? state.integral()
                        : candidate;

        double torque = Math.clamp(proportional + integral + derivative, -1.0, 1.0);
        return new AxisUpdate(torque, new PidState(integral, measured));
    }

    private static boolean pushesFurtherIntoSaturation(double unclamped, double error) {
        return Math.abs(unclamped) > 1.0 && Math.signum(unclamped) == Math.signum(error);
    }

    private static void requirePresent(RatePidState state, String name) {
        if (state == null) {
            throw new IllegalArgumentException(name + " must not be null");
        }
    }

    private static void requireFiniteRates(BodyRates rates, String name) {
        if (rates == null || !rates.isFinite()) {
            throw new IllegalArgumentException(name + " rates must be finite but was " + rates);
        }
    }

    private record AxisUpdate(double torque, PidState state) {}
}
