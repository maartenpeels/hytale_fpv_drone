package com.maartenpeels.fpv.flight;

import com.maartenpeels.fpv.control.ControlInput;
import com.maartenpeels.fpv.math.Vec3;

/**
 * The quad flight model: {@code (state, input, dt) -> state}.
 *
 * <h2>Rate-agnostic by construction</h2>
 *
 * {@link #step} takes {@code dt} and holds no notion of a tick, a substep count or a server. That is
 * CLAUDE.md decision 3's requirement, and it is what keeps {@code World.setTps(240)} a config change
 * rather than a rewrite: raising the tick rate changes how *smoothly* the drone is sampled, never
 * how it flies. {@code QuadIntegratorTest} pins that down.
 *
 * <h2>What a step does</h2>
 *
 * <ol>
 *   <li>Stick positions become demanded body rates.
 *   <li>The gap between demanded and actual rate becomes a normalised torque demand per axis.
 *   <li>{@link MotorMixer} turns collective plus torque demand into four motor commands.
 *   <li>The commands' <em>achieved</em> thrusts give the real force and torque — including whatever
 *       the mixer had to give up to stay inside {@code [0, 1]}.
 *   <li>Semi-implicit Euler advances rate then attitude, and acceleration then velocity then
 *       position.
 * </ol>
 *
 * <h2>The two placeholders</h2>
 *
 * Steps 1 and 2 have their own tickets and are deliberately the dumbest thing that flies here:
 *
 * <ul>
 *   <li>Stick to rate is <b>linear</b>. Rate and expo curves are #15.
 *   <li>Rate to torque is <b>proportional only</b>. The rate PID is #14. A proportional tracker
 *       cannot hold a rate against sustained drag, so the model will feel stiff until #14 lands —
 *       expected, not a regression.
 * </ul>
 *
 * Neither has been given an interface yet; #14 and #15 will introduce those when there is a second
 * implementation to justify one.
 *
 * <p>Because mixing is linear in command space while thrust goes with the square of the command,
 * achieved torque does not equal demanded torque. That is not an approximation to be tidied away —
 * it is exactly the error a real rate PID exists to close, and #14 inherits an honest version of the
 * problem rather than a model that pretends it away.
 *
 * <p>Instances are immutable and hold no mutable state, so one can be shared across every pilot on
 * the server.
 */
public final class QuadIntegrator {

    private final QuadParameters parameters;

    public QuadIntegrator(QuadParameters parameters) {
        if (parameters == null) {
            throw new IllegalArgumentException("parameters must not be null");
        }
        this.parameters = parameters;
    }

    public QuadParameters parameters() {
        return this.parameters;
    }

    /**
     * Advances {@code state} by {@code dt} seconds under {@code input}.
     *
     * @param dt seconds of simulated time; must be finite and positive
     */
    public DroneState step(DroneState state, ControlInput input, double dt) {
        if (!Double.isFinite(dt) || dt <= 0.0) {
            throw new IllegalArgumentException("dt must be finite and positive but was " + dt);
        }

        MotorOutputs thrusts = MotorMixer.mix(
                        input.throttle(),
                        this.torqueDemand(state.bodyRates().roll(), this.demandedRoll(input),
                                this.parameters.rollPitchAuthority()),
                        this.torqueDemand(state.bodyRates().pitch(), this.demandedPitch(input),
                                this.parameters.rollPitchAuthority()),
                        this.torqueDemand(state.bodyRates().yaw(), this.demandedYaw(input),
                                this.parameters.yawAuthority()))
                .thrusts();

        BodyRates nextRates = state.bodyRates().plus(this.angularAcceleration(state, thrusts).scale(dt));
        // Semi-implicit: attitude turns at the rate the drone has *after* this step's torque, which
        // keeps rotation from lagging a step behind the sticks.
        var nextOrientation = state.orientation().integrate(nextRates.toBodyAxes(), dt);

        Vec3 acceleration = this.linearAcceleration(state, thrusts);
        Vec3 nextVelocity = state.velocity().plus(acceleration.scale(dt));
        Vec3 nextPosition = state.position().plus(nextVelocity.scale(dt));

        return new DroneState(nextPosition, nextVelocity, nextOrientation, nextRates);
    }

    /**
     * Thrust along the body's up axis, plus gravity, plus drag.
     *
     * <p>Drag has a linear and a quadratic term. Quadratic is the physically honest one and is what
     * gives a realistic terminal velocity and the wall of air a pilot feels at speed; linear is kept
     * for gentle bleed-off when barely moving.
     */
    private Vec3 linearAcceleration(DroneState state, MotorOutputs thrusts) {
        Vec3 thrust =
                state.thrustAxis().scale(this.parameters.maxThrustAcceleration() * thrusts.mean());
        Vec3 gravity = new Vec3(0, -this.parameters.gravity(), 0);

        Vec3 velocity = state.velocity();
        double speed = velocity.length();
        Vec3 drag =
                velocity.scale(
                        -(this.parameters.linearDrag() + this.parameters.quadraticDrag() * speed));

        return thrust.plus(gravity).plus(drag);
    }

    /**
     * Angular acceleration from the differential thrust the motors actually produced, less passive
     * damping.
     *
     * <p>The differentials are read off the achieved thrusts rather than the demand, which is what
     * makes authority throttle-dependent without a special case anywhere. Because thrust goes with
     * the square of the command, the same command spread produces a much smaller thrust spread down
     * near idle — so a roll demand that snaps the drone over at mid throttle merely leans it at
     * idle.
     */
    private BodyRates angularAcceleration(DroneState state, MotorOutputs thrusts) {
        double rollPitch = this.parameters.rollPitchAuthority();
        BodyRates torque =
                new BodyRates(
                        thrusts.rollDifferential() * rollPitch,
                        thrusts.pitchDifferential() * rollPitch,
                        thrusts.yawDifferential() * this.parameters.yawAuthority());
        return torque.plus(state.bodyRates().scale(-this.parameters.angularDrag()));
    }

    /**
     * Proportional rate tracking: ask for the torque that would close the rate error in one time
     * constant, and clamp it to what one axis of the mixer can represent.
     *
     * <p>Placeholder for #14's PID.
     */
    private double torqueDemand(double actualRate, double demandedRate, double authority) {
        double error = demandedRate - actualRate;
        return Math.clamp(error / (this.parameters.rateTimeConstant() * authority), -1.0, 1.0);
    }

    /** Linear stick-to-rate mapping. Placeholder for #15's rate and expo curves. */
    private double demandedRoll(ControlInput input) {
        return input.roll() * this.parameters.maxRates().roll();
    }

    private double demandedPitch(ControlInput input) {
        return input.pitch() * this.parameters.maxRates().pitch();
    }

    private double demandedYaw(ControlInput input) {
        return input.yaw() * this.parameters.maxRates().yaw();
    }
}
