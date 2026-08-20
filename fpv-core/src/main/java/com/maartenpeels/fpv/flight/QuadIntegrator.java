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
 * <h2>The remaining placeholder</h2>
 *
 * Step 1 is deliberately the dumbest thing that flies: stick to rate is <b>linear</b>. Rate and
 * expo curves are #15, and will introduce an interface when there is a second implementation to
 * justify one.
 *
 * <p>Step 2 is no longer a placeholder — {@link RatePid} is the real rate loop as of #14.
 *
 * <p>Because mixing is linear in command space while thrust goes with the square of the command,
 * achieved torque does not equal demanded torque. That is not an approximation to be tidied away —
 * it is exactly the error the rate PID exists to close, and the loop it closes is therefore
 * genuinely non-linear and throttle-dependent rather than a textbook first-order plant.
 *
 * <p>Instances are immutable and hold no mutable state, so one can be shared across every pilot
 * flying the same airframe and tune. A retune (#5) means a new integrator rather than a mutated one.
 */
public final class QuadIntegrator {

    private final QuadParameters parameters;
    private final RatePid rateController;

    /** The default tune, {@link RatePidGains#DEFAULT}, on the given airframe. */
    public QuadIntegrator(QuadParameters parameters) {
        this(parameters, RatePidGains.DEFAULT);
    }

    public QuadIntegrator(QuadParameters parameters, RatePidGains gains) {
        if (parameters == null) {
            throw new IllegalArgumentException("parameters must not be null");
        }
        this.parameters = parameters;
        this.rateController = new RatePid(gains);
    }

    public QuadParameters parameters() {
        return this.parameters;
    }

    public RatePidGains gains() {
        return this.rateController.gains();
    }

    /**
     * Advances {@code state} by {@code dt} seconds under {@code input}.
     *
     * @param dt seconds of simulated time; must be finite and positive
     */
    public FlightState step(FlightState state, ControlInput input, double dt) {
        if (state == null) {
            throw new IllegalArgumentException("state must not be null");
        }
        if (!Double.isFinite(dt) || dt <= 0.0) {
            throw new IllegalArgumentException("dt must be finite and positive but was " + dt);
        }

        DroneState drone = state.drone();
        BodyRates rates = drone.bodyRates();
        RatePidUpdate control =
                this.rateController.update(state.controller(), this.demandedRates(input), rates, dt);
        TorqueDemand torque = control.torque();

        MotorThrusts thrusts =
                MotorMixer.mix(input.throttle(), torque.roll(), torque.pitch(), torque.yaw())
                        .thrusts();

        BodyRates nextRates = rates.plus(this.angularAcceleration(drone, thrusts).scale(dt));
        // Semi-implicit: attitude turns at the rate the drone has *after* this step's torque, which
        // keeps rotation from lagging a step behind the sticks.
        var nextOrientation = drone.orientation().integrate(nextRates.toBodyAxes(), dt);

        Vec3 acceleration = this.linearAcceleration(drone, thrusts);
        Vec3 nextVelocity = drone.velocity().plus(acceleration.scale(dt));
        Vec3 nextPosition = drone.position().plus(nextVelocity.scale(dt));

        return new FlightState(
                new DroneState(nextPosition, nextVelocity, nextOrientation, nextRates),
                control.state());
    }

    /**
     * Thrust along the body's up axis, plus gravity, plus drag.
     *
     * <p>Drag has a linear and a quadratic term. Quadratic is the physically honest one and is what
     * gives a realistic terminal velocity and the wall of air a pilot feels at speed; linear is kept
     * for gentle bleed-off when barely moving.
     */
    private Vec3 linearAcceleration(DroneState state, MotorThrusts thrusts) {
        Vec3 thrust =
                state.thrustAxis()
                        .scale(this.parameters.maxThrustAcceleration() * thrusts.collective());
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
    private BodyRates angularAcceleration(DroneState state, MotorThrusts thrusts) {
        double rollPitch = this.parameters.rollPitchAuthority();
        BodyRates torque =
                new BodyRates(
                        thrusts.rollDifferential() * rollPitch,
                        thrusts.pitchDifferential() * rollPitch,
                        thrusts.yawDifferential() * this.parameters.yawAuthority());
        return torque.plus(state.bodyRates().scale(-this.parameters.angularDrag()));
    }

    /** Linear stick-to-rate mapping. Placeholder for #15's rate and expo curves. */
    private BodyRates demandedRates(ControlInput input) {
        BodyRates maxRates = this.parameters.maxRates();
        return new BodyRates(
                input.roll() * maxRates.roll(),
                input.pitch() * maxRates.pitch(),
                input.yaw() * maxRates.yaw());
    }
}
