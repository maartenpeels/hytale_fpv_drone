package com.maartenpeels.fpv.flight;

import com.maartenpeels.fpv.control.ControlInput;

/**
 * One server tick's worth of flight: N fixed substeps of {@link QuadIntegrator} over one tick's
 * simulated time.
 *
 * <h2>Why {@code tickSeconds} and not a substep length</h2>
 *
 * {@link #advance} takes the length of the <em>tick</em> and divides it by the substep count. That is
 * the whole reason this class exists as a value rather than as a loop inside the plugin's tick
 * system, and it is what makes CLAUDE.md decision 3's promise true by construction:
 *
 * <blockquote>Changing the substep count changes CPU cost, not how the drone flies.</blockquote>
 *
 * Total simulated time per tick is {@code tickSeconds} for every substep count, so raising the count
 * refines the discretisation and nothing else. The implementation this guards against is the obvious
 * one — N substeps of a fixed 4 ms — under which going from 8 substeps to 16 would make the drone fly
 * at double speed, a change that looks like a physics bug and is really an arithmetic one.
 *
 * <p>Trajectories are not bit-identical across substep counts, and should not be: a different number
 * of steps is a different discretisation of the same differential equations, which is exactly what
 * substepping is for. What holds is convergence — the finer the substep, the closer to the true
 * trajectory — and {@code FlightTickTest} pins that rather than pinning equality.
 *
 * <h2>Input is per tick, not per substep</h2>
 *
 * {@code advance} takes one {@link ControlInput} and holds it across every substep. Pilot input
 * arrives on packets at the client's frame rate and is sampled once per tick; there is no more
 * information available at substep granularity, and pretending otherwise would put a stair-step at
 * 8× the tick frequency through the rate PID. Control latency and visual smoothness stay pinned at
 * the tick rate — decision 3 names that as a known, accepted limitation of substepping, and this
 * class is where it is visible.
 *
 * <h2>Rate-agnostic all the way down</h2>
 *
 * Nothing here knows the world's tick rate; it is told. So {@code World.setTps(240)} is a
 * configuration change rather than a rewrite, which decision 3 keeps as a standing constraint.
 *
 * <p>Immutable, and holds no per-drone state — every drone's state travels in the {@link FlightState}
 * passed in and returned. One instance serves every pilot flying the same airframe and tune.
 */
public final class FlightTick {

    private final QuadIntegrator integrator;
    private final int substeps;

    /**
     * @param integrator the flight model to step
     * @param substeps fixed integration substeps per tick; must be at least 1
     */
    public FlightTick(QuadIntegrator integrator, int substeps) {
        if (integrator == null) {
            throw new IllegalArgumentException("integrator must not be null");
        }
        if (substeps < 1) {
            throw new IllegalArgumentException("substeps must be at least 1 but was " + substeps);
        }
        this.integrator = integrator;
        this.substeps = substeps;
    }

    public QuadIntegrator integrator() {
        return this.integrator;
    }

    public int substeps() {
        return this.substeps;
    }

    /**
     * The length of one substep for a tick of {@code tickSeconds}.
     *
     * <p>Exposed because it is the number {@code FpvConfig.substepSeconds()} claims to compute, and
     * having both lets a test pin the config helper against this loop's arithmetic instead of trusting
     * that two expressions written months apart still agree.
     */
    public double substepSeconds(double tickSeconds) {
        requireUsableTickSeconds(tickSeconds);
        return tickSeconds / this.substeps;
    }

    /** Advances one tick with no per-substep hook. See {@link #advance(FlightState, ControlInput, double, SubstepListener)}. */
    public FlightState advance(FlightState state, ControlInput input, double tickSeconds) {
        return this.advance(state, input, tickSeconds, SubstepListener.NONE);
    }

    /**
     * Advances {@code state} by one tick of {@code tickSeconds}, in {@link #substeps()} equal steps
     * under a constant {@code input}.
     *
     * @param tickSeconds one tick's simulated duration; must be finite and positive
     * @param listener called after every substep, able to replace the state the next one starts from.
     *     {@link SubstepListener#NONE} for none. This is where #21's terrain collision plugs in — per
     *     substep, so a fast drone cannot tunnel between two of them.
     */
    public FlightState advance(
            FlightState state, ControlInput input, double tickSeconds, SubstepListener listener) {

        if (state == null) {
            throw new IllegalArgumentException("state must not be null");
        }
        if (input == null) {
            throw new IllegalArgumentException("input must not be null");
        }
        if (listener == null) {
            throw new IllegalArgumentException("listener must not be null");
        }
        requireUsableTickSeconds(tickSeconds);

        double dt = tickSeconds / this.substeps;
        FlightState current = state;
        for (int i = 0; i < this.substeps; i++) {
            FlightState stepped = this.integrator.step(current, input, dt);
            FlightState observed = listener.afterSubstep(current, stepped, dt);
            if (observed == null) {
                throw new IllegalStateException(
                        "SubstepListener returned null; return the state given to it to accept it unchanged");
            }
            current = observed;
        }
        return current;
    }

    private static void requireUsableTickSeconds(double tickSeconds) {
        if (!Double.isFinite(tickSeconds) || tickSeconds <= 0.0) {
            throw new IllegalArgumentException(
                    "tickSeconds must be finite and positive but was " + tickSeconds);
        }
    }
}
