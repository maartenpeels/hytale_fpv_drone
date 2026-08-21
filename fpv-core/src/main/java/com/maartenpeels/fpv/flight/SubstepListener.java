package com.maartenpeels.fpv.flight;

/**
 * A hook called after every integration substep, able to replace the state the next substep starts
 * from.
 *
 * <h2>Why this exists, and why it is per substep</h2>
 *
 * Terrain collision (#21) has to run per <em>substep</em>, not per tick. Substepping only buys
 * fidelity if nothing can pass through a wall between two substeps: a drone at 40 blocks/second on a
 * 30 TPS world covers 1.3 blocks per tick, so a once-per-tick collision test would let it tunnel
 * through a one-block wall roughly whenever it hit one squarely, which is precisely the failure
 * substepping is supposed to remove. Checking after each substep bounds the distance travelled
 * between checks to {@code speed × tickSeconds / substeps}.
 *
 * <p>So {@link FlightTick#advance(FlightState, com.maartenpeels.fpv.control.ControlInput, double,
 * SubstepListener)} calls this after each substep and integrates onward from whatever it returns.
 * Returning a modified state is what lets a swept-AABB hit clamp the drone to the impact point and
 * zero the velocity component into the surface, rather than merely observing that it happened.
 *
 * <h2>What it deliberately does not do</h2>
 *
 * There is no way to signal "stop substepping". A listener that has decided the flight is over
 * records that on its own side and the caller acts on it after {@code advance} returns; the remaining
 * substeps integrate from an already-clamped state, which is harmless. One method with one
 * responsibility was preferred over guessing at the shape of a verdict type for a ticket that has not
 * been designed yet.
 *
 * <p>Nothing here reaches outside {@code :fpv-core}. A listener that needs to read blocks lives in
 * {@code :fpv-plugin} and is injected — which is also what keeps the flight system testable without a
 * world.
 */
@FunctionalInterface
public interface SubstepListener {

    /** Observes nothing and changes nothing. The default, and what tests inject. */
    SubstepListener NONE = (before, after, dt) -> after;

    /**
     * Called once after each substep.
     *
     * @param before the state the substep started from
     * @param after the state the integrator produced
     * @param dt the substep's length in seconds — the same value for every substep of a tick
     * @return the state to carry into the next substep; return {@code after} to accept it unchanged.
     *     Must not be {@code null}.
     */
    FlightState afterSubstep(FlightState before, FlightState after, double dt);
}
