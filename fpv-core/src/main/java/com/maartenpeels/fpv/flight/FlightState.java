package com.maartenpeels.fpv.flight;

import com.maartenpeels.fpv.math.Vec3;

/**
 * Everything one drone carries from one integration step to the next: where it is and how it is
 * moving ({@link DroneState}), plus what its rate controller remembers ({@link RatePidState}).
 *
 * <p>Two values rather than one flat record on purpose. {@link DroneState} documents itself as
 * carrying no accumulated PID error, so that it stays a description of a rigid body and nothing
 * else — collision (#21), the body-follow teleport (#20) and any future replay want geometry, not a
 * controller's memory. This type is the seam where the two meet, and it is what
 * {@link QuadIntegrator#step} takes and returns, keeping a step a pure function of
 * {@code (state, input, dt)}.
 */
public record FlightState(DroneState drone, RatePidState controller) {

    public FlightState {
        if (drone == null) {
            throw new IllegalArgumentException("drone must not be null");
        }
        if (controller == null) {
            throw new IllegalArgumentException("controller must not be null");
        }
    }

    /** A level, stationary drone at {@code position} with a fresh controller — {@code /fpv launch}. */
    public static FlightState restingAt(Vec3 position) {
        return new FlightState(DroneState.restingAt(position), RatePidState.ZERO);
    }

    /**
     * A drone in whatever state {@code drone} describes, with a controller primed for the rates it
     * is already turning at — see {@link PidState#at}. The general-purpose way to start flying an
     * existing drone, where {@link #restingAt} is the specific one.
     */
    public static FlightState beginning(DroneState drone) {
        if (drone == null) {
            throw new IllegalArgumentException("drone must not be null");
        }
        return new FlightState(drone, RatePidState.at(drone.bodyRates()));
    }
}
