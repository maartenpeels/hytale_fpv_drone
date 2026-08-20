package com.maartenpeels.fpv.flight;

/**
 * One axis' worth of controller memory — everything {@link RatePid} needs to carry from one step to
 * the next.
 *
 * <p>Two fields, for two reasons:
 *
 * <ul>
 *   <li>{@code integral} is the accumulated integral term, <em>already scaled by the gain</em>
 *       rather than a raw {@code ∫e·dt}. That makes {@link PidGains#integralLimit()} directly a
 *       fraction of output authority, which is the quantity worth bounding, and it means a live
 *       retune (#5) does not retroactively rescale stored history into a jolt.
 *   <li>{@code lastRate} is the previously <em>measured</em> body rate, not the previous error,
 *       because {@link RatePid} derives on measurement. See its javadoc for why.
 * </ul>
 *
 * <p>Held separately from {@link DroneState}, which documents itself as carrying no accumulated PID
 * error; {@link FlightState} is what pairs the two.
 */
public record PidState(double integral, double lastRate) {

    public static final PidState ZERO = new PidState(0.0, 0.0);

    public PidState {
        requireFinite(integral, "integral");
        requireFinite(lastRate, "lastRate");
    }

    /**
     * Fresh memory for an axis already turning at {@code rate}.
     *
     * <p>Not the same as {@link #ZERO}: because the derivative term is taken on the measurement,
     * a controller that believes the axis was stationary last step will see the drone's actual
     * rotation as one enormous rate change and answer it with a phantom torque spike. Anything
     * spawning a controller for a moving drone wants this.
     */
    public static PidState at(double rate) {
        return new PidState(0.0, rate);
    }

    private static void requireFinite(double value, String name) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite but was " + value);
        }
    }
}
