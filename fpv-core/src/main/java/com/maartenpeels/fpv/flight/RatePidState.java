package com.maartenpeels.fpv.flight;

/**
 * The rate controller's memory for all three axes — the second half of {@link FlightState}.
 *
 * <p>A value, like everything else in the flight model, so a step stays a pure function of
 * {@code (state, input, dt)} and a drone can be rewound, copied or replayed without the controller
 * quietly remembering a timeline that no longer happened.
 */
public record RatePidState(PidState roll, PidState pitch, PidState yaw) {

    public static final RatePidState ZERO =
            new RatePidState(PidState.ZERO, PidState.ZERO, PidState.ZERO);

    public RatePidState {
        requirePresent(roll, "roll");
        requirePresent(pitch, "pitch");
        requirePresent(yaw, "yaw");
    }

    /**
     * Fresh memory for a drone already rotating at {@code rates} — see {@link PidState#at}. A drone
     * spawned at rest wants {@link #ZERO}, which is this at {@link BodyRates#ZERO}.
     */
    public static RatePidState at(BodyRates rates) {
        if (rates == null || !rates.isFinite()) {
            throw new IllegalArgumentException("rates must be finite but was " + rates);
        }
        return new RatePidState(
                PidState.at(rates.roll()), PidState.at(rates.pitch()), PidState.at(rates.yaw()));
    }

    private static void requirePresent(PidState state, String axis) {
        if (state == null) {
            throw new IllegalArgumentException(axis + " state must not be null");
        }
    }
}
