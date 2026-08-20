package com.maartenpeels.fpv.flight;

/**
 * {@link RatePid#update}'s two answers: the torque to apply now, and the memory to carry into the
 * next step.
 */
public record RatePidUpdate(TorqueDemand torque, RatePidState state) {

    public RatePidUpdate {
        if (torque == null) {
            throw new IllegalArgumentException("torque must not be null");
        }
        if (state == null) {
            throw new IllegalArgumentException("state must not be null");
        }
    }
}
