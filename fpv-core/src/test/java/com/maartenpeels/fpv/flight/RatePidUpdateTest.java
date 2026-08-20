package com.maartenpeels.fpv.flight;

import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;

class RatePidUpdateTest {

    @Test
    void rejectsEitherHalfBeingMissingBecauseDroppingTheStateWouldSilentlyDisableTheIntegrator() {
        assertThrows(
                IllegalArgumentException.class,
                () -> new RatePidUpdate(null, RatePidState.ZERO));
        assertThrows(
                IllegalArgumentException.class, () -> new RatePidUpdate(TorqueDemand.NONE, null));
    }
}
