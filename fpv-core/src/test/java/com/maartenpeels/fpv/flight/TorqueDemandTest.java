package com.maartenpeels.fpv.flight;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class TorqueDemandTest {

    @Nested
    class MixerContract {

        @Test
        void feedsTheMixerInTheSameSignConventionItExpects() {
            // The type exists to stop a BodyRates being passed where a normalised torque belongs, so
            // the thing worth asserting is that its signs really do line up with the mixer's: a
            // positive roll demand has to drive the left pair harder than the right.
            TorqueDemand rollRight = new TorqueDemand(0.4, 0, 0);
            MotorOutputs mixed =
                    MotorMixer.mix(0.5, rollRight.roll(), rollRight.pitch(), rollRight.yaw());

            assertTrue(
                    mixed.frontLeft() > mixed.frontRight(),
                    "a positive roll demand should push the left side harder");
            assertTrue(mixed.rearLeft() > mixed.rearRight());
        }

        @Test
        void aFullDemandIsAsFarAsTheMixerCanBeAsked() {
            // 1 is not "maximum torque in some absolute sense", it is the whole of the range the
            // mixer represents. Anything beyond it could only be clipped, so it is rejected instead.
            assertEquals(1.0, new TorqueDemand(1, -1, 1).roll(), 1e-12);
            assertThrows(IllegalArgumentException.class, () -> new TorqueDemand(1.000001, 0, 0));
        }
    }

    @Nested
    class Validation {

        @Test
        void rejectsAnAxisOutsideTheMixersRange() {
            assertThrows(IllegalArgumentException.class, () -> new TorqueDemand(2, 0, 0));
            assertThrows(IllegalArgumentException.class, () -> new TorqueDemand(0, -2, 0));
            assertThrows(IllegalArgumentException.class, () -> new TorqueDemand(0, 0, 1.5));
        }

        @Test
        void rejectsNonFiniteValues() {
            assertThrows(IllegalArgumentException.class, () -> new TorqueDemand(Double.NaN, 0, 0));
            assertThrows(
                    IllegalArgumentException.class,
                    () -> new TorqueDemand(0, Double.NEGATIVE_INFINITY, 0));
        }

        @Test
        void noneIsCentredOnEveryAxis() {
            assertEquals(new TorqueDemand(0, 0, 0), TorqueDemand.NONE);
        }
    }
}
