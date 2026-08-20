package com.maartenpeels.fpv.flight;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import com.maartenpeels.fpv.math.Quat;
import com.maartenpeels.fpv.math.Vec3;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class DroneStateTest {

    private static final double TOLERANCE = 1e-12;

    @Nested
    class RestingAt {

        @Test
        void startsLevelAndStationary() {
            DroneState state = DroneState.restingAt(new Vec3(10, 64, -30));

            assertEquals(new Vec3(10, 64, -30), state.position());
            assertEquals(Vec3.ZERO, state.velocity());
            assertEquals(Quat.IDENTITY, state.orientation());
            assertEquals(BodyRates.ZERO, state.bodyRates());
        }
    }

    @Nested
    class BodyAxes {

        // The frame convention, asserted rather than only documented. Every sign downstream --
        // mixer, rates, the plugin's camera adapter -- is derived from these three.

        @Test
        void aLevelDroneThrustsStraightUp() {
            assertEquals(Vec3.UP, DroneState.restingAt(Vec3.ZERO).thrustAxis());
        }

        @Test
        void aLevelDroneFacesNegativeZToMatchHytalesYawZero() {
            assertEquals(new Vec3(0, 0, -1), DroneState.restingAt(Vec3.ZERO).forward());
        }

        @Test
        void aLevelDronesRightHandSideIsPositiveX() {
            assertEquals(new Vec3(1, 0, 0), DroneState.restingAt(Vec3.ZERO).right());
        }

        @Test
        void invertedFlightPutsTheThrustAxisDownwards() {
            DroneState upsideDown =
                    new DroneState(
                            Vec3.ZERO,
                            Vec3.ZERO,
                            Quat.fromAxisAngle(new Vec3(0, 0, -1), Math.PI),
                            BodyRates.ZERO);

            assertEquals(-1.0, upsideDown.thrustAxis().y(), 1e-15);
        }
    }

    @Nested
    class Speed {

        @Test
        void speedIsTheMagnitudeOfVelocity() {
            DroneState state =
                    new DroneState(Vec3.ZERO, new Vec3(3, 4, 0), Quat.IDENTITY, BodyRates.ZERO);

            assertEquals(5.0, state.speed(), TOLERANCE);
        }
    }

    @Nested
    class Validation {

        @Test
        void rejectsNonFiniteStateSoOneBadTickCannotPoisonTheRest() {
            assertThrows(
                    IllegalArgumentException.class,
                    () ->
                            new DroneState(
                                    new Vec3(Double.NaN, 0, 0),
                                    Vec3.ZERO,
                                    Quat.IDENTITY,
                                    BodyRates.ZERO));
            assertThrows(
                    IllegalArgumentException.class,
                    () ->
                            new DroneState(
                                    Vec3.ZERO,
                                    new Vec3(0, Double.POSITIVE_INFINITY, 0),
                                    Quat.IDENTITY,
                                    BodyRates.ZERO));
            assertThrows(
                    IllegalArgumentException.class,
                    () ->
                            new DroneState(
                                    Vec3.ZERO,
                                    Vec3.ZERO,
                                    Quat.IDENTITY,
                                    new BodyRates(Double.NaN, 0, 0)));
        }

        @Test
        void rejectsMissingComponents() {
            assertThrows(
                    IllegalArgumentException.class,
                    () -> new DroneState(null, Vec3.ZERO, Quat.IDENTITY, BodyRates.ZERO));
            assertThrows(
                    IllegalArgumentException.class,
                    () -> new DroneState(Vec3.ZERO, Vec3.ZERO, null, BodyRates.ZERO));
        }
    }
}
