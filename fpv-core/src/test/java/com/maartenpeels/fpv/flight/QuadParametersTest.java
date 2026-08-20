package com.maartenpeels.fpv.flight;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class QuadParametersTest {

    private static final double TOLERANCE = 1e-12;

    @Nested
    class Defaults {

        @Test
        void gravityMatchesTheServersOwnFigureRatherThanRealGravity() {
            // PhysicsConstants.GRAVITY_ACCELERATION in the decompiled server. Hytale's world is not
            // metric-consistent; a drone built around 9.81 would feel lunar next to everything else.
            assertEquals(32.0, QuadParameters.DEFAULT.gravity(), TOLERANCE);
        }

        @Test
        void yawAuthorityIsWeakerThanRollAndPitchAsOnARealAirframe() {
            assertTrue(
                    QuadParameters.DEFAULT.yawAuthority()
                            < QuadParameters.DEFAULT.rollPitchAuthority());
        }
    }

    @Nested
    class Derived {

        @Test
        void maxThrustAccelerationIsThrustToWeightTimesGravity() {
            QuadParameters parameters =
                    QuadParameters.builder().gravity(32).thrustToWeight(8).build();

            assertEquals(256.0, parameters.maxThrustAcceleration(), TOLERANCE);
        }

        @Test
        void hoverCollectiveIsTheRootOfTheInverseThrustToWeightBecauseThrustIsQuadratic() {
            QuadParameters parameters = QuadParameters.builder().thrustToWeight(8).build();

            assertEquals(Math.sqrt(1.0 / 8.0), parameters.hoverCollective(), TOLERANCE);
        }

        @Test
        void hoverCollectiveLandsNearAThirdOfStickForARealisticThrustToWeight() {
            // The feel check behind the quadratic thrust model. A linear model would hover at 12.5%.
            double hover = QuadParameters.builder().thrustToWeight(8).build().hoverCollective();

            assertTrue(hover > 0.30 && hover < 0.40, "hover was " + hover);
        }

        @Test
        void hoverCollectiveIsOneWhenThrustExactlyEqualsWeight() {
            assertEquals(
                    1.0,
                    QuadParameters.builder().thrustToWeight(1).build().hoverCollective(),
                    TOLERANCE);
        }
    }

    @Nested
    class Validation {

        @Test
        void rejectsNonPositiveGravityAndThrust() {
            assertThrows(
                    IllegalArgumentException.class,
                    () -> QuadParameters.builder().gravity(0).build());
            assertThrows(
                    IllegalArgumentException.class,
                    () -> QuadParameters.builder().thrustToWeight(-1).build());
        }

        @Test
        void acceptsZeroDragBecauseADragFreeModelIsHowTheIntegratorIsTested() {
            QuadParameters parameters = QuadParameters.builder().withoutDrag().build();

            assertEquals(0.0, parameters.linearDrag(), TOLERANCE);
            assertEquals(0.0, parameters.quadraticDrag(), TOLERANCE);
            assertEquals(0.0, parameters.angularDrag(), TOLERANCE);
        }

        @Test
        void rejectsNegativeDrag() {
            assertThrows(
                    IllegalArgumentException.class,
                    () -> QuadParameters.builder().quadraticDrag(-0.01).build());
            assertThrows(
                    IllegalArgumentException.class,
                    () -> QuadParameters.builder().angularDrag(-1).build());
        }

        @Test
        void rejectsNonFiniteValues() {
            assertThrows(
                    IllegalArgumentException.class,
                    () -> QuadParameters.builder().gravity(Double.NaN).build());
            assertThrows(
                    IllegalArgumentException.class,
                    () -> QuadParameters.builder().linearDrag(Double.POSITIVE_INFINITY).build());
        }

        @Test
        void rejectsAnAxisThatCannotRotateAtAll() {
            // A zero max rate would silently disable an axis, which is far harder to diagnose in
            // the air than a refusal to build.
            assertThrows(
                    IllegalArgumentException.class,
                    () -> QuadParameters.builder().maxRates(new BodyRates(1, 0, 1)).build());
            assertThrows(
                    IllegalArgumentException.class,
                    () -> QuadParameters.builder().maxRates(null).build());
        }

        @Test
        void rejectsAnAxisWithNoTorqueAuthority() {
            // Authority scales the achieved thrust differential into angular acceleration, so zero
            // is an axis that cannot be commanded at all.
            assertThrows(
                    IllegalArgumentException.class,
                    () -> QuadParameters.builder().rollPitchAuthority(0).build());
            assertThrows(
                    IllegalArgumentException.class,
                    () -> QuadParameters.builder().yawAuthority(-1).build());
        }
    }
}
