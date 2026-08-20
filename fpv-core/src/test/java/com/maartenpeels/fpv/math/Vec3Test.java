package com.maartenpeels.fpv.math;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class Vec3Test {

    private static final double TOLERANCE = 1e-12;

    @Nested
    class Algebra {

        @Test
        void addsAndSubtractsComponentwise() {
            Vec3 sum = new Vec3(1, 2, 3).plus(new Vec3(10, 20, 30));
            Vec3 difference = new Vec3(1, 2, 3).minus(new Vec3(10, 20, 30));

            assertEquals(new Vec3(11, 22, 33), sum);
            assertEquals(new Vec3(-9, -18, -27), difference);
        }

        @Test
        void scalesAndNegates() {
            assertEquals(new Vec3(2, 4, 6), new Vec3(1, 2, 3).scale(2));
            assertEquals(new Vec3(-1, -2, -3), new Vec3(1, 2, 3).negated());
        }

        @Test
        void dotProductIsTheSumOfProducts() {
            assertEquals(1 * 4 + 2 * 5 + 3 * 6, new Vec3(1, 2, 3).dot(new Vec3(4, 5, 6)));
        }

        @Test
        void crossProductIsRightHandedSoTheFrameConventionHolds() {
            // X cross Y == Z is the definition the whole flight model's sign conventions rest on;
            // if this flipped, every rotation in BodyRates would silently reverse.
            assertEquals(new Vec3(0, 0, 1), new Vec3(1, 0, 0).cross(new Vec3(0, 1, 0)));
            assertEquals(new Vec3(1, 0, 0), new Vec3(0, 1, 0).cross(new Vec3(0, 0, 1)));
            assertEquals(new Vec3(0, 1, 0), new Vec3(0, 0, 1).cross(new Vec3(1, 0, 0)));
        }

        @Test
        void crossProductOfParallelVectorsIsZero() {
            assertEquals(Vec3.ZERO, new Vec3(2, 4, 6).cross(new Vec3(1, 2, 3)));
        }
    }

    @Nested
    class Length {

        @Test
        void lengthIsEuclidean() {
            assertEquals(5.0, new Vec3(3, 4, 0).length(), TOLERANCE);
            assertEquals(25.0, new Vec3(3, 4, 0).lengthSquared(), TOLERANCE);
        }

        @Test
        void normalisedHasUnitLength() {
            assertEquals(1.0, new Vec3(3, -4, 12).normalised().length(), TOLERANCE);
        }

        @Test
        void normalisingZeroYieldsZeroRatherThanNaN() {
            // A zero velocity vector is normal, not exceptional -- drag must not poison the state
            // with NaN the moment a drone comes to rest.
            assertEquals(Vec3.ZERO, Vec3.ZERO.normalised());
        }
    }

    @Nested
    class Finiteness {

        @Test
        void detectsNonFiniteComponents() {
            assertTrue(new Vec3(1, 2, 3).isFinite());
            assertFalse(new Vec3(Double.NaN, 0, 0).isFinite());
            assertFalse(new Vec3(0, Double.POSITIVE_INFINITY, 0).isFinite());
            assertFalse(new Vec3(0, 0, Double.NEGATIVE_INFINITY).isFinite());
        }
    }
}
