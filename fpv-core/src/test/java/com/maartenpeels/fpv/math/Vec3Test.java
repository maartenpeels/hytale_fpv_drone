package com.maartenpeels.fpv.math;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Map;
import java.util.Set;
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

    /**
     * #38. A record's {@code equals} compares {@code double}s with {@code Double.compare}, so
     * {@code −0.0} is unequal to {@code 0.0} and hashes differently. Axis-aligned unit vectors are
     * the worst case — two components exactly zero — and they are what collision and gate crossing
     * compare, so the canonical constructor holds every component free of {@code −0.0}.
     *
     * <p>These are the tests that must fail if that normalisation is ever "simplified" away. Some
     * assert raw bits, because {@code assertEquals(0.0, -0.0)} passes for {@code double} and would
     * hide the bug; the rest assert {@code equals} and {@code hashCode} directly, which is what
     * callers actually depend on.
     */
    @Nested
    class SignedZero {

        private static final long NEGATIVE_ZERO_BITS = Double.doubleToRawLongBits(-0.0);

        private static void assertNoNegativeZero(Vec3 vector) {
            for (double component : new double[] {vector.x(), vector.y(), vector.z()}) {
                assertNotEquals(
                        NEGATIVE_ZERO_BITS,
                        Double.doubleToRawLongBits(component),
                        component + " of " + vector + " must not be negative zero");
            }
        }

        @Test
        void negatingAnAxisVectorStaysEqualToTheSameVectorBuiltDirectly() {
            // The exact case in the ticket: without the fix this assertion fails, because the two
            // zero components come back as −0.0.
            assertEquals(new Vec3(-1, 0, 0), new Vec3(1, 0, 0).negated());
            assertEquals(new Vec3(0, -1, 0), new Vec3(0, 1, 0).negated());
            assertEquals(new Vec3(0, 0, -1), new Vec3(0, 0, 1).negated());
        }

        @Test
        void negatingAnAxisVectorAlsoHashesTheSameSoSetAndMapLookupsWork() {
            // equals without hashCode would still lose every HashSet and HashMap lookup, which is
            // one of the two ways #21 and #6 would have hit this.
            assertEquals(new Vec3(-1, 0, 0).hashCode(), new Vec3(1, 0, 0).negated().hashCode());
            assertTrue(Set.of(new Vec3(-1, 0, 0)).contains(new Vec3(1, 0, 0).negated()));
            assertEquals(
                    "found",
                    Map.of(new Vec3(0, 0, -1), "found").get(new Vec3(0, 0, 1).negated()));
        }

        @Test
        void everyOperationThatCanLandOnAZeroKeepsItPositive() {
            // negated() was the reported symptom, not the bug. Fixing it alone would have left all
            // of these, which is why the normalisation is in the constructor.
            assertNoNegativeZero(new Vec3(1, 0, 0).negated());
            assertNoNegativeZero(new Vec3(1, 2, 3).scale(-1));
            assertNoNegativeZero(new Vec3(-1, -2, -3).scale(0));
            assertNoNegativeZero(new Vec3(2, 4, 6).cross(new Vec3(1, 2, 3)));
            assertNoNegativeZero(new Vec3(1, 2, 3).minus(new Vec3(1, 2, 3)));
            assertNoNegativeZero(new Vec3(-1, 0, 0).normalised());
            assertNoNegativeZero(new Vec3(-0.0, -0.0, -0.0));
        }

        @Test
        void aParallelCrossProductEqualsZeroRatherThanMerelyBeingNumericallyZero() {
            // scale(0) and a parallel cross both produce a mix of −0.0 and 0.0 arithmetically, so
            // these two are the strongest single check that the constructor and not the caller is
            // doing the work.
            assertEquals(Vec3.ZERO, new Vec3(1, -2, 3).cross(new Vec3(-2, 4, -6)));
            assertEquals(Vec3.ZERO, new Vec3(-1, -2, -3).scale(0));
            assertEquals(Vec3.ZERO, Vec3.ZERO.negated());
            assertEquals(Vec3.ZERO.hashCode(), Vec3.ZERO.negated().hashCode());
        }

        @Test
        void normalisationDoesNotDisturbNonZeroComponents() {
            // The whole safety argument for doing this in the constructor is that it can only change
            // the sign of a zero, never any other value. Bit-exact, not within a tolerance.
            Vec3 vector = new Vec3(0.1, -1.0 / 3.0, Double.MIN_VALUE);

            assertEquals(Double.doubleToRawLongBits(0.1), Double.doubleToRawLongBits(vector.x()));
            assertEquals(
                    Double.doubleToRawLongBits(-1.0 / 3.0), Double.doubleToRawLongBits(vector.y()));
            assertEquals(
                    Double.doubleToRawLongBits(Double.MIN_VALUE),
                    Double.doubleToRawLongBits(vector.z()));
        }

        @Test
        void doesNotSwallowNonFiniteComponentsWhichIsFinitenessStillNeedsToSee() {
            assertFalse(new Vec3(Double.NaN, 0, 0).isFinite());
            assertFalse(new Vec3(0, Double.NEGATIVE_INFINITY, 0).isFinite());
            assertEquals(Double.NEGATIVE_INFINITY, new Vec3(0, Double.NEGATIVE_INFINITY, 0).y());
        }

        @Test
        void holdsUnderJitCompilationRatherThanOnlyWhenInterpreted() {
            // C2 folds some float-add identities, and the constructor's normalisation is one that
            // must survive. Hot enough to get the constructor compiled.
            int failures = 0;
            for (int i = 0; i < 1_000_000; i++) {
                Vec3 negated = new Vec3(i % 3 + 1, 0, 0).negated();
                if (!negated.equals(new Vec3(-(i % 3 + 1), 0, 0))) {
                    failures++;
                }
            }
            assertEquals(0, failures, "negated axis vectors stopped comparing equal once compiled");
        }
    }
}
