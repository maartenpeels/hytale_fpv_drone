package com.maartenpeels.fpv.math;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

/**
 * Guards the one-line body of {@link SignedZero#canonical}.
 *
 * <p>Every assertion here is on raw bits rather than on {@code double} values, because
 * {@code assertEquals(0.0, -0.0)} <em>passes</em> for {@code double} — {@code assertEquals} uses
 * {@code ==} for primitives, which is exactly the comparison that cannot see the bug. Asserting on
 * values would produce a test that is green whether or not the method does anything.
 *
 * <p>The {@code Identity} group is the load-bearing half: it is what fails if someone reads
 * {@code value + 0.0}, concludes it is dead code, and replaces the body with {@code return value}.
 */
class SignedZeroTest {

    private static long bits(double value) {
        return Double.doubleToRawLongBits(value);
    }

    private static final long POSITIVE_ZERO_BITS = bits(0.0);
    private static final long NEGATIVE_ZERO_BITS = bits(-0.0);

    @Nested
    class Canonicalisation {

        @Test
        void mapsNegativeZeroOntoPositiveZeroBecauseRecordEqualityDistinguishesThem() {
            assertEquals(POSITIVE_ZERO_BITS, bits(SignedZero.canonical(-0.0)));
        }

        @Test
        void leavesPositiveZeroAlone() {
            assertEquals(POSITIVE_ZERO_BITS, bits(SignedZero.canonical(0.0)));
        }

        @Test
        void neverReturnsNegativeZeroForAnyInput() {
            double[] inputs = {
                -0.0, 0.0, -1.0, 1.0, -1e-320, 1e-320, Double.MIN_VALUE, -Double.MIN_VALUE,
                Double.MAX_VALUE, -Double.MAX_VALUE
            };
            for (double input : inputs) {
                assertTrue(
                        bits(SignedZero.canonical(input)) != NEGATIVE_ZERO_BITS,
                        "canonical(" + input + ") must not be negative zero");
            }
        }
    }

    /**
     * Adding zero has to be the identity on everything that is not {@code −0.0}, or the fix would be
     * changing physics rather than a sign bit. These are the assertions a
     * {@code return value + 1e-300}-style "equivalent" would fail.
     */
    @Nested
    class Identity {

        @Test
        void returnsOrdinaryValuesBitIdenticallySoNoArithmeticResultCanChange() {
            double[] inputs = {
                1.0, -1.0, 32.0, -32.0, 0.1, -0.1, 1.0 / 3.0, -1.0 / 240.0,
                Math.PI, -Math.E, 1e-16, -1e16, Double.MAX_VALUE, -Double.MAX_VALUE
            };
            for (double input : inputs) {
                assertEquals(bits(input), bits(SignedZero.canonical(input)), "canonical(" + input + ")");
            }
        }

        @Test
        void preservesSubnormalsBecauseAddingZeroIsExactRatherThanRounded() {
            // The smallest magnitudes are where a sloppy "equivalent" implementation would round to
            // zero and silently change a value instead of a sign.
            double[] subnormals = {
                Double.MIN_VALUE, -Double.MIN_VALUE, 1e-320, -1e-320, Double.MIN_NORMAL / 2
            };
            for (double input : subnormals) {
                assertEquals(bits(input), bits(SignedZero.canonical(input)), "canonical(" + input + ")");
            }
        }

        @Test
        void preservesInfinitiesAndNaNSoDownstreamFinitenessChecksStillSeeThem() {
            assertEquals(
                    bits(Double.POSITIVE_INFINITY),
                    bits(SignedZero.canonical(Double.POSITIVE_INFINITY)));
            assertEquals(
                    bits(Double.NEGATIVE_INFINITY),
                    bits(SignedZero.canonical(Double.NEGATIVE_INFINITY)));
            assertTrue(Double.isNaN(SignedZero.canonical(Double.NaN)));
        }
    }

    /**
     * The property has to hold in JIT-compiled code, not only interpreted code: C2 does fold some
     * float-add identities, and this method's whole purpose is one that must not be folded. The loop
     * is long enough to get the call site compiled.
     *
     * <p>The {@code −0.0} is <em>computed</em> rather than written as a literal, so that constant
     * folding at the call site cannot make the test pass for the wrong reason.
     */
    @Nested
    class UnderJitCompilation {

        @Test
        void survivesCompilationRatherThanBeingFoldedAwayAsAnAddOfZero() {
            int failures = 0;
            int canonicalised = 0;
            for (int i = 0; i < 2_000_000; i++) {
                double negativeZero = 0.0 * -(i % 5 + 1);
                if (bits(negativeZero) != NEGATIVE_ZERO_BITS) {
                    failures++;   // the premise broke; the input was not −0.0 to begin with
                } else if (bits(SignedZero.canonical(negativeZero)) == NEGATIVE_ZERO_BITS) {
                    failures++;
                } else {
                    canonicalised++;
                }
            }
            assertEquals(0, failures, "negative zero survived canonical() after JIT compilation");
            assertEquals(2_000_000, canonicalised);
        }
    }
}
