package com.maartenpeels.fpv.control;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class ControlInputTest {

    @Nested
    class CanonicalConstructor {

        @Test
        void acceptsFullStickDeflectionOnEveryAxis() {
            ControlInput input = new ControlInput(1f, -1f, 1f, -1f);

            assertEquals(1f, input.throttle());
            assertEquals(-1f, input.roll());
            assertEquals(1f, input.pitch());
            assertEquals(-1f, input.yaw());
        }

        @Test
        void rejectsNegativeThrottleBecauseThrottleIsUnidirectional() {
            assertThrows(IllegalArgumentException.class, () -> new ControlInput(-0.1f, 0f, 0f, 0f));
        }

        @Test
        void rejectsAttitudeAxisBeyondFullDeflection() {
            assertThrows(IllegalArgumentException.class, () -> new ControlInput(0f, 1.01f, 0f, 0f));
            assertThrows(IllegalArgumentException.class, () -> new ControlInput(0f, 0f, -1.01f, 0f));
            assertThrows(IllegalArgumentException.class, () -> new ControlInput(0f, 0f, 0f, 2f));
        }

        @Test
        void rejectsNonFiniteValues() {
            assertThrows(IllegalArgumentException.class,
                    () -> new ControlInput(Float.NaN, 0f, 0f, 0f));
            assertThrows(IllegalArgumentException.class,
                    () -> new ControlInput(0f, Float.POSITIVE_INFINITY, 0f, 0f));
        }
    }

    @Nested
    class Clamped {

        @Test
        void clampsRatherThanThrowingSoANoisyClientCannotBreakTheTick() {
            ControlInput input = ControlInput.clamped(4f, -3f, 2f, -9f);

            assertEquals(1f, input.throttle());
            assertEquals(-1f, input.roll());
            assertEquals(1f, input.pitch());
            assertEquals(-1f, input.yaw());
        }

        @Test
        void clampsThrottleToClosedRatherThanNegative() {
            assertEquals(0f, ControlInput.clamped(-0.5f, 0f, 0f, 0f).throttle());
        }

        @Test
        void collapsesNonFiniteAxesToNeutral() {
            ControlInput input = ControlInput.clamped(
                    Float.NaN, Float.NaN, Float.NEGATIVE_INFINITY, Float.POSITIVE_INFINITY);

            assertEquals(0f, input.throttle());
            assertEquals(0f, input.roll());
            assertEquals(-1f, input.pitch());
            assertEquals(1f, input.yaw());
        }

        @Test
        void leavesInRangeValuesUntouched() {
            ControlInput input = ControlInput.clamped(0.42f, -0.25f, 0.75f, 0.1f);

            assertEquals(0.42f, input.throttle());
            assertEquals(-0.25f, input.roll());
            assertEquals(0.75f, input.pitch());
            assertEquals(0.1f, input.yaw());
        }
    }

    @Nested
    class Neutral {

        @Test
        void isCentredWithThrottleClosed() {
            assertEquals(0f, ControlInput.NEUTRAL.throttle());
            assertTrue(ControlInput.NEUTRAL.sticksCentred());
        }

        @Test
        void sticksCentredIgnoresThrottle() {
            assertTrue(new ControlInput(1f, 0f, 0f, 0f).sticksCentred());
            assertFalse(new ControlInput(0f, 0.01f, 0f, 0f).sticksCentred());
        }
    }
}
