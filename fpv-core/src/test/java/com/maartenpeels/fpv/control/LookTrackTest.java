package com.maartenpeels.fpv.control;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class LookTrackTest {

    @Nested
    class Unset {

        @Test
        void isDistinguishableFromLookingStraightAheadSoAFirstSampleCannotReadAsAFlick() {
            assertFalse(LookTrack.UNSET.present());
            assertTrue(LookTrack.at(0.0, 0.0).present());
            assertNotEquals(LookTrack.at(0.0, 0.0), LookTrack.UNSET);
        }
    }

    @Nested
    class At {

        @Test
        void holdsTheAnglesItWasGiven() {
            LookTrack track = LookTrack.at(1.25, -0.5);

            assertEquals(1.25, track.yaw());
            assertEquals(-0.5, track.pitch());
            assertTrue(track.present());
        }

        @Test
        void rejectsNonFiniteAnglesBecauseAPresentTrackIsSomethingWeChoseToStore() {
            assertThrows(IllegalArgumentException.class, () -> LookTrack.at(Double.NaN, 0.0));
            assertThrows(
                    IllegalArgumentException.class,
                    () -> LookTrack.at(0.0, Double.POSITIVE_INFINITY));
        }

        @Test
        void allowsNonFiniteAnglesOnAnAbsentTrackBecauseTheyAreNeverRead() {
            LookTrack absent = new LookTrack(Double.NaN, Double.NaN, false);

            assertFalse(absent.present());
        }
    }
}
