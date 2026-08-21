package com.maartenpeels.fpv.control;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
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

        @Test
        void canonicalisesAnyAbsentTrackSoEveryNothingSeenYetComparesEqual() {
            assertEquals(LookTrack.UNSET, new LookTrack(5.0, -2.0, 9.0, false));
            assertEquals(LookTrack.UNSET.hashCode(), new LookTrack(5.0, -2.0, 9.0, false).hashCode());
        }

        @Test
        void toleratesNonFiniteAnglesWhileAbsentBecauseTheyAreNeverRead() {
            LookTrack absent = new LookTrack(Double.NaN, Double.NaN, Double.NaN, false);

            assertFalse(absent.present());
            assertEquals(0.0, absent.yaw());
        }
    }

    @Nested
    class At {

        @Test
        void holdsTheAnglesItWasGivenAndStartsTheClockAtZero() {
            LookTrack track = LookTrack.at(1.25, -0.5);

            assertEquals(1.25, track.yaw());
            assertEquals(-0.5, track.pitch());
            assertEquals(0.0, track.secondsSinceSample());
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
        void rejectsANegativeElapsedTimeBecauseTheSampleCannotBeInTheFuture() {
            assertThrows(
                    IllegalArgumentException.class, () -> new LookTrack(0.0, 0.0, -0.01, true));
        }
    }

    @Nested
    class Aged {

        @Test
        void accumulatesTheIntervalTheNextDeltaWillBeMeasuredOver() {
            LookTrack track = LookTrack.at(1.0, 0.5).aged(0.1).aged(0.2);

            assertEquals(1.0, track.yaw());
            assertEquals(0.5, track.pitch());
            assertEquals(0.3, track.secondsSinceSample(), 1e-12);
        }

        @Test
        void leavesAnAbsentTrackAbsentBecauseThereIsNoSampleForTheClockToRunFrom() {
            assertSame(LookTrack.UNSET, LookTrack.UNSET.aged(0.5));
        }

        @Test
        void rejectsANegativeOrNonFiniteInterval() {
            LookTrack track = LookTrack.at(0.0, 0.0);

            assertThrows(IllegalArgumentException.class, () -> track.aged(-0.1));
            assertThrows(IllegalArgumentException.class, () -> track.aged(Double.NaN));
        }
    }
}
