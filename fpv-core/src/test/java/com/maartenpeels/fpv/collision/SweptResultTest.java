package com.maartenpeels.fpv.collision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.maartenpeels.fpv.math.Vec3;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class SweptResultTest {

    private static SweptResult.Contact contactWithNormal(Vec3 normal) {
        return new SweptResult.Contact(0.5, 0.75, normal);
    }

    @Nested
    class Constants {

        @Test
        void areEqualToAFreshlyBuiltCaseSoTestsCanCompareByValue() {
            assertEquals(new SweptResult.Miss(), SweptResult.MISS);
            assertEquals(new SweptResult.AlreadyOverlapping(), SweptResult.ALREADY_OVERLAPPING);
        }

        @Test
        void areDistinctFromEachOtherBecauseTheyMeanOppositeProblems() {
            assertFalse(SweptResult.MISS.equals(SweptResult.ALREADY_OVERLAPPING));
        }
    }

    @Nested
    class ContactValidation {

        @Test
        void rejectsAnEntryTimeOutsideTheSegment() {
            assertThrows(IllegalArgumentException.class,
                    () -> new SweptResult.Contact(-0.1, 0.5, Vec3.UP));
            assertThrows(IllegalArgumentException.class,
                    () -> new SweptResult.Contact(1.0, 1.5, Vec3.UP));
        }

        @Test
        void rejectsAnExitTimeThatIsNotAfterTheEntryTimeBecauseThatIsNotAnInterval() {
            assertThrows(IllegalArgumentException.class,
                    () -> new SweptResult.Contact(0.5, 0.5, Vec3.UP));
            assertThrows(IllegalArgumentException.class,
                    () -> new SweptResult.Contact(0.5, 0.25, Vec3.UP));
        }

        @Test
        void acceptsAnInfiniteExitTimeBecauseAVanishingDisplacementNeverLeaves() {
            SweptResult.Contact creeping =
                    new SweptResult.Contact(0.0, Double.POSITIVE_INFINITY, Vec3.UP);

            assertFalse(creeping.passedFullyThrough());
        }

        @Test
        void rejectsANonUnitNormalBecauseEveryConsumerReadsItsDirectionAndItsSign() {
            assertThrows(IllegalArgumentException.class,
                    () -> contactWithNormal(Vec3.ZERO));
            assertThrows(IllegalArgumentException.class,
                    () -> contactWithNormal(new Vec3(2, 0, 0)));
            assertThrows(IllegalArgumentException.class,
                    () -> contactWithNormal(new Vec3(1, 1, 0)));
            assertThrows(IllegalArgumentException.class,
                    () -> contactWithNormal(null));
        }

        @Test
        void acceptsANormalRotatedOutOfAnotherFrameDespiteTheRoundingItPicksUp() {
            Vec3 rotated = new Vec3(0.6, 0.8, 0).normalised();

            assertEquals(rotated, contactWithNormal(rotated).normal());
        }

        @Test
        void normalisesNegativeZeroEntryTimeBecauseItBreaksEqualityAgainstZero() {
            SweptResult.Contact hit = new SweptResult.Contact(-0.0, 1.0, Vec3.UP);

            assertEquals(0.0, hit.entryTime());
            assertEquals(new SweptResult.Contact(0.0, 1.0, Vec3.UP), hit);
        }
    }

    @Nested
    class EnteredAlong {

        @Test
        void isTrueWhenTheEnteredFaceOpposesTheDirectionOfTravel() {
            assertTrue(contactWithNormal(new Vec3(0, 0, 1)).enteredAlong(new Vec3(0, 0, -1)));
        }

        @Test
        void isFalseForTheWrongWayRound() {
            assertFalse(contactWithNormal(new Vec3(0, 0, -1)).enteredAlong(new Vec3(0, 0, -1)));
        }

        @Test
        void isFalseForAPerpendicularEntryBecauseNeitherDirectionWasTravelled() {
            assertFalse(contactWithNormal(new Vec3(1, 0, 0)).enteredAlong(new Vec3(0, 0, -1)));
            assertFalse(contactWithNormal(new Vec3(1, 0, 0)).enteredAlong(new Vec3(0, 0, 1)));
        }

        @Test
        void ignoresTheMagnitudeOfTheDirectionAndReadsOnlyItsSign() {
            SweptResult.Contact hit = contactWithNormal(new Vec3(0, 0, 1));

            assertTrue(hit.enteredAlong(new Vec3(0, 0, -1e-9)));
            assertTrue(hit.enteredAlong(new Vec3(0, 0, -1e9)));
        }

        @Test
        void treatsAGlancingApproachByItsDominantComponent() {
            // Mostly forward, slightly sideways: still a pass in the gate's direction.
            assertTrue(contactWithNormal(new Vec3(0, 0, 1)).enteredAlong(new Vec3(0.1, 0, -1)));
        }

        @Test
        void rejectsANonFiniteDirection() {
            SweptResult.Contact hit = contactWithNormal(Vec3.UP);

            assertThrows(IllegalArgumentException.class,
                    () -> hit.enteredAlong(new Vec3(Double.NaN, 0, 0)));
            assertThrows(IllegalArgumentException.class, () -> hit.enteredAlong(null));
        }
    }

    @Nested
    class PassedFullyThrough {

        @Test
        void isTrueWhenTheMoverIsClearAgainByTheEndOfTheSegment() {
            assertTrue(new SweptResult.Contact(0.2, 0.8, Vec3.UP).passedFullyThrough());
            assertTrue(new SweptResult.Contact(0.2, 1.0, Vec3.UP).passedFullyThrough());
        }

        @Test
        void isFalseWhenTheSegmentEndsWithTheBoxesStillOverlapping() {
            assertFalse(new SweptResult.Contact(0.2, 1.0001, Vec3.UP).passedFullyThrough());
        }
    }

    @Nested
    class PositionAt {

        @Test
        void interpolatesTheMoversCentreToTheMomentOfContact() {
            SweptResult.Contact hit = new SweptResult.Contact(0.25, 0.75, Vec3.UP);

            assertEquals(new Vec3(1, 2, 3),
                    hit.positionAt(new Vec3(0, 0, 0), new Vec3(4, 8, 12)));
        }
    }
}
