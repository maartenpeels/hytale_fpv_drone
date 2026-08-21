package com.maartenpeels.fpv.math;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class AabbTest {

    private static final Aabb UNIT_BLOCK = new Aabb(Vec3.ZERO, new Vec3(1, 1, 1));

    @Nested
    class CanonicalConstructor {

        @Test
        void rejectsAnInvertedBoxBecauseThatIsATransposedArgumentRatherThanAShape() {
            assertThrows(IllegalArgumentException.class,
                    () -> new Aabb(new Vec3(1, 0, 0), Vec3.ZERO));
            assertThrows(IllegalArgumentException.class,
                    () -> new Aabb(new Vec3(0, 1, 0), Vec3.ZERO));
            assertThrows(IllegalArgumentException.class,
                    () -> new Aabb(new Vec3(0, 0, 1), Vec3.ZERO));
        }

        @Test
        void acceptsADegenerateBoxBecauseAGatePlaneIsAFairThingToAskAbout() {
            Aabb plane = new Aabb(new Vec3(-2, -2, 0), new Vec3(2, 2, 0));

            assertEquals(0.0, plane.size().z());
            assertEquals(new Vec3(0, 0, 0), plane.centre());
        }

        @Test
        void rejectsNonFiniteCorners() {
            assertThrows(IllegalArgumentException.class,
                    () -> new Aabb(new Vec3(Double.NaN, 0, 0), new Vec3(1, 1, 1)));
            assertThrows(IllegalArgumentException.class,
                    () -> new Aabb(Vec3.ZERO, new Vec3(Double.POSITIVE_INFINITY, 1, 1)));
        }
    }

    @Nested
    class Factories {

        @Test
        void centredAtRoundTripsThroughCentreAndHalfExtents() {
            Vec3 centre = new Vec3(3, -4, 5.5);
            Vec3 halfExtents = new Vec3(0.25, 0.1, 0.25);

            Aabb box = Aabb.centredAt(centre, halfExtents);

            // Not bit-exact, and cannot be: min/max storage rounds twice on any non-dyadic extent.
            assertEquals(centre.x(), box.centre().x(), 1e-15);
            assertEquals(centre.y(), box.centre().y(), 1e-15);
            assertEquals(centre.z(), box.centre().z(), 1e-15);
            assertEquals(halfExtents.x(), box.halfExtents().x(), 1e-15);
            assertEquals(halfExtents.y(), box.halfExtents().y(), 1e-15);
            assertEquals(halfExtents.z(), box.halfExtents().z(), 1e-15);
        }

        @Test
        void centredAtRoundTripsExactlyForADyadicExtentBecauseThatRoundsNotAtAll() {
            Vec3 centre = new Vec3(3, -4, 5.5);
            Vec3 halfExtents = new Vec3(0.25, 0.125, 0.5);

            Aabb box = Aabb.centredAt(centre, halfExtents);

            assertEquals(centre, box.centre());
            assertEquals(halfExtents, box.halfExtents());
        }

        @Test
        void centredAtWithZeroHalfExtentsGivesAPointBecauseAPointSweepIsARayCast() {
            Aabb point = Aabb.centredAt(new Vec3(1, 2, 3), Vec3.ZERO);

            assertEquals(point.min(), point.max());
            assertEquals(Vec3.ZERO, point.halfExtents());
        }

        @Test
        void centredAtRejectsNegativeHalfExtents() {
            assertThrows(IllegalArgumentException.class,
                    () -> Aabb.centredAt(Vec3.ZERO, new Vec3(-0.1, 0, 0)));
        }

        @Test
        void cubeAtBuildsABlockFromItsLowestCorner() {
            Aabb block = Aabb.cubeAt(new Vec3(4, 5, 6), 1.0);

            assertEquals(new Vec3(4, 5, 6), block.min());
            assertEquals(new Vec3(5, 6, 7), block.max());
            assertEquals(new Vec3(4.5, 5.5, 6.5), block.centre());
        }

        @Test
        void cubeAtRejectsANegativeSize() {
            assertThrows(IllegalArgumentException.class, () -> Aabb.cubeAt(Vec3.ZERO, -1.0));
        }
    }

    @Nested
    class ExpandedBy {

        @Test
        void growsOutwardOnBothSidesOfEveryAxis() {
            Aabb expanded = UNIT_BLOCK.expandedBy(new Vec3(0.25, 0.5, 1));

            assertEquals(new Vec3(-0.25, -0.5, -1), expanded.min());
            assertEquals(new Vec3(1.25, 1.5, 2), expanded.max());
        }

        @Test
        void makesOverlapEquivalentToTheExpandedBoxContainingTheMoversCentre() {
            // The Minkowski identity the sweep rests on, checked rather than assumed.
            Vec3 halfExtents = new Vec3(0.3, 0.2, 0.4);
            Aabb expanded = UNIT_BLOCK.expandedBy(halfExtents);

            for (double x = -1; x <= 2; x += 0.07) {
                for (double y = -1; y <= 2; y += 0.11) {
                    for (double z = -1; z <= 2; z += 0.13) {
                        Vec3 centre = new Vec3(x, y, z);
                        boolean overlaps = Aabb.centredAt(centre, halfExtents).overlaps(UNIT_BLOCK);

                        assertEquals(overlaps, strictlyInside(expanded, centre), "at " + centre);
                    }
                }
            }
        }

        @Test
        void rejectsNegativeHalfExtentsBecauseShrinkingIsADifferentOperation() {
            assertThrows(IllegalArgumentException.class,
                    () -> UNIT_BLOCK.expandedBy(new Vec3(0, -0.1, 0)));
        }

        private static boolean strictlyInside(Aabb box, Vec3 point) {
            return point.x() > box.min().x() && point.x() < box.max().x()
                    && point.y() > box.min().y() && point.y() < box.max().y()
                    && point.z() > box.min().z() && point.z() < box.max().z();
        }
    }

    @Nested
    class Contains {

        @Test
        void countsTheSurfaceBecauseTheBoxIsClosed() {
            assertTrue(UNIT_BLOCK.contains(Vec3.ZERO));
            assertTrue(UNIT_BLOCK.contains(new Vec3(1, 1, 1)));
            assertTrue(UNIT_BLOCK.contains(new Vec3(0.5, 0, 0.5)));
        }

        @Test
        void excludesAPointOutsideAnySingleAxis() {
            assertFalse(UNIT_BLOCK.contains(new Vec3(-0.001, 0.5, 0.5)));
            assertFalse(UNIT_BLOCK.contains(new Vec3(0.5, 1.001, 0.5)));
            assertFalse(UNIT_BLOCK.contains(new Vec3(0.5, 0.5, -0.001)));
        }
    }

    @Nested
    class Overlaps {

        @Test
        void findsAGenuineOverlap() {
            assertTrue(UNIT_BLOCK.overlaps(Aabb.centredAt(new Vec3(1, 1, 1), new Vec3(0.5, 0.5, 0.5))));
        }

        @Test
        void isStrictAboutTouchingBecauseZeroVolumeIsNotAnOverlap() {
            Aabb restingOnTop = new Aabb(new Vec3(0, 1, 0), new Vec3(1, 2, 1));

            assertFalse(UNIT_BLOCK.overlaps(restingOnTop));
            assertTrue(UNIT_BLOCK.overlaps(restingOnTop.translatedBy(new Vec3(0, -1e-12, 0))));
        }

        @Test
        void isStrictAboutACornerTouchToo() {
            assertFalse(UNIT_BLOCK.overlaps(new Aabb(new Vec3(1, 1, 1), new Vec3(2, 2, 2))));
        }

        @Test
        void isSymmetric() {
            Aabb other = Aabb.centredAt(new Vec3(0.5, 1.4, 0.5), new Vec3(0.5, 0.5, 0.5));

            assertEquals(UNIT_BLOCK.overlaps(other), other.overlaps(UNIT_BLOCK));
        }

        @Test
        void neverOverlapsWhenOneBoxIsDegenerateOnTheSharedAxis() {
            Aabb plane = new Aabb(new Vec3(-1, 0.5, -1), new Vec3(2, 0.5, 2));

            assertFalse(UNIT_BLOCK.overlaps(plane));
        }

        @Test
        void answersFalseForADegenerateBoxWhereTheSweepAnswersAlreadyOverlapping() {
            // Pins the documented disagreement rather than leaving it to be discovered: this measures
            // volume at an instant, the sweep measures duration of containment. A #6 gate plane is
            // exactly the case where using this as a pre-check beside a sweep would answer false
            // every single time.
            Aabb gatePlane = new Aabb(new Vec3(-2, -2, 0), new Vec3(2, 2, 0));
            Aabb drone = Aabb.centredAt(Vec3.ZERO, new Vec3(0.25, 0.1, 0.25));

            assertFalse(gatePlane.overlaps(drone));
            assertFalse(drone.overlaps(gatePlane));
            assertTrue(gatePlane.expandedBy(drone.halfExtents()).contains(drone.centre()));
        }

        @Test
        void rejectsNull() {
            assertThrows(IllegalArgumentException.class, () -> UNIT_BLOCK.overlaps(null));
        }
    }

    @Nested
    class Union {

        @Test
        void boundsBothBoxesOnEveryAxis() {
            Aabb other = new Aabb(new Vec3(-2, 0.25, 3), new Vec3(-1, 0.5, 4));

            Aabb combined = UNIT_BLOCK.union(other);

            assertEquals(new Vec3(-2, 0, 0), combined.min());
            assertEquals(new Vec3(1, 1, 4), combined.max());
        }

        @Test
        void boundsTheWholeRegionAMovingBoxPassesThroughWhichIsWhySweptCallersWantIt() {
            Aabb start = Aabb.centredAt(new Vec3(0, 0, 0), new Vec3(0.25, 0.25, 0.25));
            Vec3 displacement = new Vec3(10, -3, 0);

            Aabb swept = start.union(start.translatedBy(displacement));

            assertTrue(swept.contains(start.min()));
            assertTrue(swept.contains(start.translatedBy(displacement).max()));
            for (double t = 0; t <= 1; t += 0.05) {
                Vec3 centre = displacement.scale(t);
                assertTrue(swept.contains(centre), "centre at t " + t);
            }
        }

        @Test
        void isIdempotentAndSymmetric() {
            Aabb other = new Aabb(new Vec3(-2, 0.25, 3), new Vec3(-1, 0.5, 4));

            assertEquals(UNIT_BLOCK, UNIT_BLOCK.union(UNIT_BLOCK));
            assertEquals(UNIT_BLOCK.union(other), other.union(UNIT_BLOCK));
        }

        @Test
        void absorbsAContainedBoxWithoutGrowing() {
            Aabb inner = Aabb.centredAt(new Vec3(0.5, 0.5, 0.5), new Vec3(0.1, 0.1, 0.1));

            assertEquals(UNIT_BLOCK, UNIT_BLOCK.union(inner));
        }

        @Test
        void rejectsNull() {
            assertThrows(IllegalArgumentException.class, () -> UNIT_BLOCK.union(null));
        }
    }

    @Nested
    class TranslatedBy {

        @Test
        void movesBothCornersAndLeavesTheSizeAlone() {
            Aabb moved = UNIT_BLOCK.translatedBy(new Vec3(10, -3, 0.5));

            assertEquals(new Vec3(10, -3, 0.5), moved.min());
            assertEquals(UNIT_BLOCK.size(), moved.size());
        }

        @Test
        void rejectsANonFiniteOffset() {
            assertThrows(IllegalArgumentException.class,
                    () -> UNIT_BLOCK.translatedBy(new Vec3(Double.NaN, 0, 0)));
        }
    }
}
