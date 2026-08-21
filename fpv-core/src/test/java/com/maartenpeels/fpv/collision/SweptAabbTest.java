package com.maartenpeels.fpv.collision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.maartenpeels.fpv.math.Aabb;
import com.maartenpeels.fpv.math.Quat;
import com.maartenpeels.fpv.math.Vec3;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class SweptAabbTest {

    /** One block at the origin, the thing a drone actually flies into. */
    private static final Aabb BLOCK = Aabb.cubeAt(Vec3.ZERO, 1.0);

    private static final Vec3 BLOCK_CENTRE = new Vec3(0.5, 0.5, 0.5);

    /** A cube drone, so every face hit produces the same numbers and a sign error stands out. */
    private static final Vec3 DRONE = new Vec3(0.25, 0.25, 0.25);

    /**
     * The six ways to fly at a box, each paired with the face normal it must report. Written out
     * rather than derived from the direction, because {@code Vec3.negated()} yields {@code −0.0}
     * components that compare unequal to {@code 0.0} — deriving it would test the derivation.
     */
    private record Approach(Vec3 direction, Vec3 expectedNormal) {}

    private static final Approach[] APPROACHES = {
        new Approach(new Vec3(1, 0, 0), new Vec3(-1, 0, 0)),
        new Approach(new Vec3(-1, 0, 0), new Vec3(1, 0, 0)),
        new Approach(new Vec3(0, 1, 0), new Vec3(0, -1, 0)),
        new Approach(new Vec3(0, -1, 0), new Vec3(0, 1, 0)),
        new Approach(new Vec3(0, 0, 1), new Vec3(0, 0, -1)),
        new Approach(new Vec3(0, 0, -1), new Vec3(0, 0, 1)),
    };

    private static SweptResult sweep(Vec3 from, Vec3 displacement, Aabb target) {
        return SweptAabb.sweep(Aabb.centredAt(from, DRONE), displacement, target);
    }

    private static SweptResult.Contact contact(Vec3 from, Vec3 displacement, Aabb target) {
        return assertInstanceOf(SweptResult.Contact.class, sweep(from, displacement, target));
    }

    @Nested
    class CleanMiss {

        @Test
        void missesABoxNowhereNearThePath() {
            assertSame(SweptResult.MISS, sweep(new Vec3(10, 10, 10), new Vec3(1, 1, 1), BLOCK));
        }

        @Test
        void missesABoxBehindTheStartBecauseTheSweepOnlyLooksForward() {
            assertSame(SweptResult.MISS, sweep(new Vec3(5, 0.5, 0.5), new Vec3(5, 0, 0), BLOCK));
        }

        @Test
        void missesABoxBeyondTheEndBecauseTheSweepIsASegmentAndNotARay() {
            // Dead on line, and hit at t = 4.75 if the segment were extended. It is not.
            assertSame(SweptResult.MISS, sweep(new Vec3(-5, 0.5, 0.5), new Vec3(1, 0, 0), BLOCK));
        }

        @Test
        void missesWhenMotionIsParallelToTheBoxAndOutsideItSideways() {
            assertSame(SweptResult.MISS, sweep(new Vec3(0.5, 5, 0.5), new Vec3(1, 0, 0), BLOCK));
        }

        @Test
        void missesWithZeroDisplacementWhenNotAlreadyOverlapping() {
            assertSame(SweptResult.MISS, sweep(new Vec3(5, 5, 5), Vec3.ZERO, BLOCK));
        }
    }

    @Nested
    class FaceHit {

        @Test
        void reportsTheOutwardNormalOfEachOfTheSixFacesItCanBeEnteredThrough() {
            for (Approach approach : APPROACHES) {
                Vec3 from = BLOCK_CENTRE.plus(approach.direction().scale(-2.75));

                SweptResult.Contact hit = contact(from, approach.direction().scale(4), BLOCK);

                assertEquals(approach.expectedNormal(), hit.normal(),
                        "approaching along " + approach.direction());
            }
        }

        @Test
        void solvesAnExactTimeOfImpactRatherThanASampledOne() {
            for (Approach approach : APPROACHES) {
                Vec3 from = BLOCK_CENTRE.plus(approach.direction().scale(-2.75));

                SweptResult.Contact hit = contact(from, approach.direction().scale(4), BLOCK);

                // 2.0 / 4.0 and 3.5 / 4.0 -- asserted without a delta, on purpose.
                assertEquals(0.5, hit.entryTime(), "approaching along " + approach.direction());
                assertEquals(0.875, hit.exitTime(), "approaching along " + approach.direction());
            }
        }

        @Test
        void putsTheMoverFlushAgainstTheFaceAtTheReportedTime() {
            Vec3 from = new Vec3(-2.25, 0.5, 0.5);
            Vec3 displacement = new Vec3(4, 0, 0);

            SweptResult.Contact hit = contact(from, displacement, BLOCK);

            Vec3 atImpact = hit.positionAt(from, displacement);
            assertEquals(BLOCK.min().x() - DRONE.x(), atImpact.x());
            assertFalse(Aabb.centredAt(atImpact, DRONE).overlaps(BLOCK));
        }

        @Test
        void alwaysReportsANormalThatOpposesTheMotionSoResponseCanUseItDirectly() {
            for (double dx = -3; dx <= 3; dx += 1.5) {
                for (double dy = -3; dy <= 3; dy += 1.5) {
                    for (double dz = -3; dz <= 3; dz += 1.5) {
                        Vec3 displacement = new Vec3(dx, dy, dz);
                        if (displacement.equals(Vec3.ZERO)) {
                            continue;
                        }
                        Vec3 from = BLOCK_CENTRE.minus(displacement.normalised().scale(3));

                        if (sweep(from, displacement, BLOCK) instanceof SweptResult.Contact hit) {
                            assertTrue(hit.normal().dot(displacement) < 0.0,
                                    "normal " + hit.normal() + " should oppose " + displacement);
                            assertEquals(1.0, hit.normal().length(), 1e-12);
                        }
                    }
                }
            }
        }

        @Test
        void resolvesAnExactCornerApproachInAxisOrderRatherThanArbitrarily() {
            // At t = 0.4 the mover reaches the −X and +Y faces of the expanded block simultaneously.
            SweptResult.Contact hit =
                    contact(new Vec3(-1.25, 2.25, 0.5), new Vec3(2.5, -2.5, 0), BLOCK);

            assertEquals(0.4, hit.entryTime(), 1e-15);
            assertEquals(new Vec3(-1, 0, 0), hit.normal());
        }
    }

    @Nested
    class Tunnelling {

        /** A thin wall, the shape that defeats a containment test. */
        private static final Aabb WALL =
                new Aabb(new Vec3(-0.5, -50, -50), new Vec3(0.5, 50, 50));

        @Test
        void findsAWallThatANaiveContainmentTestWouldTunnelStraightThrough() {
            Vec3 from = new Vec3(-100, 0, 0);
            Vec3 displacement = new Vec3(200, 0, 0);

            // The test the routine exists to replace: sample the ends and hope.
            assertFalse(Aabb.centredAt(from, DRONE).overlaps(WALL), "start already overlaps");
            assertFalse(Aabb.centredAt(from.plus(displacement), DRONE).overlaps(WALL),
                    "end already overlaps");

            SweptResult.Contact hit = contact(from, displacement, WALL);

            assertEquals(99.25 / 200.0, hit.entryTime());
            assertEquals(new Vec3(-1, 0, 0), hit.normal());
            assertTrue(hit.passedFullyThrough());
        }

        @Test
        void staysExactAsTheStepGrowsByOrdersOfMagnitudeRelativeToTheWall() {
            for (double halfStep = 10; halfStep <= 1e9; halfStep *= 10) {
                Vec3 from = new Vec3(-halfStep, 0, 0);
                Vec3 displacement = new Vec3(2 * halfStep, 0, 0);

                assertFalse(Aabb.centredAt(from, DRONE).overlaps(WALL), "step " + halfStep);
                assertFalse(Aabb.centredAt(from.plus(displacement), DRONE).overlaps(WALL),
                        "step " + halfStep);

                SweptResult.Contact hit = contact(from, displacement, WALL);

                assertEquals((halfStep - 0.75) / (2 * halfStep), hit.entryTime(),
                        "step " + halfStep);
                assertEquals(new Vec3(-1, 0, 0), hit.normal(), "step " + halfStep);
            }
        }

        @Test
        void findsAWallThinnerThanTheDroneItself() {
            Aabb paper = new Aabb(new Vec3(0, -50, -50), new Vec3(1e-6, 50, 50));

            SweptResult.Contact hit = contact(new Vec3(-500, 0, 0), new Vec3(1000, 0, 0), paper);

            assertEquals(new Vec3(-1, 0, 0), hit.normal());
            assertTrue(hit.entryTime() > 0.0 && hit.entryTime() < 1.0);
        }
    }

    @Nested
    class Glancing {

        @Test
        void slidingExactlyAlongAFaceIsAMissSoARestingDroneDoesNotCrashEveryTick() {
            Vec3 restingOnTop = new Vec3(-2, BLOCK.max().y() + DRONE.y(), 0.5);

            assertSame(SweptResult.MISS, sweep(restingOnTop, new Vec3(4, 0, 0), BLOCK));
        }

        @Test
        void slidingAHairBelowThatFaceIsAContactBecauseTheOverlapNowHasVolume() {
            Vec3 justInside = new Vec3(-2, BLOCK.max().y() + DRONE.y() - 1e-9, 0.5);

            SweptResult.Contact hit = contact(justInside, new Vec3(4, 0, 0), BLOCK);

            assertEquals(new Vec3(-1, 0, 0), hit.normal());
        }

        @Test
        void descendingOntoAFaceIsAContactBecauseThatIsWhatLandingIs() {
            Vec3 restingOnTop = new Vec3(0.5, BLOCK.max().y() + DRONE.y(), 0.5);

            SweptResult.Contact hit = contact(restingOnTop, new Vec3(0, -1e-9, 0), BLOCK);

            assertEquals(new Vec3(0, 1, 0), hit.normal());
            assertEquals(0.0, hit.entryTime());
        }

        @Test
        void clippingExactlyOneCornerIsAMissBecauseZeroDurationIsNotACollision() {
            // At t = 0.4 the mover touches the expanded block's (−X, +Y) corner and nothing else.
            assertSame(SweptResult.MISS,
                    sweep(new Vec3(-1.25, 0.25, 0.5), new Vec3(2.5, 2.5, 0), BLOCK));
        }

        @Test
        void dippingJustPastThatCornerIsAContact() {
            SweptResult.Contact hit =
                    contact(new Vec3(-1.25, 0.25, 0.5), new Vec3(2.5, 2.4, 0), BLOCK);

            assertEquals(0.4, hit.entryTime(), 1e-15);
            assertEquals(new Vec3(-1, 0, 0), hit.normal());
        }

        @Test
        void arrivingExactlyTangentAtTheEndOfTheSegmentDefersToTheNextSegment() {
            Vec3 from = new Vec3(-1.25, 0.5, 0.5);
            Vec3 step = new Vec3(1, 0, 0);

            assertSame(SweptResult.MISS, sweep(from, step, BLOCK));

            // Nothing is skipped: the very next segment starts tangent and reports contact at once.
            SweptResult.Contact next = contact(from.plus(step), step, BLOCK);
            assertEquals(0.0, next.entryTime());
            assertEquals(new Vec3(-1, 0, 0), next.normal());
        }

        @Test
        void beingTangentAtTheStartAndMovingAwayIsAMiss() {
            assertSame(SweptResult.MISS, sweep(new Vec3(-0.25, 0.5, 0.5), new Vec3(-1, 0, 0), BLOCK));
        }
    }

    @Nested
    class AlreadyOverlapping {

        @Test
        void reportsOverlapRatherThanInventingAFaceTheMoverNeverCrossed() {
            assertSame(SweptResult.ALREADY_OVERLAPPING,
                    sweep(BLOCK_CENTRE, new Vec3(1, 0, 0), BLOCK));
        }

        @Test
        void reportsOverlapWithZeroDisplacementBecauseTheTestDegeneratesCleanly() {
            assertSame(SweptResult.ALREADY_OVERLAPPING, sweep(BLOCK_CENTRE, Vec3.ZERO, BLOCK));
        }

        @Test
        void reportsOverlapEvenWhenTheMoverIsOnItsWayOut() {
            assertSame(SweptResult.ALREADY_OVERLAPPING,
                    sweep(BLOCK_CENTRE, new Vec3(100, 0, 0), BLOCK));
        }

        @Test
        void isDistinctFromStartingExactlyTangentAndMovingIn() {
            SweptResult tangent = sweep(new Vec3(-0.25, 0.5, 0.5), new Vec3(1, 0, 0), BLOCK);

            SweptResult.Contact hit = assertInstanceOf(SweptResult.Contact.class, tangent);
            assertEquals(0.0, hit.entryTime());
        }

        @Test
        void agreesWithAStaticOverlapTestAtTheStartOfTheSegment() {
            for (double x = -1.5; x <= 2.5; x += 0.05) {
                Vec3 from = new Vec3(x, 0.5, 0.5);
                boolean overlapsNow = Aabb.centredAt(from, DRONE).overlaps(BLOCK);

                SweptResult result = sweep(from, new Vec3(0.01, 0, 0), BLOCK);

                assertEquals(overlapsNow, result == SweptResult.ALREADY_OVERLAPPING, "at x " + x);
            }
        }
    }

    @Nested
    class GateCrossing {

        /** A gate: wide, tall and thin, facing along its own −Z, matching the drone's forward. */
        private static final Aabb GATE = new Aabb(new Vec3(-2, -2, -0.1), new Vec3(2, 2, 0.1));

        private static final Vec3 FORWARD = new Vec3(0, 0, -1);

        @Test
        void acceptsAPassInTheGatesOwnDirection() {
            SweptResult.Contact hit = contact(new Vec3(0, 0, 5), new Vec3(0, 0, -10), GATE);

            assertTrue(hit.enteredAlong(FORWARD));
            assertTrue(hit.passedFullyThrough());
        }

        @Test
        void refusesAPassInTheWrongDirection() {
            SweptResult.Contact hit = contact(new Vec3(0, 0, -5), new Vec3(0, 0, 10), GATE);

            assertFalse(hit.enteredAlong(FORWARD));
        }

        @Test
        void refusesASidewaysClipBecauseNeitherDirectionWasTravelled() {
            SweptResult.Contact hit = contact(new Vec3(-5, 0, 0), new Vec3(10, 0, 0), GATE);

            assertEquals(new Vec3(-1, 0, 0), hit.normal());
            assertFalse(hit.enteredAlong(FORWARD));
            assertFalse(hit.enteredAlong(FORWARD.negated()));
        }

        @Test
        void acceptsAPassThroughARotatedGateWorkedInTheGatesOwnFrame() {
            Quat rotation = Quat.fromAxisAngle(Vec3.UP, Math.toRadians(37));
            Vec3 worldForward = rotation.rotate(FORWARD);

            SweptResult.Contact hit = sweepInGateFrame(rotation, worldForward.scale(-5),
                    worldForward.scale(10));

            Vec3 worldNormal = rotation.rotate(hit.normal());
            assertTrue(new SweptResult.Contact(hit.entryTime(), hit.exitTime(), worldNormal)
                    .enteredAlong(worldForward));
        }

        @Test
        void refusesTheWrongWayThroughARotatedGateToo() {
            Quat rotation = Quat.fromAxisAngle(Vec3.UP, Math.toRadians(37));
            Vec3 worldForward = rotation.rotate(FORWARD);

            SweptResult.Contact hit = sweepInGateFrame(rotation, worldForward.scale(5),
                    worldForward.scale(-10));

            Vec3 worldNormal = rotation.rotate(hit.normal());
            assertFalse(new SweptResult.Contact(hit.entryTime(), hit.exitTime(), worldNormal)
                    .enteredAlong(worldForward));
        }

        @Test
        void findsAGatePassAtSpeedsThatWouldTunnelThroughAThinGate() {
            for (double speed = 10; speed <= 1e6; speed *= 10) {
                Vec3 from = new Vec3(0, 0, speed / 2);

                SweptResult.Contact hit = contact(from, new Vec3(0, 0, -speed), GATE);

                assertTrue(hit.enteredAlong(FORWARD), "speed " + speed);
                assertTrue(hit.passedFullyThrough(), "speed " + speed);
            }
        }

        private static SweptResult.Contact sweepInGateFrame(
                Quat rotation, Vec3 worldFrom, Vec3 worldDisplacement) {
            Vec3 localFrom = rotation.inverseRotate(worldFrom);
            Vec3 localDisplacement = rotation.inverseRotate(worldDisplacement);

            return assertInstanceOf(SweptResult.Contact.class,
                    SweptAabb.sweep(Aabb.centredAt(localFrom, DRONE), localDisplacement, GATE));
        }
    }

    @Nested
    class ExitTime {

        @Test
        void reportsPassedFullyThroughWhenTheSweepClearsTheBoxInsideTheSegment() {
            SweptResult.Contact hit = contact(new Vec3(-2.25, 0.5, 0.5), new Vec3(4, 0, 0), BLOCK);

            assertTrue(hit.exitTime() < 1.0);
            assertTrue(hit.passedFullyThrough());
        }

        @Test
        void goesPastOneWhenTheSegmentEndsWithTheBoxesStillOverlapping() {
            Vec3 from = new Vec3(-1.25, 0.5, 0.5);
            Vec3 displacement = new Vec3(1.5, 0, 0);

            SweptResult.Contact hit = contact(from, displacement, BLOCK);

            assertTrue(hit.exitTime() > 1.0);
            assertFalse(hit.passedFullyThrough());
            assertTrue(Aabb.centredAt(from.plus(displacement), DRONE).overlaps(BLOCK));
        }
    }

    @Nested
    class PointMover {

        @Test
        void sweepsAZeroExtentBoxAsAPlainRayCast() {
            SweptResult result = SweptAabb.sweep(Aabb.centredAt(new Vec3(-1, 0.5, 0.5), Vec3.ZERO),
                    new Vec3(3, 0, 0), BLOCK);

            SweptResult.Contact hit = assertInstanceOf(SweptResult.Contact.class, result);
            assertEquals(1.0 / 3.0, hit.entryTime());
            assertEquals(2.0 / 3.0, hit.exitTime());
            assertEquals(new Vec3(-1, 0, 0), hit.normal());
        }

        @Test
        void missesADegenerateTargetBecauseAPointThroughAPlaneHasNoVolumeToShare() {
            Aabb plane = new Aabb(new Vec3(-2, -2, 0), new Vec3(2, 2, 0));

            assertSame(SweptResult.MISS,
                    SweptAabb.sweep(Aabb.centredAt(new Vec3(0, 0, -1), Vec3.ZERO),
                            new Vec3(0, 0, 2), plane));
        }

        @Test
        void findsADegenerateTargetOnceTheMoverHasExtentToShareWithIt() {
            Aabb plane = new Aabb(new Vec3(-2, -2, 0), new Vec3(2, 2, 0));

            SweptResult.Contact hit = contact(new Vec3(0, 0, -1), new Vec3(0, 0, 2), plane);

            assertEquals(new Vec3(0, 0, -1), hit.normal());
        }
    }

    @Nested
    class Validation {

        @Test
        void rejectsANonFiniteDisplacementRatherThanLettingNaNLookLikeAnOverlap() {
            Aabb mover = Aabb.centredAt(new Vec3(-2, 0.5, 0.5), DRONE);

            assertThrows(IllegalArgumentException.class,
                    () -> SweptAabb.sweep(mover, new Vec3(Double.NaN, 0, 0), BLOCK));
            assertThrows(IllegalArgumentException.class,
                    () -> SweptAabb.sweep(mover, new Vec3(Double.POSITIVE_INFINITY, 0, 0), BLOCK));
        }

        @Test
        void rejectsNullArguments() {
            Aabb mover = Aabb.centredAt(Vec3.ZERO, DRONE);

            assertThrows(IllegalArgumentException.class,
                    () -> SweptAabb.sweep(null, Vec3.UP, BLOCK));
            assertThrows(IllegalArgumentException.class,
                    () -> SweptAabb.sweep(mover, Vec3.UP, null));
            assertThrows(IllegalArgumentException.class,
                    () -> SweptAabb.sweep(mover, null, BLOCK));
        }
    }

    @Nested
    class Determinism {

        @Test
        void producesBitIdenticalResultsForIdenticalArguments() {
            Vec3 from = new Vec3(-3.1, 1.7, -0.4);
            Vec3 displacement = new Vec3(6.2, -2.9, 1.1);

            assertEquals(sweep(from, displacement, BLOCK), sweep(from, displacement, BLOCK));
        }
    }
}
