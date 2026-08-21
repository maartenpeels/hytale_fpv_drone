package com.maartenpeels.fpv.control;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

class PilotInputMapperTest {

    private static final double TOLERANCE = 1e-5;

    /** 30 TPS, the Hytale default. */
    private static final double DT = 1.0 / 30.0;

    private final PilotInputMapper mapper = new PilotInputMapper(PilotInputMapping.DEFAULT);

    /** Hytale's world-space wish vector for a pilot at {@code yaw} pushing the stick fully forward. */
    private static PilotInputSample forwardStickAt(double yaw) {
        return PilotInputSample.lookRelative(-Math.sin(yaw), -Math.cos(yaw), yaw, 0.0);
    }

    /** …and fully right, {@code +X} being the aircraft's right at yaw zero. */
    private static PilotInputSample rightStickAt(double yaw) {
        return PilotInputSample.lookRelative(Math.cos(yaw), -Math.sin(yaw), yaw, 0.0);
    }

    private ControlInput fly(LookTrack track, PilotInputSample sample) {
        return this.mapper.map(track, sample, DT).input();
    }

    @Nested
    class ThrottleFromForwardAxis {

        @Test
        void restsAtMidStickWithTheStickCentredBecauseTheStickIsSpringCentred() {
            assertEquals(0.5f, fly(LookTrack.UNSET, PilotInputSample.EMPTY).throttle(), TOLERANCE);
        }

        @Test
        void reachesFullAtFullForwardAndClosedAtFullBack() {
            assertEquals(1f, fly(LookTrack.UNSET, forwardStickAt(0.0)).throttle(), TOLERANCE);

            PilotInputSample back = PilotInputSample.lookRelative(0.0, 1.0, 0.0, 0.0);
            assertEquals(0f, fly(LookTrack.UNSET, back).throttle(), TOLERANCE);
        }

        @Test
        void mapsHalfStickToThreeQuartersBecauseTheRemapIsBipolarNotClipped() {
            PilotInputSample halfForward = PilotInputSample.lookRelative(0.0, -0.5, 0.0, 0.0);

            assertEquals(0.75f, fly(LookTrack.UNSET, halfForward).throttle(), TOLERANCE);
        }

        @Test
        void keepsDescendingTravelInsteadOfClippingItAwayAtMidStick() {
            PilotInputSample halfBack = PilotInputSample.lookRelative(0.0, 0.5, 0.0, 0.0);

            assertEquals(0.25f, fly(LookTrack.UNSET, halfBack).throttle(), TOLERANCE);
        }

        @Test
        void scalesByTheConfiguredFullScaleSoAnUnnormalisedWishVectorCanBeCorrected() {
            PilotInputMapper scaled =
                    new PilotInputMapper(new PilotInputMapping(4.0, 2.0 * Math.PI, 2.0 * Math.PI));
            PilotInputSample fullForwardAtScaleFour =
                    PilotInputSample.lookRelative(0.0, -4.0, 0.0, 0.0);

            assertEquals(
                    1f,
                    scaled.map(LookTrack.UNSET, fullForwardAtScaleFour, DT).input().throttle(),
                    TOLERANCE);
            assertEquals(
                    0.75f,
                    scaled.map(LookTrack.UNSET, PilotInputSample.lookRelative(0.0, -2.0, 0.0, 0.0), DT)
                            .input()
                            .throttle(),
                    TOLERANCE);
        }
    }

    @Nested
    class YawFromLateralAxis {

        @Test
        void mapsStrafeRightToYawRightWithoutNegatingBecauseTheLateralAxisIsSpatialNotAnAngle() {
            assertEquals(1f, fly(LookTrack.UNSET, rightStickAt(0.0)).yaw(), TOLERANCE);
        }

        @Test
        void mapsStrafeLeftToYawLeft() {
            PilotInputSample left = PilotInputSample.lookRelative(-1.0, 0.0, 0.0, 0.0);

            assertEquals(-1f, fly(LookTrack.UNSET, left).yaw(), TOLERANCE);
        }

        @Test
        void leavesThrottleAtMidStickWhenOnlyTheLateralAxisMoves() {
            assertEquals(0.5f, fly(LookTrack.UNSET, rightStickAt(0.0)).throttle(), TOLERANCE);
        }
    }

    @Nested
    class WishFrameRotation {

        @ParameterizedTest
        @ValueSource(doubles = {0.0, 0.7, 1.5707963, 3.1415926, -1.5707963, -2.4, 6.0})
        void givesTheSameSticksAtEveryHeadingBecauseWishMovementArrivesInWorldSpace(double yaw) {
            ControlInput forward = fly(LookTrack.UNSET, forwardStickAt(yaw));
            assertEquals(1f, forward.throttle(), TOLERANCE);
            assertEquals(0f, forward.yaw(), TOLERANCE);

            ControlInput right = fly(LookTrack.UNSET, rightStickAt(yaw));
            assertEquals(0.5f, right.throttle(), TOLERANCE);
            assertEquals(1f, right.yaw(), TOLERANCE);
        }

        @Test
        void treatsAPinnedFrameAsAFixedZeroYawSoACustomMovementRotationStillWorks() {
            PilotInputSample pinned = new PilotInputSample(0.0, -1.0, 0.0, 2.5, 0.3);

            ControlInput input = fly(LookTrack.UNSET, pinned);

            assertEquals(1f, input.throttle(), TOLERANCE);
            assertEquals(0f, input.yaw(), TOLERANCE);
        }

        @Test
        void fallsBackToTheTrackedYawWhenThePacketCarriedNoOrientationAtAll() {
            double heading = Math.PI / 2.0;
            PilotInputSample wishOnly =
                    new PilotInputSample(
                            Math.cos(heading), -Math.sin(heading),
                            Double.NaN, Double.NaN, Double.NaN);

            ControlInput input = fly(LookTrack.at(heading, 0.0), wishOnly);

            assertEquals(1f, input.yaw(), TOLERANCE);
            assertEquals(0.5f, input.throttle(), TOLERANCE);
        }

        @ParameterizedTest
        @ValueSource(doubles = {0.0, 1.5707963, 3.1415926, -1.5707963})
        void centresTheWishAxesWhenNoSourceNamesTheFrameRatherThanGuessingNorth(double heading) {
            PilotInputSample uninterpretable =
                    new PilotInputSample(
                            -Math.sin(heading), -Math.cos(heading),
                            Double.NaN, Double.NaN, Double.NaN);

            ControlInput input = fly(LookTrack.UNSET, uninterpretable);

            assertEquals(0.5f, input.throttle(), TOLERANCE);
            assertEquals(0f, input.yaw(), TOLERANCE);
        }
    }

    @Nested
    class PitchFromLookPitchDelta {

        @Test
        void pitchesNoseDownWhenTheViewLooksDownBecauseHytalePitchIsPositiveNoseUp() {
            ControlInput input =
                    fly(LookTrack.at(0.0, 0.0), PilotInputSample.lookRelative(0.0, 0.0, 0.0, -0.1));

            assertTrue(input.pitch() > 0f, "looking down must demand nose-down pitch");
            assertEquals((float) ((0.1 / DT) / (2.0 * Math.PI)), input.pitch(), TOLERANCE);
        }

        @Test
        void pitchesNoseUpWhenTheViewLooksUp() {
            ControlInput input =
                    fly(LookTrack.at(0.0, 0.0), PilotInputSample.lookRelative(0.0, 0.0, 0.0, 0.1));

            assertTrue(input.pitch() < 0f);
        }

        @Test
        void reachesFullStickAtTheConfiguredFullScaleLookRate() {
            double fullScale = PilotInputMapping.DEFAULT.pitchLookRateFullScale();
            PilotInputSample oneFullScaleTickDown =
                    PilotInputSample.lookRelative(0.0, 0.0, 0.0, -fullScale * DT);

            assertEquals(1f, fly(LookTrack.at(0.0, 0.0), oneFullScaleTickDown).pitch(), TOLERANCE);
        }

        @Test
        void doesNotWrapPitchBecausePitchStopsRatherThanWrapping() {
            PilotInputSample absurd = PilotInputSample.lookRelative(0.0, 0.0, 0.0, -100.0);

            assertEquals(1f, fly(LookTrack.at(0.0, 0.0), absurd).pitch(), TOLERANCE);
        }
    }

    @Nested
    class RollFromLookYawDelta {

        @Test
        void banksRightWhenTheViewTurnsRightBecauseIncreasingHytaleYawTurnsTheNoseLeft() {
            ControlInput input =
                    fly(LookTrack.at(0.0, 0.0), PilotInputSample.lookRelative(0.0, 0.0, -0.1, 0.0));

            assertTrue(input.roll() > 0f, "turning the view right must demand a right bank");
            assertEquals((float) ((0.1 / DT) / (2.0 * Math.PI)), input.roll(), TOLERANCE);
        }

        @Test
        void banksLeftWhenTheViewTurnsLeft() {
            ControlInput input =
                    fly(LookTrack.at(0.0, 0.0), PilotInputSample.lookRelative(0.0, 0.0, 0.1, 0.0));

            assertTrue(input.roll() < 0f);
        }

        @Test
        void takesTheShortWayRoundWhenTheViewCrossesTheYawWrap() {
            double justUnderPi = Math.PI - 0.05;
            PilotInputSample acrossTheWrap =
                    PilotInputSample.lookRelative(0.0, 0.0, -justUnderPi, 0.0);

            ControlInput wrapped = fly(LookTrack.at(justUnderPi, 0.0), acrossTheWrap);
            ControlInput equivalentSmallTurn =
                    fly(LookTrack.at(0.0, 0.0), PilotInputSample.lookRelative(0.0, 0.0, 0.1, 0.0));

            assertEquals(equivalentSmallTurn.roll(), wrapped.roll(), TOLERANCE);
        }

        @Test
        void banksLeftForAnOddMultipleOfPiLeftwardsRatherThanInvertingAtTheWrapBoundary() {
            ControlInput threeHalfTurnsLeft =
                    fly(LookTrack.at(0.0, 0.0), PilotInputSample.lookRelative(0.0, 0.0, 3 * Math.PI, 0.0));

            assertTrue(
                    threeHalfTurnsLeft.roll() < 0f,
                    "a 540 degree leftward flick must not come out banking right");
        }

        @Test
        void saturatesRatherThanExplodingOnAViolentFlick() {
            PilotInputSample flick = PilotInputSample.lookRelative(0.0, 0.0, -3.0, 0.0);

            assertEquals(1f, fly(LookTrack.at(0.0, 0.0), flick).roll(), TOLERANCE);
        }
    }

    @Nested
    class TickRateIndependence {

        @Test
        void givesTheSameDeflectionForTheSameLookRateAtAnyTickRate() {
            PilotInputUpdate atThirty =
                    mapper.map(
                            LookTrack.at(0.0, 0.0),
                            PilotInputSample.lookRelative(0.0, 0.0, -0.2, -0.2),
                            1.0 / 30.0);
            PilotInputUpdate atTwoForty =
                    mapper.map(
                            LookTrack.at(0.0, 0.0),
                            PilotInputSample.lookRelative(0.0, 0.0, -0.025, -0.025),
                            1.0 / 240.0);

            assertEquals(atThirty.input().roll(), atTwoForty.input().roll(), TOLERANCE);
            assertEquals(atThirty.input().pitch(), atTwoForty.input().pitch(), TOLERANCE);
        }
    }

    @Nested
    class LookTracking {

        @Test
        void centresPitchAndRollOnTheFirstSampleBecauseArmingMustNotFireAFlick() {
            ControlInput input =
                    fly(LookTrack.UNSET, PilotInputSample.lookRelative(0.0, 0.0, 2.5, -0.9));

            assertEquals(0f, input.roll());
            assertEquals(0f, input.pitch());
        }

        @Test
        void carriesTheSampledLookForwardAsTheNextDeltasReference() {
            PilotInputUpdate update =
                    mapper.map(
                            LookTrack.UNSET, PilotInputSample.lookRelative(0.0, 0.0, 2.5, -0.9), DT);

            assertEquals(LookTrack.at(2.5, -0.9), update.track());
        }

        @Test
        void keepsTheTrackedAnglesWhenAPacketCarriesNoLookBecauseAbsentMeansUnchanged() {
            LookTrack track = LookTrack.at(1.25, 0.5);
            PilotInputSample wishOnly =
                    new PilotInputSample(
                            -Math.sin(1.25), -Math.cos(1.25), 1.25, Double.NaN, Double.NaN);

            PilotInputUpdate update = mapper.map(track, wishOnly, DT);

            assertEquals(1.25, update.track().yaw());
            assertEquals(0.5, update.track().pitch());
            assertEquals(1f, update.input().throttle(), TOLERANCE);
            assertEquals(0f, update.input().roll(), TOLERANCE);
            assertEquals(0f, update.input().pitch(), TOLERANCE);
        }

        @Test
        void agesTheTrackOnALookLessTickSoTheNextDeltaKnowsHowLongItSpanned() {
            LookTrack track = LookTrack.at(1.0, 0.0);
            PilotInputSample noLook = new PilotInputSample(0.0, 0.0, 1.0, Double.NaN, Double.NaN);

            assertEquals(DT, mapper.map(track, noLook, DT).track().secondsSinceSample(), 1e-12);
        }

        @Test
        void measuresTheNextDeltaOverTheIntervalItActuallySpannedNotOneTick() {
            LookTrack track = LookTrack.at(1.0, 0.0);
            PilotInputSample noLook = new PilotInputSample(0.0, 0.0, 1.0, Double.NaN, Double.NaN);

            LookTrack afterGap = mapper.map(track, noLook, DT).track();
            ControlInput resumed =
                    mapper.map(afterGap, PilotInputSample.lookRelative(0.0, 0.0, 0.9, 0.0), DT)
                            .input();

            assertEquals((float) ((0.1 / (2 * DT)) / (2.0 * Math.PI)), resumed.roll(), TOLERANCE);
        }

        @Test
        void reportsTheSameRateWhetherLookSamplesArriveEveryTickOrEveryFourth() {
            double turnRate = Math.PI;

            ControlInput everyTick =
                    fly(
                            LookTrack.at(0.0, 0.0),
                            PilotInputSample.lookRelative(0.0, 0.0, -turnRate * DT, 0.0));

            LookTrack sparse = LookTrack.at(0.0, 0.0).aged(3 * DT);
            ControlInput everyFourth =
                    fly(
                            sparse,
                            PilotInputSample.lookRelative(0.0, 0.0, -turnRate * 4 * DT, 0.0));

            assertEquals(0.5f, everyTick.roll(), TOLERANCE);
            assertEquals(everyTick.roll(), everyFourth.roll(), TOLERANCE);
        }
    }

    @Nested
    class UntrustedInput {

        @Test
        void restsThrottleAtMidStickOnANonFiniteWishVectorRatherThanCuttingTheMotors() {
            PilotInputSample garbage =
                    new PilotInputSample(Double.NaN, Double.POSITIVE_INFINITY, 0.0, 0.0, 0.0);

            ControlInput input = fly(LookTrack.at(0.0, 0.0), garbage);

            assertEquals(0.5f, input.throttle(), TOLERANCE);
            assertEquals(0f, input.yaw(), TOLERANCE);
        }

        @Test
        void saturatesAnOutOfRangeWishVectorInsteadOfThrowing() {
            PilotInputSample overdriven = PilotInputSample.lookRelative(50.0, -50.0, 0.0, 0.0);

            ControlInput input = fly(LookTrack.UNSET, overdriven);

            assertEquals(1f, input.throttle(), TOLERANCE);
            assertEquals(1f, input.yaw(), TOLERANCE);
        }

        @Test
        void centresPitchAndRollOnANonFiniteLookAngle() {
            PilotInputSample garbage =
                    new PilotInputSample(0.0, 0.0, 0.0, Double.NaN, Double.NEGATIVE_INFINITY);

            ControlInput input = fly(LookTrack.at(0.0, 0.0), garbage);

            assertEquals(0f, input.roll());
            assertEquals(0f, input.pitch());
        }

        @Test
        void rejectsANonPositiveOrNonFiniteDtBecauseThatIsOurBugNotTheClients() {
            PilotInputSample sample = PilotInputSample.EMPTY;

            assertThrows(
                    IllegalArgumentException.class, () -> mapper.map(LookTrack.UNSET, sample, 0.0));
            assertThrows(
                    IllegalArgumentException.class, () -> mapper.map(LookTrack.UNSET, sample, -DT));
            assertThrows(
                    IllegalArgumentException.class,
                    () -> mapper.map(LookTrack.UNSET, sample, Double.NaN));
        }

        @Test
        void neverEmitsANegativeZeroBecauseItWouldCompareUnequalToACentredInput() {
            PilotInputSample centredAtSouth =
                    new PilotInputSample(0.0, 0.0, Math.PI, Math.PI, 0.0);

            ControlInput input = fly(LookTrack.at(Math.PI, 0.0), centredAtSouth);

            assertEquals(new ControlInput(0.5f, 0f, 0f, 0f), input);
        }

        @Test
        void rejectsMissingArguments() {
            assertThrows(
                    IllegalArgumentException.class,
                    () -> mapper.map(null, PilotInputSample.EMPTY, DT));
            assertThrows(
                    IllegalArgumentException.class, () -> mapper.map(LookTrack.UNSET, null, DT));
            assertThrows(IllegalArgumentException.class, () -> new PilotInputMapper(null));
        }
    }

    @Nested
    class Centred {

        @Test
        void restsThrottleAtMidStickSoASilentClientHoversRatherThanFalling() {
            PilotInputUpdate update = mapper.centred(LookTrack.at(1.0, 0.5), DT);

            assertEquals(0.5f, update.input().throttle());
            assertTrue(update.input().sticksCentred());
        }

        @Test
        void preservesTheTrackedAnglesSoTheNextPacketDoesNotFlick() {
            PilotInputUpdate update = mapper.centred(LookTrack.at(1.0, 0.5), DT);

            assertEquals(1.0, update.track().yaw());
            assertEquals(0.5, update.track().pitch());
        }

        @Test
        void agesTheTrackSoASkippedTickIsNotChargedToTheNextDelta() {
            assertEquals(
                    DT,
                    mapper.centred(LookTrack.at(1.0, 0.5), DT).track().secondsSinceSample(),
                    1e-12);
        }

        @Test
        void rejectsAMissingTrackOrAnUnusableDt() {
            assertThrows(IllegalArgumentException.class, () -> mapper.centred(null, DT));
            assertThrows(
                    IllegalArgumentException.class,
                    () -> mapper.centred(LookTrack.UNSET, 0.0));
        }
    }

    @Nested
    class ModeTwoLayout {

        @Test
        void putsThrottleAndYawOnOneStickAndPitchAndRollOnTheOther() {
            ControlInput leftStickOnly =
                    fly(LookTrack.at(0.0, 0.0), PilotInputSample.lookRelative(1.0, -1.0, 0.0, 0.0));

            assertEquals(1f, leftStickOnly.throttle(), TOLERANCE);
            assertEquals(1f, leftStickOnly.yaw(), TOLERANCE);
            assertEquals(0f, leftStickOnly.roll(), TOLERANCE);
            assertEquals(0f, leftStickOnly.pitch(), TOLERANCE);

            ControlInput rightStickOnly =
                    fly(LookTrack.at(0.0, 0.0), PilotInputSample.lookRelative(0.0, 0.0, -0.1, -0.1));

            assertEquals(0.5f, rightStickOnly.throttle(), TOLERANCE);
            assertEquals(0f, rightStickOnly.yaw(), TOLERANCE);
            assertFalse(rightStickOnly.sticksCentred());
            assertTrue(rightStickOnly.roll() > 0f);
            assertTrue(rightStickOnly.pitch() > 0f);
        }
    }
}
