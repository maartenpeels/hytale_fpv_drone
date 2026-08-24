package com.maartenpeels.fpv.plugin.input;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.maartenpeels.fpv.control.ControlInput;
import com.maartenpeels.fpv.control.PilotInputMapper;
import com.maartenpeels.fpv.control.PilotInputMapping;
import com.maartenpeels.fpv.control.PilotInputSample;
import com.maartenpeels.fpv.flight.QuadParameters;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

/**
 * The interval the look delta is divided by, which is the one thing in this feature with a known
 * wrong answer.
 *
 * <p>No harness and no server: a slot is a plain object, which is the point of keeping the thread
 * handoff out of the ECS. The mapping arithmetic itself is {@code PilotInputMapperTest}'s subject in
 * {@code :fpv-core}; what is under test here is only <em>which</em> numbers this slot feeds it.
 */
class PilotInputSlotTest {

    private static final double TICK_30_TPS = 1.0 / 30.0;

    /** Where a centred throttle stick sits on the default airframe. Derived, never restated (#45). */
    private static final float HOVER = (float) QuadParameters.DEFAULT.hoverCollective();

    /** The mapper is stateless and shared, exactly as it is in production. */
    private static final PilotInputMapper MAPPER = new PilotInputMapper(
                    PilotInputMapping.DEFAULT, QuadParameters.DEFAULT.hoverCollective());

    /**
     * The look-yaw change that means full roll stick in one 30 TPS tick.
     *
     * <p>{@code PilotInputMapping.DEFAULT}'s full-scale look rate is 2π rad/s, so this is 2π/30. Every
     * delta below is expressed as a fraction of it, because a delta chosen by eye saturates
     * {@code ControlInput}'s ±1 and turns a ratio assertion into a comparison of two clamped ones —
     * which is how the first draft of the four-tick test below passed while measuring nothing.
     */
    private static final double FULL_STICK_DELTA = 2.0 * Math.PI * TICK_30_TPS;

    /** A tenth of full stick in one tick: small enough that four ticks' worth still cannot saturate. */
    private static final double GENTLE_DELTA = FULL_STICK_DELTA / 10.0;

    /** A packet holding the left stick forward, looking along the given yaw. */
    private static PilotInputSample lookingAt(double yaw) {
        return PilotInputSample.lookRelative(0.0, -1.0, yaw, 0.0);
    }

    /** Consumes one tick's worth of nothing. */
    private static ControlInput quietTick(PilotInputSlot slot) {
        return slot.nextInput(MAPPER, TICK_30_TPS);
    }

    @Nested
    class LookIntervalAccumulation {

        @Test
        void aDeltaSpanningFourTicksGivesAQuarterTheDeflectionOfTheSameDeltaInOneTick() {
            // The 4x over-report, pinned. #17's self-review found this exact bug: a look delta that
            // arrived after several silent ticks, divided by one tick's dt. It is worst where decision
            // 3's escape hatch lives -- at 240 TPS with a 60 FPS client the factor is 4, and a steady
            // half-stick input becomes a full-stick square wave through the rate PID.
            PilotInputSlot fast = new PilotInputSlot();
            fast.offer(lookingAt(0.0));
            fast.nextInput(MAPPER, TICK_30_TPS);
            fast.offer(lookingAt(GENTLE_DELTA));
            float inOneTick = fast.nextInput(MAPPER, TICK_30_TPS).roll();

            PilotInputSlot slow = new PilotInputSlot();
            slow.offer(lookingAt(0.0));
            slow.nextInput(MAPPER, TICK_30_TPS);
            quietTick(slow);
            quietTick(slow);
            quietTick(slow);
            slow.offer(lookingAt(GENTLE_DELTA));
            float inFourTicks = slow.nextInput(MAPPER, TICK_30_TPS).roll();

            assertTrue(Math.abs(inOneTick) > 1e-4, "the one-tick case must actually deflect");
            assertTrue(
                    Math.abs(inOneTick) < 0.99f,
                    "the one-tick case must not be clamped, or the ratio below is vacuous");
            assertEquals(inOneTick / 4.0f, inFourTicks, Math.abs(inOneTick) * 1e-5);
        }

        @Test
        void aQuietTickAgesTheTrackRatherThanResettingItsClock() {
            // The mechanism behind the test above. Re-feeding the held sample would answer a
            // LookTrack sampled *now*, zeroing secondsSinceSample -- so the interval would never
            // accumulate and the delta above would come out four times too large.
            PilotInputSlot slot = new PilotInputSlot();
            slot.offer(lookingAt(0.0));
            slot.nextInput(MAPPER, TICK_30_TPS);
            assertEquals(0.0, slot.track().secondsSinceSample(), 1e-12);

            quietTick(slot);
            assertEquals(TICK_30_TPS, slot.track().secondsSinceSample(), 1e-12);

            quietTick(slot);
            quietTick(slot);
            assertEquals(3.0 * TICK_30_TPS, slot.track().secondsSinceSample(), 1e-12);
        }

        @Test
        void aFreshSampleResetsTheClockSoTheNextIntervalStartsFromIt() {
            PilotInputSlot slot = new PilotInputSlot();
            slot.offer(lookingAt(0.0));
            slot.nextInput(MAPPER, TICK_30_TPS);
            quietTick(slot);
            quietTick(slot);

            slot.offer(lookingAt(GENTLE_DELTA));
            slot.nextInput(MAPPER, TICK_30_TPS);

            assertEquals(0.0, slot.track().secondsSinceSample(), 1e-12);
        }

        @Test
        void isTickRateAgnosticSoRaisingWorldTpsDoesNotMultiplySensitivity() {
            // Same look rate, different tick rate: half the delta over half the tick is the same
            // deflection. Decision 3's escape hatch depends on this.
            PilotInputSlot at30 = new PilotInputSlot();
            at30.offer(lookingAt(0.0));
            at30.nextInput(MAPPER, 1.0 / 30.0);
            at30.offer(lookingAt(GENTLE_DELTA));
            float deflection30 = at30.nextInput(MAPPER, 1.0 / 30.0).roll();

            PilotInputSlot at240 = new PilotInputSlot();
            at240.offer(lookingAt(0.0));
            at240.nextInput(MAPPER, 1.0 / 240.0);
            at240.offer(lookingAt(GENTLE_DELTA * 30.0 / 240.0));
            float deflection240 = at240.nextInput(MAPPER, 1.0 / 240.0).roll();

            assertTrue(Math.abs(deflection30) > 1e-4, "must actually deflect");
            assertTrue(Math.abs(deflection30) < 0.99f, "must not be clamped");
            assertEquals(deflection30, deflection240, Math.abs(deflection30) * 1e-4);
        }
    }

    @Nested
    class LatestInputWins {

        @Test
        void severalPacketsInOneTickCollapseToTheNewestSoAHighFrameRateClientGainsNoAuthority() {
            // Two clients, same total look movement over the same tick, different packet rates. The
            // deflection must match: a delta is measured against the last *consumed* angle, so
            // intermediate samples add resolution and nothing else.
            PilotInputSlot chatty = new PilotInputSlot();
            chatty.offer(lookingAt(0.0));
            chatty.nextInput(MAPPER, TICK_30_TPS);
            chatty.offer(lookingAt(GENTLE_DELTA / 3.0));
            chatty.offer(lookingAt(GENTLE_DELTA * 2.0 / 3.0));
            chatty.offer(lookingAt(GENTLE_DELTA));
            float chattyRoll = chatty.nextInput(MAPPER, TICK_30_TPS).roll();

            PilotInputSlot terse = new PilotInputSlot();
            terse.offer(lookingAt(0.0));
            terse.nextInput(MAPPER, TICK_30_TPS);
            terse.offer(lookingAt(GENTLE_DELTA));
            float terseRoll = terse.nextInput(MAPPER, TICK_30_TPS).roll();

            assertTrue(Math.abs(terseRoll) < 0.99f, "must not be clamped, or this compares two 1.0s");
            assertEquals(terseRoll, chattyRoll, Math.abs(terseRoll) * 1e-5);
        }

        @Test
        void consumesThePendingSampleSoTheSameOneIsNeverCountedTwice() {
            PilotInputSlot slot = new PilotInputSlot();
            slot.offer(lookingAt(0.0));
            slot.nextInput(MAPPER, TICK_30_TPS);

            assertEquals(null, slot.peekPending());
        }
    }

    @Nested
    class HoldingTheLeftStick {

        @Test
        void keepsThrottleAndYawThroughAQuietTickBecauseAPhysicalStickStaysWhereItIsPut() {
            PilotInputSlot slot = new PilotInputSlot();
            // Full forward on the wish vector at yaw 0: forward is -Z, so wishZ = -1 is full throttle.
            slot.offer(PilotInputSample.lookRelative(0.6, -0.8, 0.0, 0.0));
            ControlInput first = slot.nextInput(MAPPER, TICK_30_TPS);

            ControlInput held = quietTick(slot);

            assertEquals(first.throttle(), held.throttle(), 1e-6);
            assertEquals(first.yaw(), held.yaw(), 1e-6);
        }

        @Test
        void centresPitchAndRollThroughAQuietTickBecauseNoNewLookMeansNoDeflection() {
            PilotInputSlot slot = new PilotInputSlot();
            slot.offer(lookingAt(0.0));
            slot.nextInput(MAPPER, TICK_30_TPS);
            slot.offer(lookingAt(GENTLE_DELTA));
            ControlInput moving = slot.nextInput(MAPPER, TICK_30_TPS);
            assertTrue(Math.abs(moving.roll()) > 1e-4, "the moving tick must actually deflect");

            ControlInput quiet = quietTick(slot);

            assertEquals(0f, quiet.roll());
            assertEquals(0f, quiet.pitch());
        }

        @Test
        void restsThrottleAtHoverBeforeAnyPacketHasArrived() {
            // A drone that armed and has not yet heard from its pilot. Motors-off would be the worst
            // available answer, and mid-scale -- what this asserted before #45 -- is a 1 g climb.
            assertEquals(
                    HOVER, new PilotInputSlot().nextInput(MAPPER, TICK_30_TPS).throttle(), 1e-6);
        }
    }

    @Nested
    class StalenessCutoff {

        private static int ticksToExceedCutoff() {
            return (int) Math.ceil(PilotInputSlot.MAX_HELD_SECONDS / TICK_30_TPS) + 1;
        }

        @Test
        void centresTheSticksOnceTheHeldInputIsTooOldSoAFrozenClientDoesNotFlyAway() {
            // A frozen client is not a disconnected one -- #18's teardown never fires -- so without
            // this the drone keeps whatever throttle it last had, indefinitely.
            PilotInputSlot slot = new PilotInputSlot();
            slot.offer(PilotInputSample.lookRelative(0.0, -1.0, 0.0, 0.0));
            ControlInput commanded = slot.nextInput(MAPPER, TICK_30_TPS);
            assertTrue(commanded.throttle() > 0.9f, "the pilot really did command full throttle");

            ControlInput last = commanded;
            for (int i = 0; i < ticksToExceedCutoff(); i++) {
                last = quietTick(slot);
            }

            assertEquals(HOVER, last.throttle(), 1e-6);
            assertEquals(0f, last.yaw());
        }

        @Test
        void holdsTheSticksRightUpToTheCutoffRatherThanFadingOut() {
            PilotInputSlot slot = new PilotInputSlot();
            slot.offer(PilotInputSample.lookRelative(0.0, -1.0, 0.0, 0.0));
            ControlInput commanded = slot.nextInput(MAPPER, TICK_30_TPS);

            int wholeTicksInsideCutoff =
                    (int) Math.floor(PilotInputSlot.MAX_HELD_SECONDS / TICK_30_TPS);
            ControlInput last = commanded;
            for (int i = 0; i < wholeTicksInsideCutoff; i++) {
                last = quietTick(slot);
            }

            assertEquals(commanded.throttle(), last.throttle(), 1e-6);
        }

        @Test
        void keepsAccumulatingTheLookIntervalAcrossTheCutoffSoARecoveringClientIsNotOverReported() {
            // Past the cutoff the slot answers `centred`, which also ages the track. If it stopped
            // ageing, a client that stalled for two seconds and then moved the mouse would have that
            // delta divided by one tick.
            PilotInputSlot slot = new PilotInputSlot();
            slot.offer(lookingAt(0.0));
            slot.nextInput(MAPPER, TICK_30_TPS);

            int ticks = ticksToExceedCutoff();
            for (int i = 0; i < ticks; i++) {
                quietTick(slot);
            }

            assertEquals(ticks * TICK_30_TPS, slot.track().secondsSinceSample(), 1e-9);
        }

        @Test
        void recoversFullyWhenPacketsResume() {
            PilotInputSlot slot = new PilotInputSlot();
            slot.offer(PilotInputSample.lookRelative(0.0, -1.0, 0.0, 0.0));
            slot.nextInput(MAPPER, TICK_30_TPS);
            for (int i = 0; i < ticksToExceedCutoff(); i++) {
                quietTick(slot);
            }

            slot.offer(PilotInputSample.lookRelative(0.0, -1.0, 0.0, 0.0));
            ControlInput resumed = slot.nextInput(MAPPER, TICK_30_TPS);

            assertTrue(resumed.throttle() > 0.9f, "throttle should obey the pilot again, was " + resumed);
        }
    }
}
