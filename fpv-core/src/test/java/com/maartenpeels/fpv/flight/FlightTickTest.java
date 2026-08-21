package com.maartenpeels.fpv.flight;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.maartenpeels.fpv.control.ControlInput;
import com.maartenpeels.fpv.math.Vec3;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

/**
 * The substep schedule, which is where CLAUDE.md decision 3 either holds or does not.
 *
 * <p>The dynamics themselves are {@link QuadIntegratorTest}'s subject. What is under test here is
 * only the arithmetic of dividing a tick into substeps — and that arithmetic is the whole of the
 * ticket's done-when, so it gets pinned from several directions.
 */
class FlightTickTest {

    private static final double TICK_30_TPS = 1.0 / 30.0;

    /** Climbing hard: the axis where a substep-length error shows up fastest. */
    private static final ControlInput FULL_THROTTLE = new ControlInput(1f, 0f, 0f, 0f);

    private static FlightTick at(int substeps) {
        return new FlightTick(new QuadIntegrator(QuadParameters.DEFAULT), substeps);
    }

    /** Flies for one simulated second, whatever the substep count, and answers the height gained. */
    private static double climbInOneSecond(int substeps, int tps) {
        FlightTick tick = at(substeps);
        FlightState state = FlightState.restingAt(Vec3.ZERO);
        for (int i = 0; i < tps; i++) {
            state = tick.advance(state, FULL_THROTTLE, 1.0 / tps);
        }
        return state.drone().position().y();
    }

    /** Fractional disagreement between two climbs, which is the scale the failure mode lives on. */
    private static double relativeGap(double a, double b) {
        return Math.abs(a - b) / Math.abs(b);
    }

    @Nested
    class SimulatedTimePerTick {

        @Test
        void isTheTickLengthWhateverTheSubstepCountBecauseThatIsWhatMakesSubstepsFreeOfFlightEffect() {
            // The bug this exists to catch: N substeps of a *fixed* length, under which the simulated
            // time per tick is proportional to N and the drone flies N times as fast. That failure is
            // enormous -- 1 substep against 128 would differ by 128x, i.e. 12,700% -- so a few
            // percent of tolerance separates it cleanly from the genuine discretisation difference
            // measured by the convergence test below.
            double reference = climbInOneSecond(8, 30);

            for (int substeps : new int[] {1, 2, 4, 8, 32, 128}) {
                double climb = climbInOneSecond(substeps, 30);
                assertTrue(
                        relativeGap(climb, reference) < 0.05,
                        "climb at " + substeps + " substeps was " + climb + ", reference " + reference);
            }
        }

        @Test
        void wouldNotSurviveSubstepsScalingTheSimulatedTime() {
            // Guards the tolerance above from being vacuous. If `advance` multiplied rather than
            // divided, doubling the substep count would double the climb; this pins that a 2x change
            // in N really is far outside the tolerance the invariance test allows.
            double eight = climbInOneSecond(8, 30);
            double sixteenIfItScaled = climbInOneSecond(16, 30) * 2.0;

            assertTrue(
                    relativeGap(sixteenIfItScaled, eight) > 0.5,
                    "a 2x scaling error must be unmistakable, but reads as "
                            + relativeGap(sixteenIfItScaled, eight));
        }

        @Test
        void convergesAsTheSubstepShrinksRatherThanBeingIdenticalAcrossCounts() {
            // Substepping is a discretisation refinement, so exact equality is the wrong assertion:
            // it would only pass if substepping made no difference at all, i.e. if it were pointless.
            // The real property is that the error against a very fine reference shrinks with N.
            double fine = climbInOneSecond(512, 30);
            double coarse = Math.abs(climbInOneSecond(2, 30) - fine);
            double medium = Math.abs(climbInOneSecond(8, 30) - fine);
            double finer = Math.abs(climbInOneSecond(64, 30) - fine);

            assertTrue(medium < coarse, "8 substeps (" + medium + ") should beat 2 (" + coarse + ")");
            assertTrue(finer < medium, "64 substeps (" + finer + ") should beat 8 (" + medium + ")");
        }

        @Test
        void isIndependentOfTickRateSoRaisingWorldTpsIsAConfigChange() {
            // Decision 3's escape hatch: World.setTps(120..240) on a dedicated drone world. One
            // simulated second must be one simulated second at 30, 120 and 240 TPS -- if `advance`
            // ignored tickSeconds, 240 TPS would integrate eight times the simulated time.
            double at30 = climbInOneSecond(8, 30);

            for (int tps : new int[] {60, 120, 240}) {
                double climb = climbInOneSecond(8, tps);
                assertTrue(
                        relativeGap(climb, at30) < 0.05,
                        "30 TPS gave " + at30 + ", " + tps + " gave " + climb);
            }
        }
    }

    @Nested
    class SubstepLength {

        @Test
        void isTheTickDividedByTheSubstepCount() {
            assertEquals(TICK_30_TPS / 8.0, at(8).substepSeconds(TICK_30_TPS), 0.0);
            assertEquals(TICK_30_TPS, at(1).substepSeconds(TICK_30_TPS), 0.0);
        }

        @Test
        void agreesWithTheFigureFpvConfigAdvertisesForTheSameNumbers() {
            // FpvConfig.substepSeconds() is `1.0f / (worldTps * physicsSubsteps)`, computed in the
            // plugin module where :fpv-core cannot see it. Two expressions written months apart that
            // are meant to be the same number, so they are pinned against each other here rather
            // than trusted.
            //
            // The tolerance is float-shaped on purpose: the config helper returns `float` while
            // everything here is `double`, so the two agree only to about seven significant digits.
            // That is why the plugin passes `1.0 / worldTps` as a double and lets this class do the
            // division, rather than handing the float substep length to the integrator -- at 240 TPS
            // the drone takes 1,920 steps a second and float rounding on the step length would
            // accumulate, which is the drift CLAUDE.md's world-units note warns about.
            for (int tps : new int[] {30, 60, 120, 240}) {
                for (int substeps : new int[] {1, 4, 8, 16}) {
                    double fromConfig = 1.0f / (tps * substeps);
                    double fromLoop = at(substeps).substepSeconds(1.0 / tps);
                    assertEquals(
                            fromConfig,
                            fromLoop,
                            fromLoop * 1e-6,
                            "tps=" + tps + " substeps=" + substeps);
                }
            }
        }

        @Test
        void rejectsANonPositiveOrNonFiniteTickBecauseItComesFromOurOwnLoopNotFromAClient() {
            FlightTick tick = at(8);
            assertThrows(IllegalArgumentException.class, () -> tick.substepSeconds(0.0));
            assertThrows(IllegalArgumentException.class, () -> tick.substepSeconds(-TICK_30_TPS));
            assertThrows(IllegalArgumentException.class, () -> tick.substepSeconds(Double.NaN));
            assertThrows(
                    IllegalArgumentException.class,
                    () -> tick.advance(FlightState.restingAt(Vec3.ZERO), FULL_THROTTLE, 0.0));
            assertThrows(
                    IllegalArgumentException.class,
                    () ->
                            tick.advance(
                                    FlightState.restingAt(Vec3.ZERO),
                                    FULL_THROTTLE,
                                    Double.POSITIVE_INFINITY));
        }
    }

    @Nested
    class Construction {

        @Test
        void rejectsFewerThanOneSubstepBecauseATickHasToAdvanceTheSimulation() {
            QuadIntegrator integrator = new QuadIntegrator(QuadParameters.DEFAULT);
            assertThrows(IllegalArgumentException.class, () -> new FlightTick(integrator, 0));
            assertThrows(IllegalArgumentException.class, () -> new FlightTick(integrator, -8));
        }

        @Test
        void rejectsANullIntegrator() {
            assertThrows(IllegalArgumentException.class, () -> new FlightTick(null, 8));
        }

        @Test
        void keepsItsIntegratorAndSubstepCountReadable() {
            QuadIntegrator integrator = new QuadIntegrator(QuadParameters.DEFAULT);
            FlightTick tick = new FlightTick(integrator, 12);
            assertSame(integrator, tick.integrator());
            assertEquals(12, tick.substeps());
        }
    }

    @Nested
    class PerSubstepListener {

        /** Records every call, and hands back whatever it was given. */
        private static final class Recorder implements SubstepListener {
            final List<Double> intervals = new ArrayList<>();
            final List<FlightState> results = new ArrayList<>();

            @Override
            public FlightState afterSubstep(FlightState before, FlightState after, double dt) {
                this.intervals.add(dt);
                this.results.add(after);
                return after;
            }
        }

        @Test
        void runsOncePerSubstepNotOncePerTickBecauseOtherwiseAFastDroneTunnels() {
            // This is #21's requirement, tested here because it is a property of this loop rather
            // than of collision. A once-per-tick hook would see 1 call, not 8.
            Recorder recorder = new Recorder();
            at(8).advance(FlightState.restingAt(Vec3.ZERO), FULL_THROTTLE, TICK_30_TPS, recorder);

            assertEquals(8, recorder.intervals.size());
        }

        @Test
        void isToldTheSubstepLengthNotTheTickLengthSoASweptTestCoversTheRightSpan() {
            Recorder recorder = new Recorder();
            at(4).advance(FlightState.restingAt(Vec3.ZERO), FULL_THROTTLE, TICK_30_TPS, recorder);

            for (double interval : recorder.intervals) {
                assertEquals(TICK_30_TPS / 4.0, interval, 0.0);
            }
        }

        @Test
        void seesConsecutiveStatesSoTheSweptSpanIsContiguous() {
            // A swept AABB test needs `before` to be exactly where the previous substep left the
            // drone, or there are gaps in the volume it checks. Recording `after` and comparing it to
            // the next call's `before` is what proves the chain is unbroken.
            List<FlightState> befores = new ArrayList<>();
            List<FlightState> afters = new ArrayList<>();
            at(5).advance(
                            FlightState.restingAt(Vec3.ZERO),
                            FULL_THROTTLE,
                            TICK_30_TPS,
                            (before, after, dt) -> {
                                befores.add(before);
                                afters.add(after);
                                return after;
                            });

            for (int i = 1; i < befores.size(); i++) {
                assertSame(afters.get(i - 1), befores.get(i));
            }
        }

        @Test
        void integratesOnwardFromTheStateItReturnsSoACollisionCanClampThePosition() {
            // The point of returning a state: #21 clamps the drone to the impact point and zeroes the
            // velocity into the surface. Here the listener pins the drone in place every substep, and
            // the tick must therefore end where it began despite full throttle.
            Vec3 pinned = new Vec3(1, 2, 3);
            FlightState after =
                    at(8).advance(
                            FlightState.restingAt(pinned),
                            FULL_THROTTLE,
                            TICK_30_TPS,
                            (before, stepped, dt) ->
                                    new FlightState(
                                            new DroneState(
                                                    pinned,
                                                    Vec3.ZERO,
                                                    stepped.drone().orientation(),
                                                    stepped.drone().bodyRates()),
                                            stepped.controller()));

            assertEquals(pinned, after.drone().position());
            assertEquals(Vec3.ZERO, after.drone().velocity());
        }

        @Test
        void withoutAListenerTheDroneMovesSoThePinningTestAboveIsMeasuringSomething() {
            FlightState after =
                    at(8).advance(FlightState.restingAt(new Vec3(1, 2, 3)), FULL_THROTTLE, TICK_30_TPS);
            assertNotEquals(new Vec3(1, 2, 3), after.drone().position());
        }

        @Test
        void noneAcceptsTheSteppedStateUnchanged() {
            FlightState withNone =
                    at(8).advance(
                            FlightState.restingAt(Vec3.ZERO),
                            FULL_THROTTLE,
                            TICK_30_TPS,
                            SubstepListener.NONE);
            FlightState withoutListener =
                    at(8).advance(FlightState.restingAt(Vec3.ZERO), FULL_THROTTLE, TICK_30_TPS);

            assertEquals(withoutListener.drone(), withNone.drone());
        }

        @Test
        void refusesANullReturnLoudlyRatherThanNullPointeringOneSubstepLater() {
            FlightTick tick = at(8);
            FlightState resting = FlightState.restingAt(Vec3.ZERO);
            assertThrows(
                    IllegalStateException.class,
                    () -> tick.advance(resting, FULL_THROTTLE, TICK_30_TPS, (b, a, dt) -> null));
        }

        @Test
        void rejectsANullListenerRatherThanSilentlyRunningWithoutOne() {
            FlightTick tick = at(8);
            FlightState resting = FlightState.restingAt(Vec3.ZERO);
            assertThrows(
                    IllegalArgumentException.class,
                    () -> tick.advance(resting, FULL_THROTTLE, TICK_30_TPS, null));
        }
    }

    @Nested
    class Arguments {

        @Test
        void rejectsANullStateOrInput() {
            FlightTick tick = at(8);
            assertThrows(
                    IllegalArgumentException.class, () -> tick.advance(null, FULL_THROTTLE, TICK_30_TPS));
            assertThrows(
                    IllegalArgumentException.class,
                    () -> tick.advance(FlightState.restingAt(Vec3.ZERO), null, TICK_30_TPS));
        }
    }
}
