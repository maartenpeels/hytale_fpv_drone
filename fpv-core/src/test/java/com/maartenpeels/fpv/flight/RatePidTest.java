package com.maartenpeels.fpv.flight;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.maartenpeels.fpv.control.ControlInput;
import com.maartenpeels.fpv.math.Vec3;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

/**
 * Two kinds of test here, deliberately.
 *
 * <p>The tracking tests drive the controller through the <em>real</em> {@link QuadIntegrator}, because
 * the plant is what makes the question interesting: mixing is linear in command space while thrust
 * goes with the square of the command, so achieved torque never equals demanded torque, and the
 * loop is non-linear, throttle-dependent and saturating. Asking whether the controller converges
 * against a tidy first-order plant it will never meet would prove nothing.
 *
 * <p>The windup and derivative tests call {@link RatePid#update} directly with the measurement
 * pinned, because the situations they describe — an axis that physically cannot reach its demand, a
 * setpoint that steps while the drone does not move — are ones the closed loop will not sit still in
 * long enough to observe.
 */
class RatePidTest {

    /** 30 TPS with 8 substeps — the configured default in {@code FpvConfig}. */
    private static final double DEFAULT_SUBSTEP = 1.0 / (30 * 8);

    private static final QuadParameters AIRFRAME = QuadParameters.DEFAULT;

    /**
     * The <em>linear</em> rate profile, not {@link RateProfile#DEFAULT}.
     *
     * <p>These tests are about the rate loop, so the stick-to-rate map should not be a variable in
     * them: a half-stick demand here means half of max rate, exactly as it did before #15 replaced
     * the linear map with a curve. That keeps every number this file asserts — and every figure in
     * #14's plan document — directly comparable, and it makes this file the check that relocating
     * full-stick rate out of the airframe changed no behaviour. {@code RateCurveTest} is where the
     * curve's own shape is tested.
     */
    private static final RateProfile RATES =
            RateProfile.DEFAULT.onEveryAxis(RateCurve::asLinear);

    private static FlightState fly(
            RatePidGains gains, ControlInput input, double dt, double seconds) {
        QuadIntegrator integrator = new QuadIntegrator(AIRFRAME, gains, RATES);
        FlightState state = FlightState.restingAt(Vec3.ZERO);
        int steps = (int) Math.round(seconds / dt);
        for (int i = 0; i < steps; i++) {
            state = integrator.step(state, input, dt);
        }
        return state;
    }

    /** Peak roll rate reached over {@code seconds}, for asking about overshoot. */
    private static double peakRollRate(
            RatePidGains gains, ControlInput input, double dt, double seconds) {
        QuadIntegrator integrator = new QuadIntegrator(AIRFRAME, gains, RATES);
        FlightState state = FlightState.restingAt(Vec3.ZERO);
        double peak = 0;
        int steps = (int) Math.round(seconds / dt);
        for (int i = 0; i < steps; i++) {
            state = integrator.step(state, input, dt);
            peak = Math.max(peak, state.drone().bodyRates().roll());
        }
        return peak;
    }

    private static double demandedRoll(float stick) {
        return RATES.roll().rateFor(stick);
    }

    @Nested
    class Tracking {

        @Test
        void convergesOnTheDemandedRateFromRest() {
            float stick = 0.5f;
            double demanded = demandedRoll(stick);

            double settled =
                    fly(
                                    RatePidGains.DEFAULT,
                                    new ControlInput(0.5f, stick, 0f, 0f),
                                    DEFAULT_SUBSTEP,
                                    1.0)
                            .drone()
                            .bodyRates()
                            .roll();

            // Measured at +0.11% of demand. A tenth of a percent of 400 deg/s is 0.4 deg/s, which
            // is far below anything a pilot could perceive.
            assertEquals(demanded, settled, demanded * 0.005);
        }

        @Test
        void leavesAStandingErrorWithoutTheIntegralTermWhichIsWhyTheIntegralTermExists() {
            // This is the stiffness #13 shipped and documented: a proportional tracker settles where
            // its output balances drag and the mixer's losses, which is never quite the demand. The
            // gap is what the integral term is for, so it is worth pinning both halves.
            float stick = 0.5f;
            double demanded = demandedRoll(stick);
            ControlInput rolling = new ControlInput(0.5f, stick, 0f, 0f);

            double withIntegral =
                    fly(RatePidGains.DEFAULT, rolling, DEFAULT_SUBSTEP, 1.0)
                            .drone()
                            .bodyRates()
                            .roll();
            double withoutIntegral =
                    fly(
                                    RatePidGains.DEFAULT.onEveryAxis(PidGains::withoutIntegral),
                                    rolling,
                                    DEFAULT_SUBSTEP,
                                    1.0)
                            .drone()
                            .bodyRates()
                            .roll();

            // Measured: -1.478% of demand without the integral term, +0.11% with it. An order of
            // magnitude is the claim; the exact figures are airframe-dependent.
            double errorWithout = Math.abs(demanded - withoutIntegral);
            double errorWith = Math.abs(demanded - withIntegral);

            assertTrue(
                    withoutIntegral < demanded,
                    "a proportional-only loop should undershoot, not overshoot: " + withoutIntegral);
            assertTrue(
                    errorWith < errorWithout * 0.2,
                    "the integral term should cut the standing error by at least 5x, but went from "
                            + errorWithout
                            + " to "
                            + errorWith);
        }

        @Test
        void tracksEveryAxisSeparatelyBecauseYawIsFarWeakerThanRollAndPitch() {
            // Yaw torque comes from prop reaction rather than differential lift, so its authority is
            // a quarter of roll and pitch. Per-axis gains are what let all three still land on their
            // demand; one shared set could not.
            FlightState after =
                    fly(
                            RatePidGains.DEFAULT,
                            new ControlInput(0.5f, 0.4f, -0.3f, 0.5f),
                            DEFAULT_SUBSTEP,
                            1.0);
            BodyRates maxRates = RATES.maxRates();
            BodyRates rates = after.drone().bodyRates();

            assertEquals(0.4 * maxRates.roll(), rates.roll(), 0.4 * maxRates.roll() * 0.02);
            assertEquals(-0.3 * maxRates.pitch(), rates.pitch(), 0.3 * maxRates.pitch() * 0.02);
            assertEquals(0.5 * maxRates.yaw(), rates.yaw(), 0.5 * maxRates.yaw() * 0.02);
        }

        @Test
        void overshootsFarMoreAtHighProportionalGain() {
            // A small stick, so the loop stays inside the mixer's unsaturated branch and the gain
            // really is what governs the response. At full stick the mixer's ceiling dominates and
            // raising P changes almost nothing -- which is true of a real quad too.
            float stick = 0.04f;
            double demanded = demandedRoll(stick);
            ControlInput gentle = new ControlInput(0.5f, stick, 0f, 0f);

            double atDefault = peakRollRate(RatePidGains.DEFAULT, gentle, DEFAULT_SUBSTEP, 2.0);
            double atHighGain =
                    peakRollRate(
                            RatePidGains.DEFAULT.onEveryAxis(
                                    g -> g.withProportional(g.proportional() * 4)),
                            gentle,
                            DEFAULT_SUBSTEP,
                            2.0);

            // Measured: 2.8% overshoot at the default gain, 11.5% at four times it. Past about
            // eight times, the loop stops settling at all -- the point of the default being where
            // it is.
            assertTrue(
                    atDefault < demanded * 1.05,
                    "the default tune should barely overshoot but peaked at " + atDefault);
            assertTrue(
                    atHighGain > demanded * 1.08,
                    "four times the gain should visibly overshoot but peaked at " + atHighGain);
        }

        @Test
        void tracksAtEverySubstepCountFromTwoUpwards() {
            // The floor is the plant's, not the controller's: at one substep per 30 TPS tick the
            // whole loop is being sampled so coarsely that a proportional-only tracker mistracks by
            // the same +37%. FpvConfig defaults to 8, so this documents a 4x margin -- and documents
            // that 1 is not a usable setting, which is worth knowing before someone tries it.
            float stick = 0.5f;
            double demanded = demandedRoll(stick);
            ControlInput rolling = new ControlInput(0.5f, stick, 0f, 0f);

            for (int substeps : new int[] {2, 4, 8, 16, 64}) {
                double settled =
                        fly(RatePidGains.DEFAULT, rolling, 1.0 / (30 * substeps), 1.0)
                                .drone()
                                .bodyRates()
                                .roll();
                assertEquals(
                        demanded,
                        settled,
                        demanded * 0.01,
                        substeps + " substeps per tick should still track the demand");
            }

            double atOneSubstep =
                    fly(RatePidGains.DEFAULT, rolling, 1.0 / 30, 1.0).drone().bodyRates().roll();
            assertTrue(
                    atOneSubstep > demanded * 1.2,
                    "one substep per tick is expected to mistrack badly (measured +37%); if this"
                            + " now passes, the plant got better and the floor can be lowered");
        }
    }

    @Nested
    class Windup {

        /** Full stick on every axis, which no airframe can hold against a wall. */
        private static final BodyRates UNREACHABLE = RATES.maxRates();

        private static RatePidState hold(
                RatePid pid, BodyRates demanded, BodyRates measured, int steps) {
            RatePidState state = RatePidState.ZERO;
            for (int i = 0; i < steps; i++) {
                state = pid.update(state, demanded, measured, DEFAULT_SUBSTEP).state();
            }
            return state;
        }

        @Test
        void storesNothingWhileHeldAgainstAWallAtFullStick() {
            // The case the ticket names. Full stick with the rate pinned at zero saturates the
            // output on the proportional term alone, so accumulation freezes on the very first step
            // and ten seconds of it changes nothing.
            RatePid pid = new RatePid(RatePidGains.DEFAULT);

            RatePidState afterTenSeconds = hold(pid, UNREACHABLE, BodyRates.ZERO, 2400);
            TorqueDemand torque =
                    pid.update(afterTenSeconds, UNREACHABLE, BodyRates.ZERO, DEFAULT_SUBSTEP)
                            .torque();

            assertEquals(0.0, afterTenSeconds.roll().integral(), 1e-12);
            assertEquals(0.0, afterTenSeconds.pitch().integral(), 1e-12);
            assertEquals(0.0, afterTenSeconds.yaw().integral(), 1e-12);
            assertEquals(1.0, torque.roll(), 1e-12, "should still be commanding all it can");
        }

        @Test
        void storesNothingWhenPinnedAgainstTheWallInTheOtherDirectionEither() {
            // The freeze is a sign comparison, which is the kind of thing that works on one side and
            // not the other. Full stick the other way, and the answers should mirror exactly.
            RatePid pid = new RatePid(RatePidGains.DEFAULT);
            BodyRates reversed = UNREACHABLE.scale(-1);

            RatePidState afterTenSeconds = hold(pid, reversed, BodyRates.ZERO, 2400);
            TorqueDemand torque =
                    pid.update(afterTenSeconds, reversed, BodyRates.ZERO, DEFAULT_SUBSTEP)
                            .torque();

            assertEquals(0.0, afterTenSeconds.roll().integral(), 1e-12);
            assertEquals(0.0, afterTenSeconds.yaw().integral(), 1e-12);
            assertEquals(-1.0, torque.roll(), 1e-12);
            assertEquals(-1.0, torque.yaw(), 1e-12);
        }

        @Test
        void clampsTheIntegralInTheNegativeDirectionToo() {
            RatePid pid = new RatePid(RatePidGains.DEFAULT);
            double limit = RatePidGains.DEFAULT.roll().integralLimit();

            RatePidState wound = hold(pid, new BodyRates(-2, -2, -2), BodyRates.ZERO, 2400);

            assertEquals(-limit, wound.roll().integral(), 1e-12);
            assertEquals(-limit, wound.pitch().integral(), 1e-12);
        }

        @Test
        void clampsTheIntegralWhenTheErrorPersistsWithoutSaturatingTheOutput() {
            // The gap the freeze alone would leave: a demand small enough that the proportional term
            // does not saturate, but which the axis still cannot reach. Nothing ever triggers the
            // freeze, so the clamp is the only thing bounding the accumulator -- which is exactly
            // why both guards are there.
            RatePid pid = new RatePid(RatePidGains.DEFAULT);
            BodyRates modest = new BodyRates(2, 2, 2);
            double limit = RatePidGains.DEFAULT.roll().integralLimit();

            RatePidState afterTenSeconds = hold(pid, modest, BodyRates.ZERO, 2400);
            double torque =
                    pid.update(afterTenSeconds, modest, BodyRates.ZERO, DEFAULT_SUBSTEP)
                            .torque()
                            .roll();

            assertEquals(limit, afterTenSeconds.roll().integral(), 1e-12);
            assertTrue(
                    torque < 1.0,
                    "the output never saturated here, so the clamp -- not the freeze -- is what"
                            + " bounded the accumulator; torque was "
                            + torque);
        }

        @Test
        void aFrozenIntegralMovesAgainOnTheStepTheErrorReverses() {
            // Recovery has to be immediate. A guard that waited for the accumulator to bleed down
            // would give the drone back its authority some tens of milliseconds late, which is the
            // snap a pilot feels when a quad comes free of whatever it was against.
            RatePid pid = new RatePid(RatePidGains.DEFAULT);
            BodyRates modest = new BodyRates(2, 2, 2);
            RatePidState wound = hold(pid, modest, BodyRates.ZERO, 2400);

            // The demand reverses: the axis is now over-rotating, so the stored correction is wrong
            // and must start unwinding at once.
            RatePidState next =
                    pid.update(wound, new BodyRates(-2, -2, -2), BodyRates.ZERO, DEFAULT_SUBSTEP)
                            .state();

            assertTrue(
                    next.roll().integral() < wound.roll().integral(),
                    "a reversed error should immediately unwind the accumulator, but it went from "
                            + wound.roll().integral()
                            + " to "
                            + next.roll().integral());
        }

        @Test
        void keepsTheOutputInsideTheMixersRangeWhateverTheGainsAndTheError() {
            // TorqueDemand rejects out-of-range values on construction, so a leak here is a thrown
            // exception rather than a silent one. Absurd gains, absurd error, and a dt far from the
            // design point, all at once.
            RatePid absurd =
                    new RatePid(RatePidGains.uniform(new PidGains(1e6, 1e6, 1e6, 1.0)));
            BodyRates enormous = new BodyRates(1e4, -1e4, 1e4);

            TorqueDemand first =
                    absurd.update(RatePidState.ZERO, enormous, BodyRates.ZERO, 1e-6).torque();
            TorqueDemand later =
                    absurd.update(
                                    RatePidState.at(enormous),
                                    BodyRates.ZERO,
                                    enormous,
                                    1.0)
                            .torque();

            assertEquals(1.0, first.roll(), 1e-12);
            assertEquals(-1.0, first.pitch(), 1e-12);
            assertEquals(-1.0, later.roll(), 1e-12);
        }

        @Test
        void anIntegralLimitOfZeroTurnsTheIntegratorOffEntirely() {
            RatePid pid =
                    new RatePid(
                            RatePidGains.DEFAULT.onEveryAxis(g -> g.withIntegralLimit(0.0)));

            RatePidState after = hold(pid, new BodyRates(2, 2, 2), BodyRates.ZERO, 240);

            assertEquals(0.0, after.roll().integral(), 1e-12);
        }
    }

    @Nested
    class DerivativeOnMeasurement {

        private static final PidGains WITH_DERIVATIVE =
                PidGains.fromTimes(RatePidGains.DEFAULT_ROLL_PITCH_PROPORTIONAL, 0.3, 0.008);

        @Test
        void aSetpointStepProducesNoDerivativeActionAtAll() {
            // The reason for deriving on the measurement. A stick snap is a setpoint step, and
            // d(setpoint)/dt at a 4 ms step is enormous: differentiating the error rather than the
            // measurement would add 0.37 of full authority here, purely as an artefact of the pilot
            // moving their thumb.
            RatePid without =
                    new RatePid(RatePidGains.uniform(WITH_DERIVATIVE.withoutDerivative()));
            RatePid with = new RatePid(RatePidGains.uniform(WITH_DERIVATIVE));
            BodyRates snapped = new BodyRates(1, 0, 0);

            double withoutD =
                    without.update(RatePidState.ZERO, snapped, BodyRates.ZERO, DEFAULT_SUBSTEP)
                            .torque()
                            .roll();
            double withD =
                    with.update(RatePidState.ZERO, snapped, BodyRates.ZERO, DEFAULT_SUBSTEP)
                            .torque()
                            .roll();

            assertEquals(
                    withoutD,
                    withD,
                    1e-12,
                    "the derivative gain must not change the answer to a pure setpoint step");
        }

        @Test
        void aChangeInTheMeasuredRateDoesProduceDerivativeAction() {
            // The other half: D still has to do its job. Same gains, but now it is the drone that
            // moved rather than the stick.
            RatePid without =
                    new RatePid(RatePidGains.uniform(WITH_DERIVATIVE.withoutDerivative()));
            RatePid with = new RatePid(RatePidGains.uniform(WITH_DERIVATIVE));
            BodyRates rotating = new BodyRates(1, 0, 0);

            double withoutD =
                    without.update(RatePidState.ZERO, BodyRates.ZERO, rotating, DEFAULT_SUBSTEP)
                            .torque()
                            .roll();
            double withD =
                    with.update(RatePidState.ZERO, BodyRates.ZERO, rotating, DEFAULT_SUBSTEP)
                            .torque()
                            .roll();

            // Measured -0.194 without, -0.560 with: the derivative term opposes the rotation on top
            // of what the proportional term was already doing.
            assertTrue(
                    withD < withoutD - 0.1,
                    "the derivative term should push back harder against a rate that is changing,"
                            + " but went from "
                            + withoutD
                            + " to "
                            + withD);
        }

        @Test
        void spawningAControllerAtTheCurrentRateAvoidsAPhantomKick() {
            // A controller handed ZERO for a drone already rotating believes the whole of that
            // rotation appeared in one step, and answers a zero error with full opposing torque.
            // RatePidState.at() is the fix, and this is the size of the problem it fixes.
            RatePid pid = new RatePid(RatePidGains.uniform(WITH_DERIVATIVE));
            BodyRates spinning = new BodyRates(6, 0, 0);

            double fromZeroState =
                    pid.update(RatePidState.ZERO, spinning, spinning, DEFAULT_SUBSTEP)
                            .torque()
                            .roll();
            double fromPrimedState =
                    pid.update(RatePidState.at(spinning), spinning, spinning, DEFAULT_SUBSTEP)
                            .torque()
                            .roll();

            assertEquals(-1.0, fromZeroState, 1e-12, "unprimed state should kick at full authority");
            assertEquals(
                    0.0,
                    fromPrimedState,
                    1e-12,
                    "a primed controller should ask for nothing when the rate is already right");
        }
    }

    @Nested
    class Purity {

        @Test
        void theSameArgumentsAlwaysGiveTheSameAnswer() {
            RatePid pid = new RatePid(RatePidGains.DEFAULT);
            RatePidState state = new RatePidState(
                    new PidState(0.11, 1.3), new PidState(-0.07, -0.4), new PidState(0.02, 0.9));
            BodyRates demanded = new BodyRates(3.0, -1.5, 0.7);
            BodyRates measured = new BodyRates(2.2, -1.9, 0.5);

            assertEquals(
                    pid.update(state, demanded, measured, DEFAULT_SUBSTEP),
                    pid.update(state, demanded, measured, DEFAULT_SUBSTEP));
        }

        @Test
        void oneAxisNeverInfluencesAnother() {
            RatePid pid = new RatePid(RatePidGains.DEFAULT);

            TorqueDemand rollOnly =
                    pid.update(
                                    RatePidState.ZERO,
                                    new BodyRates(3, 0, 0),
                                    BodyRates.ZERO,
                                    DEFAULT_SUBSTEP)
                            .torque();

            assertTrue(rollOnly.roll() > 0);
            assertEquals(0.0, rollOnly.pitch(), 1e-12);
            assertEquals(0.0, rollOnly.yaw(), 1e-12);
        }
    }

    @Nested
    class Validation {

        @Test
        void rejectsMissingGainsStateOrRates() {
            RatePid pid = new RatePid(RatePidGains.DEFAULT);

            assertThrows(IllegalArgumentException.class, () -> new RatePid(null));
            assertThrows(
                    IllegalArgumentException.class,
                    () -> pid.update(null, BodyRates.ZERO, BodyRates.ZERO, DEFAULT_SUBSTEP));
            assertThrows(
                    IllegalArgumentException.class,
                    () -> pid.update(RatePidState.ZERO, null, BodyRates.ZERO, DEFAULT_SUBSTEP));
            assertThrows(
                    IllegalArgumentException.class,
                    () -> pid.update(RatePidState.ZERO, BodyRates.ZERO, null, DEFAULT_SUBSTEP));
        }

        @Test
        void rejectsANonPositiveOrNonFiniteStepBecauseTheDerivativeDividesByIt() {
            RatePid pid = new RatePid(RatePidGains.DEFAULT);

            assertThrows(
                    IllegalArgumentException.class,
                    () -> pid.update(RatePidState.ZERO, BodyRates.ZERO, BodyRates.ZERO, 0));
            assertThrows(
                    IllegalArgumentException.class,
                    () -> pid.update(RatePidState.ZERO, BodyRates.ZERO, BodyRates.ZERO, -0.001));
            assertThrows(
                    IllegalArgumentException.class,
                    () ->
                            pid.update(
                                    RatePidState.ZERO,
                                    BodyRates.ZERO,
                                    BodyRates.ZERO,
                                    Double.NaN));
        }

        @Test
        void rejectsNonFiniteRates() {
            RatePid pid = new RatePid(RatePidGains.DEFAULT);
            BodyRates broken = new BodyRates(Double.NaN, 0, 0);

            assertThrows(
                    IllegalArgumentException.class,
                    () -> pid.update(RatePidState.ZERO, broken, BodyRates.ZERO, DEFAULT_SUBSTEP));
        }
    }
}
