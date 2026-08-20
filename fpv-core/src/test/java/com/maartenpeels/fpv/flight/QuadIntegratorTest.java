package com.maartenpeels.fpv.flight;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.maartenpeels.fpv.control.ControlInput;
import com.maartenpeels.fpv.math.Vec3;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

/**
 * The dynamics: forces, torques and integration. How well the rate loop <em>tracks</em> a demand is
 * {@link RatePidTest}'s subject, and it drives this same integrator to ask.
 */
class QuadIntegratorTest {

    /** 30 TPS with 8 substeps — the configured default in {@code FpvConfig}. */
    private static final double DEFAULT_SUBSTEP = 1.0 / (30 * 8);

    private static FlightState simulate(
            QuadIntegrator integrator, FlightState from, ControlInput input, double dt, int steps) {
        FlightState state = from;
        for (int i = 0; i < steps; i++) {
            state = integrator.step(state, input, dt);
        }
        return state;
    }

    /** Runs for a fixed span of simulated time, whatever the step size. */
    private static FlightState simulateSeconds(
            QuadIntegrator integrator, ControlInput input, double seconds, int steps) {
        return simulate(
                integrator, FlightState.restingAt(Vec3.ZERO), input, seconds / steps, steps);
    }

    @Nested
    class Hover {

        @Test
        void holdsPositionAtTheHoverCollectiveBecauseThrustExactlyCancelsGravity() {
            QuadParameters parameters = QuadParameters.DEFAULT;
            QuadIntegrator integrator = new QuadIntegrator(parameters);
            ControlInput hover =
                    new ControlInput((float) parameters.hoverCollective(), 0f, 0f, 0f);

            FlightState after =
                    simulate(
                            integrator,
                            FlightState.restingAt(new Vec3(0, 100, 0)),
                            hover,
                            DEFAULT_SUBSTEP,
                            2400);

            // Ten seconds of hover. The residual is float rounding on the throttle axis, nothing
            // structural, so the tolerance is tight.
            assertEquals(100.0, after.drone().position().y(), 1e-3);
            assertEquals(0.0, after.drone().speed(), 1e-3);
        }

        @Test
        void sinksBelowTheHoverCollectiveAndClimbsAboveIt() {
            QuadParameters parameters = QuadParameters.DEFAULT;
            QuadIntegrator integrator = new QuadIntegrator(parameters);
            float hover = (float) parameters.hoverCollective();

            FlightState sinking =
                    simulate(
                            integrator,
                            FlightState.restingAt(Vec3.ZERO),
                            new ControlInput(hover * 0.5f, 0f, 0f, 0f),
                            DEFAULT_SUBSTEP,
                            240);
            FlightState climbing =
                    simulate(
                            integrator,
                            FlightState.restingAt(Vec3.ZERO),
                            new ControlInput(hover * 1.5f, 0f, 0f, 0f),
                            DEFAULT_SUBSTEP,
                            240);

            assertTrue(sinking.drone().velocity().y() < 0, "should sink but y velocity was "
                    + sinking.drone().velocity().y());
            assertTrue(climbing.drone().velocity().y() > 0, "should climb but y velocity was "
                    + climbing.drone().velocity().y());
        }

        @Test
        void hoveringLevelIntroducesNoRotation() {
            QuadParameters parameters = QuadParameters.DEFAULT;
            QuadIntegrator integrator = new QuadIntegrator(parameters);

            FlightState after =
                    simulate(
                            integrator,
                            FlightState.restingAt(Vec3.ZERO),
                            new ControlInput((float) parameters.hoverCollective(), 0f, 0f, 0f),
                            DEFAULT_SUBSTEP,
                            2400);

            assertEquals(0.0, after.drone().bodyRates().roll(), 1e-12);
            assertEquals(0.0, after.drone().bodyRates().pitch(), 1e-12);
            assertEquals(0.0, after.drone().bodyRates().yaw(), 1e-12);
            assertEquals(1.0, after.drone().thrustAxis().y(), 1e-12);
        }
    }

    @Nested
    class BallisticFall {

        @Test
        void fallsUnderGravityAloneAtZeroThrottle() {
            QuadIntegrator integrator =
                    new QuadIntegrator(QuadParameters.builder().withoutDrag().build());
            double gravity = QuadParameters.DEFAULT_GRAVITY;
            int steps = 240;

            FlightState after =
                    simulate(
                            integrator,
                            FlightState.restingAt(Vec3.ZERO),
                            ControlInput.NEUTRAL,
                            DEFAULT_SUBSTEP,
                            steps);

            // The closed form of the discrete scheme, not of the continuous one: semi-implicit
            // Euler advances position with the velocity it has *after* the acceleration, giving
            // -g*dt^2*n*(n+1)/2. Asserting that exactly is what proves nothing else is acting.
            double expectedVelocity = -gravity * steps * DEFAULT_SUBSTEP;
            double expectedDrop =
                    -gravity * DEFAULT_SUBSTEP * DEFAULT_SUBSTEP * steps * (steps + 1) / 2.0;

            assertEquals(expectedVelocity, after.drone().velocity().y(), 1e-9);
            assertEquals(expectedDrop, after.drone().position().y(), 1e-9);
            assertEquals(0.0, after.drone().position().x(), 1e-12);
            assertEquals(0.0, after.drone().position().z(), 1e-12);
        }

        @Test
        void staysWithinAStepOfTheContinuousSolution() {
            QuadIntegrator integrator =
                    new QuadIntegrator(QuadParameters.builder().withoutDrag().build());
            double gravity = QuadParameters.DEFAULT_GRAVITY;
            double seconds = 1.0;
            int steps = 240;

            FlightState after = simulateSeconds(integrator, ControlInput.NEUTRAL, seconds, steps);

            // First-order truncation over the whole fall comes to exactly half a step of velocity,
            // so that is the bound worth asserting rather than a round number.
            double continuousDrop = -0.5 * gravity * seconds * seconds;
            double oneStepOfVelocity = gravity * (seconds / steps) * seconds * 0.5;

            assertEquals(continuousDrop, after.drone().position().y(), oneStepOfVelocity * 1.01);
        }
    }

    @Nested
    class TerminalVelocity {

        @Test
        void convergesOnTheSpeedWhereQuadraticDragBalancesGravity() {
            double quadraticDrag = 0.01;
            QuadParameters parameters =
                    QuadParameters.builder()
                            .linearDrag(0)
                            .quadraticDrag(quadraticDrag)
                            .angularDrag(0)
                            .build();
            QuadIntegrator integrator = new QuadIntegrator(parameters);
            double terminal = Math.sqrt(parameters.gravity() / quadraticDrag);

            FlightState after =
                    simulate(
                            integrator,
                            FlightState.restingAt(Vec3.ZERO),
                            ControlInput.NEUTRAL,
                            DEFAULT_SUBSTEP,
                            30 * 8 * 30);

            assertEquals(-terminal, after.drone().velocity().y(), 0.01);
        }

        @Test
        void neverOvershootsTerminalVelocity() {
            double quadraticDrag = 0.01;
            QuadParameters parameters =
                    QuadParameters.builder().linearDrag(0).quadraticDrag(quadraticDrag).build();
            QuadIntegrator integrator = new QuadIntegrator(parameters);
            double terminal = Math.sqrt(parameters.gravity() / quadraticDrag);

            FlightState state = FlightState.restingAt(Vec3.ZERO);
            for (int i = 0; i < 30 * 8 * 30; i++) {
                state = integrator.step(state, ControlInput.NEUTRAL, DEFAULT_SUBSTEP);
                assertTrue(
                        state.drone().speed() <= terminal,
                        "speed " + state.drone().speed() + " exceeded terminal " + terminal);
            }
        }

        @Test
        void linearDragAlsoBoundsTheFall() {
            double linearDrag = 0.5;
            QuadParameters parameters =
                    QuadParameters.builder().linearDrag(linearDrag).quadraticDrag(0).build();
            QuadIntegrator integrator = new QuadIntegrator(parameters);

            FlightState after =
                    simulate(
                            integrator,
                            FlightState.restingAt(Vec3.ZERO),
                            ControlInput.NEUTRAL,
                            DEFAULT_SUBSTEP,
                            30 * 8 * 60);

            assertEquals(
                    -parameters.gravity() / linearDrag, after.drone().velocity().y(), 0.01);
        }
    }

    @Nested
    class RateAgnosticism {

        // The tests that protect CLAUDE.md decision 3's escape hatch. If raising World.setTps
        // changed where the drone ends up, "just raise the tick rate" would stop being a config
        // change and start being a re-tune.

        /**
         * A gentle off-axis input. Every axis moves, so thrust direction, drag and rotation all
         * matter — but the demands stay inside the rate loop's unsaturated region and inside the
         * mixer's unsaturated branch, so the integrand is smooth and the error really is O(dt).
         */
        private static final ControlInput CRUISING = new ControlInput(0.5f, 0.06f, -0.05f, 0.03f);

        @Test
        void landsInTheSamePlaceWhenDtIsHalvedAndStepCountDoubled() {
            QuadIntegrator integrator = new QuadIntegrator(QuadParameters.DEFAULT);
            double seconds = 1.0;

            FlightState coarse = simulateSeconds(integrator, CRUISING, seconds, 240);
            FlightState fine = simulateSeconds(integrator, CRUISING, seconds, 480);

            double travelled = coarse.drone().position().length();
            double difference =
                    coarse.drone().position().minus(fine.drone().position()).length();

            assertTrue(travelled > 5.0, "the test input should actually move the drone");
            assertTrue(
                    difference < 0.01 * travelled,
                    "moved " + travelled + " but the two step sizes disagreed by " + difference);
        }

        @Test
        void convergesAsTheStepShrinksRatherThanMerelyStayingClose() {
            // Staying within a tolerance could just mean the whole thing is insensitive. Halving dt
            // must actually reduce the error against a much finer reference.
            QuadIntegrator integrator = new QuadIntegrator(QuadParameters.DEFAULT);
            double seconds = 1.0;

            Vec3 reference =
                    simulateSeconds(integrator, CRUISING, seconds, 15_360).drone().position();
            double coarseError =
                    simulateSeconds(integrator, CRUISING, seconds, 240)
                            .drone()
                            .position()
                            .minus(reference)
                            .length();
            double fineError =
                    simulateSeconds(integrator, CRUISING, seconds, 480)
                            .drone()
                            .position()
                            .minus(reference)
                            .length();

            assertTrue(
                    fineError < coarseError * 0.6,
                    "halving dt took the error from " + coarseError + " only to " + fineError);
        }

        @Test
        void staysStableAtTheCoarsestStepTheConfigCanProduce() {
            // FpvConfig.physicsSubsteps is a config value, so someone can legitimately set it to 1
            // and hand the integrator a whole 30 TPS tick. It must degrade in accuracy, not blow up
            // -- and the rate loop's derivative term, which divides by dt, is the part most likely
            // to misbehave when dt is eight times its design point.
            QuadIntegrator integrator = new QuadIntegrator(QuadParameters.DEFAULT);
            FlightState state = FlightState.restingAt(Vec3.ZERO);

            for (int i = 0; i < 300; i++) {
                state = integrator.step(state, new ControlInput(0.8f, 0.9f, -0.7f, 0.5f), 1.0 / 30);
            }

            // The constructor rejects non-finite state, so reaching here at all proves it held
            // together; the bounds then check it did not diverge to something absurd.
            assertTrue(state.drone().position().isFinite(), "position went non-finite");
            assertTrue(
                    state.drone().speed() < 1000,
                    "speed ran away to " + state.drone().speed() + " at one substep per tick");
            assertEquals(
                    1.0,
                    state.drone().orientation().norm(),
                    1e-9,
                    "orientation stopped being a rotation");
        }

        @Test
        void reachesTheSameAttitudeWhateverTheStepSize() {
            QuadIntegrator integrator = new QuadIntegrator(QuadParameters.DEFAULT);
            double seconds = 1.0;

            Vec3 coarseNose = simulateSeconds(integrator, CRUISING, seconds, 240).drone().forward();
            Vec3 fineNose = simulateSeconds(integrator, CRUISING, seconds, 480).drone().forward();

            assertEquals(0.0, coarseNose.minus(fineNose).length(), 0.005);
        }

        @Test
        void flightIsUnchangedByTheWorldTickRateItIsSubsteppedInside() {
            // 30 TPS x 8 substeps against 240 TPS x 8 -- the actual escape hatch in decision 3.
            QuadIntegrator integrator = new QuadIntegrator(QuadParameters.DEFAULT);
            double seconds = 2.0;

            FlightState atThirtyTps =
                    simulateSeconds(integrator, CRUISING, seconds, (int) (30 * 8 * seconds));
            FlightState atTwoFortyTps =
                    simulateSeconds(integrator, CRUISING, seconds, (int) (240 * 8 * seconds));

            double travelled = atThirtyTps.drone().position().length();
            double difference =
                    atThirtyTps
                            .drone()
                            .position()
                            .minus(atTwoFortyTps.drone().position())
                            .length();

            assertTrue(
                    difference < 0.01 * travelled,
                    "moved " + travelled + " but the two tick rates disagreed by " + difference);
        }
    }

    @Nested
    class AttitudeResponse {

        @Test
        void aRollInputRollsRightWithoutTouchingTheOtherAxes() {
            QuadIntegrator integrator = new QuadIntegrator(QuadParameters.DEFAULT);
            ControlInput rollRight = new ControlInput(0.5f, 0.5f, 0f, 0f);

            FlightState after =
                    simulate(
                            integrator,
                            FlightState.restingAt(Vec3.ZERO),
                            rollRight,
                            DEFAULT_SUBSTEP,
                            48);

            assertTrue(after.drone().bodyRates().roll() > 0, "should be rolling right");
            assertEquals(
                    0.0, after.drone().bodyRates().pitch(), 1e-9, "pitch must stay uncommanded");
            assertEquals(0.0, after.drone().bodyRates().yaw(), 1e-9, "yaw must stay uncommanded");
        }

        @Test
        void aPitchForwardInputTipsTheNoseDownAndAcceleratesForward() {
            // The whole point of transmitter pitch convention: stick forward, nose down, drone goes
            // where it is looking.
            QuadIntegrator integrator = new QuadIntegrator(QuadParameters.DEFAULT);
            ControlInput noseDown = new ControlInput(0.6f, 0f, 0.4f, 0f);

            FlightState after =
                    simulate(
                            integrator,
                            FlightState.restingAt(Vec3.ZERO),
                            noseDown,
                            DEFAULT_SUBSTEP,
                            60);

            assertTrue(after.drone().forward().y() < 0, "nose should be down");
            assertTrue(after.drone().velocity().z() < 0, "should be accelerating towards -Z, forward");
        }

        @Test
        void authorityGrowsWithThrottleBecauseThrustGoesWithTheSquareOfTheCommand() {
            // A roll demand that snaps the drone over at mid throttle should merely lean it at
            // idle. The demand is small on purpose: a saturated one pins the collective to mid
            // throttle regardless, which the mixer's own test covers.
            QuadIntegrator integrator = new QuadIntegrator(QuadParameters.DEFAULT);
            FlightState resting = FlightState.restingAt(Vec3.ZERO);

            double atIdle =
                    integrator
                            .step(resting, new ControlInput(0f, 0.1f, 0f, 0f), DEFAULT_SUBSTEP)
                            .drone()
                            .bodyRates()
                            .roll();
            double atMidThrottle =
                    integrator
                            .step(resting, new ControlInput(0.5f, 0.1f, 0f, 0f), DEFAULT_SUBSTEP)
                            .drone()
                            .bodyRates()
                            .roll();

            assertTrue(atIdle > 0, "air mode should leave some authority even at closed throttle");
            assertTrue(
                    atMidThrottle > atIdle * 1.5,
                    "mid-throttle authority " + atMidThrottle + " should dwarf idle " + atIdle);
        }

        @Test
        void angularDragBleedsOffRotationOnceTheSticksCentre() {
            QuadIntegrator integrator = new QuadIntegrator(QuadParameters.DEFAULT);
            FlightState spinning =
                    simulate(
                            integrator,
                            FlightState.restingAt(Vec3.ZERO),
                            new ControlInput(0.5f, 1f, 0f, 0f),
                            DEFAULT_SUBSTEP,
                            120);

            FlightState coasting =
                    simulate(integrator, spinning, ControlInput.NEUTRAL, DEFAULT_SUBSTEP, 120);

            assertTrue(spinning.drone().bodyRates().roll() > 0, "should have built up a roll rate");
            assertTrue(
                    coasting.drone().bodyRates().roll() < spinning.drone().bodyRates().roll(),
                    "centring the sticks should bleed the rate off, not hold it");
        }
    }

    @Nested
    class Purity {

        @Test
        void theSameStateInputAndDtAlwaysGiveTheSameResult() {
            QuadIntegrator integrator = new QuadIntegrator(QuadParameters.DEFAULT);
            FlightState start =
                    simulate(
                            integrator,
                            FlightState.restingAt(new Vec3(3, 70, -12)),
                            new ControlInput(0.7f, -0.3f, 0.2f, 0.1f),
                            DEFAULT_SUBSTEP,
                            37);
            ControlInput input = new ControlInput(0.4f, 0.1f, -0.2f, 0.3f);

            assertEquals(
                    integrator.step(start, input, DEFAULT_SUBSTEP),
                    integrator.step(start, input, DEFAULT_SUBSTEP));
        }

        @Test
        void steppingDoesNotMutateTheStateItWasGiven() {
            QuadIntegrator integrator = new QuadIntegrator(QuadParameters.DEFAULT);
            FlightState start = FlightState.restingAt(new Vec3(1, 2, 3));

            integrator.step(start, new ControlInput(0.8f, 0.2f, 0.2f, 0.2f), DEFAULT_SUBSTEP);

            assertEquals(FlightState.restingAt(new Vec3(1, 2, 3)), start);
        }

        @Test
        void carriesTheControllersMemoryForwardRatherThanRestartingItEveryStep() {
            // The seam that would silently break if someone reconstructed the controller state per
            // step: an integral term that resets each time cannot accumulate, so the drone would
            // never null a standing rate error however long it held the stick.
            QuadIntegrator integrator = new QuadIntegrator(QuadParameters.DEFAULT);
            ControlInput rolling = new ControlInput(0.5f, 0.3f, 0f, 0f);

            FlightState before =
                    simulate(
                            integrator, FlightState.restingAt(Vec3.ZERO), rolling,
                            DEFAULT_SUBSTEP, 80);
            FlightState after = integrator.step(before, rolling, DEFAULT_SUBSTEP);

            assertTrue(
                    after.controller().roll().integral() > 0,
                    "a sustained roll demand should have built up integral term");
            // The derivative term is taken on the measurement, so what the controller remembers is
            // the rate it was handed at the start of the step -- not the one the step produced.
            assertEquals(
                    before.drone().bodyRates().roll(),
                    after.controller().roll().lastRate(),
                    1e-12,
                    "the controller should remember the rate it last measured");
        }
    }

    @Nested
    class Validation {

        @Test
        void rejectsANonPositiveOrNonFiniteStep() {
            QuadIntegrator integrator = new QuadIntegrator(QuadParameters.DEFAULT);
            FlightState state = FlightState.restingAt(Vec3.ZERO);

            assertThrows(
                    IllegalArgumentException.class,
                    () -> integrator.step(state, ControlInput.NEUTRAL, 0));
            assertThrows(
                    IllegalArgumentException.class,
                    () -> integrator.step(state, ControlInput.NEUTRAL, -DEFAULT_SUBSTEP));
            assertThrows(
                    IllegalArgumentException.class,
                    () -> integrator.step(state, ControlInput.NEUTRAL, Double.NaN));
        }

        @Test
        void rejectsMissingParametersGainsOrState() {
            QuadIntegrator integrator = new QuadIntegrator(QuadParameters.DEFAULT);

            assertThrows(IllegalArgumentException.class, () -> new QuadIntegrator(null));
            assertThrows(
                    IllegalArgumentException.class,
                    () -> new QuadIntegrator(QuadParameters.DEFAULT, null));
            assertThrows(
                    IllegalArgumentException.class,
                    () -> integrator.step(null, ControlInput.NEUTRAL, DEFAULT_SUBSTEP));
        }

        @Test
        void defaultsToTheDefaultTuneWhenNoneIsGiven() {
            assertEquals(
                    RatePidGains.DEFAULT, new QuadIntegrator(QuadParameters.DEFAULT).gains());
        }
    }
}
