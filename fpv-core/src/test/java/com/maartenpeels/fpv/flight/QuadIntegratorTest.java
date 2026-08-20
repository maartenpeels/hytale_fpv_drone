package com.maartenpeels.fpv.flight;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.maartenpeels.fpv.control.ControlInput;
import com.maartenpeels.fpv.math.Vec3;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class QuadIntegratorTest {

    /** 30 TPS with 8 substeps — the configured default in {@code FpvConfig}. */
    private static final double DEFAULT_SUBSTEP = 1.0 / (30 * 8);

    private static DroneState simulate(
            QuadIntegrator integrator, DroneState from, ControlInput input, double dt, int steps) {
        DroneState state = from;
        for (int i = 0; i < steps; i++) {
            state = integrator.step(state, input, dt);
        }
        return state;
    }

    /** Runs for a fixed span of simulated time, whatever the step size. */
    private static DroneState simulateSeconds(
            QuadIntegrator integrator, ControlInput input, double seconds, int steps) {
        return simulate(
                integrator, DroneState.restingAt(Vec3.ZERO), input, seconds / steps, steps);
    }

    @Nested
    class Hover {

        @Test
        void holdsPositionAtTheHoverCollectiveBecauseThrustExactlyCancelsGravity() {
            QuadParameters parameters = QuadParameters.DEFAULT;
            QuadIntegrator integrator = new QuadIntegrator(parameters);
            ControlInput hover =
                    new ControlInput((float) parameters.hoverCollective(), 0f, 0f, 0f);

            DroneState after =
                    simulate(
                            integrator,
                            DroneState.restingAt(new Vec3(0, 100, 0)),
                            hover,
                            DEFAULT_SUBSTEP,
                            2400);

            // Ten seconds of hover. The residual is float rounding on the throttle axis, nothing
            // structural, so the tolerance is tight.
            assertEquals(100.0, after.position().y(), 1e-3);
            assertEquals(0.0, after.speed(), 1e-3);
        }

        @Test
        void sinksBelowTheHoverCollectiveAndClimbsAboveIt() {
            QuadParameters parameters = QuadParameters.DEFAULT;
            QuadIntegrator integrator = new QuadIntegrator(parameters);
            float hover = (float) parameters.hoverCollective();

            DroneState sinking =
                    simulate(
                            integrator,
                            DroneState.restingAt(Vec3.ZERO),
                            new ControlInput(hover * 0.5f, 0f, 0f, 0f),
                            DEFAULT_SUBSTEP,
                            240);
            DroneState climbing =
                    simulate(
                            integrator,
                            DroneState.restingAt(Vec3.ZERO),
                            new ControlInput(hover * 1.5f, 0f, 0f, 0f),
                            DEFAULT_SUBSTEP,
                            240);

            assertTrue(sinking.velocity().y() < 0, "should sink but y velocity was "
                    + sinking.velocity().y());
            assertTrue(climbing.velocity().y() > 0, "should climb but y velocity was "
                    + climbing.velocity().y());
        }

        @Test
        void hoveringLevelIntroducesNoRotation() {
            QuadParameters parameters = QuadParameters.DEFAULT;
            QuadIntegrator integrator = new QuadIntegrator(parameters);

            DroneState after =
                    simulate(
                            integrator,
                            DroneState.restingAt(Vec3.ZERO),
                            new ControlInput((float) parameters.hoverCollective(), 0f, 0f, 0f),
                            DEFAULT_SUBSTEP,
                            2400);

            assertEquals(0.0, after.bodyRates().roll(), 1e-12);
            assertEquals(0.0, after.bodyRates().pitch(), 1e-12);
            assertEquals(0.0, after.bodyRates().yaw(), 1e-12);
            assertEquals(1.0, after.thrustAxis().y(), 1e-12);
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

            DroneState after =
                    simulate(
                            integrator,
                            DroneState.restingAt(Vec3.ZERO),
                            ControlInput.NEUTRAL,
                            DEFAULT_SUBSTEP,
                            steps);

            // The closed form of the discrete scheme, not of the continuous one: semi-implicit
            // Euler advances position with the velocity it has *after* the acceleration, giving
            // -g*dt^2*n*(n+1)/2. Asserting that exactly is what proves nothing else is acting.
            double expectedVelocity = -gravity * steps * DEFAULT_SUBSTEP;
            double expectedDrop =
                    -gravity * DEFAULT_SUBSTEP * DEFAULT_SUBSTEP * steps * (steps + 1) / 2.0;

            assertEquals(expectedVelocity, after.velocity().y(), 1e-9);
            assertEquals(expectedDrop, after.position().y(), 1e-9);
            assertEquals(0.0, after.position().x(), 1e-12);
            assertEquals(0.0, after.position().z(), 1e-12);
        }

        @Test
        void staysWithinAStepOfTheContinuousSolution() {
            QuadIntegrator integrator =
                    new QuadIntegrator(QuadParameters.builder().withoutDrag().build());
            double gravity = QuadParameters.DEFAULT_GRAVITY;
            double seconds = 1.0;
            int steps = 240;

            DroneState after = simulateSeconds(integrator, ControlInput.NEUTRAL, seconds, steps);

            // First-order truncation over the whole fall comes to exactly half a step of velocity,
            // so that is the bound worth asserting rather than a round number.
            double continuousDrop = -0.5 * gravity * seconds * seconds;
            double oneStepOfVelocity = gravity * (seconds / steps) * seconds * 0.5;

            assertEquals(continuousDrop, after.position().y(), oneStepOfVelocity * 1.01);
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

            DroneState after =
                    simulate(
                            integrator,
                            DroneState.restingAt(Vec3.ZERO),
                            ControlInput.NEUTRAL,
                            DEFAULT_SUBSTEP,
                            30 * 8 * 30);

            assertEquals(-terminal, after.velocity().y(), 0.01);
        }

        @Test
        void neverOvershootsTerminalVelocity() {
            double quadraticDrag = 0.01;
            QuadParameters parameters =
                    QuadParameters.builder().linearDrag(0).quadraticDrag(quadraticDrag).build();
            QuadIntegrator integrator = new QuadIntegrator(parameters);
            double terminal = Math.sqrt(parameters.gravity() / quadraticDrag);

            DroneState state = DroneState.restingAt(Vec3.ZERO);
            for (int i = 0; i < 30 * 8 * 30; i++) {
                state = integrator.step(state, ControlInput.NEUTRAL, DEFAULT_SUBSTEP);
                assertTrue(
                        state.speed() <= terminal,
                        "speed " + state.speed() + " exceeded terminal " + terminal);
            }
        }

        @Test
        void linearDragAlsoBoundsTheFall() {
            double linearDrag = 0.5;
            QuadParameters parameters =
                    QuadParameters.builder().linearDrag(linearDrag).quadraticDrag(0).build();
            QuadIntegrator integrator = new QuadIntegrator(parameters);

            DroneState after =
                    simulate(
                            integrator,
                            DroneState.restingAt(Vec3.ZERO),
                            ControlInput.NEUTRAL,
                            DEFAULT_SUBSTEP,
                            30 * 8 * 60);

            assertEquals(-parameters.gravity() / linearDrag, after.velocity().y(), 0.01);
        }
    }

    @Nested
    class RateAgnosticism {

        // The tests that protect CLAUDE.md decision 3's escape hatch. If raising World.setTps
        // changed where the drone ends up, "just raise the tick rate" would stop being a config
        // change and start being a re-tune.

        /**
         * A gentle off-axis input. Every axis moves, so thrust direction, drag and rotation all
         * matter — but the demands stay inside the rate tracker's linear region and inside the
         * mixer's unsaturated branch, so the integrand is smooth and the error really is O(dt).
         */
        private static final ControlInput CRUISING = new ControlInput(0.5f, 0.06f, -0.05f, 0.03f);

        @Test
        void landsInTheSamePlaceWhenDtIsHalvedAndStepCountDoubled() {
            QuadIntegrator integrator = new QuadIntegrator(QuadParameters.DEFAULT);
            double seconds = 1.0;

            DroneState coarse = simulateSeconds(integrator, CRUISING, seconds, 240);
            DroneState fine = simulateSeconds(integrator, CRUISING, seconds, 480);

            double travelled = coarse.position().length();
            double difference = coarse.position().minus(fine.position()).length();

            assertTrue(travelled > 5.0, "the test input should actually move the drone");
            assertTrue(
                    difference < 0.01 * travelled,
                    "moved " + travelled + " but the two step sizes disagreed by " + difference);
        }

        @Test
        void convergesAsTheStepShrinksRatherThanMerelyStayingClose() {
            // Staying within a tolerance could just mean the whole thing is insensitive. Halving dt
            // must actually halve the error against a much finer reference.
            QuadIntegrator integrator = new QuadIntegrator(QuadParameters.DEFAULT);
            double seconds = 1.0;

            Vec3 reference = simulateSeconds(integrator, CRUISING, seconds, 15_360).position();
            double coarseError =
                    simulateSeconds(integrator, CRUISING, seconds, 240)
                            .position()
                            .minus(reference)
                            .length();
            double fineError =
                    simulateSeconds(integrator, CRUISING, seconds, 480)
                            .position()
                            .minus(reference)
                            .length();

            assertTrue(
                    fineError < coarseError * 0.6,
                    "halving dt took the error from " + coarseError + " only to " + fineError);
        }

        @Test
        void reachesTheSameAttitudeWhateverTheStepSize() {
            QuadIntegrator integrator = new QuadIntegrator(QuadParameters.DEFAULT);
            double seconds = 1.0;

            Vec3 coarseNose = simulateSeconds(integrator, CRUISING, seconds, 240).forward();
            Vec3 fineNose = simulateSeconds(integrator, CRUISING, seconds, 480).forward();

            assertEquals(0.0, coarseNose.minus(fineNose).length(), 0.005);
        }

        @Test
        void flightIsUnchangedByTheWorldTickRateItIsSubsteppedInside() {
            // 30 TPS x 8 substeps against 240 TPS x 8 -- the actual escape hatch in decision 3.
            QuadIntegrator integrator = new QuadIntegrator(QuadParameters.DEFAULT);
            double seconds = 2.0;

            DroneState atThirtyTps =
                    simulateSeconds(integrator, CRUISING, seconds, (int) (30 * 8 * seconds));
            DroneState atTwoFortyTps =
                    simulateSeconds(integrator, CRUISING, seconds, (int) (240 * 8 * seconds));

            double travelled = atThirtyTps.position().length();
            double difference =
                    atThirtyTps.position().minus(atTwoFortyTps.position()).length();

            // Measured at 0.42% -- 25 cm after 58 m of flight, well inside gate-detection noise.
            assertTrue(
                    difference < 0.01 * travelled,
                    "moved " + travelled + " but the two tick rates disagreed by " + difference);
        }
    }

    @Nested
    class AttitudeResponse {

        @Test
        void aRollInputRollsTowardsTheDemandedRateWithoutTouchingTheOtherAxes() {
            QuadIntegrator integrator = new QuadIntegrator(QuadParameters.DEFAULT);
            ControlInput rollRight = new ControlInput(0.5f, 0.5f, 0f, 0f);

            DroneState after =
                    simulate(
                            integrator,
                            DroneState.restingAt(Vec3.ZERO),
                            rollRight,
                            DEFAULT_SUBSTEP,
                            48);

            double demanded = 0.5 * QuadParameters.DEFAULT.maxRates().roll();
            assertTrue(after.bodyRates().roll() > 0, "should be rolling right");
            assertTrue(
                    after.bodyRates().roll() <= demanded,
                    "should not overshoot the demanded rate on a proportional tracker");
            assertEquals(0.0, after.bodyRates().pitch(), 1e-9, "pitch must stay uncommanded");
            assertEquals(0.0, after.bodyRates().yaw(), 1e-9, "yaw must stay uncommanded");
        }

        @Test
        void aPitchForwardInputTipsTheNoseDownAndAcceleratesForward() {
            // The whole point of transmitter pitch convention: stick forward, nose down, drone goes
            // where it is looking.
            QuadIntegrator integrator = new QuadIntegrator(QuadParameters.DEFAULT);
            ControlInput noseDown = new ControlInput(0.6f, 0f, 0.4f, 0f);

            DroneState after =
                    simulate(
                            integrator,
                            DroneState.restingAt(Vec3.ZERO),
                            noseDown,
                            DEFAULT_SUBSTEP,
                            60);

            assertTrue(after.forward().y() < 0, "nose should be down");
            assertTrue(after.velocity().z() < 0, "should be accelerating towards -Z, forward");
        }

        @Test
        void authorityGrowsWithThrottleBecauseThrustGoesWithTheSquareOfTheCommand() {
            // A roll demand that snaps the drone over at mid throttle should merely lean it at
            // idle. The demand is small on purpose: a saturated one pins the collective to mid
            // throttle regardless, which the mixer's own test covers.
            QuadIntegrator integrator = new QuadIntegrator(QuadParameters.DEFAULT);
            DroneState resting = DroneState.restingAt(Vec3.ZERO);

            double atIdle =
                    integrator
                            .step(resting, new ControlInput(0f, 0.1f, 0f, 0f), DEFAULT_SUBSTEP)
                            .bodyRates()
                            .roll();
            double atMidThrottle =
                    integrator
                            .step(resting, new ControlInput(0.5f, 0.1f, 0f, 0f), DEFAULT_SUBSTEP)
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
            DroneState spinning =
                    simulate(
                            integrator,
                            DroneState.restingAt(Vec3.ZERO),
                            new ControlInput(0.5f, 1f, 0f, 0f),
                            DEFAULT_SUBSTEP,
                            120);

            DroneState coasting =
                    simulate(integrator, spinning, ControlInput.NEUTRAL, DEFAULT_SUBSTEP, 120);

            assertTrue(spinning.bodyRates().roll() > 0, "should have built up a roll rate");
            assertTrue(
                    coasting.bodyRates().roll() < spinning.bodyRates().roll(),
                    "centring the sticks should bleed the rate off, not hold it");
        }
    }

    @Nested
    class Purity {

        @Test
        void theSameStateInputAndDtAlwaysGiveTheSameResult() {
            QuadIntegrator integrator = new QuadIntegrator(QuadParameters.DEFAULT);
            DroneState start =
                    simulate(
                            integrator,
                            DroneState.restingAt(new Vec3(3, 70, -12)),
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
            DroneState start = DroneState.restingAt(new Vec3(1, 2, 3));

            integrator.step(start, new ControlInput(0.8f, 0.2f, 0.2f, 0.2f), DEFAULT_SUBSTEP);

            assertEquals(DroneState.restingAt(new Vec3(1, 2, 3)), start);
        }
    }

    @Nested
    class Validation {

        @Test
        void rejectsANonPositiveOrNonFiniteStep() {
            QuadIntegrator integrator = new QuadIntegrator(QuadParameters.DEFAULT);
            DroneState state = DroneState.restingAt(Vec3.ZERO);

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
        void rejectsMissingParameters() {
            assertThrows(IllegalArgumentException.class, () -> new QuadIntegrator(null));
        }
    }
}
