package com.maartenpeels.fpv.flight;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import com.maartenpeels.fpv.math.Quat;
import com.maartenpeels.fpv.math.Vec3;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class FlightStateTest {

    @Nested
    class Launching {

        @Test
        void restingAtIsLevelStationaryAndCarriesNoCorrection() {
            FlightState launched = FlightState.restingAt(new Vec3(10, 64, -3));

            assertEquals(DroneState.restingAt(new Vec3(10, 64, -3)), launched.drone());
            assertEquals(RatePidState.ZERO, launched.controller());
        }

        @Test
        void beginningPrimesTheControllerForARotationAlreadyUnderway() {
            // The reason this factory exists rather than callers reaching for ZERO: a drone handed
            // to the integrator mid-rotation with an unprimed controller gets hit with a phantom
            // derivative kick on its first step. RatePidTest measures how large.
            BodyRates spinning = new BodyRates(4, -1, 2);
            DroneState tumbling =
                    new DroneState(Vec3.ZERO, Vec3.ZERO, Quat.IDENTITY, spinning);

            FlightState begun = FlightState.beginning(tumbling);

            assertEquals(RatePidState.at(spinning), begun.controller());
            assertEquals(tumbling, begun.drone());
        }

        @Test
        void beginningFromRestAgreesWithRestingAt() {
            assertEquals(
                    FlightState.restingAt(new Vec3(1, 2, 3)),
                    FlightState.beginning(DroneState.restingAt(new Vec3(1, 2, 3))));
        }
    }

    @Nested
    class Validation {

        @Test
        void rejectsEitherHalfBeingMissing() {
            assertThrows(
                    IllegalArgumentException.class,
                    () -> new FlightState(null, RatePidState.ZERO));
            assertThrows(
                    IllegalArgumentException.class,
                    () -> new FlightState(DroneState.restingAt(Vec3.ZERO), null));
            assertThrows(IllegalArgumentException.class, () -> FlightState.beginning(null));
        }
    }
}
