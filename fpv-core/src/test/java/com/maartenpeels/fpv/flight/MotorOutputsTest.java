package com.maartenpeels.fpv.flight;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class MotorOutputsTest {

    private static final double TOLERANCE = 1e-12;

    @Nested
    class Thrusts {

        @Test
        void thrustIsTheSquareOfTheCommandSoHoverLandsNearAThirdOfStick() {
            MotorThrusts thrusts = new MotorOutputs(0.5, 0.25, 1.0, 0.0).thrusts();

            assertEquals(0.25, thrusts.frontLeft(), TOLERANCE);
            assertEquals(0.0625, thrusts.frontRight(), TOLERANCE);
            assertEquals(1.0, thrusts.rearLeft(), TOLERANCE);
            assertEquals(0.0, thrusts.rearRight(), TOLERANCE);
        }

        @Test
        void fullCommandOnEveryMotorIsFullThrust() {
            assertEquals(1.0, new MotorOutputs(1, 1, 1, 1).thrusts().collective(), TOLERANCE);
        }

        @Test
        void idleCommandsProduceNoThrustAtAll() {
            assertEquals(0.0, MotorOutputs.IDLE.thrusts().collective(), TOLERANCE);
        }
    }
}
