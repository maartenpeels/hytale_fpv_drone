package com.maartenpeels.fpv.plugin.input;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.hypixel.hytale.protocol.Direction;
import com.hypixel.hytale.protocol.Position;
import com.hypixel.hytale.protocol.packets.player.ClientMovement;
import com.maartenpeels.fpv.control.ControlInput;
import com.maartenpeels.fpv.control.LookTrack;
import com.maartenpeels.fpv.control.PilotInputMapper;
import com.maartenpeels.fpv.control.PilotInputMapping;
import com.maartenpeels.fpv.control.PilotInputSample;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

/**
 * Pins what the packet-boundary shell is allowed to be: field copies, nulls to {@code NaN}, and no
 * arithmetic.
 *
 * <p>No {@link com.maartenpeels.fpv.plugin.ecs.HytaleEcsHarness} needed. {@code ClientMovement},
 * {@code Position} and {@code Direction} are plain public-field protocol objects with no static
 * initialisers, no component registry and no world behind them, so the adapter is directly testable
 * on the server jar alone.
 */
class ClientMovementAdapterTest {

    private static final double TOLERANCE = 1e-5;
    private static final double DT = 1.0 / 30.0;

    private static ClientMovement packet(Position wishMovement, Direction lookOrientation) {
        ClientMovement packet = new ClientMovement();
        packet.wishMovement = wishMovement;
        packet.lookOrientation = lookOrientation;
        return packet;
    }

    @Nested
    class FieldCopies {

        @Test
        void copiesTheHorizontalWishComponentsVerbatimWithoutConvertingAnything() {
            PilotInputSample sample =
                    ClientMovementAdapter.sample(
                            packet(new Position(0.25, 9.0, -0.75), new Direction(1.5f, -0.5f, 0f)));

            assertEquals(0.25, sample.wishX());
            assertEquals(-0.75, sample.wishZ());
            assertEquals(1.5, sample.lookYaw(), TOLERANCE);
            assertEquals(-0.5, sample.lookPitch(), TOLERANCE);
        }

        @Test
        void treatsTheLookYawAsTheWishFrameBecauseHytaleRotatesTheStickVectorByTheHead() {
            PilotInputSample sample =
                    ClientMovementAdapter.sample(
                            packet(new Position(0.0, 0.0, -1.0), new Direction(1.5f, 0f, 0f)));

            assertEquals(sample.lookYaw(), sample.wishFrameYaw());
        }

        @Test
        void takesAnExplicitFrameYawForACameraThatPinsTheMovementRotation() {
            PilotInputSample sample =
                    ClientMovementAdapter.sample(
                            packet(new Position(0.0, 0.0, -1.0), new Direction(1.5f, 0f, 0f)), 0.0);

            assertEquals(0.0, sample.wishFrameYaw());
            assertEquals(1.5, sample.lookYaw(), TOLERANCE);
        }
    }

    @Nested
    class MissingFields {

        @Test
        void readsAMissingWishMovementAsACentredLeftStick() {
            PilotInputSample sample =
                    ClientMovementAdapter.sample(packet(null, new Direction(1.5f, -0.5f, 0f)));

            assertEquals(0.0, sample.wishX());
            assertEquals(0.0, sample.wishZ());
            assertTrue(sample.hasLook());
        }

        @Test
        void readsAMissingLookOrientationAsAbsentRatherThanAsZeroDegrees() {
            PilotInputSample sample =
                    ClientMovementAdapter.sample(packet(new Position(0.0, 0.0, -1.0), null));

            assertFalse(sample.hasLook());
            assertFalse(Double.isFinite(sample.wishFrameYaw()));
        }

        @Test
        void readsAnEntirelyEmptyPacketAsSayingNothing() {
            PilotInputSample sample = ClientMovementAdapter.sample(new ClientMovement());

            assertEquals(PilotInputSample.EMPTY, sample);
        }

        @Test
        void survivesANullPacket() {
            assertEquals(PilotInputSample.EMPTY, ClientMovementAdapter.sample(null));
            assertEquals(PilotInputSample.EMPTY, ClientMovementAdapter.sample(null, 0.0));
        }
    }

    @Nested
    class EndToEnd {

        @Test
        void turnsAFullForwardKeypressIntoFullThrottleAtAnyHeading() {
            PilotInputMapper mapper = new PilotInputMapper(PilotInputMapping.DEFAULT);
            float heading = 2.25f;
            Position worldWish =
                    new Position(-Math.sin(heading), 0.0, -Math.cos(heading));

            ControlInput input =
                    mapper.map(
                                    LookTrack.UNSET,
                                    ClientMovementAdapter.sample(
                                            packet(worldWish, new Direction(heading, 0f, 0f))),
                                    DT)
                            .input();

            assertEquals(1f, input.throttle(), TOLERANCE);
            assertEquals(0f, input.yaw(), TOLERANCE);
        }

        @Test
        void clampsRatherThanThrowingOnAPacketFullOfGarbage() {
            PilotInputMapper mapper = new PilotInputMapper(PilotInputMapping.DEFAULT);
            ClientMovement hostile =
                    packet(
                            new Position(Double.NaN, Double.NaN, Double.POSITIVE_INFINITY),
                            new Direction(Float.NaN, Float.NaN, Float.NaN));

            ControlInput input =
                    mapper.map(LookTrack.at(0.0, 0.0), ClientMovementAdapter.sample(hostile), DT)
                            .input();

            assertEquals(0.5f, input.throttle(), TOLERANCE);
            assertTrue(input.sticksCentred());
        }
    }
}
