package com.maartenpeels.fpv.plugin.input;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;

import com.hypixel.hytale.protocol.Direction;
import com.hypixel.hytale.protocol.Packet;
import com.hypixel.hytale.protocol.Position;
import com.hypixel.hytale.protocol.packets.camera.SetServerCamera;
import com.hypixel.hytale.protocol.packets.player.ClientMovement;
import com.maartenpeels.fpv.control.PilotInputSample;
import java.util.UUID;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

/**
 * The watcher's counters — #49's netty-side instrument.
 *
 * <p>Tested through the package-private {@code accept(UUID, Packet)} seam rather than the
 * {@code PlayerRef} overload, because the harness cannot construct a {@code PlayerRef}. That is the
 * same constraint that let #47's key-mismatch hypothesis survive four flights, and the reason the
 * seam exists: everything with behaviour is on this side of it.
 */
class ClientMovementWatcherTest {

    private static ClientMovement movement() {
        ClientMovement packet = new ClientMovement();
        packet.wishMovement = new Position(0.0, 0.0, -1.0);
        packet.lookOrientation = new Direction(0f, 0f, 0f);
        return packet;
    }

    @Nested
    class Counting {

        @Test
        void countsEveryInboundPacketSoAWatcherOutsideThePipelineIsDistinguishable() {
            // inbound == 0 is the only evidence that separates "registered but not in the pipeline"
            // from "in the pipeline but the client sends no 108". Both look like dead input.
            PilotInputBuffer buffer = new PilotInputBuffer();
            ClientMovementWatcher watcher = new ClientMovementWatcher(buffer);

            watcher.accept(UUID.randomUUID(), new SetServerCamera());
            watcher.accept(UUID.randomUUID(), movement());

            assertEquals(2, watcher.inboundPackets());
            assertEquals(1, watcher.movementPackets());
        }

        @Test
        void doesNotCountANonMovementPacketAsMovement() {
            PilotInputBuffer buffer = new PilotInputBuffer();
            ClientMovementWatcher watcher = new ClientMovementWatcher(buffer);

            watcher.accept(UUID.randomUUID(), new SetServerCamera());

            assertEquals(0, watcher.movementPackets());
            assertEquals(0, watcher.watcherErrors());
        }

        @Test
        void countsAMovementPacketFromANullPlayerAsInboundButNotAsMovement() {
            // A null PlayerRef reaches accept as a null UUID. It must not be mistaken for input, and
            // it must not be mistaken for an error either -- it is neither.
            PilotInputBuffer buffer = new PilotInputBuffer();
            ClientMovementWatcher watcher = new ClientMovementWatcher(buffer);

            watcher.accept((UUID) null, movement());

            assertEquals(1, watcher.inboundPackets());
            assertEquals(0, watcher.movementPackets());
            assertEquals(0, watcher.watcherErrors());
        }
    }

    @Nested
    class Errors {

        /** A buffer that fails the way a broken watcher would: on every offer. */
        private static final class ThrowingBuffer extends PilotInputBuffer {
            @Override
            public void offer(UUID pilotId, PilotInputSample sample) {
                throw new IllegalStateException("boom");
            }
        }

        @Test
        void countsAThrowSoAWatcherFailingEveryPacketIsNotMistakenForOneThatNeverRan() {
            // PacketAdapters swallows and logs at SEVERE, so before #49 a watcher throwing on every
            // packet and a watcher never invoked produced identical evidence: all counters zero.
            ClientMovementWatcher watcher = new ClientMovementWatcher(new ThrowingBuffer());

            assertThrows(
                    IllegalStateException.class,
                    () -> watcher.accept(UUID.randomUUID(), movement()));

            assertEquals(1, watcher.watcherErrors());
            assertEquals(1, watcher.inboundPackets());
        }

        @Test
        void rethrowsSoPacketAdaptersStillLogsTheCauseAtSevere() {
            // Counting must not become swallowing: the SEVERE log is the only place the stack trace
            // survives, and a silently-counted error would be a worse instrument than none.
            ClientMovementWatcher watcher = new ClientMovementWatcher(new ThrowingBuffer());

            IllegalStateException thrown = assertThrows(
                    IllegalStateException.class,
                    () -> watcher.accept(UUID.randomUUID(), movement()));

            assertNotNull(thrown);
            assertEquals("boom", thrown.getMessage());
        }
    }

    @Nested
    class Routing {

        @Test
        void routesToTheSlotOpenedOnTheSameUuid() {
            PilotInputBuffer buffer = new PilotInputBuffer();
            ClientMovementWatcher watcher = new ClientMovementWatcher(buffer);
            UUID pilot = UUID.randomUUID();
            buffer.open(pilot);

            watcher.accept(pilot, movement());

            assertEquals(1, buffer.offersAccepted());
            assertEquals(0, buffer.offersDropped());
        }

        @Test
        void dropsAndCountsWhenTheSlotWasOpenedOnADifferentUuid() {
            // The exact shape of #47's eliminated hypothesis. Kept as a test because the reason it
            // was invisible -- offer writing existing slots only -- is still the design.
            PilotInputBuffer buffer = new PilotInputBuffer();
            ClientMovementWatcher watcher = new ClientMovementWatcher(buffer);
            buffer.open(UUID.randomUUID());

            watcher.accept(UUID.randomUUID(), movement());

            assertEquals(0, buffer.offersAccepted());
            assertEquals(1, buffer.offersDropped());
        }
    }

    /** Guards the assumption the seam rests on: the packet-type check, not a cast. */
    @Nested
    class PacketTyping {

        @Test
        void ignoresAnUnrelatedPacketWithoutThrowing() {
            PilotInputBuffer buffer = new PilotInputBuffer();
            ClientMovementWatcher watcher = new ClientMovementWatcher(buffer);
            UUID pilot = UUID.randomUUID();
            buffer.open(pilot);

            Packet unrelated = new SetServerCamera();
            watcher.accept(pilot, unrelated);

            assertEquals(0, buffer.offersAccepted());
            assertEquals(0, watcher.watcherErrors());
        }
    }
}
