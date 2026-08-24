package com.maartenpeels.fpv.plugin.input;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.maartenpeels.fpv.control.PilotInputMapper;
import com.maartenpeels.fpv.control.PilotInputMapping;
import com.maartenpeels.fpv.control.PilotInputSample;
import com.maartenpeels.fpv.flight.QuadParameters;
import java.util.UUID;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

/**
 * The map between the netty thread and the world thread: what it accepts, what it drops, and why it
 * cannot grow without bound.
 */
class PilotInputBufferTest {

    private static final PilotInputMapper MAPPER = new PilotInputMapper(
                    PilotInputMapping.DEFAULT, QuadParameters.DEFAULT.hoverCollective());

    private static PilotInputSample forward() {
        return PilotInputSample.lookRelative(0.0, -1.0, 0.0, 0.0);
    }

    @Nested
    class Boundedness {

        @Test
        void dropsInputForAPilotWithNoOpenSlotSoNonFlyingPlayersCostNothing() {
            // Packet 108 arrives many times a second from every connected player. If `offer` created
            // slots on demand, this map would hold one entry per player who has ever moved, forever.
            PilotInputBuffer buffer = new PilotInputBuffer();

            buffer.offer(UUID.randomUUID(), forward());

            assertEquals(0, buffer.size());
        }

        @Test
        void holdsExactlyThePilotsWhoseSlotsWereOpened() {
            PilotInputBuffer buffer = new PilotInputBuffer();
            UUID flying = UUID.randomUUID();
            UUID walking = UUID.randomUUID();

            buffer.open(flying);
            buffer.offer(flying, forward());
            buffer.offer(walking, forward());

            assertEquals(1, buffer.size());
            assertNull(buffer.slotOf(walking));
        }

        @Test
        void closingRemovesTheSlotSoLaterOffersAreDroppedAgain() {
            PilotInputBuffer buffer = new PilotInputBuffer();
            UUID pilot = UUID.randomUUID();
            buffer.open(pilot);

            buffer.close(pilot);
            buffer.offer(pilot, forward());

            assertEquals(0, buffer.size());
            assertNull(buffer.slotOf(pilot));
        }

        @Test
        void closingAPilotWhoIsNotFlyingIsHarmlessBecauseTeardownRunsOnPathsWeDoNotControl() {
            PilotInputBuffer buffer = new PilotInputBuffer();
            buffer.close(UUID.randomUUID());
            assertEquals(0, buffer.size());
        }
    }

    @Nested
    class Slots {

        @Test
        void reopeningGivesAFreshSlotSoALandedFlightsLookMemoryIsNotInheritedByTheNextOne() {
            // A LookTrack carried over from a landing an hour ago would make the first tick of the
            // new flight a full-deflection flick from an angle nobody chose.
            PilotInputBuffer buffer = new PilotInputBuffer();
            UUID pilot = UUID.randomUUID();

            PilotInputSlot first = buffer.open(pilot);
            first.offer(PilotInputSample.lookRelative(0.0, 0.0, 2.5, 0.5));
            first.nextInput(MAPPER, 1.0 / 30.0);
            PilotInputSlot second = buffer.open(pilot);

            assertNotSame(first, second);
            assertTrue(first.track().present(), "the first slot did see a look");
            assertEquals(false, second.track().present(), "the second must start with no memory");
        }

        @Test
        void answersTheSameSlotForTheSamePilotSoATickAndAPacketAgreeOnWhereInputGoes() {
            PilotInputBuffer buffer = new PilotInputBuffer();
            UUID pilot = UUID.randomUUID();

            PilotInputSlot opened = buffer.open(pilot);

            assertSame(opened, buffer.slotOf(pilot));
        }

        @Test
        void routesAnOfferToTheRightPilotsSlot() {
            PilotInputBuffer buffer = new PilotInputBuffer();
            UUID one = UUID.randomUUID();
            UUID two = UUID.randomUUID();
            buffer.open(one);
            PilotInputSlot slotTwo = buffer.open(two);

            buffer.offer(one, forward());

            assertNull(slotTwo.peekPending(), "the other pilot's slot must be untouched");
        }
    }

    @Nested
    class AcrossThreads {

        @Test
        void aSampleOfferedFromAnotherThreadIsVisibleToTheDrainingThread() throws Exception {
            // The real writer is the netty thread. This does not prove memory-model correctness --
            // that comes from the AtomicReference -- but it does prove the class does not, say, bind
            // itself to a constructing thread the way Store does, which would fail here immediately.
            PilotInputBuffer buffer = new PilotInputBuffer();
            UUID pilot = UUID.randomUUID();
            buffer.open(pilot);

            CountDownLatch offered = new CountDownLatch(1);
            Thread netty = new Thread(() -> {
                buffer.offer(pilot, forward());
                offered.countDown();
            }, "fake-netty");
            netty.start();
            assertTrue(offered.await(5, TimeUnit.SECONDS), "the offering thread should have finished");
            netty.join();

            float throttle = buffer.slotOf(pilot).nextInput(MAPPER, 1.0 / 30.0).throttle();
            assertTrue(throttle > 0.9f, "full forward should read as full throttle, was " + throttle);
        }
    }
}
