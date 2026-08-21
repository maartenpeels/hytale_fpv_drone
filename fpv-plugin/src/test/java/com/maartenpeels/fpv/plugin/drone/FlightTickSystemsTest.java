package com.maartenpeels.fpv.plugin.drone;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.hypixel.hytale.component.Ref;
import com.hypixel.hytale.component.RemoveReason;
import com.hypixel.hytale.component.Store;
import com.hypixel.hytale.logger.backend.HytaleLoggerBackend;
import com.hypixel.hytale.math.vector.Rotation3f;
import com.hypixel.hytale.server.core.modules.entity.component.HeadRotation;
import com.hypixel.hytale.server.core.modules.entity.component.TransformComponent;
import com.hypixel.hytale.server.core.universe.world.storage.EntityStore;
import com.maartenpeels.fpv.control.PilotInputSample;
import com.maartenpeels.fpv.flight.FlightTick;
import com.maartenpeels.fpv.flight.QuadIntegrator;
import com.maartenpeels.fpv.flight.QuadParameters;
import com.maartenpeels.fpv.flight.SubstepListener;
import com.maartenpeels.fpv.plugin.ecs.HytaleEcsHarness;
import java.util.List;
import java.util.UUID;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.logging.Level;
import java.util.logging.LogRecord;
import org.joml.Vector3d;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

/**
 * The ECS half of #23: seeding a drone's flight state, owning its input slot, and getting the
 * simulated position onto the entity.
 *
 * <p>The substep arithmetic is {@code FlightTickTest}'s subject in {@code :fpv-core} and the input
 * interval is {@code PilotInputSlotTest}'s; both are plain JUnit. What is left for the harness is the
 * wiring, which is exactly the split the ECS conventions are meant to produce.
 */
class FlightTickSystemsTest {

    private static final Vector3d SPAWN = new Vector3d(10.0, 70.0, -20.0);
    private static final int SUBSTEPS = 8;
    private static final int TPS = 30;
    private static final double TICK_SECONDS = 1.0 / TPS;
    private static final float TICK_DT = (float) TICK_SECONDS;

    private static FlightTick flightTick() {
        return new FlightTick(new QuadIntegrator(QuadParameters.DEFAULT), SUBSTEPS);
    }

    /** Full forward on the wish vector at yaw 0 — forward is −Z, so this is full throttle. */
    private static PilotInputSample fullThrottle() {
        return PilotInputSample.lookRelative(0.0, -1.0, 0.0, 0.0);
    }

    /** No forward component at all — the bipolar remap makes this throttle closed. */
    private static PilotInputSample throttleClosed() {
        return PilotInputSample.lookRelative(0.0, 1.0, 0.0, 0.0);
    }

    private static DroneFlight flightOf(Store<EntityStore> store, DroneTestFixture f, Ref<EntityStore> drone) {
        return store.getComponent(drone, f.types().droneFlight());
    }

    private static TransformComponent transformOf(
            Store<EntityStore> store, DroneTestFixture f, Ref<EntityStore> drone) {
        return store.getComponent(drone, f.types().transform());
    }

    @Nested
    class SeedingFlightState {

        @Test
        void aLaunchedDroneCarriesFlightStateBecauseOtherwiseItWouldHangMotionless() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture f = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = f.newPilot(store);

                Ref<EntityStore> drone =
                        f.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);

                assertNotNull(flightOf(store, f, drone));
            }
        }

        @Test
        void startsAtRestAtTheSpawnPositionSoTheSimAndTheEntityAgreeOnTickZero() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture f = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = f.newPilot(store);

                Ref<EntityStore> drone =
                        f.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);
                var position = flightOf(store, f, drone).getState().drone().position();

                assertEquals(SPAWN.x, position.x(), 1e-9);
                assertEquals(SPAWN.y, position.y(), 1e-9);
                assertEquals(SPAWN.z, position.z(), 1e-9);
                assertEquals(0.0, flightOf(store, f, drone).getState().drone().speed(), 1e-9);
            }
        }

        @Test
        void facesWhereThePilotFacedRatherThanYawZero() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture f = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = f.newPilot(store);

                Ref<EntityStore> drone = f.sessions()
                        .launch(store, pilot, SPAWN, new Rotation3f(0f, 1.1f, 0f), null);
                Rotation3f attitude =
                        DroneRotation.toRotation(flightOf(store, f, drone).getState().drone().orientation());

                assertEquals(1.1f, attitude.yaw(), 1e-5);
            }
        }

        @Test
        void dropsThePilotsPitchAndRollSoADroneNeverArmsAlreadyBanked() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture f = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = f.newPilot(store);

                Ref<EntityStore> drone = f.sessions()
                        .launch(store, pilot, SPAWN, new Rotation3f(0.6f, 1.1f, -0.8f), null);
                Rotation3f attitude =
                        DroneRotation.toRotation(flightOf(store, f, drone).getState().drone().orientation());

                assertEquals(0f, attitude.pitch(), 1e-5);
                assertEquals(0f, attitude.roll(), 1e-5);
            }
        }

        @Test
        void namesThePilotByUuidSoTheSlotCanBeClosedAfterThePilotRefGoesInvalid() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture f = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = f.newPilot(store);

                Ref<EntityStore> drone =
                        f.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);

                assertEquals(f.uuidOf(store, pilot), flightOf(store, f, drone).getPilotId());
            }
        }

        @Test
        void isSkippedForAPilotWithNoUuidBecauseNoInputCouldEverBeRoutedToIt() {
            // Not a player. Player entities always carry UUIDComponent, so this is a drone spawned for
            // something that cannot supply input; a drone that exists and does not fly is the honest
            // outcome, and it must not take the whole launch down with it.
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture f = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = f.newPilotWithoutUuid(store);

                Ref<EntityStore> drone =
                        f.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);

                assertNotNull(drone, "the launch itself must still succeed");
                assertNull(flightOf(store, f, drone));
                assertEquals(0, f.inputs().size());
            }
        }
    }

    @Nested
    class InputSlotLifetime {

        @Test
        void opensOnLaunchSoPacketsArrivingImmediatelyAreNotDropped() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture f = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = f.newPilot(store);

                f.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);

                assertNotNull(f.inputs().slotOf(f.uuidOf(store, pilot)));
            }
        }

        @Test
        void closesOnLandSoALandedPilotsPacketsStopBeingCollected() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture f = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = f.newPilot(store);
                UUID pilotId = f.uuidOf(store, pilot);
                f.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);

                f.sessions().land(store, pilot);

                assertNull(f.inputs().slotOf(pilotId));
                assertEquals(0, f.inputs().size());
            }
        }

        @Test
        void closesWhenThePilotEntityIsRemovedWhichIsTheDisconnectAndWorldSwitchPath() {
            // The pilot's own Ref is already invalid by the time the drone's removal callback runs, so
            // this only works because DroneFlight caches the UUID on the drone.
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture f = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = f.newPilot(store);
                UUID pilotId = f.uuidOf(store, pilot);
                f.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);

                store.removeEntity(pilot, RemoveReason.UNLOAD);

                assertEquals(0, f.inputs().size(), "a disconnect mid-flight must not leak a slot");
                assertNull(f.inputs().slotOf(pilotId));
            }
        }

        @Test
        void closesWhenTheDroneDiesByAnyOtherRoute() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture f = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = f.newPilot(store);
                Ref<EntityStore> drone =
                        f.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);

                store.removeEntity(drone, RemoveReason.REMOVE);

                assertEquals(0, f.inputs().size());
            }
        }

        @Test
        void relaunchingGivesAFreshSlotRatherThanTheOldFlightsLookMemory() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture f = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = f.newPilot(store);
                UUID pilotId = f.uuidOf(store, pilot);

                f.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);
                f.inputs().offer(pilotId, PilotInputSample.lookRelative(0.0, 0.0, 2.5, 0.5));
                f.inputs().slotOf(pilotId).nextInput(
                        new com.maartenpeels.fpv.control.PilotInputMapper(
                                com.maartenpeels.fpv.control.PilotInputMapping.DEFAULT),
                        TICK_SECONDS);
                f.sessions().land(store, pilot);
                f.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);

                assertFalse(
                        f.inputs().slotOf(pilotId).track().present(),
                        "a new flight must not inherit the last one's look memory");
            }
        }
    }

    @Nested
    class Ticking {

        @Test
        void fullThrottleClimbsAndTheNewPositionReachesTheEntity() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture f = DroneTestFixture.install(harness);
                f.installFlightTick(harness, flightTick(), TICK_SECONDS, null);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = f.newPilot(store);
                Ref<EntityStore> drone =
                        f.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);
                f.inputs().offer(f.uuidOf(store, pilot), fullThrottle());

                store.tick(TICK_DT);

                assertTrue(
                        flightOf(store, f, drone).getState().drone().position().y() > SPAWN.y,
                        "full throttle should climb");
                assertEquals(
                        flightOf(store, f, drone).getState().drone().position().y(),
                        transformOf(store, f, drone).getPosition().y,
                        1e-9,
                        "the entity's transform must carry the simulated position");
            }
        }

        @Test
        void aClosedThrottleFallsBecauseGravityIsTheOnlyOtherForce() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture f = DroneTestFixture.install(harness);
                f.installFlightTick(harness, flightTick(), TICK_SECONDS, null);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = f.newPilot(store);
                Ref<EntityStore> drone =
                        f.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);
                f.inputs().offer(f.uuidOf(store, pilot), throttleClosed());

                store.tick(TICK_DT);

                assertTrue(
                        transformOf(store, f, drone).getPosition().y < SPAWN.y,
                        "a closed throttle should sink");
            }
        }

        @Test
        void keepsFlyingWhenThePilotSentNothingBecauseNoPacketIsNotAnError() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture f = DroneTestFixture.install(harness);
                f.installFlightTick(harness, flightTick(), TICK_SECONDS, null);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = f.newPilot(store);
                Ref<EntityStore> drone =
                        f.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);

                store.tick(TICK_DT);
                store.tick(TICK_DT);

                // Mid-throttle against a thrust-to-weight of 8 is well above hover, so it climbs. The
                // assertion that matters is that it ticked at all rather than throwing or freezing.
                assertNotNull(flightOf(store, f, drone));
                assertTrue(flightOf(store, f, drone).getState().drone().position().y() != SPAWN.y);
            }
        }

        @Test
        void carriesStateAcrossTicksRatherThanRestartingFromTheSpawn() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture f = DroneTestFixture.install(harness);
                f.installFlightTick(harness, flightTick(), TICK_SECONDS, null);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = f.newPilot(store);
                Ref<EntityStore> drone =
                        f.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);
                UUID pilotId = f.uuidOf(store, pilot);

                f.inputs().offer(pilotId, fullThrottle());
                store.tick(TICK_DT);
                double afterOne = flightOf(store, f, drone).getState().drone().position().y();
                f.inputs().offer(pilotId, fullThrottle());
                store.tick(TICK_DT);
                double afterTwo = flightOf(store, f, drone).getState().drone().position().y();

                // Accelerating from rest, so the second tick must cover strictly more ground than the
                // first -- which is only true if velocity survived the tick boundary.
                assertTrue(
                        afterTwo - afterOne > afterOne - SPAWN.y,
                        "velocity must carry over: tick 1 gained "
                                + (afterOne - SPAWN.y)
                                + ", tick 2 gained "
                                + (afterTwo - afterOne));
            }
        }

        @Test
        void writesTheAttitudeOntoBothTheBodyAndTheHeadSoTheTrackerSendsOneConsistentOrientation() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture f = DroneTestFixture.install(harness);
                f.installFlightTick(harness, flightTick(), TICK_SECONDS, null);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = f.newPilot(store);
                Ref<EntityStore> drone = f.sessions()
                        .launch(store, pilot, SPAWN, new Rotation3f(0f, 1.1f, 0f), null);

                store.tick(TICK_DT);

                Rotation3f body = transformOf(store, f, drone).getRotation();
                HeadRotation head = store.getComponent(drone, f.types().headRotation());
                assertNotNull(head);
                assertEquals(body.yaw(), head.getRotation().yaw(), 1e-6);
                assertEquals(body.pitch(), head.getRotation().pitch(), 1e-6);
                assertEquals(body.roll(), head.getRotation().roll(), 1e-6);
            }
        }

        @Test
        void runsTheSubstepListenerOncePerSubstepPerDronePerTick() {
            // #21's collision hook, verified through the real system rather than only through
            // FlightTick: this is what proves the system actually passes the listener down.
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture f = DroneTestFixture.install(harness);
                AtomicInteger calls = new AtomicInteger();
                SubstepListener counting = (before, after, dt) -> {
                    calls.incrementAndGet();
                    return after;
                };
                f.installFlightTick(harness, flightTick(), TICK_SECONDS, counting);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = f.newPilot(store);
                f.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);

                store.tick(TICK_DT);

                assertEquals(SUBSTEPS, calls.get());
            }
        }

        @Test
        void isUnaffectedByTheWallClockDtBecauseTheSubstepLengthComesFromConfig() {
            // A lag spike hands `tick` a much larger dt. If the system used it, the drone would jump
            // -- which is the tunnelling #21 has to worry about. The configured tick length is what
            // makes a lag spike slow the sim down instead.
            //
            // The 2-second tick below also trips the tick-rate mismatch warning, so this test prints a
            // WARNING. That is the behaviour TickRateMismatchWarning asserts, not a defect here.
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture f = DroneTestFixture.install(harness);
                f.installFlightTick(harness, flightTick(), TICK_SECONDS, null);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = f.newPilot(store);
                Ref<EntityStore> drone =
                        f.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);
                UUID pilotId = f.uuidOf(store, pilot);

                f.inputs().offer(pilotId, fullThrottle());
                store.tick(TICK_DT);
                double normal = flightOf(store, f, drone).getState().drone().position().y() - SPAWN.y;

                // A second drone, same everything, handed a two-second tick.
                Ref<EntityStore> other = f.newPilot(store);
                Ref<EntityStore> otherDrone =
                        f.sessions().launch(store, other, SPAWN, Rotation3f.IDENTITY, null);
                f.inputs().offer(f.uuidOf(store, other), fullThrottle());
                store.tick(2.0f);
                double afterSpike =
                        flightOf(store, f, otherDrone).getState().drone().position().y() - SPAWN.y;

                assertEquals(normal, afterSpike, Math.abs(normal) * 1e-6);
            }
        }

        @Test
        void tickingWithNoDroneAtAllIsHarmless() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture f = DroneTestFixture.install(harness);
                f.installFlightTick(harness, flightTick(), TICK_SECONDS, null);

                harness.store().tick(TICK_DT);
            }
        }

        @Test
        void fliesTwoDronesIndependentlyOnTheirOwnPilotsInput() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture f = DroneTestFixture.install(harness);
                f.installFlightTick(harness, flightTick(), TICK_SECONDS, null);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> climbing = f.newPilot(store);
                Ref<EntityStore> sinking = f.newPilot(store);
                Ref<EntityStore> climber =
                        f.sessions().launch(store, climbing, SPAWN, Rotation3f.IDENTITY, null);
                Ref<EntityStore> sinker =
                        f.sessions().launch(store, sinking, SPAWN, Rotation3f.IDENTITY, null);

                f.inputs().offer(f.uuidOf(store, climbing), fullThrottle());
                f.inputs().offer(f.uuidOf(store, sinking), throttleClosed());
                store.tick(TICK_DT);

                assertTrue(flightOf(store, f, climber).getState().drone().position().y() > SPAWN.y);
                assertTrue(flightOf(store, f, sinker).getState().drone().position().y() < SPAWN.y);
            }
        }
    }

    /**
     * The guard on the one real cost of a fixed tick length: if {@code WorldTps} disagrees with the
     * world's actual rate, the drone flies at the wrong speed and nothing else says so.
     *
     * <p>Assertable because {@code HytaleLoggerBackend.subscribe} takes a list the backend appends
     * every {@code LogRecord} to (`logger/backend/HytaleLoggerBackend.java:97-107`). Worth doing rather
     * than trusting the log line exists: a warning that never fires is indistinguishable from no
     * warning at all, and this one is the only thing standing between an operator and a silently
     * quarter-speed drone.
     */
    @Nested
    class TickRateMismatchWarning {

        private static List<LogRecord> captureLogs() {
            CopyOnWriteArrayList<LogRecord> records = new CopyOnWriteArrayList<>();
            HytaleLoggerBackend.subscribe(records);
            return records;
        }

        private static long warnings(List<LogRecord> records) {
            return records.stream()
                    .filter(record -> record.getLevel().intValue() >= Level.WARNING.intValue())
                    .filter(record -> String.valueOf(record.getMessage()).contains("FPV flight is simulating"))
                    .count();
        }

        @Test
        void firesWhenTheObservedTickIsNowhereNearTheConfiguredOne() {
            CopyOnWriteArrayList<LogRecord> records = new CopyOnWriteArrayList<>();
            HytaleLoggerBackend.subscribe(records);
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture f = DroneTestFixture.install(harness);
                f.installFlightTick(harness, flightTick(), TICK_SECONDS, null);
                Store<EntityStore> store = harness.store();
                f.sessions().launch(store, f.newPilot(store), SPAWN, Rotation3f.IDENTITY, null);

                // Configured for 30 TPS, ticked as if the world were running at 4.
                store.tick(1.0f / 4.0f);

                assertEquals(1, warnings(records));
            } finally {
                HytaleLoggerBackend.unsubscribe(records);
            }
        }

        @Test
        void reportsSlowerThanRealTimeWhenTheWorldTicksSlowerThanConfigured() {
            // The direction of the ratio, pinned. 4 TPS delivering 1/30 s of simulation per tick is
            // 4/30 = 0.13x real speed -- the drone crawls. The first draft of this warning printed the
            // reciprocal (7.5x) and would have sent an operator looking for the wrong problem.
            CopyOnWriteArrayList<LogRecord> records = new CopyOnWriteArrayList<>();
            HytaleLoggerBackend.subscribe(records);
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture f = DroneTestFixture.install(harness);
                f.installFlightTick(harness, flightTick(), TICK_SECONDS, null);
                Store<EntityStore> store = harness.store();
                f.sessions().launch(store, f.newPilot(store), SPAWN, Rotation3f.IDENTITY, null);

                store.tick(1.0f / 4.0f);

                String message = String.valueOf(records.get(records.size() - 1).getMessage());
                assertTrue(
                        message.contains("0.13x real speed"),
                        "a slow world must read as slow, but said: " + message);
            } finally {
                HytaleLoggerBackend.unsubscribe(records);
            }
        }

        @Test
        void reportsFasterThanRealTimeWhenTheWorldTicksFasterThanConfigured() {
            // The other direction: WorldTps left at 30 after `/world tps 120` means four simulated
            // seconds per real second.
            CopyOnWriteArrayList<LogRecord> records = new CopyOnWriteArrayList<>();
            HytaleLoggerBackend.subscribe(records);
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture f = DroneTestFixture.install(harness);
                f.installFlightTick(harness, flightTick(), TICK_SECONDS, null);
                Store<EntityStore> store = harness.store();
                f.sessions().launch(store, f.newPilot(store), SPAWN, Rotation3f.IDENTITY, null);

                store.tick(1.0f / 120.0f);

                String message = String.valueOf(records.get(records.size() - 1).getMessage());
                assertTrue(
                        message.contains("4.00x real speed"),
                        "a fast world must read as fast, but said: " + message);
            } finally {
                HytaleLoggerBackend.unsubscribe(records);
            }
        }

        @Test
        void staysQuietWhenTheRatesAgreeSoTheLogIsNotFloodedEveryTick() {
            CopyOnWriteArrayList<LogRecord> records = new CopyOnWriteArrayList<>();
            HytaleLoggerBackend.subscribe(records);
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture f = DroneTestFixture.install(harness);
                f.installFlightTick(harness, flightTick(), TICK_SECONDS, null);
                Store<EntityStore> store = harness.store();
                f.sessions().launch(store, f.newPilot(store), SPAWN, Rotation3f.IDENTITY, null);

                for (int i = 0; i < 10; i++) {
                    // A little jitter, as a real server produces. Well inside the 2x factor.
                    store.tick(TICK_DT * (i % 2 == 0 ? 1.1f : 0.9f));
                }

                assertEquals(0, warnings(records));
            } finally {
                HytaleLoggerBackend.unsubscribe(records);
            }
        }

        @Test
        void firesOnlyOnceBecauseAMismatchPersistsForEveryTickOfTheWorldsLife() {
            CopyOnWriteArrayList<LogRecord> records = new CopyOnWriteArrayList<>();
            HytaleLoggerBackend.subscribe(records);
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture f = DroneTestFixture.install(harness);
                f.installFlightTick(harness, flightTick(), TICK_SECONDS, null);
                Store<EntityStore> store = harness.store();
                f.sessions().launch(store, f.newPilot(store), SPAWN, Rotation3f.IDENTITY, null);

                for (int i = 0; i < 20; i++) {
                    store.tick(1.0f / 4.0f);
                }

                assertEquals(1, warnings(records), "30 warnings a second would be worse than none");
            } finally {
                HytaleLoggerBackend.unsubscribe(records);
            }
        }
    }

    @Nested
    class Construction {

        @Test
        void rejectsANonPositiveTickLengthBecauseItComesFromConfigNotFromAClient() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture f = DroneTestFixture.install(harness);

                assertThrows(
                        IllegalArgumentException.class,
                        () -> f.installFlightTick(harness, flightTick(), 0.0, null));
                assertThrows(
                        IllegalArgumentException.class,
                        () -> f.installFlightTick(harness, flightTick(), Double.NaN, null));
            }
        }
    }

    @Nested
    class SlotDrain {

        @Test
        void holdsTheLastPacketsThrottleThroughAQuietTickRatherThanCentringIt() {
            // The tick drains the slot, so a quiet tick has no fresh sample. It must still obey the
            // throttle the pilot last commanded -- a physical stick stays where it is put -- which is
            // what PilotInputSample.withoutLook() is for. If the tick centred instead, the second
            // climb would be visibly smaller than the first despite accelerating from a faster start.
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture f = DroneTestFixture.install(harness);
                f.installFlightTick(harness, flightTick(), TICK_SECONDS, null);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = f.newPilot(store);
                Ref<EntityStore> drone =
                        f.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);

                f.inputs().offer(f.uuidOf(store, pilot), fullThrottle());
                store.tick(TICK_DT);
                double firstTickGain =
                        flightOf(store, f, drone).getState().drone().position().y() - SPAWN.y;
                double afterOne = flightOf(store, f, drone).getState().drone().position().y();

                store.tick(TICK_DT);
                double quietTickGain =
                        flightOf(store, f, drone).getState().drone().position().y() - afterOne;

                assertTrue(
                        quietTickGain > firstTickGain,
                        "the held throttle should keep accelerating: first tick gained "
                                + firstTickGain
                                + ", the quiet tick gained "
                                + quietTickGain);
            }
        }

        @Test
        void advancesTheLookMemoryOnAQuietTickSoTheNextDeltaSpansTheRightInterval() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture f = DroneTestFixture.install(harness);
                f.installFlightTick(harness, flightTick(), TICK_SECONDS, null);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = f.newPilot(store);
                f.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);
                UUID pilotId = f.uuidOf(store, pilot);

                f.inputs().offer(pilotId, fullThrottle());
                store.tick(TICK_DT);
                store.tick(TICK_DT);
                store.tick(TICK_DT);

                assertEquals(
                        2 * TICK_SECONDS,
                        f.inputs().slotOf(pilotId).track().secondsSinceSample(),
                        1e-9);
            }
        }
    }
}
