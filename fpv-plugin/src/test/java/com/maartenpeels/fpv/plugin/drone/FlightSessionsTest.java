package com.maartenpeels.fpv.plugin.drone;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.hypixel.hytale.component.Ref;
import com.hypixel.hytale.component.Store;
import com.hypixel.hytale.math.vector.Rotation3f;
import com.hypixel.hytale.server.core.modules.entity.component.Intangible;
import com.hypixel.hytale.server.core.modules.entity.component.Invulnerable;
import com.hypixel.hytale.server.core.modules.entity.component.TransformComponent;
import com.hypixel.hytale.server.core.universe.world.storage.EntityStore;
import com.maartenpeels.fpv.plugin.ecs.HytaleEcsHarness;
import org.joml.Vector3d;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

/**
 * The launch and land halves of the ownership model.
 *
 * <p>These assert the ticket's Done-when directly: exactly one drone per launch, land removes it.
 * The involuntary exit paths are {@link DroneLifecycleSystemsTest}.
 */
class FlightSessionsTest {

    private static final Vector3d SPAWN = new Vector3d(10.0, 64.0, -20.0);

    @Nested
    class Launching {

        @Test
        void spawnsExactlyOneDroneAndRecordsItOnThePilot() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture fixture = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = fixture.newPilot(store);

                assertFalse(fixture.sessions().isFlying(store, pilot), "a fresh pilot is not flying");

                Ref<EntityStore> drone = fixture.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);

                assertNotNull(drone, "launch must return the spawned drone");
                assertTrue(drone.isValid(), "the drone entity must be live in the store");
                assertTrue(fixture.sessions().isFlying(store, pilot));
                assertSame(
                        drone,
                        fixture.sessions().droneOf(store, pilot),
                        "the session must point at the drone launch returned");
            }
        }

        @Test
        void refusesASecondLaunchBecauseTheSessionComponentIsTheGuard() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture fixture = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = fixture.newPilot(store);

                Ref<EntityStore> first = fixture.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);
                Ref<EntityStore> second = fixture.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);

                assertNull(second, "a pilot already flying must not get a second drone");
                assertSame(first, fixture.sessions().droneOf(store, pilot), "the first drone must survive the refusal");
                assertTrue(first.isValid(), "the refused launch must not have disturbed the live drone");
            }
        }

        @Test
        void putsTheDroneAtTheRequestedTransform() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture fixture = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = fixture.newPilot(store);

                Ref<EntityStore> drone = fixture.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);

                TransformComponent transform = store.getComponent(drone, fixture.types().transform());
                assertNotNull(transform);
                assertEquals(SPAWN, transform.getPosition());
            }
        }

        @Test
        void namesThePilotOnTheDroneSoTheFlightTickCanFindTheirInput() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture fixture = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = fixture.newPilot(store);

                Ref<EntityStore> drone = fixture.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);

                DroneComponent droneComponent = store.getComponent(drone, fixture.types().drone());
                assertNotNull(droneComponent);
                assertSame(pilot, droneComponent.getPilot(), "the back-reference must name the launching pilot");
            }
        }

        @Test
        void marksTheDroneNonSerializedSoACrashCannotLeaveOneInASaveFile() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture fixture = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = fixture.newPilot(store);

                Ref<EntityStore> drone = fixture.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);

                // This is the guarantee that makes `kill -9` mid-flight safe. It is not a shutdown
                // hook; a non-serialized entity is simply never written to a chunk or world save.
                assertTrue(
                        store.getArchetype(drone).contains(fixture.types().nonSerialized()),
                        "a drone must never be persistable");
            }
        }

        @Test
        void givesEachDroneItsOwnNetworkIdSoClientsCanTellThemApart() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture fixture = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();

                Ref<EntityStore> firstDrone = fixture.sessions()
                        .launch(store, fixture.newPilot(store), SPAWN, Rotation3f.IDENTITY, null);
                Ref<EntityStore> secondDrone = fixture.sessions()
                        .launch(store, fixture.newPilot(store), SPAWN, Rotation3f.IDENTITY, null);

                int firstId = store.getComponent(firstDrone, fixture.types().networkId()).getId();
                int secondId = store.getComponent(secondDrone, fixture.types().networkId()).getId();

                assertFalse(firstId == secondId, "two concurrent drones must not share a network id");
            }
        }
    }

    @Nested
    class ParkingThePilotsCharacter {

        @Test
        void addsBothBodyMarkersAndRecordsThatItAddedThem() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture fixture = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = fixture.newPilot(store);

                fixture.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);

                assertTrue(store.getArchetype(pilot).contains(fixture.types().invulnerable()));
                assertTrue(store.getArchetype(pilot).contains(fixture.types().intangible()));

                ParkedBody parked = store.getComponent(pilot, fixture.types().parkedBody());
                assertNotNull(parked, "a parked body must be recorded, or nothing can restore it");
                assertTrue(parked.addedInvulnerable());
                assertTrue(parked.addedIntangible());
            }
        }

        @Test
        void recordsThatItDidNotAddAMarkerThePilotAlreadyHad() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture fixture = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = fixture.newPilot(store);
                // Creative mode does exactly this: Player.setGameModeInternal adds Invulnerable
                // (`server/core/entity/entities/Player.java:752`).
                store.putComponent(pilot, fixture.types().invulnerable(), Invulnerable.INSTANCE);

                fixture.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);

                ParkedBody parked = store.getComponent(pilot, fixture.types().parkedBody());
                assertNotNull(parked);
                assertFalse(parked.addedInvulnerable(), "we must not claim credit for a pre-existing marker");
                assertTrue(parked.addedIntangible());
            }
        }

        @Test
        void leavesAPreExistingMarkerAloneOnLanding() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture fixture = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = fixture.newPilot(store);
                store.putComponent(pilot, fixture.types().invulnerable(), Invulnerable.INSTANCE);

                fixture.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);
                fixture.sessions().land(store, pilot);

                assertTrue(
                        store.getArchetype(pilot).contains(fixture.types().invulnerable()),
                        "landing must not strip the Creative-mode invulnerability the pilot arrived with");
                assertFalse(
                        store.getArchetype(pilot).contains(fixture.types().intangible()),
                        "the marker we added must still come off");
            }
        }
    }

    @Nested
    class Landing {

        @Test
        void removesTheDroneAndClearsBothPilotComponents() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture fixture = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = fixture.newPilot(store);
                Ref<EntityStore> drone = fixture.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);

                assertTrue(fixture.sessions().land(store, pilot));

                assertFalse(drone.isValid(), "land must destroy the drone entity");
                assertFalse(fixture.sessions().isFlying(store, pilot));
                assertNull(store.getComponent(pilot, fixture.types().parkedBody()));
                assertFalse(store.getArchetype(pilot).contains(fixture.types().invulnerable()));
                assertFalse(store.getArchetype(pilot).contains(fixture.types().intangible()));
            }
        }

        @Test
        void reportsFalseWhenThePilotWasNotFlyingSoCallersCanSaySo() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture fixture = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();

                assertFalse(fixture.sessions().land(store, fixture.newPilot(store)));
            }
        }

        @Test
        void allowsRelaunchingAfterwards() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture fixture = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = fixture.newPilot(store);

                fixture.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);
                fixture.sessions().land(store, pilot);
                Ref<EntityStore> second = fixture.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);

                assertNotNull(second, "landing must leave no residue that blocks the next launch");
                assertTrue(second.isValid());
            }
        }

        @Test
        void isSafeToCallTwice() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture fixture = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = fixture.newPilot(store);
                fixture.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);

                assertTrue(fixture.sessions().land(store, pilot));
                assertFalse(fixture.sessions().land(store, pilot), "the second land is a no-op, not a crash");
            }
        }

        @Test
        void doesNotTouchAnotherPilotsFlight() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture fixture = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> landing = fixture.newPilot(store);
                Ref<EntityStore> flying = fixture.newPilot(store);
                fixture.sessions().launch(store, landing, SPAWN, Rotation3f.IDENTITY, null);
                Ref<EntityStore> keptDrone = fixture.sessions().launch(store, flying, SPAWN, Rotation3f.IDENTITY, null);

                fixture.sessions().land(store, landing);

                assertTrue(keptDrone.isValid(), "one pilot landing must not disturb another's drone");
                assertTrue(fixture.sessions().isFlying(store, flying));
                assertTrue(store.getArchetype(flying).contains(fixture.types().intangible()));
            }
        }
    }

    @Nested
    class UnparkedMarkersAreTheOnesWeAdded {

        @Test
        void landingRestoresACharacterThatHadNeitherMarker() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture fixture = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = fixture.newPilot(store);
                store.putComponent(pilot, fixture.types().intangible(), Intangible.INSTANCE);

                fixture.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);
                fixture.sessions().land(store, pilot);

                assertTrue(
                        store.getArchetype(pilot).contains(fixture.types().intangible()),
                        "a pre-existing Intangible must survive the round trip");
                assertFalse(store.getArchetype(pilot).contains(fixture.types().invulnerable()));
            }
        }
    }
}
