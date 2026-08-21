package com.maartenpeels.fpv.plugin.drone;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.hypixel.hytale.component.AddReason;
import com.hypixel.hytale.component.Holder;
import com.hypixel.hytale.component.Ref;
import com.hypixel.hytale.component.RemoveReason;
import com.hypixel.hytale.component.Store;
import com.hypixel.hytale.math.vector.Rotation3f;
import com.hypixel.hytale.server.core.modules.entity.component.Intangible;
import com.hypixel.hytale.server.core.modules.entity.component.Invulnerable;
import com.hypixel.hytale.server.core.modules.entity.component.TransformComponent;
import com.hypixel.hytale.server.core.modules.entity.tracker.EntityTrackerSystems;
import com.hypixel.hytale.server.core.universe.world.storage.EntityStore;
import com.maartenpeels.fpv.plugin.ecs.HytaleEcsHarness;
import org.joml.Vector3d;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

/**
 * The involuntary exit paths — the half of the ticket that is actually hard.
 *
 * <p>Every one of disconnect, kick, world switch, world unload and world crash ends as one
 * {@code Store.removeEntity(pilotRef, RemoveReason.UNLOAD)} via {@code PlayerRef.removeFromStore()}
 * (`universe/PlayerRef.java:161`). So the removal assertions below are not a proxy for those
 * paths; they are those paths, minus the code that decides to walk them.
 */
class DroneLifecycleSystemsTest {

    private static final Vector3d SPAWN = new Vector3d(0.0, 70.0, 0.0);
    private static final float TICK_DT = 1.0f / 30.0f;

    @Nested
    class WhenThePilotLeaves {

        @Test
        void removingThePilotWithUnloadRemovesTheDroneBecauseThatIsTheDisconnectPath() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture fixture = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = fixture.newPilot(store);
                Ref<EntityStore> drone = fixture.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);

                // UNLOAD, not REMOVE: players are always removed with UNLOAD, including on a
                // permanent disconnect. Branching on the reason would be branching on noise.
                store.removeEntity(pilot, RemoveReason.UNLOAD);

                assertFalse(drone.isValid(), "a hard disconnect mid-flight must leave nothing behind");
            }
        }

        @Test
        void removingThePilotWithRemoveAlsoRemovesTheDrone() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture fixture = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = fixture.newPilot(store);
                Ref<EntityStore> drone = fixture.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);

                store.removeEntity(pilot, RemoveReason.REMOVE);

                assertFalse(drone.isValid(), "teardown must not depend on which reason the caller passed");
            }
        }

        @Test
        void keepsTheParkedBodyRecordOnTheOutgoingHolderSoTheNextAddCanRestoreIt() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture fixture = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = fixture.newPilot(store);
                fixture.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);

                Holder<EntityStore> outgoing = store.removeEntity(pilot, RemoveReason.UNLOAD);

                // This is what makes a mid-flight crash recoverable rather than permanent: the
                // record rides out with the pilot and RestoreParkedBodyOnAdd unwinds it wherever
                // they turn up next.
                assertNotNull(
                        outgoing.getComponent(fixture.types().parkedBody()),
                        "ParkedBody must survive the removal, or nothing can unpark the pilot");
                assertTrue(outgoing.getArchetype().contains(fixture.types().invulnerable()));
            }
        }

        @Test
        void removingOnePilotLeavesAnotherPilotsDroneAlone() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture fixture = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> leaving = fixture.newPilot(store);
                Ref<EntityStore> staying = fixture.newPilot(store);
                fixture.sessions().launch(store, leaving, SPAWN, Rotation3f.IDENTITY, null);
                Ref<EntityStore> keptDrone = fixture.sessions().launch(store, staying, SPAWN, Rotation3f.IDENTITY, null);

                store.removeEntity(leaving, RemoveReason.UNLOAD);

                assertTrue(keptDrone.isValid());
                assertTrue(fixture.sessions().isFlying(store, staying));
            }
        }

        @Test
        void aPilotWhoNeverLaunchedIsRemovedWithoutIncident() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture fixture = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = fixture.newPilot(store);

                store.removeEntity(pilot, RemoveReason.UNLOAD);

                assertFalse(pilot.isValid());
            }
        }
    }

    @Nested
    class WhenTheDroneDiesFirst {

        @Test
        void clearsThePilotsSessionSoTheyCanLaunchAgain() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture fixture = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = fixture.newPilot(store);
                Ref<EntityStore> drone = fixture.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);

                // Stands in for anything that kills a drone without going through land():
                // a chunk unload, /entity remove, a future crash-on-collision.
                store.removeEntity(drone, RemoveReason.REMOVE);

                assertFalse(fixture.sessions().isFlying(store, pilot), "a stale session would block relaunching");
                assertNotNull(
                        fixture.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null),
                        "the pilot must be able to launch a replacement");
            }
        }

        @Test
        void unparksThePilotSoTheyAreNotStrandedInvisibleWithNothingToFly() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture fixture = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = fixture.newPilot(store);
                Ref<EntityStore> drone = fixture.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);

                store.removeEntity(drone, RemoveReason.REMOVE);

                assertNull(store.getComponent(pilot, fixture.types().parkedBody()));
                assertFalse(store.getArchetype(pilot).contains(fixture.types().invulnerable()));
                assertFalse(store.getArchetype(pilot).contains(fixture.types().intangible()));
            }
        }

        @Test
        void doesNotClearASessionThatAlreadyPointsAtADifferentDrone() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture fixture = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = fixture.newPilot(store);
                Ref<EntityStore> stale = fixture.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);

                // Re-point the session at a second drone without removing the first, then kill the
                // first. The identity check in ClearSessionOnDroneRemoved must spare the session.
                Ref<EntityStore> current = fixture.sessions()
                        .launch(store, fixture.newPilot(store), SPAWN, Rotation3f.IDENTITY, null);
                store.putComponent(pilot, fixture.types().flightSession(), new FlightSession(current));

                store.removeEntity(stale, RemoveReason.REMOVE);

                assertTrue(
                        fixture.sessions().isFlying(store, pilot),
                        "an outdated drone's death must not cancel the session that superseded it");
            }
        }
    }

    @Nested
    class RestoringAParkedBodyOnAdd {

        @Test
        void stripsTheMarkersTheRecordSaysWeAdded() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture fixture = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();

                // Exactly the state a crash leaves on disk: both markers, plus a record saying we
                // put them there. Adding the entity is a relog, or arriving in the next world.
                Holder<EntityStore> holder = harness.registry().newHolder();
                holder.addComponent(fixture.types().transform(), new TransformComponent());
                holder.addComponent(fixture.types().invulnerable(), Invulnerable.INSTANCE);
                holder.addComponent(fixture.types().intangible(), Intangible.INSTANCE);
                holder.addComponent(fixture.types().parkedBody(), new ParkedBody(true, true));

                Ref<EntityStore> pilot = store.addEntity(holder, AddReason.LOAD);

                assertNotNull(pilot);
                assertNull(
                        store.getComponent(pilot, fixture.types().parkedBody()),
                        "the record must be consumed, or the pilot re-restores forever");
                assertFalse(store.getArchetype(pilot).contains(fixture.types().invulnerable()));
                assertFalse(store.getArchetype(pilot).contains(fixture.types().intangible()));
            }
        }

        @Test
        void leavesAMarkerTheRecordSaysWeDidNotAdd() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture fixture = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();

                Holder<EntityStore> holder = harness.registry().newHolder();
                holder.addComponent(fixture.types().transform(), new TransformComponent());
                holder.addComponent(fixture.types().invulnerable(), Invulnerable.INSTANCE);
                holder.addComponent(fixture.types().intangible(), Intangible.INSTANCE);
                holder.addComponent(fixture.types().parkedBody(), new ParkedBody(false, true));

                Ref<EntityStore> pilot = store.addEntity(holder, AddReason.LOAD);

                assertNotNull(pilot);
                assertTrue(
                        store.getArchetype(pilot).contains(fixture.types().invulnerable()),
                        "a Creative-mode pilot must stay invulnerable across a crash");
                assertFalse(store.getArchetype(pilot).contains(fixture.types().intangible()));
            }
        }

        @Test
        void ignoresAnEntityWithNoParkedBodyRecord() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture fixture = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();

                Holder<EntityStore> holder = harness.registry().newHolder();
                holder.addComponent(fixture.types().transform(), new TransformComponent());
                holder.addComponent(fixture.types().invulnerable(), Invulnerable.INSTANCE);

                Ref<EntityStore> pilot = store.addEntity(holder, AddReason.LOAD);

                assertNotNull(pilot);
                assertTrue(
                        store.getArchetype(pilot).contains(fixture.types().invulnerable()),
                        "without a record we have no claim on the marker");
            }
        }
    }

    @Nested
    class HidingAParkedBody {

        @Test
        void prunesAParkedBodyFromAViewersVisibleSet() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture fixture = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = fixture.newPilot(store);
                fixture.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);

                Ref<EntityStore> viewer = fixture.newViewer(store);
                EntityTrackerSystems.EntityViewer viewerComponent =
                        store.getComponent(viewer, fixture.types().entityViewer());
                // CollectVisible would have put the pilot here; we stand in for it.
                viewerComponent.visible.add(pilot);

                store.tick(TICK_DT);

                assertFalse(
                        viewerComponent.visible.contains(pilot),
                        "a parked body must be pruned so SendPackets despawns it clientside");
                assertTrue(viewerComponent.hiddenCount > 0, "the tracker diagnostics must stay honest");
            }
        }

        @Test
        void leavesAnUnparkedEntityVisible() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture fixture = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> bystander = fixture.newPilot(store);

                Ref<EntityStore> viewer = fixture.newViewer(store);
                EntityTrackerSystems.EntityViewer viewerComponent =
                        store.getComponent(viewer, fixture.types().entityViewer());
                viewerComponent.visible.add(bystander);

                store.tick(TICK_DT);

                assertTrue(viewerComponent.visible.contains(bystander), "hiding must be opt-in, not a blanket cull");
            }
        }

        @Test
        void stopsHidingOnceThePilotLands() {
            try (HytaleEcsHarness harness = HytaleEcsHarness.create()) {
                DroneTestFixture fixture = DroneTestFixture.install(harness);
                Store<EntityStore> store = harness.store();
                Ref<EntityStore> pilot = fixture.newPilot(store);
                fixture.sessions().launch(store, pilot, SPAWN, Rotation3f.IDENTITY, null);

                Ref<EntityStore> viewer = fixture.newViewer(store);
                EntityTrackerSystems.EntityViewer viewerComponent =
                        store.getComponent(viewer, fixture.types().entityViewer());

                fixture.sessions().land(store, pilot);
                viewerComponent.visible.add(pilot);
                store.tick(TICK_DT);

                assertTrue(viewerComponent.visible.contains(pilot), "the pilot's body must come back on landing");
            }
        }
    }
}
