package com.maartenpeels.fpv.plugin.camera;

import com.hypixel.hytale.component.ComponentAccessor;
import com.hypixel.hytale.component.Ref;
import com.hypixel.hytale.component.Store;
import com.hypixel.hytale.math.vector.Rotation3f;
import com.hypixel.hytale.protocol.ToClientPacket;
import com.hypixel.hytale.protocol.packets.camera.SetServerCamera;
import com.hypixel.hytale.server.core.modules.entity.component.TransformComponent;
import com.hypixel.hytale.server.core.modules.entity.tracker.NetworkId;
import com.hypixel.hytale.server.core.receiver.IPacketReceiver;
import com.hypixel.hytale.server.core.universe.world.storage.EntityStore;
import com.maartenpeels.fpv.plugin.drone.DroneTestFixture;
import com.maartenpeels.fpv.plugin.ecs.HytaleEcsHarness;
import org.joml.Vector3d;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;
import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * The session lifecycle half of #19, driven through a real ECS store.
 *
 * <p>This is as far into the feature as any automated test can reach. It proves that packet 280 is
 * built and handed to the pilot's sink at the right moments, carrying the network id of a
 * <em>really spawned</em> drone — the parts that can regress silently. It proves nothing about
 * whether the client honours the packet; only a human can.
 *
 * <p>Reaching this far needs the {@link PilotSink} seam: {@code PlayerRef} cannot be constructed in
 * a harness, so the production {@link PlayerRefSink} is swapped for a recorder.
 */
class DroneCameraSystemsTest {

    /** Captures what the plugin would have put on the wire. */
    private static final class RecordingSink implements PilotSink, IPacketReceiver {

        final List<SetServerCamera> cameraPackets = new ArrayList<>();
        final List<ToClientPacket> allPackets = new ArrayList<>();

        @Nullable
        @Override
        public IPacketReceiver receiverFor(
                @Nonnull ComponentAccessor<EntityStore> accessor, @Nonnull Ref<EntityStore> pilot) {
            return this;
        }

        @Override
        public void write(@Nonnull ToClientPacket packet) {
            record(packet);
        }

        @Override
        public void writeNoCache(@Nonnull ToClientPacket packet) {
            record(packet);
        }

        private void record(ToClientPacket packet) {
            this.allPackets.add(packet);
            if (packet instanceof SetServerCamera camera) {
                this.cameraPackets.add(camera);
            }
        }

        void clear() {
            this.cameraPackets.clear();
            this.allPackets.clear();
        }
    }

    private HytaleEcsHarness harness;
    private DroneTestFixture fixture;
    private RecordingSink sink;
    private DroneCamera camera;

    @BeforeEach
    void setUp() {
        this.harness = HytaleEcsHarness.create();
        this.fixture = DroneTestFixture.install(this.harness);
        this.sink = new RecordingSink();
        this.camera = new DroneCamera(this.fixture.types(), this.sink, DroneCameraTuning.DEFAULT);

        this.harness.registry().registerSystem(
                new DroneCameraSystems.AttachOnSession(this.fixture.types(), this.camera));
        this.harness.registry().registerSystem(
                new DroneCameraSystems.PushDrivenCamera(this.fixture.types(), this.camera));
    }

    @AfterEach
    void tearDown() {
        this.harness.close();
    }

    private Store<EntityStore> store() {
        return this.harness.store();
    }

    private Ref<EntityStore> launch(Ref<EntityStore> pilot) {
        return this.fixture.sessions().launch(
                store(), pilot, new Vector3d(1.0, 2.0, 3.0), new Rotation3f(0.0f, 0.0f, 0.0f), null);
    }

    @Nested
    class AttachOnSessionStart {

        @Test
        void sendsExactlyOneCameraPacketWhenAFlightSessionAppearsOnAPilot() {
            // The assertion the ticket brief asks for -- "we sent packet 280" -- at the moment the
            // lifecycle says we should. A count of zero here is the failure that would otherwise
            // only surface as a human staring at an unchanged view.
            Ref<EntityStore> pilot = DroneCameraSystemsTest.this.fixture.newPilot(store());

            assertNotNull(launch(pilot));

            assertEquals(1, DroneCameraSystemsTest.this.sink.cameraPackets.size());
            assertEquals(280, SetServerCamera.PACKET_ID);
        }

        @Test
        void firesFromAComponentTransitionWhichIsWhyThisIsARefChangeSystem() {
            // RefSystem.onEntityAdded would never run here: launch() puts a component on an
            // already-live entity, and RefSystem's callbacks are only invoked from Store.addEntity
            // / addEntities / removeEntity / removeEntities. If this system were a RefSystem this
            // test would see zero packets.
            Ref<EntityStore> pilot = DroneCameraSystemsTest.this.fixture.newPilot(store());
            assertTrue(DroneCameraSystemsTest.this.sink.cameraPackets.isEmpty(),
                    "spawning a pilot must not attach a camera");

            launch(pilot);

            assertEquals(1, DroneCameraSystemsTest.this.sink.cameraPackets.size());
        }

        @Test
        void carriesTheRealSpawnedDronesNetworkIdNotItsEcsIndex() {
            // Ref.getIndex() is a store slot and is not the wire id. Using it would attach the
            // camera to whatever entity happened to hold that network id -- or to nothing.
            Ref<EntityStore> pilot = DroneCameraSystemsTest.this.fixture.newPilot(store());
            Ref<EntityStore> drone = launch(pilot);
            assertNotNull(drone);

            NetworkId networkId =
                    store().getComponent(drone, DroneCameraSystemsTest.this.fixture.types().networkId());
            assertNotNull(networkId, "EnsureDroneNetworkSendable should have added one");

            SetServerCamera packet = DroneCameraSystemsTest.this.sink.cameraPackets.get(0);
            assertNotNull(packet.cameraSettings);
            assertEquals(networkId.getId(), packet.cameraSettings.attachedToEntityId);
        }

        @Test
        void carriesTheDronesActualSpawnPosition() {
            DroneCameraSystemsTest.this.camera.setTuning(
                    DroneCameraTuning.DEFAULT.withView(DroneCameraView.DRIVEN));
            Ref<EntityStore> pilot = DroneCameraSystemsTest.this.fixture.newPilot(store());

            launch(pilot);

            SetServerCamera packet = DroneCameraSystemsTest.this.sink.cameraPackets.get(0);
            assertNotNull(packet.cameraSettings);
            assertNotNull(packet.cameraSettings.position);
            assertEquals(1.0, packet.cameraSettings.position.x, 1.0e-6);
            assertEquals(3.0, packet.cameraSettings.position.z, 1.0e-6);
        }
    }

    @Nested
    class DetachOnSessionEnd {

        @Test
        void releasesTheCameraWhenTheSessionIsRemovedWhichIsTheTicketsDoneWhen() {
            Ref<EntityStore> pilot = DroneCameraSystemsTest.this.fixture.newPilot(store());
            launch(pilot);
            DroneCameraSystemsTest.this.sink.clear();

            assertTrue(DroneCameraSystemsTest.this.fixture.sessions().land(store(), pilot));

            assertEquals(1, DroneCameraSystemsTest.this.sink.cameraPackets.size());
            SetServerCamera release = DroneCameraSystemsTest.this.sink.cameraPackets.get(0);
            assertNull(release.cameraSettings, "a release must carry no settings");
            assertEquals(false, release.isLocked);
        }

        @Test
        void alsoReleasesWhenTheDroneDiesByARouteWeDidNotInitiate() {
            // ClearSessionOnDroneRemoved strips the session when a drone is removed by something
            // else -- a chunk unload, /entity remove. That path must return the pilot's view too,
            // or they are left looking through a drone that no longer exists.
            Ref<EntityStore> pilot = DroneCameraSystemsTest.this.fixture.newPilot(store());
            Ref<EntityStore> drone = launch(pilot);
            assertNotNull(drone);
            DroneCameraSystemsTest.this.sink.clear();

            store().removeEntity(drone, com.hypixel.hytale.component.RemoveReason.REMOVE);

            assertEquals(1, DroneCameraSystemsTest.this.sink.cameraPackets.size());
            assertNull(DroneCameraSystemsTest.this.sink.cameraPackets.get(0).cameraSettings);
        }
    }

    @Nested
    class PerTickPush {

        @Test
        void drivenModeResendsEveryTickBecauseItsPositionAndRotationAreAbsolute() {
            DroneCameraSystemsTest.this.camera.setTuning(
                    DroneCameraTuning.DEFAULT.withView(DroneCameraView.DRIVEN));
            Ref<EntityStore> pilot = DroneCameraSystemsTest.this.fixture.newPilot(store());
            launch(pilot);
            DroneCameraSystemsTest.this.sink.clear();

            store().tick(1.0f / 30.0f);
            store().tick(1.0f / 30.0f);

            assertEquals(2, DroneCameraSystemsTest.this.sink.cameraPackets.size(),
                    "DRIVEN must push once per tick or the camera sits frozen at the launch point");
        }

        @Test
        void trackedModeSendsNothingPerTickBecauseTheClientFollowsTheEntityItself() {
            Ref<EntityStore> pilot = DroneCameraSystemsTest.this.fixture.newPilot(store());
            launch(pilot);
            DroneCameraSystemsTest.this.sink.clear();

            store().tick(1.0f / 30.0f);
            store().tick(1.0f / 30.0f);

            assertTrue(DroneCameraSystemsTest.this.sink.cameraPackets.isEmpty(),
                    "TRACKED must cost no per-tick bandwidth");
        }

        @Test
        void switchingModeAtRuntimeTakesEffectOnTheNextTickWithNoRestart() {
            // This is what makes /fpv camera set a usable bisecting tool: a human can flip modes
            // mid-flight and watch the horizon, rather than restarting the server per attempt.
            Ref<EntityStore> pilot = DroneCameraSystemsTest.this.fixture.newPilot(store());
            launch(pilot);
            DroneCameraSystemsTest.this.sink.clear();

            store().tick(1.0f / 30.0f);
            assertTrue(DroneCameraSystemsTest.this.sink.cameraPackets.isEmpty());

            DroneCameraSystemsTest.this.camera.setTuning(
                    DroneCameraTuning.DEFAULT.withView(DroneCameraView.DRIVEN));
            store().tick(1.0f / 30.0f);

            assertEquals(1, DroneCameraSystemsTest.this.sink.cameraPackets.size());
        }
    }

    @Nested
    class AttachPacketResolution {

        @Test
        void reflectsTheDronesCurrentAttitudeSoForcedRollShowsUpInTheNextPacket() {
            // The mechanism behind /fpv camera attitude, and behind whatever #24 does every tick.
            DroneCameraSystemsTest.this.camera.setTuning(
                    DroneCameraTuning.DEFAULT.withView(DroneCameraView.DRIVEN));
            Ref<EntityStore> pilot = DroneCameraSystemsTest.this.fixture.newPilot(store());
            Ref<EntityStore> drone = launch(pilot);
            assertNotNull(drone);

            TransformComponent transform =
                    store().getComponent(drone, DroneCameraSystemsTest.this.fixture.types().transform());
            assertNotNull(transform);
            transform.setRotation(new Rotation3f(0.0f, 0.0f, (float) Math.toRadians(30.0)));

            SetServerCamera packet =
                    DroneCameraSystemsTest.this.camera.attachPacket(store(), drone);

            assertNotNull(packet);
            assertNotNull(packet.cameraSettings);
            assertNotNull(packet.cameraSettings.rotation);
            assertEquals(Math.toRadians(30.0), packet.cameraSettings.rotation.roll, 1.0e-6);
        }

        @Test
        void returnsNullForAnInvalidDroneRatherThanThrowing() {
            Ref<EntityStore> pilot = DroneCameraSystemsTest.this.fixture.newPilot(store());
            Ref<EntityStore> drone = launch(pilot);
            assertNotNull(drone);
            store().removeEntity(drone, com.hypixel.hytale.component.RemoveReason.REMOVE);

            assertNull(DroneCameraSystemsTest.this.camera.attachPacket(store(), drone));
        }
    }

    @Nested
    class Tuning {

        @Test
        void readsBackWhatWasSetSoTheStatusCommandReportsTheTruth() {
            DroneCameraTuning replacement =
                    DroneCameraTuning.DEFAULT.withView(DroneCameraView.DRIVEN).withLerpSpeeds(0.5f, 0.6f);

            DroneCameraSystemsTest.this.camera.setTuning(replacement);

            assertSame(replacement, DroneCameraSystemsTest.this.camera.getTuning());
        }
    }
}
