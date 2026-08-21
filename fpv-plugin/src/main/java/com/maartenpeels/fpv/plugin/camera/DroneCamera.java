package com.maartenpeels.fpv.plugin.camera;

import com.hypixel.hytale.component.ComponentAccessor;
import com.hypixel.hytale.component.Ref;
import com.hypixel.hytale.protocol.packets.camera.SetServerCamera;
import com.hypixel.hytale.server.core.modules.entity.component.TransformComponent;
import com.hypixel.hytale.server.core.modules.entity.tracker.NetworkId;
import com.hypixel.hytale.server.core.receiver.IPacketReceiver;
import com.hypixel.hytale.server.core.universe.PlayerRef;
import com.hypixel.hytale.server.core.universe.world.storage.EntityStore;
import com.maartenpeels.fpv.plugin.drone.FlightComponentTypes;
import com.maartenpeels.fpv.plugin.drone.FlightSession;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;

/**
 * Puts a pilot's camera in their drone, and takes it back out. The side-effect half of the camera
 * feature; all construction is in {@link DroneCameraPackets}.
 *
 * <p>Holds the live {@link DroneCameraTuning}, which {@code /fpv camera set} replaces at runtime.
 * That mutability is the point: #19 cannot be verified by any test, so the loop for getting the
 * camera right is a human flying and changing knobs, and a rebuild per knob would make that loop
 * useless.
 *
 * <h2>Sending is fire-and-forget, and that is fine</h2>
 *
 * <p>No system in the server re-sends {@code SetServerCamera} — there are five call sites, all
 * one-shot. Camera state is sticky client-side until something changes it. Two things change it
 * without asking us, and neither needs handling:
 *
 * <ul>
 *   <li><strong>World transfer</strong> resets the camera: {@code Universe.java:1239} calls
 *       {@code Player.resetManagers}, which calls {@code CameraManager.resetCamera}
 *       (`server/core/entity/entities/Player.java:348-356`) and also clears
 *       {@code EntityTrackerSystems} state, invalidating any tracked network id. But #18's
 *       {@code EndSessionOnPilotRemoved} ends the session on that same transition, so there is no
 *       session left wanting a camera.
 *   <li><strong>Reconnect</strong> is a fresh client with no session.
 * </ul>
 *
 * <p>There is no camera reset on respawn or death.
 */
public final class DroneCamera {

    @Nonnull
    private final FlightComponentTypes types;

    @Nonnull
    private final PilotSink sink;

    @Nonnull
    private volatile DroneCameraTuning tuning;

    public DroneCamera(
            @Nonnull FlightComponentTypes types,
            @Nonnull PilotSink sink,
            @Nonnull DroneCameraTuning tuning) {
        this.types = types;
        this.sink = sink;
        this.tuning = tuning;
    }

    /**
     * The tuning currently in force.
     *
     * <p>{@code volatile} because commands run on the world thread while
     * {@code /fpv camera status} may report from a command thread; a torn read of an immutable
     * record reference is the only hazard and volatile removes it.
     */
    @Nonnull
    public DroneCameraTuning getTuning() {
        return this.tuning;
    }

    /** Replaces the tuning. Callers that want it to take effect immediately must re-attach. */
    public void setTuning(@Nonnull DroneCameraTuning tuning) {
        this.tuning = tuning;
    }

    /**
     * Builds the packet that would attach a pilot to this drone, or {@code null} if the drone is
     * not yet describable to a client.
     *
     * <p>Split out from {@link #attach} on purpose: this half needs no {@code PlayerRef} and no
     * connection, so it is the deepest point in the feature a {@code HytaleEcsHarness} test can
     * reach. It resolves a <em>real spawned drone</em>'s network id and attitude, which is the part
     * of #19 that can actually regress silently.
     *
     * <p>{@code null} means the drone lacks {@code NetworkId} or {@code TransformComponent}.
     * #18's {@code EnsureDroneNetworkSendable} adds the former during {@code addEntity} and
     * {@code FlightSessions.launch} adds the latter, so in practice this is unreachable for a
     * drone we spawned — it is here so a drone created by some future path cannot crash a camera
     * attach.
     */
    @Nullable
    public SetServerCamera attachPacket(
            @Nonnull ComponentAccessor<EntityStore> accessor, @Nonnull Ref<EntityStore> drone) {

        if (!drone.isValid()) {
            return null;
        }

        // NetworkId, never Ref.getIndex(). getIndex() is an ECS store slot with no relationship to
        // the wire id; the network id is what every entity-referencing packet uses
        // (`server/core/entity/AnimationUtils.java:52-54`,
        // `server/core/entity/InteractionContext.java:342-344`).
        NetworkId networkId = accessor.getComponent(drone, this.types.networkId());
        TransformComponent transform = accessor.getComponent(drone, this.types.transform());
        if (networkId == null || transform == null) {
            return null;
        }

        return DroneCameraPackets.attach(
                this.tuning, networkId.getId(), transform.getRotation(), transform.getPosition());
    }

    /**
     * Attaches the pilot's camera to the drone.
     *
     * @return {@code false} if the pilot has no live connection or the drone is not sendable, so a
     *     caller can report it rather than silently leaving a pilot in their own body
     */
    public boolean attach(
            @Nonnull ComponentAccessor<EntityStore> accessor,
            @Nonnull Ref<EntityStore> pilot,
            @Nonnull Ref<EntityStore> drone) {

        IPacketReceiver receiver = this.sink.receiverFor(accessor, pilot);
        if (receiver == null) {
            return false;
        }
        SetServerCamera packet = attachPacket(accessor, drone);
        if (packet == null) {
            return false;
        }
        // writeNoCache, not write: write() wraps the packet in a shared CachedPacket
        // (`server/core/io/PacketHandler.java:217-255`), which is wrong for a payload that carries
        // one pilot's drone id. Every in-tree camera sender uses writeNoCache for the same reason.
        receiver.writeNoCache(packet);
        return true;
    }

    /**
     * Attaches using the pilot's own {@link FlightSession} to find the drone.
     *
     * @return {@code false} if the pilot is not flying, or attaching failed
     */
    public boolean attachToSessionDrone(
            @Nonnull ComponentAccessor<EntityStore> accessor, @Nonnull Ref<EntityStore> pilot) {

        FlightSession session = accessor.getComponent(pilot, this.types.flightSession());
        return session != null && attach(accessor, pilot, session.getDrone());
    }

    /**
     * Returns the pilot to a normal view. The ticket's Done-when.
     *
     * <p>Unconditional: it does not check whether we ever attached. Sending a release to a pilot
     * whose camera is already normal is a no-op client-side, and the alternative — tracking
     * attachment in a component — would be plugin state that can disagree with the client, which is
     * strictly worse than one redundant 157-byte packet.
     *
     * @return {@code false} only if the pilot has no live connection
     */
    public boolean detach(
            @Nonnull ComponentAccessor<EntityStore> accessor, @Nonnull Ref<EntityStore> pilot) {

        IPacketReceiver receiver = this.sink.receiverFor(accessor, pilot);
        if (receiver == null) {
            return false;
        }
        receiver.writeNoCache(DroneCameraPackets.detach());
        return true;
    }
}
