package com.maartenpeels.fpv.plugin.camera;

import com.hypixel.hytale.component.ArchetypeChunk;
import com.hypixel.hytale.component.CommandBuffer;
import com.hypixel.hytale.component.ComponentType;
import com.hypixel.hytale.component.Ref;
import com.hypixel.hytale.component.Store;
import com.hypixel.hytale.component.query.Query;
import com.hypixel.hytale.component.system.RefChangeSystem;
import com.hypixel.hytale.component.system.tick.EntityTickingSystem;
import com.hypixel.hytale.server.core.universe.world.storage.EntityStore;
import com.maartenpeels.fpv.plugin.drone.FlightComponentTypes;
import com.maartenpeels.fpv.plugin.drone.FlightSession;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;

/**
 * The systems that keep a pilot's camera in step with their flight session.
 *
 * <p>Neither touches {@code World}, so both run inside {@code HytaleEcsHarness}.
 */
public final class DroneCameraSystems {

    private DroneCameraSystems() {}

    /**
     * Attaches the camera when a {@link FlightSession} appears on a pilot, and releases it when the
     * session goes away.
     *
     * <h2>Why {@code RefChangeSystem} and not {@code RefSystem}</h2>
     *
     * <p>Because a session is a component put onto an <em>already live</em> player entity, and
     * <strong>{@code RefSystem} does not see that.</strong> {@code RefSystem.onEntityAdded} /
     * {@code onEntityRemove} are invoked from exactly four places, all entity-lifecycle:
     * {@code Store.addEntity}, {@code addEntities}, {@code removeEntity}, {@code removeEntities}.
     * No component-mutation path touches them.
     *
     * <p>Component transitions dispatch to {@code RefChangeSystem} instead —
     * {@code Store.datachunk_addComponent} at `component/Store.java:2052-2057`,
     * {@code Store.putComponent}'s already-present branch at `:951-964`,
     * {@code removeComponent} at `:1054` and {@code removeComponentIfExists} at `:1105`. All three
     * conditions there are satisfied here: this extends {@code RefChangeSystem}, its
     * {@code componentType()} is the very instance {@code FlightSessions} writes, and its query is
     * that same type so it matches the post-transition archetype.
     *
     * <h2>Why this and not a call inside {@code FlightSessions}</h2>
     *
     * <p>{@code docs/plans/18.md:249-256} kept camera concerns out of {@code FlightSessions}
     * deliberately, so that #22's "a failure partway through launch must not leave a player
     * camera-less" question stays in the ticket that owns launch. Hanging off the component
     * transition preserves that: every route that creates or destroys a session — launch, land,
     * a drone removed by a chunk unload via {@code ClearSessionOnDroneRemoved} — goes through here
     * without any of them knowing a camera exists.
     *
     * <p>One route deliberately not covered: a pilot whose <em>entity</em> is removed (disconnect,
     * world switch) gets no release packet, because {@code onComponentRemoved} does not fire for a
     * wholesale entity removal. That is correct — a disconnecting client has nothing to release,
     * and a world switch already resets the camera itself at {@code Universe.java:1239}.
     */
    public static final class AttachOnSession extends RefChangeSystem<EntityStore, FlightSession> {

        @Nonnull
        private final FlightComponentTypes types;
        @Nonnull
        private final DroneCamera camera;

        public AttachOnSession(@Nonnull FlightComponentTypes types, @Nonnull DroneCamera camera) {
            this.types = types;
            this.camera = camera;
        }

        @Nonnull
        @Override
        public ComponentType<EntityStore, FlightSession> componentType() {
            return this.types.flightSession();
        }

        @Nonnull
        @Override
        public Query<EntityStore> getQuery() {
            return this.types.flightSession();
        }

        @Override
        public void onComponentAdded(
                @Nonnull Ref<EntityStore> ref,
                @Nonnull FlightSession session,
                @Nonnull Store<EntityStore> store,
                @Nonnull CommandBuffer<EntityStore> commandBuffer) {

            this.camera.attach(commandBuffer, ref, session.getDrone());
        }

        /**
         * A session replaced in place — the pilot has been pointed at a different drone.
         *
         * <p>Implemented because a second {@code putComponent} of an existing component yields
         * {@code onComponentSet}, <strong>not</strong> {@code onComponentAdded}
         * (`component/Store.java:951-964`). Nothing does that today —
         * {@code FlightSessions.launch} refuses to run for a pilot who already has a session — but
         * a relaunch-without-landing path added later would otherwise leave the camera on the old
         * drone, which is a silent bug of exactly the kind this ticket cannot test for.
         */
        @Override
        public void onComponentSet(
                @Nonnull Ref<EntityStore> ref,
                @Nullable FlightSession previous,
                @Nonnull FlightSession session,
                @Nonnull Store<EntityStore> store,
                @Nonnull CommandBuffer<EntityStore> commandBuffer) {

            this.camera.attach(commandBuffer, ref, session.getDrone());
        }

        @Override
        public void onComponentRemoved(
                @Nonnull Ref<EntityStore> ref,
                @Nonnull FlightSession session,
                @Nonnull Store<EntityStore> store,
                @Nonnull CommandBuffer<EntityStore> commandBuffer) {

            this.camera.detach(commandBuffer, ref);
        }
    }

    /**
     * Re-sends the camera every tick, for {@link DroneCameraView#DRIVEN} only.
     *
     * <p>{@code DRIVEN} puts absolute position and rotation in the packet
     * ({@code PositionType.Custom} / {@code RotationType.Custom}), so without a per-tick push the
     * camera would sit frozen where the drone was at launch. {@code TRACKED} needs none of this —
     * the client follows the entity itself — so this system checks the live tuning and returns
     * immediately, which is what makes the mode a runtime switch rather than a restart.
     *
     * <p>Costs one 157-byte packet per flying pilot per tick: ~4.7 kB/s each at 30 TPS, ~38 kB/s at
     * decision 9's eight-pilot target. Small next to chunk streaming, and only paid in the mode
     * that needs it.
     *
     * <p>Not parallel. {@code EntityTickingSystem.maybeUseParallel} exists and
     * {@code PlayerSendInventorySystem} uses it, but at eight pilots there is nothing to gain and
     * serial keeps packet order per pilot obvious.
     */
    public static final class PushDrivenCamera extends EntityTickingSystem<EntityStore> {

        @Nonnull
        private final FlightComponentTypes types;
        @Nonnull
        private final DroneCamera camera;
        @Nonnull
        private final Query<EntityStore> query;

        public PushDrivenCamera(@Nonnull FlightComponentTypes types, @Nonnull DroneCamera camera) {
            this.types = types;
            this.camera = camera;
            // Just the session, deliberately not Query.and(session, PlayerRef). Adding PlayerRef
            // would filter out nothing real -- only pilots have sessions -- while making this
            // system unreachable in a harness, which has no PlayerRef instances. DroneCamera
            // already no-ops when a pilot has no sink.
            this.query = types.flightSession();
        }

        @Nonnull
        @Override
        public Query<EntityStore> getQuery() {
            return this.query;
        }

        @Override
        public void tick(
                float dt,
                int index,
                @Nonnull ArchetypeChunk<EntityStore> archetypeChunk,
                @Nonnull Store<EntityStore> store,
                @Nonnull CommandBuffer<EntityStore> commandBuffer) {

            if (!this.camera.getTuning().view().needsPerTickPush()) {
                return;
            }

            FlightSession session = archetypeChunk.getComponent(index, this.types.flightSession());
            if (session == null) {
                return;
            }

            this.camera.attach(commandBuffer, archetypeChunk.getReferenceTo(index), session.getDrone());
        }
    }
}
