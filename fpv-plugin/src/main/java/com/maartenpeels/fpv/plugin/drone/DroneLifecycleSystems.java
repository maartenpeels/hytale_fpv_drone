package com.maartenpeels.fpv.plugin.drone;

import com.hypixel.hytale.component.AddReason;
import com.hypixel.hytale.component.ArchetypeChunk;
import com.hypixel.hytale.component.CommandBuffer;
import com.hypixel.hytale.component.Holder;
import com.hypixel.hytale.component.Ref;
import com.hypixel.hytale.component.RemoveReason;
import com.hypixel.hytale.component.Store;
import com.hypixel.hytale.component.SystemGroup;
import com.hypixel.hytale.component.dependency.Dependency;
import com.hypixel.hytale.component.query.Query;
import com.hypixel.hytale.component.system.HolderSystem;
import com.hypixel.hytale.component.system.RefSystem;
import com.hypixel.hytale.component.system.tick.EntityTickingSystem;
import com.hypixel.hytale.server.core.modules.entity.tracker.EntityTrackerSystems;
import com.hypixel.hytale.server.core.modules.entity.tracker.NetworkId;
import com.hypixel.hytale.server.core.universe.world.storage.EntityStore;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;
import java.util.Iterator;
import java.util.Set;

/**
 * The systems that make a flight session impossible to leak.
 *
 * <p>Grouped in one holder class following the server's own convention for related systems —
 * {@code EntityTrackerSystems}, {@code HideEntitySystems}, {@code MountSystems},
 * {@code PlayerSystems} all do this.
 *
 * <p>Every one of them takes its {@link com.hypixel.hytale.component.ComponentType}s through
 * {@link FlightComponentTypes} in the constructor, per CLAUDE.md. None of them touch
 * {@code World}, so all five run inside {@code HytaleEcsHarness}.
 */
public final class DroneLifecycleSystems {

    private DroneLifecycleSystems() {}

    /**
     * Gives every drone entity a {@code NetworkId} and the {@code NonSerialized} marker.
     *
     * <p>Why a system rather than two more lines in {@link FlightSessions#launch}: the
     * {@code NonSerialized} marker is what makes {@code kill -9} mid-flight safe, and that
     * guarantee must not depend on a caller remembering it. Keyed on {@link DroneComponent}, so
     * anything that ever creates a drone holder gets both, including code written later.
     *
     * <p>{@code NetworkId} plus {@code TransformComponent} is exactly what makes an entity
     * network-sendable — that pair is the query of
     * {@code NetworkSendableSpatialSystem} (`modules/entity/system/NetworkSendableSpatialSystem.java:17`),
     * and everything downstream (visibility collection, model updates, the batched
     * {@code EntityUpdates} packet) follows automatically. There is no spawn packet to send.
     *
     * <p>The idiom has five precedents: {@code MountSystems.EnsureMinecartComponents:95-130},
     * {@code ParkourCheckpointSystems.EnsureNetworkSendable:38},
     * {@code ReachLocationMarkerSystems.EnsureNetworkSendable:46},
     * {@code SpawnMarkerSystems:133}, {@code ItemSystems.EnsureRequiredComponents:37}.
     */
    public static final class EnsureDroneNetworkSendable extends HolderSystem<EntityStore> {

        @Nonnull
        private final FlightComponentTypes types;
        @Nonnull
        private final Query<EntityStore> query;

        public EnsureDroneNetworkSendable(@Nonnull FlightComponentTypes types) {
            this.types = types;
            this.query = Query.and(types.drone(), Query.not(types.networkId()));
        }

        @Nonnull
        @Override
        public Query<EntityStore> getQuery() {
            return this.query;
        }

        @Override
        public void onEntityAdd(
                @Nonnull Holder<EntityStore> holder, @Nonnull AddReason reason, @Nonnull Store<EntityStore> store) {
            holder.putComponent(
                    this.types.networkId(), new NetworkId(store.getExternalData().takeNextNetworkId()));
            holder.ensureComponent(this.types.nonSerialized());
        }

        @Override
        public void onEntityRemoved(
                @Nonnull Holder<EntityStore> holder, @Nonnull RemoveReason reason, @Nonnull Store<EntityStore> store) {
            // Nothing to do: a drone's components die with its detached Holder.
        }
    }

    /**
     * Removes the drone when the pilot's entity leaves the store.
     *
     * <p>This is the single choke point for every involuntary exit. Disconnect, kick, world
     * switch, world unload and world crash all end as one
     * {@code Store.removeEntity(pilotRef, RemoveReason.UNLOAD)} via
     * {@code PlayerRef.removeFromStore()} (`universe/PlayerRef.java:161`), and
     * {@code onEntityRemove} runs while the pilot ref is still valid
     * (`component/Store.java:655-664`).
     *
     * <p>It deliberately ignores {@link RemoveReason}. Players are removed with {@code UNLOAD}
     * on <em>every</em> path including a permanent disconnect, so branching on the reason would
     * be branching on noise.
     */
    public static final class EndSessionOnPilotRemoved extends RefSystem<EntityStore> {

        @Nonnull
        private final FlightComponentTypes types;

        public EndSessionOnPilotRemoved(@Nonnull FlightComponentTypes types) {
            this.types = types;
        }

        @Nonnull
        @Override
        public Query<EntityStore> getQuery() {
            return this.types.flightSession();
        }

        @Override
        public void onEntityAdded(
                @Nonnull Ref<EntityStore> ref,
                @Nonnull AddReason reason,
                @Nonnull Store<EntityStore> store,
                @Nonnull CommandBuffer<EntityStore> commandBuffer) {
            // A session is never present on an entity being added: FlightSession is not
            // serialized, so it cannot arrive from disk, and launch() adds it to a live entity.
        }

        @Override
        public void onEntityRemove(
                @Nonnull Ref<EntityStore> ref,
                @Nonnull RemoveReason reason,
                @Nonnull Store<EntityStore> store,
                @Nonnull CommandBuffer<EntityStore> commandBuffer) {
            FlightSession session = commandBuffer.getComponent(ref, this.types.flightSession());
            if (session != null) {
                commandBuffer.tryRemoveEntity(session.getDrone(), RemoveReason.REMOVE);
            }
            // The parked body is deliberately NOT restored here. ParkedBody rides the pilot's
            // detached Holder and RestoreParkedBodyOnAdd unwinds it when they are next added to a
            // store -- next world, or next login. That is what makes a crash recoverable.
        }
    }

    /**
     * Clears the pilot's session when the drone dies by any route we did not initiate.
     *
     * <p>The symmetric guard. Without it, a drone removed by a chunk unload or
     * {@code /entity remove} leaves the pilot holding a stale ref: unable to relaunch, and
     * invisible and invulnerable with nothing to fly.
     *
     * <p>It cannot recurse with {@link EndSessionOnPilotRemoved}: removing a <em>component</em>
     * does not remove an entity, and {@code tryRemoveEntity}/{@code tryRemoveComponent} both
     * no-op on an invalid ref (`component/CommandBuffer.java:124-139`, `:243-252`).
     */
    public static final class ClearSessionOnDroneRemoved extends RefSystem<EntityStore> {

        @Nonnull
        private final FlightComponentTypes types;

        public ClearSessionOnDroneRemoved(@Nonnull FlightComponentTypes types) {
            this.types = types;
        }

        @Nonnull
        @Override
        public Query<EntityStore> getQuery() {
            return this.types.drone();
        }

        @Override
        public void onEntityAdded(
                @Nonnull Ref<EntityStore> ref,
                @Nonnull AddReason reason,
                @Nonnull Store<EntityStore> store,
                @Nonnull CommandBuffer<EntityStore> commandBuffer) {
            // The session is written by launch() right after addEntity returns, so there is
            // nothing to reconcile at add time.
        }

        @Override
        public void onEntityRemove(
                @Nonnull Ref<EntityStore> ref,
                @Nonnull RemoveReason reason,
                @Nonnull Store<EntityStore> store,
                @Nonnull CommandBuffer<EntityStore> commandBuffer) {
            DroneComponent drone = commandBuffer.getComponent(ref, this.types.drone());
            if (drone == null) {
                return;
            }

            Ref<EntityStore> pilot = drone.getPilot();
            // Routinely invalid: when the pilot goes first, Store.removeEntity invalidates their
            // ref (`component/Store.java:666-681`) before consuming the buffered drone removal
            // (`:686`). In that case the pilot's own teardown has it covered.
            if (!pilot.isValid()) {
                return;
            }

            FlightSession session = commandBuffer.getComponent(pilot, this.types.flightSession());
            // Identity, not equals: Ref does not override equals, and the store hands out one Ref
            // instance per entity, so the ref in the session is the same object as `ref`. The
            // check stops us clobbering a session that has already been pointed at a new drone.
            if (session == null || session.getDrone() != ref) {
                return;
            }

            commandBuffer.tryRemoveComponent(pilot, this.types.flightSession());
            FlightSessions.unpark(commandBuffer, pilot, this.types);
        }
    }

    /**
     * Restores a parked character when the pilot entity is added to a store.
     *
     * <p>Restoration hangs off <em>add</em> rather than remove, and that is the whole reason a
     * mid-flight crash is recoverable. {@link ParkedBody} and the markers it describes are all
     * serialized, so they survive together or not at all; whenever the pilot next appears in a
     * store — next world after a switch, next login after a crash — this unwinds them.
     * {@code FlightSessions.land} calls the same helper eagerly, so a voluntary land needs no
     * round trip.
     *
     * <p>Mutating the holder here is supported: {@code Store.addEntity} re-reads
     * {@code holder.getArchetype()} before testing each holder system
     * (`component/Store.java:409`), which is also what
     * {@code BuilderToolsUserDataSystem.onEntityAdd} relies on.
     */
    public static final class RestoreParkedBodyOnAdd extends HolderSystem<EntityStore> {

        @Nonnull
        private final FlightComponentTypes types;

        public RestoreParkedBodyOnAdd(@Nonnull FlightComponentTypes types) {
            this.types = types;
        }

        @Nonnull
        @Override
        public Query<EntityStore> getQuery() {
            return this.types.parkedBody();
        }

        @Override
        public void onEntityAdd(
                @Nonnull Holder<EntityStore> holder, @Nonnull AddReason reason, @Nonnull Store<EntityStore> store) {
            // Reason is ignored on purpose. A ParkedBody on an entity being added always means an
            // interrupted flight, whether the holder came off disk (LOAD) or out of another
            // world's store (SPAWN) -- FlightSession is not serialized, so the session is
            // definitionally gone either way.
            FlightSessions.unpark(holder, this.types);
        }

        @Override
        public void onEntityRemoved(
                @Nonnull Holder<EntityStore> holder, @Nonnull RemoveReason reason, @Nonnull Store<EntityStore> store) {
            // Deliberately empty. ParkedBody must survive on the outgoing Holder -- that is the
            // record the add-side restore reads.
        }
    }

    /**
     * Hides parked characters from every client, unconditionally.
     *
     * <p>Nothing in the server hides an entity unconditionally, so this is a near-verbatim copy of
     * {@code HideEntitySystems.AdventurePlayerSystem} (`modules/entity/system/HideEntitySystems.java:28-92`)
     * with its gamemode/settings gate dropped and {@link ParkedBody} in place of
     * {@code HiddenFromAdventurePlayers}. Pruning {@code EntityViewer.visible} after
     * {@code CollectVisible} is a real extension point — three builtin systems use it — and
     * {@code SendPackets} emits the despawn for free, because it treats "no longer visible" and
     * "was destroyed" identically (`modules/entity/tracker/EntityTrackerSystems.java:1029-1031`).
     *
     * <p>The alternatives were all checked and all rejected. Removing {@code ModelComponent} does
     * nothing — {@code EntityTrackerSystems.EntityModel}'s query requires it (`:488`) and asserts
     * it non-null (`:519`), so the null-model branch at `:545` is unreachable and no packet is
     * sent at all. A model scale of {@code 0} is the <em>unset</em> sentinel (`:520`). Removing
     * {@code NetworkId} is not reversible — the id is reallocated and
     * {@code PlayerSystems.sendPlayerSelf} throws without one (`PlayerSystems.java:571-573`). No
     * invisibility entity effect exists anywhere in the tree. And both built-in hide mechanisms
     * are conditional on client state: {@code HiddenPlayersManager} is gated on
     * {@code !showEntityMarkers()} (`EntityTrackerSystems.java:801`), a client-sent preference,
     * and {@code HiddenFromAdventurePlayers} on {@code gameMode == Adventure ||
     * !showEntityMarkers()} (`HideEntitySystems.java:81`).
     *
     * <p><strong>This is the most fragile file in the feature.</strong> It reaches into a public
     * mutable field of an internal class. It is also one system whose worst failure is a visible
     * parked body, never a crash, which is why the trade was worth taking.
     *
     * <p>The group and dependencies are constructor parameters because
     * {@code SystemGroup.validateRegistry} throws for a group belonging to another registry
     * (`component/SystemGroup.java:36-40`) — the plugin passes
     * {@code FIND_VISIBLE_ENTITIES_GROUP} and an {@code AFTER CollectVisible} dependency, and a
     * test passes {@code null} and an empty set so the pruning logic stays inside the harness.
     */
    public static final class HideParkedBody extends EntityTickingSystem<EntityStore> {

        @Nonnull
        private final FlightComponentTypes types;
        @Nonnull
        private final Query<EntityStore> query;
        @Nullable
        private final SystemGroup<EntityStore> group;
        @Nonnull
        private final Set<Dependency<EntityStore>> dependencies;

        public HideParkedBody(
                @Nonnull FlightComponentTypes types,
                @Nullable SystemGroup<EntityStore> group,
                @Nonnull Set<Dependency<EntityStore>> dependencies) {
            this.types = types;
            this.query = types.entityViewer();
            this.group = group;
            this.dependencies = dependencies;
        }

        @Nullable
        @Override
        public SystemGroup<EntityStore> getGroup() {
            return this.group;
        }

        @Nonnull
        @Override
        public Set<Dependency<EntityStore>> getDependencies() {
            return this.dependencies;
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

            EntityTrackerSystems.EntityViewer viewer =
                    archetypeChunk.getComponent(index, this.types.entityViewer());
            if (viewer == null) {
                return;
            }

            Iterator<Ref<EntityStore>> iterator = viewer.visible.iterator();
            while (iterator.hasNext()) {
                Ref<EntityStore> candidate = iterator.next();
                if (commandBuffer.getArchetype(candidate).contains(this.types.parkedBody())) {
                    viewer.hiddenCount++;
                    iterator.remove();
                }
            }
        }
    }
}
