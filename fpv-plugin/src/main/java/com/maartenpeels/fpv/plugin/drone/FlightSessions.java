package com.maartenpeels.fpv.plugin.drone;

import com.hypixel.hytale.component.AddReason;
import com.hypixel.hytale.component.Archetype;
import com.hypixel.hytale.component.ComponentAccessor;
import com.hypixel.hytale.component.Holder;
import com.hypixel.hytale.component.Ref;
import com.hypixel.hytale.component.RemoveReason;
import com.hypixel.hytale.component.Store;
import com.hypixel.hytale.math.shape.Box;
import com.hypixel.hytale.math.vector.Rotation3fc;
import com.hypixel.hytale.server.core.asset.type.model.config.Model;
import com.hypixel.hytale.server.core.entity.UUIDComponent;
import com.hypixel.hytale.server.core.modules.entity.component.BoundingBox;
import com.hypixel.hytale.server.core.modules.entity.component.HeadRotation;
import com.hypixel.hytale.server.core.modules.entity.component.Intangible;
import com.hypixel.hytale.server.core.modules.entity.component.Invulnerable;
import com.hypixel.hytale.server.core.modules.entity.component.ModelComponent;
import com.hypixel.hytale.server.core.modules.entity.component.TransformComponent;
import com.hypixel.hytale.server.core.universe.world.storage.EntityStore;
import org.joml.Vector3dc;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;

/**
 * Starts and ends flight sessions. The one place that knows where session state lives.
 *
 * <h2>The ownership model</h2>
 *
 * A pilot is flying exactly when their entity carries a {@link FlightSession}. Nothing else is
 * authoritative — there is no plugin-side {@code Map<UUID, ...>}, no registry of live drones, and
 * no shutdown hook. Two facts about the server make that sufficient, and both were read out of
 * the decompiled sources:
 *
 * <ol>
 *   <li><strong>Every player exit is one ECS operation.</strong>
 *       {@code PlayerRef.removeFromStore()} (`universe/PlayerRef.java:161`) is a single
 *       {@code Store.removeEntity(ref, RemoveReason.UNLOAD)}, and disconnect, kick, world switch
 *       and world crash all reach it — via {@code Universe.removePlayer}
 *       (`Universe.java:1136-1148`) and {@code World.stopIndividualWorld}
 *       (`World.java:286-291`). So one {@code RefSystem} keyed on {@link FlightSession} sees
 *       every exit exactly once, with the pilot ref still valid and a {@code CommandBuffer} in
 *       hand (`component/Store.java:655-664`). The reason is <em>always</em> {@code UNLOAD},
 *       never {@code REMOVE}, so nothing may branch on it.</li>
 *   <li><strong>A non-serialized entity cannot outlive the process.</strong> Drones carry
 *       {@code NonSerialized} (`component/ComponentRegistry.java:246`), the marker projectiles
 *       and dropped items use, so a drone is never written to a chunk or world save.
 *       {@code kill -9} mid-flight leaves nothing behind by construction — which is necessary,
 *       because the shutdown hooks are not trustworthy: {@code PluginManager.shutdown()} only
 *       calls a plugin's {@code shutdown()} when its state is {@code ENABLED}
 *       (`PluginManager.java:398`), and {@code Universe.shutdownAllWorlds()} never even fires
 *       {@code RemoveWorldEvent} (`Universe.java:678-686`).</li>
 * </ol>
 *
 * Eight exit paths — land, disconnect, kick, world switch, world unload, world crash, clean
 * shutdown, {@code kill -9} — therefore collapse to two ECS callbacks and one marker component.
 *
 * <h2>Threading</h2>
 *
 * {@link #launch} and {@link #land} must run on the world thread: {@code Store.addEntity} calls
 * {@code assertThread()} (`component/Store.java:361`). A caller that might be off-thread must hop
 * through {@code World.execute} (`World.java:750`).
 *
 * <p>They also must not be called from inside a system {@code tick} — {@code Store.addEntity}
 * calls {@code assertWriteProcessing()}. That is the CLAUDE.md convention, and it is why these
 * take a {@link Store} rather than a {@code CommandBuffer}: the signature refuses the misuse.
 */
public final class FlightSessions {

    @Nonnull
    private final FlightComponentTypes types;

    public FlightSessions(@Nonnull FlightComponentTypes types) {
        this.types = types;
    }

    /** Whether this pilot currently has a flight session. */
    public boolean isFlying(@Nonnull ComponentAccessor<EntityStore> accessor, @Nonnull Ref<EntityStore> pilot) {
        return accessor.getComponent(pilot, this.types.flightSession()) != null;
    }

    /** The drone this pilot is flying, or {@code null} if they are not flying. */
    @Nullable
    public Ref<EntityStore> droneOf(@Nonnull ComponentAccessor<EntityStore> accessor, @Nonnull Ref<EntityStore> pilot) {
        FlightSession session = accessor.getComponent(pilot, this.types.flightSession());
        return session == null ? null : session.getDrone();
    }

    /**
     * Spawns a drone for this pilot and parks their character.
     *
     * <p>Refuses and returns {@code null} if the pilot is already flying, which is what makes
     * "launch spawns exactly one drone" true without a race: the session component is the guard
     * and it is written in the same synchronous block as the spawn.
     *
     * @param model the drone's appearance, or {@code null} for an invisible drone. Nullable on
     *     purpose — resolving it needs {@code ModelAsset.getAssetMap()}, i.e. a loaded asset
     *     registry, which the test harness does not have. Keeping it a parameter puts the whole
     *     lifecycle inside the test loop and confines the untestable line to {@link DroneModel}.
     * @return the drone entity, or {@code null} if the pilot was already flying or a
     *     {@code RefSystem} rejected the spawn ({@code Store.addEntity} returns null in that
     *     case — `component/Store.java:453`)
     */
    @Nullable
    public Ref<EntityStore> launch(
            @Nonnull Store<EntityStore> store,
            @Nonnull Ref<EntityStore> pilot,
            @Nonnull Vector3dc position,
            @Nonnull Rotation3fc rotation,
            @Nullable Model model) {

        if (isFlying(store, pilot)) {
            return null;
        }

        // store.getRegistry(), not EntityStore.REGISTRY. Every spawn precedent in the server
        // hardcodes the static registry (ProjectileModule.java:115, DeployablesUtils.java:63);
        // going through the store is what lets this run against a test harness registry.
        Holder<EntityStore> holder = store.getRegistry().newHolder();
        holder.addComponent(this.types.transform(), new TransformComponent(position, rotation));
        holder.addComponent(this.types.headRotation(), new HeadRotation(rotation));
        holder.addComponent(this.types.uuid(), UUIDComponent.randomUUID());
        holder.addComponent(this.types.drone(), new DroneComponent(pilot));
        if (model != null) {
            holder.addComponent(this.types.model(), new ModelComponent(model));
            Box boundingBox = model.getBoundingBox();
            if (boundingBox != null) {
                holder.addComponent(this.types.boundingBox(), new BoundingBox(boundingBox));
            }
        }

        // NetworkId and NonSerialized are added by DroneLifecycleSystems.EnsureDroneNetworkSendable
        // during this call, not here -- see that system for why.
        Ref<EntityStore> drone = store.addEntity(holder, AddReason.SPAWN);
        if (drone == null) {
            return null;
        }

        store.putComponent(pilot, this.types.flightSession(), new FlightSession(drone));
        park(store, pilot);
        return drone;
    }

    /**
     * Ends the session: removes the drone and restores the pilot's character.
     *
     * @return {@code false} if the pilot was not flying, so a caller can report the misuse
     */
    public boolean land(@Nonnull Store<EntityStore> store, @Nonnull Ref<EntityStore> pilot) {
        FlightSession session = store.getComponent(pilot, this.types.flightSession());
        if (session == null) {
            return false;
        }

        // Order matters. Clearing the session and unparking before removing the drone means
        // ClearSessionOnDroneRemoved finds nothing left to do, so the two teardown routes stay
        // idempotent with respect to each other instead of racing.
        store.tryRemoveComponent(pilot, this.types.flightSession());
        unpark(store, pilot, this.types);

        Ref<EntityStore> drone = session.getDrone();
        if (drone.isValid()) {
            store.removeEntity(drone, RemoveReason.REMOVE);
        }
        return true;
    }

    /**
     * Makes the pilot's character invulnerable and intangible, and records which of those we
     * actually added.
     *
     * <p>Recording matters: Creative mode already adds {@code Invulnerable}
     * (`server/core/entity/entities/Player.java:752`), and stripping a marker the pilot owned
     * before the flight would be a bug. {@code Intangible} additionally drops the body out of
     * the collision spatial structure (`modules/collision/TangiableEntitySpatialSystem.java:18`),
     * which is what stops a parked body from blocking other entities.
     */
    private void park(@Nonnull Store<EntityStore> store, @Nonnull Ref<EntityStore> pilot) {
        Archetype<EntityStore> archetype = store.getArchetype(pilot);
        boolean addInvulnerable = !archetype.contains(this.types.invulnerable());
        boolean addIntangible = !archetype.contains(this.types.intangible());

        if (addInvulnerable) {
            store.putComponent(pilot, this.types.invulnerable(), Invulnerable.INSTANCE);
        }
        if (addIntangible) {
            store.putComponent(pilot, this.types.intangible(), Intangible.INSTANCE);
        }
        store.putComponent(pilot, this.types.parkedBody(), new ParkedBody(addInvulnerable, addIntangible));
    }

    /**
     * Undoes {@link #park}, taking away only the markers we added.
     *
     * <p>Takes a {@link ComponentAccessor} so the same code serves a {@link Store} (voluntary
     * land) and a {@code CommandBuffer} (a system tearing a session down); both implement it.
     */
    static void unpark(
            @Nonnull ComponentAccessor<EntityStore> accessor,
            @Nonnull Ref<EntityStore> pilot,
            @Nonnull FlightComponentTypes types) {

        ParkedBody parked = accessor.getComponent(pilot, types.parkedBody());
        if (parked == null) {
            return;
        }
        if (parked.addedInvulnerable()) {
            accessor.tryRemoveComponent(pilot, types.invulnerable());
        }
        if (parked.addedIntangible()) {
            accessor.tryRemoveComponent(pilot, types.intangible());
        }
        accessor.tryRemoveComponent(pilot, types.parkedBody());
    }

    /**
     * The {@link Holder} form of {@link #unpark}, for a pilot entity being added to a store.
     *
     * <p>This is the path that survives a crash. {@code Invulnerable} and {@code Intangible} are
     * serialized components (`modules/entity/EntityModule.java:328,330` register both with an id
     * and a codec), so a server that dies mid-flight would otherwise bring the pilot back
     * permanently invulnerable. {@link ParkedBody} is serialized by the same mechanism, so it and
     * the markers it describes can never disagree.
     */
    static void unpark(@Nonnull Holder<EntityStore> holder, @Nonnull FlightComponentTypes types) {
        ParkedBody parked = holder.getComponent(types.parkedBody());
        if (parked == null) {
            return;
        }
        if (parked.addedInvulnerable()) {
            holder.tryRemoveComponent(types.invulnerable());
        }
        if (parked.addedIntangible()) {
            holder.tryRemoveComponent(types.intangible());
        }
        holder.tryRemoveComponent(types.parkedBody());
    }
}
