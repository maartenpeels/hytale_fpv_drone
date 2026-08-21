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
 * <p>A pilot is flying exactly when their entity carries a {@link FlightSession}. Nothing else is
 * authoritative — no plugin-side {@code Map<UUID, ...>}, no registry of live drones, no shutdown
 * hook. Two verified facts make that sufficient: every player exit is one
 * {@code Store.removeEntity(ref, RemoveReason.UNLOAD)} via {@code PlayerRef.removeFromStore()}
 * (`universe/PlayerRef.java:161`), so a {@code RefSystem} on the session sees all of them; and a
 * {@code NonSerialized} drone is never written to a save, so a hard kill cannot leave one behind.
 * The reasoning, the rejected alternatives and the evidence are in {@code docs/plans/18.md}.
 *
 * <p><strong>Caller must be on the world thread, outside any system tick.</strong>
 * {@code Store.addEntity} calls both {@code assertThread()} and {@code assertWriteProcessing()}
 * (`component/Store.java:361-362`), so an off-thread caller must hop through
 * {@code World.execute} (`World.java:750`). Taking a {@link Store} documents the requirement but
 * does not enforce it — a system's {@code tick} is handed a {@code Store} too, and calling this
 * from one throws.
 */
public final class FlightSessions {

    @Nonnull
    private final FlightComponentTypes types;

    public FlightSessions(@Nonnull FlightComponentTypes types) {
        this.types = types;
    }

    public boolean isFlying(@Nonnull ComponentAccessor<EntityStore> accessor, @Nonnull Ref<EntityStore> pilot) {
        return accessor.getComponent(pilot, this.types.flightSession()) != null;
    }

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

    /** Undoes {@link #park}, taking away only the markers we added. */
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
     * The {@link Holder} form of {@link #unpark}, for a pilot entity being added to a store. This
     * is the path that survives a crash; see {@link ParkedBody}.
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
