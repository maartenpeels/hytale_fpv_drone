package com.maartenpeels.fpv.plugin.drone;

import com.hypixel.hytale.component.ComponentType;
import com.hypixel.hytale.component.NonSerialized;
import com.hypixel.hytale.server.core.entity.UUIDComponent;
import com.hypixel.hytale.server.core.modules.entity.component.BoundingBox;
import com.hypixel.hytale.server.core.modules.entity.component.HeadRotation;
import com.hypixel.hytale.server.core.modules.entity.component.Intangible;
import com.hypixel.hytale.server.core.modules.entity.component.Invulnerable;
import com.hypixel.hytale.server.core.modules.entity.component.ModelComponent;
import com.hypixel.hytale.server.core.modules.entity.component.TransformComponent;
import com.hypixel.hytale.server.core.modules.entity.tracker.EntityTrackerSystems;
import com.hypixel.hytale.server.core.modules.entity.tracker.NetworkId;
import com.hypixel.hytale.server.core.universe.world.storage.EntityStore;

/**
 * Every {@link ComponentType} the drone feature touches, resolved once and passed everywhere.
 *
 * <p>This exists because of the constructor-injection convention in CLAUDE.md, and it exists as
 * one object rather than thirteen constructor parameters per system. The static accessors on
 * Hytale's components — {@code TransformComponent.getComponentType()} and friends — resolve
 * through {@code EntityModule.get()}, whose singleton is only assigned in a booted server
 * (`modules/entity/EntityModule.java:304`), so anything that calls them is untestable. The
 * caller resolves them; {@code FPVDrone.setup()} does that for the server, and the test source
 * set builds an instance against a {@code HytaleEcsHarness} registry.
 *
 * <p>All thirteen have distinct type arguments, so the compiler catches a mis-ordered
 * construction rather than leaving it to fail at runtime.
 *
 * <p>Deliberately absent: {@code PersistentModel}. All four spawn precedents in the server
 * ({@code ProjectileModule}, {@code DeployablesUtils}, {@code SpawnMinecartInteraction},
 * {@code NPCPlugin}) add it, but it is only the <em>serialized</em> form of an appearance and a
 * drone is a {@code NonSerialized} entity, so there is nothing to serialize. Verified safe to
 * omit: {@code ModelSystems.ModelChange.getQuery()} returns the {@code PersistentModel} type
 * (`modules/entity/system/ModelSystems.java:191-194`), so its {@code assert persistentModel
 * != null` at `:215` is unreachable for an entity that never had one.
 *
 * @param flightSession our per-pilot session marker; see {@link FlightSession}
 * @param parkedBody our parked-character record; see {@link ParkedBody}
 * @param drone our drone marker; see {@link DroneComponent}
 * @param transform position and rotation
 * @param headRotation look direction, read by {@code TransformSystems.EntityTrackerUpdate}
 * @param uuid stable identity, needed for UUID lookup
 * @param networkId the client-facing id; with {@code transform} it is what makes an entity
 *     network-sendable at all (`modules/entity/system/NetworkSendableSpatialSystem.java:17`)
 * @param nonSerialized the marker that keeps a drone out of every save file
 * @param invulnerable body marker, applied while parked
 * @param intangible body marker, applied while parked
 * @param model appearance
 * @param boundingBox collision box, derived from the model
 * @param entityViewer the per-player tracker state {@code HideParkedBody} prunes
 */
public record FlightComponentTypes(
        ComponentType<EntityStore, FlightSession> flightSession,
        ComponentType<EntityStore, ParkedBody> parkedBody,
        ComponentType<EntityStore, DroneComponent> drone,
        ComponentType<EntityStore, TransformComponent> transform,
        ComponentType<EntityStore, HeadRotation> headRotation,
        ComponentType<EntityStore, UUIDComponent> uuid,
        ComponentType<EntityStore, NetworkId> networkId,
        ComponentType<EntityStore, NonSerialized<EntityStore>> nonSerialized,
        ComponentType<EntityStore, Invulnerable> invulnerable,
        ComponentType<EntityStore, Intangible> intangible,
        ComponentType<EntityStore, ModelComponent> model,
        ComponentType<EntityStore, BoundingBox> boundingBox,
        ComponentType<EntityStore, EntityTrackerSystems.EntityViewer> entityViewer) {}
