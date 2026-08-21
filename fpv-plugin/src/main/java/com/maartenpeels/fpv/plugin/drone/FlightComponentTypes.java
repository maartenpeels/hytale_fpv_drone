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
 * <p>{@code FPVDrone.setup()} builds it for the server; the test source set builds one against a
 * {@code HytaleEcsHarness} registry. See CLAUDE.md's Conventions section for why systems never
 * resolve their own types.
 *
 * <p>Deliberately absent: {@code PersistentModel}. All four spawn precedents in the server
 * ({@code ProjectileModule}, {@code DeployablesUtils}, {@code SpawnMinecartInteraction},
 * {@code NPCPlugin}) add it, but it is only the <em>serialized</em> form of an appearance and a
 * drone is a {@code NonSerialized} entity. Verified safe to omit:
 * {@code ModelSystems.ModelChange.getQuery()} returns the {@code PersistentModel} type
 * (`modules/entity/system/ModelSystems.java:191-194`), so the
 * {@code assert persistentModel != null} at `:215` is unreachable for an entity that never had
 * one.
 *
 * @param flightSession our per-pilot session marker; see {@link FlightSession}
 * @param parkedBody our parked-character record; see {@link ParkedBody}
 * @param drone our drone marker; see {@link DroneComponent}
 * @param droneFlight the simulated flight state and the pilot id it flies on; see {@link DroneFlight}
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
        ComponentType<EntityStore, DroneFlight> droneFlight,
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
