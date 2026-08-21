package com.maartenpeels.fpv.plugin.camera;

import com.hypixel.hytale.component.ComponentAccessor;
import com.hypixel.hytale.component.ComponentType;
import com.hypixel.hytale.component.Ref;
import com.hypixel.hytale.server.core.receiver.IPacketReceiver;
import com.hypixel.hytale.server.core.universe.PlayerRef;
import com.hypixel.hytale.server.core.universe.world.storage.EntityStore;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;

/**
 * The real {@link PilotSink}: a pilot's packets go to their {@code PlayerRef}'s packet handler.
 *
 * <p>{@code PlayerRef} is itself an ECS component (`universe/PlayerRef.java:44`), so this is an
 * ordinary component lookup. The {@link ComponentType} arrives by constructor injection per
 * CLAUDE.md's ECS conventions — and here that is not merely style: {@code PlayerRef}'s static
 * {@code getComponentType()} resolves through {@code Universe.get()}, so calling it would tie this
 * class to a booted server for no reason.
 */
public final class PlayerRefSink implements PilotSink {

    @Nonnull
    private final ComponentType<EntityStore, PlayerRef> playerRefType;

    public PlayerRefSink(@Nonnull ComponentType<EntityStore, PlayerRef> playerRefType) {
        this.playerRefType = playerRefType;
    }

    @Nullable
    @Override
    public IPacketReceiver receiverFor(
            @Nonnull ComponentAccessor<EntityStore> accessor, @Nonnull Ref<EntityStore> pilot) {

        if (!pilot.isValid()) {
            return null;
        }
        PlayerRef playerRef = accessor.getComponent(pilot, this.playerRefType);
        return playerRef == null ? null : playerRef.getPacketHandler();
    }
}
