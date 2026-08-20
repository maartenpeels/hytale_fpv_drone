package com.maartenpeels.fpv.plugin.command;

import com.hypixel.hytale.component.Ref;
import com.hypixel.hytale.component.Store;
import com.hypixel.hytale.protocol.ClientCameraView;
import com.hypixel.hytale.protocol.packets.camera.SetServerCamera;
import com.hypixel.hytale.server.core.Message;
import com.hypixel.hytale.server.core.command.system.CommandContext;
import com.hypixel.hytale.server.core.command.system.basecommands.AbstractTargetPlayerCommand;
import com.hypixel.hytale.server.core.permissions.provider.HytalePermissionsProvider;
import com.hypixel.hytale.server.core.universe.PlayerRef;
import com.hypixel.hytale.server.core.universe.world.World;
import com.hypixel.hytale.server.core.universe.world.storage.EntityStore;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;

/**
 * Escape hatch for {@link FpvRollTestCommand}. A locked camera at 90 degrees of roll leaves the
 * pilot looking sideways with no way back, so the instrument ships with its own undo.
 *
 * <p>Null settings with {@code isLocked = false} is how {@code PlayerCameraResetCommand} hands
 * control back to the client.
 */
public class FpvRollResetCommand extends AbstractTargetPlayerCommand {

    public FpvRollResetCommand() {
        super("rollreset", "Hand the camera back to the client after /fpv rolltest");
        this.setPermissionGroups(HytalePermissionsProvider.GROUP_ADVENTURER);
    }

    @Override
    protected void execute(
            @Nonnull CommandContext context,
            @Nullable Ref<EntityStore> sourceRef,
            @Nonnull Ref<EntityStore> ref,
            @Nonnull PlayerRef playerRef,
            @Nonnull World world,
            @Nonnull Store<EntityStore> store) {

        playerRef.getPacketHandler()
                .writeNoCache(new SetServerCamera(ClientCameraView.Custom, false, null));

        context.sendMessage(Message.raw("Camera reset."));
    }
}
