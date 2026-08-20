package com.maartenpeels.fpv.plugin.command;

import com.hypixel.hytale.server.core.Message;
import com.hypixel.hytale.server.core.command.system.AbstractCommand;
import com.hypixel.hytale.server.core.command.system.CommandContext;
import com.hypixel.hytale.server.core.permissions.provider.HytalePermissionsProvider;
import com.maartenpeels.FPVDrone;
import com.maartenpeels.fpv.plugin.config.FpvConfig;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;
import java.util.concurrent.CompletableFuture;

/**
 * Root {@code /fpv} command. Currently reports the resolved simulation settings, which is
 * what you need to confirm the plugin loaded and read its config.
 *
 * <p>{@code launch} and {@code land} land in phase 0 (CLAUDE.md roadmap).
 *
 * <p>{@code rolltest} and {@code rollreset} are throwaway spike instruments for #28 — delete
 * them once #19 attaches the camera to a real drone entity.
 */
public class FpvCommand extends AbstractCommand {

    private final FPVDrone plugin;

    public FpvCommand(@Nonnull FPVDrone plugin) {
        super("fpv", "FPV drone flight");
        this.setPermissionGroups(HytalePermissionsProvider.GROUP_ADVENTURER);
        this.plugin = plugin;
        this.addSubCommand(new FpvRollTestCommand());
        this.addSubCommand(new FpvRollResetCommand());
    }

    @Nullable
    @Override
    protected CompletableFuture<Void> execute(@Nonnull CommandContext context) {
        FpvConfig config = this.plugin.getFpvConfig().get();
        context.sendMessage(Message.raw(String.format(
                "FPV Drone — %d TPS x %d substeps (%.2f ms per step)",
                config.getWorldTps(),
                config.getPhysicsSubsteps(),
                config.substepSeconds() * 1000.0f)));
        return CompletableFuture.completedFuture(null);
    }
}
