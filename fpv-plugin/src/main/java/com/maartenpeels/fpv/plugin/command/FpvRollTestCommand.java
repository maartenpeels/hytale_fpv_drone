package com.maartenpeels.fpv.plugin.command;

import com.hypixel.hytale.component.Ref;
import com.hypixel.hytale.component.Store;
import com.hypixel.hytale.protocol.ClientCameraView;
import com.hypixel.hytale.protocol.Direction;
import com.hypixel.hytale.protocol.RotationType;
import com.hypixel.hytale.protocol.ServerCameraSettings;
import com.hypixel.hytale.protocol.packets.camera.SetServerCamera;
import com.hypixel.hytale.server.core.Message;
import com.hypixel.hytale.server.core.command.system.CommandContext;
import com.hypixel.hytale.server.core.command.system.arguments.system.OptionalArg;
import com.hypixel.hytale.server.core.command.system.arguments.types.ArgTypes;
import com.hypixel.hytale.server.core.command.system.basecommands.AbstractTargetPlayerCommand;
import com.hypixel.hytale.server.core.permissions.provider.HytalePermissionsProvider;
import com.hypixel.hytale.server.core.universe.PlayerRef;
import com.hypixel.hytale.server.core.universe.world.World;
import com.hypixel.hytale.server.core.universe.world.storage.EntityStore;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;

/**
 * Throwaway instrument for the #28 spike: does the client actually <em>render</em> camera roll?
 *
 * <p>CLAUDE.md verifies that {@code Direction} carries a {@code roll} field and that
 * {@code ServerCameraSettings} can express a custom rotation — that is representability. It does
 * not establish that the client honours roll, and CLAUDE.md decision 4 (and therefore all of
 * phase 0) assumes it does. This command exists to settle that by looking at a horizon.
 *
 * <p>Delete it once #19 attaches the camera to a real drone entity. It is scaffolding for two
 * questions, not the beginning of a camera subsystem.
 *
 * <h2>Why two modes</h2>
 *
 * <p>{@link RotationType#Custom} sets an absolute camera rotation, which freezes yaw and pitch.
 * That answers "can the client draw a rolled camera at all", but it is <em>not</em> the shape
 * decision 4 needs: a drone camera tracks the aircraft's yaw and pitch and rolls on top of them.
 * {@link RotationType#AttachedToPlusOffset} with a {@code rotationOffset} tests that composition
 * instead. Roll could plausibly work in one mode and not the other, so the spike tries both.
 *
 * <p>The angle is an argument for the same reason: one hard-coded value cannot tell "roll works"
 * apart from "roll is clamped" or "roll snaps to increments". Sweep 0/15/30/90/180.
 */
public class FpvRollTestCommand extends AbstractTargetPlayerCommand {

    private static final float DEFAULT_DEGREES = 30.0f;

    @Nonnull
    private final OptionalArg<Float> degreesArg =
            this.withOptionalArg("degrees", "Roll angle in degrees (default 30)", ArgTypes.FLOAT);

    @Nonnull
    private final OptionalArg<Boolean> offsetArg = this.withOptionalArg(
            "offset",
            "true to add roll as an offset on top of the tracked look direction "
                    + "(AttachedToPlusOffset) instead of an absolute rotation (Custom)",
            ArgTypes.BOOLEAN);

    public FpvRollTestCommand() {
        super("rolltest", "Roll the camera by a fixed angle to see whether the client renders it");
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

        float degrees = this.degreesArg.provided(context)
                ? this.degreesArg.get(context)
                : DEFAULT_DEGREES;

        // A non-finite angle would produce a NaN roll, which renders as no tilt at all — and
        // would be read as "the client ignores roll", the exact false negative this spike exists
        // to avoid.
        if (!Float.isFinite(degrees)) {
            context.sendMessage(Message.raw("Roll angle must be a finite number."));
            return;
        }

        boolean asOffset = this.offsetArg.provided(context) && this.offsetArg.get(context);

        // Direction is radians, not degrees: PlayerCameraTopdownCommand passes -PI/2 to look
        // straight down. Passing degrees here would be ~4.8 rotations for a 30 that reads as mild.
        Direction roll = new Direction(0.0f, 0.0f, (float) Math.toRadians(degrees));

        ServerCameraSettings settings = new ServerCameraSettings();
        settings.isFirstPerson = true;
        if (asOffset) {
            settings.rotationType = RotationType.AttachedToPlusOffset;
            settings.rotationOffset = roll;
        } else {
            settings.rotationType = RotationType.Custom;
            settings.rotation = roll;
        }

        // Locking is what makes an absolute rotation observable. In offset mode the whole point is
        // roll composed with a live look direction, so leaving it unlocked is deliberate — though
        // isLocked's exact effect is itself one of the unknowns to record on #28.
        boolean locked = !asOffset;

        playerRef.getPacketHandler()
                .writeNoCache(new SetServerCamera(ClientCameraView.Custom, locked, settings));

        context.sendMessage(Message.raw(String.format(
                "Roll %.1f deg (%.4f rad), mode=%s, locked=%s. %s /fpv rollreset to undo.",
                degrees,
                Math.toRadians(degrees),
                asOffset ? "AttachedToPlusOffset" : "Custom",
                locked,
                asOffset
                        ? "Mouse look should still work; roll should ride on top of it."
                        : "The view snaps to a fixed yaw/pitch and stops following the mouse — "
                                + "that is expected, not the bug you are looking for.")));
    }
}
