package com.maartenpeels.fpv.plugin.config;

import com.hypixel.hytale.codec.Codec;
import com.hypixel.hytale.codec.KeyedCodec;
import com.hypixel.hytale.codec.builder.BuilderCodec;

/**
 * Server-wide plugin configuration.
 *
 * <p>Deliberately holds only the two knobs CLAUDE.md decision 3 requires to stay cheap: the
 * substep count the integrator runs per tick, and the tick rate the drone world runs at. If
 * flight feels floaty at 30 TPS, raising {@code WorldTps} is the documented escape hatch, and
 * it has to remain a config change rather than a rewrite.
 *
 * <p>Per-pilot tuning (rates, PID, profiles) does <em>not</em> belong here — that is
 * {@code PilotProfile} in {@code :fpv-core}, persisted per player.
 */
public class FpvConfig {

    /** Hytale's own default; see {@code WorldTpsResetCommand} in the decompiled server. */
    public static final int DEFAULT_WORLD_TPS = 30;

    /** 8 substeps at 30 TPS is ~4 ms per integration step. */
    public static final int DEFAULT_PHYSICS_SUBSTEPS = 8;

    public static final BuilderCodec<FpvConfig> CODEC =
            BuilderCodec.builder(FpvConfig.class, FpvConfig::new)
                    .append(
                            new KeyedCodec<>("PhysicsSubsteps", Codec.INTEGER),
                            (config, value, extraInfo) -> config.physicsSubsteps = value,
                            (config, extraInfo) -> config.physicsSubsteps)
                    .add()
                    .append(
                            new KeyedCodec<>("WorldTps", Codec.INTEGER),
                            (config, value, extraInfo) -> config.worldTps = value,
                            (config, extraInfo) -> config.worldTps)
                    .add()
                    .build();

    private int physicsSubsteps = DEFAULT_PHYSICS_SUBSTEPS;
    private int worldTps = DEFAULT_WORLD_TPS;

    private FpvConfig() {}

    /** Fixed integration substeps per server tick. */
    public int getPhysicsSubsteps() {
        return this.physicsSubsteps;
    }

    /** Tick rate for the world drones fly in. {@code World.setTps} accepts up to 2048. */
    public int getWorldTps() {
        return this.worldTps;
    }

    /** Seconds of simulated time per integration substep at the configured tick rate. */
    public float substepSeconds() {
        return 1.0f / (this.worldTps * this.physicsSubsteps);
    }
}
