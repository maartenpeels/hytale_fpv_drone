package com.maartenpeels;

import com.hypixel.hytale.server.core.plugin.JavaPlugin;
import com.hypixel.hytale.server.core.plugin.JavaPluginInit;
import com.hypixel.hytale.server.core.util.Config;
import com.maartenpeels.fpv.plugin.command.FpvCommand;
import com.maartenpeels.fpv.plugin.config.FpvConfig;

import javax.annotation.Nonnull;

/**
 * Plugin entry point. Must stay at this fully-qualified name — {@code main_class} in
 * {@code gradle.properties} points here and {@code manifest.json} is generated from it.
 *
 * <p>This class is an adapter and nothing more. Flight physics, rate curves, gate crossing
 * and race state live in {@code :fpv-core}; see CLAUDE.md decision 10.
 */
public class FPVDrone extends JavaPlugin {

    private final Config<FpvConfig> config;

    public FPVDrone(@Nonnull JavaPluginInit init) {
        super(init);
        this.config = this.withConfig("fpv_drone", FpvConfig.CODEC);
    }

    @Override
    protected void setup() {
        this.config.save();
        this.getCommandRegistry().registerCommand(new FpvCommand(this));
    }

    @Nonnull
    public Config<FpvConfig> getFpvConfig() {
        return this.config;
    }
}
