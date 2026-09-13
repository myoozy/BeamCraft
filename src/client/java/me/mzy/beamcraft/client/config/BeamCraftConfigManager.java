package me.mzy.beamcraft.client.config;

import java.io.File;
import java.nio.file.Path;
import java.util.List;

/** Owns the single loaded config instance shared by client subsystems. */
public final class BeamCraftConfigManager {
    private static volatile BeamCraftConfig current;
    private static volatile List<File> assetRoots = List.of();

    private BeamCraftConfigManager() {
    }

    public static BeamCraftConfig initialize(Path configDir, File gameDir) {
        BeamCraftConfig loaded = BeamCraftConfig.load(configDir);
        current = loaded;
        assetRoots = List.copyOf(loaded.resolveAssetRoots(gameDir));
        return loaded;
    }

    public static BeamCraftConfig get() {
        BeamCraftConfig config = current;
        if (config == null) {
            throw new IllegalStateException("BeamCraft config has not been initialized");
        }
        return config;
    }

    public static List<File> assetRoots() {
        get();
        return assetRoots;
    }
}
