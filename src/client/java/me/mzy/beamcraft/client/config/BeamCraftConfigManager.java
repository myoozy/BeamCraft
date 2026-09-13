package me.mzy.beamcraft.client.config;

import me.mzy.beamcraft.client.material.MaterialRenderPlanner;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.nio.file.Path;
import java.util.List;

/** Owns the single loaded config instance shared by client subsystems. */
public final class BeamCraftConfigManager {
    /**
     * A logger of this class's own rather than {@code BeamCraft.LOGGER}: reaching into
     * the mod initializer would run its static initializer, which registers an entity
     * type and therefore needs a bootstrapped Minecraft registry. That is fine in game
     * and fatal in a unit test.
     */
    private static final Logger LOGGER = LoggerFactory.getLogger("beamcraft");

    private static volatile BeamCraftConfig current;
    private static volatile List<File> assetRoots = List.of();
    /** Validated at load time so the render path only ever reads it. */
    private static volatile float cutoutAlphaRef = MaterialRenderPlanner.DEFAULT_CUTOUT_ALPHA_REF;

    private BeamCraftConfigManager() {
    }

    public static BeamCraftConfig initialize(Path configDir, File gameDir) {
        BeamCraftConfig loaded = BeamCraftConfig.load(configDir);
        current = loaded;
        assetRoots = List.copyOf(loaded.resolveAssetRoots(gameDir));
        cutoutAlphaRef = resolveCutoutAlphaRef(loaded);
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

    /**
     * Threshold used for a coverage mask that declares no {@code alphaRef} of its own
     * (see {@link MaterialRenderPlanner#plan(MaterialDefinition, float)}). Falls back to
     * the built-in default rather than throwing when no config was loaded, so a render
     * path can never be the thing that crashes on a missing config.
     */
    public static float cutoutAlphaRef() {
        return cutoutAlphaRef;
    }

    /**
     * Validates the configured value once, so the per-frame render path never has to.
     * Package-private so the validation is testable without initializing the singleton.
     */
    static float resolveCutoutAlphaRef(BeamCraftConfig config) {
        float fallback = BeamCraftConfig.Materials.defaults().cutoutAlphaRef.floatValue();
        Double configured = config == null || config.materials == null
                ? null
                : config.materials.cutoutAlphaRef;
        if (configured == null) {
            return fallback;
        }
        if (!(configured > 0.0 && configured < 1.0)) {
            LOGGER.warn(
                    "Ignoring materials.cutoutAlphaRef {}: it must be strictly between 0 and 1; using {}",
                    configured, fallback);
            return fallback;
        }
        return configured.floatValue();
    }
}
