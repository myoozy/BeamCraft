package me.mzy.beamcraft.client.assets;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import java.io.File;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/**
 * BeamCraft user configuration, loaded from {@code <gameDir>/config/beamcraft.json}.
 *
 * <p>Fields are public and mirror the JSON exactly (Gson's default reflective
 * adapter), so the file stays hand-editable and new keys (e.g. future physics
 * constants) are added as new fields without touching the loader. Missing files
 * are created with defaults; a missing {@code assetRoots} defaults to the
 * historical hardcoded {@code mods/beamcraft/vehicles} so existing installs keep
 * working untouched.
 */
public final class BeamCraftConfig {

    public static final String FILE_NAME = "beamcraft.json";
    public static final String DEFAULT_ROOT = "mods/beamcraft/vehicles";

    /** Asset roots in load order; the first entry is the historical default. */
    public List<String> assetRoots = new ArrayList<>(List.of(DEFAULT_ROOT));

    public Conflict conflict = new Conflict();

    /** Conflict behaviour, separated from asset discovery. */
    public static final class Conflict {
        /** Surface a duplicate as a one-time in-game chat message. */
        public boolean notify = false;

        /** One of {@code "newer"}, {@code "later-root"}, {@code "earlier-root"}. */
        public String strategy = "later-root";
    }

    private static final Gson GSON = new GsonBuilder().setPrettyPrinting().create();

    private BeamCraftConfig() {
    }

    /**
     * Loads {@code <configDir>/beamcraft.json}, creating a default file (and the
     * parent directory) when it does not exist. On a parse failure logs a warning
     * and returns defaults without touching the user's file.
     */
    public static BeamCraftConfig load(Path configDir) {
        if (configDir == null) {
            return new BeamCraftConfig();
        }
        Path file = configDir.resolve(FILE_NAME);
        try {
            if (Files.notExists(file)) {
                Files.createDirectories(configDir);
                Files.writeString(file, GSON.toJson(new BeamCraftConfig()), StandardCharsets.UTF_8);
            }
            String json = Files.readString(file, StandardCharsets.UTF_8);
            BeamCraftConfig cfg = GSON.fromJson(json, BeamCraftConfig.class);
            if (cfg == null) {
                cfg = new BeamCraftConfig();
            }
            if (cfg.assetRoots == null || cfg.assetRoots.isEmpty()) {
                cfg.assetRoots = new ArrayList<>(List.of(DEFAULT_ROOT));
            }
            if (cfg.conflict == null) {
                cfg.conflict = new Conflict();
            }
            return cfg;
        } catch (Exception e) {
            System.err.println("⚠️ [BeamCraft] Failed to load config " + file + ": " + e.getMessage());
            return new BeamCraftConfig();
        }
    }

    /**
     * Resolves the configured asset roots to absolute files. Relative paths are
     * resolved against the game directory, so the default {@code "mods/beamcraft/vehicles"}
     * maps to the historical hardcoded location; absolute paths are used as-is.
     */
    public List<File> resolveAssetRoots(File gameDir) {
        List<File> out = new ArrayList<>();
        for (String root : assetRoots) {
            if (root == null || root.isBlank()) {
                continue;
            }
            String trimmed = root.trim();
            Path p = Path.of(trimmed);
            Path resolved = p.isAbsolute() ? p : gameDir == null ? p : gameDir.toPath().resolve(p);
            out.add(resolved.toAbsolutePath().normalize().toFile());
        }
        return out;
    }

    public ConflictStrategy strategy() {
        return ConflictStrategy.parse(conflict == null ? null : conflict.strategy);
    }

    public boolean notifyConflicts() {
        return conflict != null && conflict.notify;
    }

    public ConflictPolicy policy() {
        return new ConflictPolicy(strategy(), notifyConflicts());
    }
}
