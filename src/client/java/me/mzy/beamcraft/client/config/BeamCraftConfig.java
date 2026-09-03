package me.mzy.beamcraft.client.config;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import me.mzy.beamcraft.client.assets.ConflictPolicy;
import me.mzy.beamcraft.client.assets.ConflictStrategy;

import java.io.File;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/** User configuration loaded from {@code <gameDir>/config/beamcraft.json}. */
public final class BeamCraftConfig {

    public static final String FILE_NAME = "beamcraft.json";
    public static final String DEFAULT_ROOT = "mods/beamcraft/vehicles";

    /** Asset roots in load order; the first entry is the historical default. */
    public List<String> assetRoots = new ArrayList<>(List.of(DEFAULT_ROOT));

    public Conflict conflict = new Conflict();
    public Input input = new Input();

    public static final class Conflict {
        public boolean notify = false;
        /** One of {@code newer}, {@code later-root}, or {@code earlier-root}. */
        public String strategy = "later-root";
    }

    /** Keyboard translation keys consumed by the client input handler. */
    public static final class Input {
        public String steerLeft = "key.keyboard.left";
        public String steerRight = "key.keyboard.right";
        public String throttle = "key.keyboard.up";
        public String brake = "key.keyboard.down";
        public String clutch = "key.keyboard.left.shift";
        public String starter = "key.keyboard.v";
        public String shiftUp = "key.keyboard.x";
        public String shiftDown = "key.keyboard.z";
        public String resetVehicle = "key.keyboard.g";
    }

    private static final Gson GSON = new GsonBuilder().setPrettyPrinting().create();

    BeamCraftConfig() {
    }

    /**
     * Loads the config and creates it when absent. Missing default fields are
     * merged back into an existing JSON object without discarding unknown keys.
     */
    public static BeamCraftConfig load(Path configDir) {
        if (configDir == null) {
            return new BeamCraftConfig();
        }

        Path file = configDir.resolve(FILE_NAME);
        try {
            Files.createDirectories(configDir);
            JsonObject json;
            if (Files.notExists(file)) {
                json = GSON.toJsonTree(new BeamCraftConfig()).getAsJsonObject();
            } else {
                JsonElement parsed = JsonParser.parseString(Files.readString(file, StandardCharsets.UTF_8));
                json = parsed.getAsJsonObject();
            }

            boolean changed = mergeMissing(json, GSON.toJsonTree(new BeamCraftConfig()).getAsJsonObject());
            if (Files.notExists(file) || changed) {
                Files.writeString(file, GSON.toJson(json), StandardCharsets.UTF_8);
            }

            BeamCraftConfig config = GSON.fromJson(json, BeamCraftConfig.class);
            return config == null ? new BeamCraftConfig() : config.normalize();
        } catch (Exception e) {
            System.err.println("[BeamCraft] Failed to load config " + file + ": " + e.getMessage());
            return new BeamCraftConfig();
        }
    }

    private BeamCraftConfig normalize() {
        if (assetRoots == null || assetRoots.isEmpty()) {
            assetRoots = new ArrayList<>(List.of(DEFAULT_ROOT));
        }
        if (conflict == null) {
            conflict = new Conflict();
        }
        if (input == null) {
            input = new Input();
        }
        return this;
    }

    private static boolean mergeMissing(JsonObject target, JsonObject defaults) {
        boolean changed = false;
        for (var entry : defaults.entrySet()) {
            String key = entry.getKey();
            JsonElement defaultValue = entry.getValue();
            if (!target.has(key) || target.get(key).isJsonNull()) {
                target.add(key, defaultValue.deepCopy());
                changed = true;
            } else if (target.get(key).isJsonObject() && defaultValue.isJsonObject()) {
                changed |= mergeMissing(target.getAsJsonObject(key), defaultValue.getAsJsonObject());
            }
        }
        return changed;
    }

    public List<File> resolveAssetRoots(File gameDir) {
        List<File> out = new ArrayList<>();
        for (String root : assetRoots) {
            if (root == null || root.isBlank()) {
                continue;
            }
            Path path = Path.of(root.trim());
            Path resolved = path.isAbsolute() ? path : gameDir == null ? path : gameDir.toPath().resolve(path);
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
