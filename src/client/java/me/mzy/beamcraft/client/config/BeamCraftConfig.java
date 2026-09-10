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

    /**
     * Optional physical input overrides consumed by the client input handler. Missing or
     * empty values use runtime defaults and are deliberately not written into the config.
     * Pedal actions may define linear keyboard ramp times in seconds.
     */
    public static final class Input {
        public KeyBinding exitVehicle;
        public DirectionalBinding steering;
        public AxisBinding throttle;
        public AxisBinding brake;
        public AxisBinding clutch;
        public KeyBinding starter;
        public KeyBinding shiftUp;
        public KeyBinding shiftDown;
        public KeyBinding rangeBoxToggle;
        public KeyBinding resetVehicle;

        /** Runtime defaults; these are deliberately not serialized into a new config file. */
        public static Input defaults() {
            Input defaults = new Input();
            defaults.exitVehicle = new KeyBinding("key.keyboard.left.shift");
            defaults.steering = new DirectionalBinding(
                    new AxisKey("key.keyboard.left", -1.0),
                    new AxisKey("key.keyboard.right", 1.0));
            defaults.throttle = new AxisBinding(0.15, 0.25,
                    new AxisKey("key.keyboard.up", 1.0));
            defaults.brake = new AxisBinding(0.05, 0.15,
                    new AxisKey("key.keyboard.down", 1.0));
            defaults.clutch = new AxisBinding(0.10, 0.10,
                    new AxisKey("key.keyboard.c", 1.0));
            defaults.starter = new KeyBinding("key.keyboard.v");
            defaults.shiftUp = new KeyBinding("key.keyboard.x");
            defaults.shiftDown = new KeyBinding("key.keyboard.z");
            defaults.rangeBoxToggle = new KeyBinding("key.keyboard.b");
            defaults.resetVehicle = new KeyBinding("key.keyboard.g");
            return defaults;
        }
    }

    public static class KeyBinding {
        public List<String> keys;

        public KeyBinding() {
        }

        public KeyBinding(String... keys) {
            this.keys = new ArrayList<>(List.of(keys));
        }
    }

    public static class DirectionalBinding {
        public List<AxisKey> keys;

        public DirectionalBinding() {
        }

        public DirectionalBinding(AxisKey... keys) {
            this.keys = new ArrayList<>(List.of(keys));
        }
    }

    public static final class AxisBinding extends DirectionalBinding {
        public Double riseTime;
        public Double fallTime;

        public AxisBinding() {
        }

        public AxisBinding(double riseTime, double fallTime, AxisKey... keys) {
            super(keys);
            this.riseTime = riseTime;
            this.fallTime = fallTime;
        }
    }

    public static final class AxisKey {
        public String key = "";
        public double value = 0.0;

        public AxisKey() {
        }

        public AxisKey(String key, double value) {
            this.key = key;
            this.value = value;
        }
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

            boolean changed = resetInvalidInputSection(json);
            changed |= mergeMissing(json, GSON.toJsonTree(new BeamCraftConfig()).getAsJsonObject());
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

    /** Keeps unrelated settings usable when the input section has an invalid schema. */
    private static boolean resetInvalidInputSection(JsonObject root) {
        JsonElement input = root.get("input");
        if (input == null || input.isJsonNull()) {
            return false;
        }
        try {
            GSON.fromJson(input, Input.class);
            return false;
        } catch (RuntimeException exception) {
            System.err.println("[BeamCraft] Invalid input config; resetting only the input section: "
                    + exception.getMessage());
            root.add("input", GSON.toJsonTree(new Input()));
            return true;
        }
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
