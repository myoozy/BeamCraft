package me.mzy.beamcraft.client.config;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class BeamCraftConfigTest {
    @TempDir
    Path tempDir;

    @Test
    void createsSparseConfigWithoutSerializingInputDefaults() throws Exception {
        BeamCraftConfig config = BeamCraftConfig.load(tempDir);
        JsonObject saved = JsonParser.parseString(Files.readString(
                tempDir.resolve(BeamCraftConfig.FILE_NAME))).getAsJsonObject();

        assertEquals(0, saved.getAsJsonObject("input").size());
        assertEquals(null, config.input.exitVehicle);
        assertEquals(null, config.input.steering);
        assertEquals(null, config.input.throttle);
        assertEquals(512, config.vehicleCache.sleepingMemoryMiB);
        assertEquals(0.85, config.vehicleCache.heapHighWatermark);
        assertEquals(512L * 1024L * 1024L, config.sleepingVehicleCacheBytes());

        BeamCraftConfig.Input defaults = BeamCraftConfig.Input.defaults();
        assertEquals(List.of("key.keyboard.left.shift"), defaults.exitVehicle.keys);
        assertEquals("key.keyboard.left", defaults.steering.keys.getFirst().key);
        assertEquals(0.15, defaults.throttle.riseTime);
        assertTrue(Files.exists(tempDir.resolve(BeamCraftConfig.FILE_NAME)));
    }

    @Test
    void migratesMissingInputFieldsWithoutDiscardingUnknownFields() throws Exception {
        Path file = tempDir.resolve(BeamCraftConfig.FILE_NAME);
        Files.writeString(file, """
                {
                  "assetRoots": ["custom/vehicles"],
                  "futureSetting": {"enabled": true},
                  "input": {
                    "throttle": {
                      "keys": [
                        {"key": "key.keyboard.up", "value": 1.0},
                        {"key": "key.keyboard.w", "value": 1.0}
                      ],
                      "riseTime": 0.4,
                      "fallTime": 0.6
                    }
                  }
                }
                """, StandardCharsets.UTF_8);

        BeamCraftConfig config = BeamCraftConfig.load(tempDir);
        JsonObject saved = JsonParser.parseString(Files.readString(file)).getAsJsonObject();

        assertEquals(List.of("key.keyboard.up", "key.keyboard.w"),
                config.input.throttle.keys.stream().map(key -> key.key).toList());
        assertEquals(0.4, config.input.throttle.riseTime);
        assertEquals(0.6, config.input.throttle.fallTime);
        assertEquals(null, config.input.steering);
        assertEquals(null, config.input.brake);
        assertEquals(null, config.input.shiftUp);
        assertTrue(saved.has("futureSetting"));
        assertTrue(!saved.getAsJsonObject("input").has("shiftUp"));
    }

    @Test
    void invalidInputSectionDoesNotDiscardAssetSettings() throws Exception {
        Path file = tempDir.resolve(BeamCraftConfig.FILE_NAME);
        Files.writeString(file, """
                {
                  "assetRoots": ["custom/vehicles", "external/vehicles"],
                  "conflict": {"notify": true, "strategy": "earlier-root"},
                  "input": {
                    "exitVehicle": "",
                    "throttle": "key.keyboard.up"
                  }
                }
                """, StandardCharsets.UTF_8);

        BeamCraftConfig config = BeamCraftConfig.load(tempDir);
        JsonObject saved = JsonParser.parseString(Files.readString(file)).getAsJsonObject();

        assertEquals(List.of("custom/vehicles", "external/vehicles"), config.assetRoots);
        assertTrue(config.conflict.notify);
        assertEquals("earlier-root", config.conflict.strategy);
        assertEquals(null, config.input.steering);
        assertEquals(0, saved.getAsJsonObject("input").size());
        assertEquals("custom/vehicles", saved.getAsJsonArray("assetRoots").get(0).getAsString());
    }

    @Test
    void explicitZeroRampTimeIsPreservedWhileMissingFieldsStayUnset() throws Exception {
        Path file = tempDir.resolve(BeamCraftConfig.FILE_NAME);
        Files.writeString(file, """
                {
                  "input": {
                    "throttle": {
                      "keys": [],
                      "riseTime": 0
                    }
                  }
                }
                """, StandardCharsets.UTF_8);

        BeamCraftConfig config = BeamCraftConfig.load(tempDir);
        JsonObject saved = JsonParser.parseString(Files.readString(file)).getAsJsonObject();

        assertEquals(0.0, config.input.throttle.riseTime);
        assertEquals(null, config.input.throttle.fallTime);
        assertTrue(config.input.throttle.keys.isEmpty());
        JsonObject throttle = saved.getAsJsonObject("input").getAsJsonObject("throttle");
        assertEquals(0.0, throttle.get("riseTime").getAsDouble());
        assertTrue(!throttle.has("fallTime"));
    }

    @Test
    void normalizesUnsafeVehicleCacheSettings() throws Exception {
        Path file = tempDir.resolve(BeamCraftConfig.FILE_NAME);
        Files.writeString(file, """
                {
                  "vehicleCache": {
                    "sleepingMemoryMiB": -20,
                    "heapHighWatermark": 1.5
                  }
                }
                """, StandardCharsets.UTF_8);

        BeamCraftConfig config = BeamCraftConfig.load(tempDir);

        assertEquals(0, config.vehicleCache.sleepingMemoryMiB);
        assertEquals(0.85, config.vehicleCache.heapHighWatermark);
        assertEquals(0L, config.sleepingVehicleCacheBytes());
    }
}
