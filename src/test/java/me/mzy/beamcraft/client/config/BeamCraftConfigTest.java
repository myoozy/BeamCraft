package me.mzy.beamcraft.client.config;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class BeamCraftConfigTest {
    @TempDir
    Path tempDir;

    @Test
    void createsConfigWithInputDefaults() throws Exception {
        BeamCraftConfig config = BeamCraftConfig.load(tempDir);

        assertEquals("key.keyboard.left", config.input.steerLeft);
        assertEquals("key.keyboard.right", config.input.steerRight);
        assertEquals("key.keyboard.up", config.input.throttle);
        assertEquals("key.keyboard.down", config.input.brake);
        assertEquals("key.keyboard.left.shift", config.input.clutch);
        assertTrue(Files.exists(tempDir.resolve(BeamCraftConfig.FILE_NAME)));
    }

    @Test
    void migratesMissingInputFieldsWithoutDiscardingUnknownFields() throws Exception {
        Path file = tempDir.resolve(BeamCraftConfig.FILE_NAME);
        Files.writeString(file, """
                {
                  "assetRoots": ["custom/vehicles"],
                  "futureSetting": {"enabled": true},
                  "input": {"throttle": "key.keyboard.up"}
                }
                """, StandardCharsets.UTF_8);

        BeamCraftConfig config = BeamCraftConfig.load(tempDir);
        JsonObject saved = JsonParser.parseString(Files.readString(file)).getAsJsonObject();

        assertEquals("key.keyboard.up", config.input.throttle);
        assertEquals("key.keyboard.left", config.input.steerLeft);
        assertEquals("key.keyboard.down", config.input.brake);
        assertEquals("key.keyboard.x", config.input.shiftUp);
        assertTrue(saved.has("futureSetting"));
        assertEquals("key.keyboard.x", saved.getAsJsonObject("input").get("shiftUp").getAsString());
    }
}
