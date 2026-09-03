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
    void createsConfigWithEmptyInputOverrides() throws Exception {
        BeamCraftConfig config = BeamCraftConfig.load(tempDir);

        assertEquals("", config.input.exitVehicle);
        assertEquals("", config.input.steerLeft);
        assertEquals("", config.input.steerRight);
        assertEquals("", config.input.throttle);
        assertEquals("", config.input.brake);
        assertEquals("", config.input.clutch);
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
        assertEquals("", config.input.steerLeft);
        assertEquals("", config.input.brake);
        assertEquals("", config.input.shiftUp);
        assertTrue(saved.has("futureSetting"));
        assertEquals("", saved.getAsJsonObject("input").get("shiftUp").getAsString());
    }
}
