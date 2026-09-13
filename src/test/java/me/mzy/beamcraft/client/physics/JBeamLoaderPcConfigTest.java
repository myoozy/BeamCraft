package me.mzy.beamcraft.client.physics;

import com.google.gson.JsonObject;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Covers the {@code .pc} selection rules in {@link JBeamLoader} against a fixture
 * container laid out the BeamNG way ({@code <container>/vehicles/<name>/…}).
 *
 * <p>Two behaviours matter here and neither is exercised by the corpus tests,
 * which self-skip when the real vehicle assets are absent:
 * <ul>
 *   <li>the config name is matched case-insensitively, consistent with the
 *       case-insensitive vehicle-name matching in {@code AssetScanner};</li>
 *   <li>a named-but-missing config is a hard failure. It must not be mistaken for
 *       "no config requested", because an empty {@code userConfig} does not fail —
 *       it quietly assembles every slot from its default part.</li>
 * </ul>
 */
class JBeamLoaderPcConfigTest {

    @TempDir
    Path root;

    private Path container() {
        return root.resolve("mymod");
    }

    /** Creates {@code mymod/vehicles/foo/} with a part file and (optionally) a config. */
    private void writeVehicle(String pcName) throws IOException {
        Path vehicle = container().resolve("vehicles/foo");
        Files.createDirectories(vehicle);
        Files.writeString(vehicle.resolve("foo.jbeam"), "{\"foo\": {}}", StandardCharsets.UTF_8);
        if (pcName != null) {
            // On-disk casing is deliberately different from what the tests request.
            Files.writeString(vehicle.resolve(pcName),
                    "{\"parts\": {\"body\": \"foo_body\"}}", StandardCharsets.UTF_8);
        }
    }

    /** Runs the loader against the fixture root and returns the resolved slot config. */
    private Map<String, String> load(String pcName) {
        Map<String, String> config = new HashMap<>();
        // A throwaway registry: this test is about config selection, not part parsing.
        JBeamLoader.loadVehicle(List.of(root.toFile()), "foo", pcName, new HashMap<>(), config);
        return config;
    }

    private boolean loadInto(String pcName, Map<String, String> config) {
        Map<String, JsonObject> registry = new HashMap<>();
        return JBeamLoader.loadVehicle(List.of(root.toFile()), "foo", pcName, registry, config);
    }

    @Test
    void exactConfigNameIsResolved() throws IOException {
        writeVehicle("Foo.pc");

        assertEquals(Map.of("body", "foo_body"), load("Foo.pc"));
    }

    @Test
    void configNameMatchesCaseInsensitively() throws IOException {
        writeVehicle("Foo.pc");

        assertEquals(Map.of("body", "foo_body"), load("foo.pc"),
                "a lowercased request must still resolve Foo.pc");
    }

    @Test
    void uppercaseRequestResolvesAndSuffixIsOptional() throws IOException {
        writeVehicle("foo.pc");

        assertEquals(Map.of("body", "foo_body"), load("FOO.PC"),
                "the .pc suffix check must be case-insensitive too");
        assertEquals(Map.of("body", "foo_body"), load("foo"),
                "a request without the .pc suffix gets it appended");
    }

    @Test
    void unknownConfigNameFailsLoudly() throws IOException {
        writeVehicle("Foo.pc");

        Map<String, String> config = new HashMap<>();
        boolean resolved = loadInto("nope.pc", config);

        assertFalse(resolved, "a named-but-missing .pc must be reported as a failure");
        assertTrue(config.isEmpty(),
                "no slots may be resolved, or the vehicle would assemble with default parts");
    }

    @Test
    void vehicleWithoutAnyConfigStillLoads() throws IOException {
        // A .pc is not guaranteed: a vehicle may ship only .jbeam parts and rely on
        // every slot's authored default. Omitting the name is a valid request.
        writeVehicle(null);

        Map<String, String> config = new HashMap<>();
        boolean resolved = loadInto(null, config);

        assertTrue(resolved, "omitting the config name is not a failure");
        assertTrue(config.isEmpty());
    }

    @Test
    void commonConfigsAreNeverSelected() throws IOException {
        // The common namespace holds shared parts; its .pc files must not be
        // eligible to satisfy a vehicle's config request.
        Path common = container().resolve("vehicles/common");
        Files.createDirectories(common);
        Files.writeString(common.resolve("shared.pc"),
                "{\"parts\": {\"body\": \"shared_body\"}}", StandardCharsets.UTF_8);
        writeVehicle(null);

        Map<String, String> config = new HashMap<>();
        boolean resolved = loadInto("shared.pc", config);

        assertFalse(resolved, "a .pc living under vehicles/common must not match a vehicle request");
        assertTrue(config.isEmpty());
    }
}
