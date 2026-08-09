package me.mzy.beamcraft.client.material;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.junit.jupiter.api.Test;

import java.util.HashMap;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Unit tests for {@link MaterialLibrary#resolveMaterial}, the pure material
 * lookup precedence: direct {@code mapTo} in the vehicle namespace, then the
 * namespace's static glowMap alias target (vehicle, then common), then the
 * existing common fallback for the original key. In-memory maps only — no
 * filesystem, no GL, no Minecraft renderer.
 */
class MaterialLibraryResolveTest {

    private static MaterialDefinition def(String json) {
        JsonObject obj = JsonParser.parseString(json).getAsJsonObject();
        MaterialDefinition definition = MaterialDefinition.fromJson("x", obj, "test");
        assertTrue(definition != null, "material JSON should parse to a definition");
        return definition;
    }

    private static MaterialDefinition material(String mapTo) {
        return def("{\"name\":\"" + mapTo + "\",\"mapTo\":\"" + mapTo + "\"}");
    }

    @Test
    void directMapToInVehicleNamespaceWinsOverAliasAndCommon() {
        Map<String, MaterialDefinition> vehicle = new HashMap<>();
        vehicle.put("body", material("body"));
        Map<String, String> aliases = new HashMap<>();
        aliases.put("body", "paint"); // must not be consulted: direct hit wins
        Map<String, MaterialDefinition> common = new HashMap<>();
        common.put("body", material("body_common"));

        MaterialDefinition resolved = MaterialLibrary.resolveMaterial(vehicle, aliases, common, "body");

        assertEquals("body", resolved.mapTo, "the direct vehicle mapTo must win");
    }

    @Test
    void aliasTargetResolvedInVehicleNamespace() {
        Map<String, MaterialDefinition> vehicle = new HashMap<>();
        vehicle.put("pickup_lightglass", material("pickup_lightglass"));
        Map<String, String> aliases = new HashMap<>();
        aliases.put("pickup_lowbeamglass", "pickup_lightglass");

        MaterialDefinition resolved = MaterialLibrary.resolveMaterial(vehicle, aliases, null, "pickup_lowbeamglass");

        assertEquals("pickup_lightglass", resolved.mapTo);
    }

    @Test
    void aliasTargetResolvedFromCommonWhenNotInVehicle() {
        Map<String, MaterialDefinition> vehicle = new HashMap<>(); // empty vehicle index
        Map<String, String> aliases = new HashMap<>();
        aliases.put("pickup_lowbeamglass", "pickup_lightglass");
        Map<String, MaterialDefinition> common = new HashMap<>();
        common.put("pickup_lightglass", material("pickup_lightglass"));

        MaterialDefinition resolved = MaterialLibrary.resolveMaterial(vehicle, aliases, common, "pickup_lowbeamglass");

        assertEquals("pickup_lightglass", resolved.mapTo, "the alias target in common must resolve");
    }

    @Test
    void noAliasFallsBackToCommonForDirectKey() {
        Map<String, MaterialDefinition> vehicle = new HashMap<>();
        Map<String, String> aliases = new HashMap<>();
        Map<String, MaterialDefinition> common = new HashMap<>();
        common.put("pickup_taillightglass_r", material("pickup_taillightglass_r"));

        MaterialDefinition resolved = MaterialLibrary.resolveMaterial(vehicle, aliases, common, "pickup_taillightglass_r");

        assertEquals("pickup_taillightglass_r", resolved.mapTo, "existing common fallback must still work");
    }

    @Test
    void brokenAliasTargetDegradesToCommonFallbackForOriginalKey() {
        // Alias exists but its target resolves nowhere: the lookup must degrade to
        // the existing common fallback for the ORIGINAL key, never get worse than
        // the pre-alias behaviour.
        Map<String, MaterialDefinition> vehicle = new HashMap<>();
        Map<String, String> aliases = new HashMap<>();
        aliases.put("weird", "missing_target");
        Map<String, MaterialDefinition> common = new HashMap<>();
        common.put("weird", material("weird"));

        MaterialDefinition resolved = MaterialLibrary.resolveMaterial(vehicle, aliases, common, "weird");

        assertEquals("weird", resolved.mapTo);
    }

    @Test
    void aliasesDoNotLeakAcrossNamespaces() {
        // Two namespaces share the same DAE material key, but only namespace A
        // has an alias. Resolving under B must not use A's alias.
        Map<String, MaterialDefinition> vehicleB = new HashMap<>();
        Map<String, String> aliasesB = new HashMap<>(); // B has no alias for the key
        Map<String, MaterialDefinition> common = new HashMap<>();

        MaterialDefinition resolved = MaterialLibrary.resolveMaterial(vehicleB, aliasesB, common, "shared_light");

        assertNull(resolved, "B's lookup must not see A's alias");
    }

    @Test
    void aliasTargetPreferredOverCommonDirectForKey() {
        // The DAE key itself also exists in common; the alias must redirect to the
        // lights-off material rather than the common definition of the key.
        Map<String, MaterialDefinition> vehicle = new HashMap<>();
        Map<String, String> aliases = new HashMap<>();
        aliases.put("pickup_headlightglass", "pickup_lightglass");
        Map<String, MaterialDefinition> common = new HashMap<>();
        common.put("pickup_headlightglass", material("pickup_headlightglass")); // the raw key exists in common
        common.put("pickup_lightglass", material("pickup_lightglass"));

        MaterialDefinition resolved = MaterialLibrary.resolveMaterial(vehicle, aliases, common, "pickup_headlightglass");

        assertEquals("pickup_lightglass", resolved.mapTo, "the alias target must win over the raw-key common hit");
    }

    @Test
    void nullMapsYieldNull() {
        assertNull(MaterialLibrary.resolveMaterial(null, null, null, "anything"));
        assertNull(MaterialLibrary.resolveMaterial(new HashMap<>(), new HashMap<>(), null, "anything"));
    }
}
