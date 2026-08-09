package me.mzy.beamcraft.client.material;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.junit.jupiter.api.Test;

import java.util.LinkedHashMap;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Unit tests for {@link GlowMapAliasExtractor}, the static lights-off material
 * alias extraction from BeamNG JBeam {@code glowMap} sections. The test inputs
 * mirror the real relaxed-JSON dialect (C-style comments, missing/trailing
 * commas) seen in pickup.jbeam and sunburst2.jbeam, and all content flows
 * through the shared {@link RelaxedJson} cleaner. Backend-neutral; no GL, no
 * Minecraft renderer.
 */
class GlowMapAliasExtractorTest {

    private static Map<String, String> extract(String relaxedJson) {
        Map<String, String> out = new LinkedHashMap<>();
        GlowMapAliasExtractor.collectFromJBeam(relaxedJson, out);
        return out;
    }

    @Test
    void pickupStyleGlowMapYieldsLightsOffAliases() {
        // Relaxed JSON with C-style comments and trailing commas, as in pickup.jbeam.
        String jbeam = """
                {
                "pickup_body":{
                    "information":{"authors":"BeamNG","name":"Body","value":100,},
                    // lamp covers
                    "glowMap":{
                        "pickup_taillight_R":{"simpleFunction":"lowhighBrakeSignal_R_filament", "off":"pickup_lights", "on":"pickup_lights_on", "materialEmissiveScaling":{"on_max":1.00}},
                        "pickup_lowbeamglass":{"simpleFunction":"lowbeam_filament", "off":"pickup_lightglass", "on":"pickup_lightglass_on", "materialEmissiveScaling":{"on_max":1.00},},
                        "pickup_headlightglass":{"simpleFunction":"lowhighbeam_filament", "off":"pickup_lightglass", "on":"pickup_lightglass_on",},
                    },
                },
                }
                """;
        Map<String, String> aliases = extract(jbeam);
        assertEquals(3, aliases.size());
        assertEquals("pickup_lights", aliases.get("pickup_taillight_r"));
        assertEquals("pickup_lightglass", aliases.get("pickup_lowbeamglass"));
        assertEquals("pickup_lightglass", aliases.get("pickup_headlightglass"));
    }

    @Test
    void sunburstStyleGlowMapMapsHeadlightGlassToGlass() {
        // A "sunburst2" DAE submesh such as sunburst2_headlightglass has no
        // mapTo of its own; the glowMap redirects it to the real sunburst2_glass
        // material. simpleFunction may be an object (multi-function).
        String jbeam = """
                {
                "sunburst2_headlight":{
                    "glowMap":{
                        "sunburst2_headlightglass":{"simpleFunction":{"lowbeam_filament":0.49,"highbeam_filament":1}, "off":"sunburst2_glass", "on":"sunburst2_glass_on", "on_intense":"sunburst2_glass_on_intense", "materialEmissiveScaling":{"on_max":1.00}},
                        "sunburst2_foglightglass":{"simpleFunction":"foglight_filament", "off":"sunburst2_glass", "on":"sunburst2_glass_on_intense", "materialEmissiveScaling":{"on_max":1.00}},
                        "sunburst2_signalglass_R":{"simpleFunction":"signal_R_filament", "off":"sunburst2_glass", "on":"sunburst2_glass_on_intense", "materialEmissiveScaling":{"on_max":0.49}},
                        "sunburst2_lowbeam":{"simpleFunction":"lowhighbeam_filament", "off":"sunburst2_lights", "on":"sunburst2_lights_on_intense", "materialEmissiveScaling":{"on_max":0.49}},
                    },
                },
                }
                """;
        Map<String, String> aliases = extract(jbeam);
        assertEquals("sunburst2_glass", aliases.get("sunburst2_headlightglass"));
        assertEquals("sunburst2_glass", aliases.get("sunburst2_foglightglass"));
        assertEquals("sunburst2_glass", aliases.get("sunburst2_signalglass_r"));
        assertEquals("sunburst2_lights", aliases.get("sunburst2_lowbeam"));
    }

    @Test
    void keysAndTargetsAreNormalisedToLowerCase() {
        String jbeam = """
                {
                "part":{"glowMap":{
                    "PICKUP_LowBeamGLASS":{"off":"Pickup_LightGlass", "on":"Pickup_LightGlass_on"},
                }},
                }
                """;
        Map<String, String> aliases = extract(jbeam);
        assertEquals(1, aliases.size());
        assertTrue(aliases.containsKey("pickup_lowbeamglass"), "alias keys must be lowercase");
        assertEquals("pickup_lightglass", aliases.get("pickup_lowbeamglass"));
    }

    @Test
    void entriesWithoutAStringOffAreSkipped() {
        String jbeam = """
                {
                "part":{"glowMap":{
                    "no_off_field":{"on":"x_on"},
                    "off_is_number":{"off":7},
                    "off_is_object":{"off":{"x":1}},
                    "off_is_empty":{"off":""},
                    "off_is_null":{"off":null},
                    "valid":{"off":"good"},
                }},
                }
                """;
        Map<String, String> aliases = extract(jbeam);
        assertEquals(1, aliases.size(), "only the entry with a non-empty string off survives");
        assertEquals("good", aliases.get("valid"));
    }

    @Test
    void nonObjectGlowMapValuesAndPartsAreSkipped() {
        String jbeam = """
                {
                "part":{
                    "glowMap":{
                        "value_is_string":"just a string",
                        "value_is_array":["a", "b"],
                        "value_is_number":3,
                        "ok":{"off":"target"},
                    },
                },
                "string_part":"not an object",
                "array_part":[],
                }
                """;
        Map<String, String> aliases = extract(jbeam);
        assertEquals(1, aliases.size(), "non-object glowMap values and non-object parts are skipped");
        assertEquals("target", aliases.get("ok"));
    }

    @Test
    void duplicateKeyInOneGlowMapKeepsLastOccurrence() {
        String jbeam = """
                {
                "part":{"glowMap":{
                    "shared":{"off":"first"},
                    "shared":{"off":"second"},
                }},
                }
                """;
        Map<String, String> aliases = extract(jbeam);
        assertEquals(1, aliases.size());
        assertEquals("second", aliases.get("shared"), "the last occurrence in scan order wins");
    }

    @Test
    void missingGlowMapContributesNothing() {
        String jbeam = """
                {
                "part_without_glowmap":{
                    "nodes":[["id","posX"]],
                    "beams":[["id1:","id2:"]],
                },
                }
                """;
        assertTrue(extract(jbeam).isEmpty());
    }

    @Test
    void onlyPartsWithGlowMapAreProcessed() {
        String jbeam = """
                {
                "partA":{"glowMap":{"alpha":{"off":"targetA"}}},
                "partB":{"glowMap":{"beta":{"off":"targetB"}}},
                "partC":{"nodes":[]},
                }
                """;
        Map<String, String> aliases = extract(jbeam);
        assertEquals(2, aliases.size());
        assertEquals("targeta", aliases.get("alpha"));
        assertEquals("targetb", aliases.get("beta"));
    }

    @Test
    void malformedFileDegradesToNoAliasesWithoutThrowing() {
        Map<String, String> aliases = extract("this is { not json, [at all] ::: ");
        assertTrue(aliases.isEmpty(), "a malformed JBeam file must not throw or abort the scan");
    }

    @Test
    void nullInputsAreNoOps() {
        Map<String, String> out = new LinkedHashMap<>();
        GlowMapAliasExtractor.collectFromJBeam(null, out);
        GlowMapAliasExtractor.collectFromJBeam("{}", null);
        GlowMapAliasExtractor.collect(null, out);
        GlowMapAliasExtractor.collect(new JsonObject(), null);
        GlowMapAliasExtractor.collectFromGlowMap(null, out);
        assertTrue(out.isEmpty());
    }

    @Test
    void emptyOffTargetIsNotRegisteredEvenWithEmptyKeyCheck() {
        JsonObject glowMap = JsonParser.parseString("{\"key\":{\"off\":\"\"}}").getAsJsonObject();
        Map<String, String> out = new LinkedHashMap<>();
        GlowMapAliasExtractor.collectFromGlowMap(glowMap, out);
        assertTrue(out.isEmpty());
        assertNull(out.get("key"));
    }
}
