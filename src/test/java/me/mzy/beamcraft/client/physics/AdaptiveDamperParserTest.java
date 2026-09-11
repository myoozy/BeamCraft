package me.mzy.beamcraft.client.physics;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import me.mzy.beamcraft.client.material.RelaxedJson;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Build-time parsing of the BeamNG {@code adaptiveDampers} controller declaration.
 *
 * <p>The reference is the stock ETK800 {@code etk800_shock_R_wide_adaptive} part:
 * a {@code controller} row naming {@code drivingDynamics/actuators/adaptiveDampers}
 * with a {@code dampBeamNames} option, plus a {@code modes} header table on the
 * named controller object.
 */
class AdaptiveDamperParserTest {

    /** Verbatim ETK800 rear adaptive shock part, relaxed JBeam syntax included. */
    private static final String ETK_REAR_ADAPTIVE_PART = """
            {
            "controller": [
                ["fileName"],
                ["drivingDynamics/actuators/adaptiveDampers" {"name":"adaptiveRearDamper", "dampBeamNames":["shock_RR", "shock_RL"]}]
            ],
            "adaptiveRearDamper": {
                "modes": [
                    ["name",    "beamDampCoef", "beamDampFastCoef", "beamDampReboundCoef", "beamDampReboundFastCoef","beamDampVelocitySplitCoef"]
                    ["soft",    0.7,            1,                  0.65,                  1,                        0.7]
                    ["regular", 1,              1,                  1,                     1,                        1]
                    ["hard",    1.5,            1,                  1.4,                   1,                        1.2]
                ]
            }
            }
            """;

    // --- file name normalization -------------------------------------------

    @Test
    void normalizesControllerFileNames() {
        assertEquals("drivingdynamics/actuators/adaptivedampers",
                AdaptiveDamperParser.normalizeControllerFileName("drivingDynamics/actuators/adaptiveDampers"));
        // A mod may spell the whole loader path; matching is suffix-based.
        assertEquals("lua/vehicle/controller/drivingdynamics/actuators/adaptivedampers",
                AdaptiveDamperParser.normalizeControllerFileName(
                        "  lua\\vehicle\\controller\\drivingDynamics\\actuators\\adaptiveDampers.lua  "));
        assertEquals("drivingdynamics/actuators/adaptivedampers",
                AdaptiveDamperParser.normalizeControllerFileName("//drivingDynamics//actuators/adaptiveDampers/"));
        assertNull(AdaptiveDamperParser.normalizeControllerFileName("   "));
        assertNull(AdaptiveDamperParser.normalizeControllerFileName(null));
    }

    @Test
    void acceptsOnlyTheAdaptiveDamperControllerFile() {
        assertTrue(AdaptiveDamperParser.isAdaptiveDamperFileName("drivingDynamics/actuators/adaptiveDampers"));
        assertTrue(AdaptiveDamperParser.isAdaptiveDamperFileName(
                "lua/vehicle/controller/drivingDynamics/actuators/adaptiveDampers.lua"));
        assertFalse(AdaptiveDamperParser.isAdaptiveDamperFileName("drivingDynamics/actuators/activeDifferential"));
        assertFalse(AdaptiveDamperParser.isAdaptiveDamperFileName(""));
        assertFalse(AdaptiveDamperParser.isAdaptiveDamperFileName(null));
    }

    // --- row parsing -------------------------------------------------------

    @Test
    void parsesTheStockEtkRearController() {
        List<AdaptiveDamperSpec> specs = AdaptiveDamperParser.parsePart(
                RelaxedJson.parse(ETK_REAR_ADAPTIVE_PART), Map.of());

        assertEquals(1, specs.size());
        AdaptiveDamperSpec spec = specs.get(0);
        assertEquals("adaptiveRearDamper", spec.instanceName());
        assertEquals(List.of("shock_RR", "shock_RL"), spec.dampBeamNames());
        assertEquals(List.of("soft", "regular", "hard"), List.copyOf(spec.modes().keySet()));
    }

    @Test
    void readsTheEtkModeCoefficientsExactly() {
        Map<String, AdaptiveDamperMode> modes = AdaptiveDamperParser.parsePart(
                RelaxedJson.parse(ETK_REAR_ADAPTIVE_PART), Map.of()).get(0).modes();

        AdaptiveDamperMode soft = modes.get("soft");
        assertEquals(0.7f, soft.beamDampCoef(), 0.0f);
        assertEquals(1.0f, soft.beamDampFastCoef(), 0.0f);
        assertEquals(0.65f, soft.beamDampReboundCoef(), 0.0f);
        assertEquals(1.0f, soft.beamDampReboundFastCoef(), 0.0f);
        assertEquals(0.7f, soft.beamDampVelocitySplitCoef(), 0.0f);

        AdaptiveDamperMode regular = modes.get("regular");
        for (float coef : new float[]{regular.beamDampCoef(), regular.beamDampFastCoef(),
                regular.beamDampReboundCoef(), regular.beamDampReboundFastCoef(),
                regular.beamDampVelocitySplitCoef()}) {
            assertEquals(1.0f, coef, 0.0f);
        }

        AdaptiveDamperMode hard = modes.get("hard");
        assertEquals(1.5f, hard.beamDampCoef(), 0.0f);
        assertEquals(1.4f, hard.beamDampReboundCoef(), 0.0f);
        assertEquals(1.2f, hard.beamDampVelocitySplitCoef(), 0.0f);
    }

    @Test
    void missingCoefficientColumnsDefaultToOne() {
        JsonObject part = part("""
                {
                  "controller":[["fileName"],["drivingDynamics/actuators/adaptiveDampers",{"name":"c","dampBeamNames":["b1"]}]],
                  "c":{"modes":[["name","beamDampCoef"],["soft",0.5]]}
                }
                """);
        AdaptiveDamperMode soft = AdaptiveDamperParser.parsePart(part, Map.of())
                .get(0).modes().get("soft");

        assertEquals(0.5f, soft.beamDampCoef(), 0.0f);
        assertEquals(1.0f, soft.beamDampFastCoef(), 0.0f);
        assertEquals(1.0f, soft.beamDampReboundCoef(), 0.0f);
        assertEquals(1.0f, soft.beamDampReboundFastCoef(), 0.0f);
        assertEquals(1.0f, soft.beamDampVelocitySplitCoef(), 0.0f);
    }

    @Test
    void dampBeamNamesFallBackToTheNamedControllerObject() {
        JsonObject part = part("""
                {
                  "controller":[["fileName"],["drivingDynamics/actuators/adaptiveDampers",{"name":"c"}]],
                  "c":{
                    "dampBeamNames":["only_front"],
                    "modes":[["name"],["soft"]]
                  }
                }
                """);
        assertEquals(List.of("only_front"), AdaptiveDamperParser.parsePart(part, Map.of())
                .get(0).dampBeamNames());
    }

    @Test
    void inlineObjectOverridesAModeTableCell() {
        JsonObject part = part("""
                {
                  "controller":[["fileName"],["drivingDynamics/actuators/adaptiveDampers",{"name":"c","dampBeamNames":["b"]}]],
                  "c":{"modes":[["name","beamDampCoef"],["hard",1.5,{"beamDampCoef":2.5}]]}
                }
                """);
        assertEquals(2.5f, AdaptiveDamperParser.parsePart(part, Map.of())
                .get(0).modes().get("hard").beamDampCoef(), 0.0f);
    }

    // --- graceful degradation ---------------------------------------------

    @Test
    void malformedOrUnrelatedDeclarationsYieldNoControllers() {
        assertTrue(AdaptiveDamperParser.parsePart(null, Map.of()).isEmpty());
        assertTrue(AdaptiveDamperParser.parsePart(part("{}"), Map.of()).isEmpty());
        assertTrue(AdaptiveDamperParser.parsePart(part("{\"controller\":{}}"), Map.of()).isEmpty());

        // A different controller file name.
        assertTrue(AdaptiveDamperParser.parsePart(part("""
                {"controller":[["fileName"],["drivingDynamics/actuators/activeDifferential",{"name":"c"}]]}
                """), Map.of()).isEmpty());

        // Adaptive damper row without a resolvable instance name.
        assertTrue(AdaptiveDamperParser.parsePart(part("""
                {"controller":[["fileName"],["drivingDynamics/actuators/adaptiveDampers",{}]]}
                """), Map.of()).isEmpty());

        // Instance name points at a part section that does not exist.
        assertTrue(AdaptiveDamperParser.parsePart(part("""
                {"controller":[["fileName"],["drivingDynamics/actuators/adaptiveDampers",{"name":"ghost"}]]}
                """), Map.of()).isEmpty());

        // Named object exists but declares no mode table.
        assertTrue(AdaptiveDamperParser.parsePart(part("""
                {"controller":[["fileName"],["drivingDynamics/actuators/adaptiveDampers",{"name":"c"}]],"c":{}}
                """), Map.of()).isEmpty());

        // Mode table without the required "name" column.
        assertTrue(AdaptiveDamperParser.parsePart(part("""
                {"controller":[["fileName"],["drivingDynamics/actuators/adaptiveDampers",{"name":"c"}]],
                 "c":{"modes":[["beamDampCoef"],[0.5]]}}
                """), Map.of()).isEmpty());
    }

    @Test
    void modeRowsWithoutANameAreSkipped() {
        JsonObject part = part("""
                {
                  "controller":[["fileName"],["drivingDynamics/actuators/adaptiveDampers",{"name":"c","dampBeamNames":["b"]}]],
                  "c":{"modes":[["name","beamDampCoef"],["soft",0.5],["",0.1],[0.2]]}
                }
                """);
        assertEquals(List.of("soft"), List.copyOf(
                AdaptiveDamperParser.parsePart(part, Map.of()).get(0).modes().keySet()));
    }

    @Test
    void duplicateBeamNamesAreCollapsedInDeclarationOrder() {
        JsonObject part = part("""
                {
                  "controller":[["fileName"],["drivingDynamics/actuators/adaptiveDampers",
                                 {"name":"c","dampBeamNames":["b1","b2","b1"]}]],
                  "c":{"modes":[["name"],["soft"]]}
                }
                """);
        assertEquals(List.of("b1", "b2"), AdaptiveDamperParser.parsePart(part, Map.of())
                .get(0).dampBeamNames());
    }

    private static JsonObject part(String json) {
        return JsonParser.parseString(json).getAsJsonObject();
    }

    /** The stock controller row omits the comma before its inline options object. */
    @Test
    void theRelaxedEtkFixtureParsesThroughTheRelaxedLoader() {
        assertNotNull(RelaxedJson.parse(ETK_REAR_ADAPTIVE_PART));
    }
}
