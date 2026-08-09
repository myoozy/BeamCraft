package me.mzy.beamcraft.client.material;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Unit tests for the deterministic opaque-diffuse stage selection in
 * {@link MaterialRenderPlanner}. Backend-neutral; no GL, no Minecraft renderer.
 */
class MaterialRenderPlannerTest {

    private static MaterialDefinition def(String json) {
        JsonObject obj = JsonParser.parseString(json).getAsJsonObject();
        MaterialDefinition definition = MaterialDefinition.fromJson("body", obj, "test");
        assertNotNull(definition, "material JSON should parse to a definition");
        return definition;
    }

    @Test
    void nullMaterialDegradesToColourOnlyWhite() {
        MaterialRenderPlan plan = MaterialRenderPlanner.plan(null);
        assertFalse(plan.hasTexture());
        assertEquals(RgbaColor.WHITE, plan.colorFactor());
    }

    @Test
    void firstStageWithBaseColorMapWinsWithItsOwnFactor() {
        MaterialDefinition def = def("""
                {
                  "mapTo": "body",
                  "Stages": [
                    { "baseColorMap": "/vehicles/pickup/body_d.png", "baseColorFactor": [0.5, 0.6, 0.7, 1.0] },
                    { "baseColorMap": "/vehicles/pickup/body_d2.png" }
                  ]
                }
                """);
        MaterialRenderPlan plan = MaterialRenderPlanner.plan(def);
        assertTrue(plan.hasTexture());
        assertEquals("/vehicles/pickup/body_d.png", plan.diffusePath());
        assertEquals(new RgbaColor(0.5f, 0.6f, 0.7f, 1.0f), plan.colorFactor());
    }

    @Test
    void legacyColorMapIsAccepted() {
        MaterialDefinition def = def("""
                {
                  "mapTo": "body",
                  "Stages": [ { "colorMap": "/vehicles/pickup/body_d.png" } ]
                }
                """);
        MaterialRenderPlan plan = MaterialRenderPlanner.plan(def);
        assertTrue(plan.hasTexture());
        assertEquals("/vehicles/pickup/body_d.png", plan.diffusePath());
        assertEquals(RgbaColor.WHITE, plan.colorFactor());
    }

    @Test
    void emptyStageIsSkippedAndNextCandidateWins() {
        MaterialDefinition def = def("""
                {
                  "mapTo": "body",
                  "activeLayers": 2,
                  "Stages": [
                    { },
                    { "baseColorMap": "/vehicles/pickup/glass_d.png", "baseColorFactor": [0.1, 0.2, 0.3, 0.9] }
                  ]
                }
                """);
        MaterialRenderPlan plan = MaterialRenderPlanner.plan(def);
        assertTrue(plan.hasTexture());
        assertEquals("/vehicles/pickup/glass_d.png", plan.diffusePath());
        assertEquals(new RgbaColor(0.1f, 0.2f, 0.3f, 0.9f), plan.colorFactor());
    }

    @Test
    void activeLayersBoundsTheCandidates() {
        // activeLayers = 1 makes the empty first stage the only candidate, so
        // the diffuse stage 1 must not be selected.
        MaterialDefinition def = def("""
                {
                  "mapTo": "body",
                  "activeLayers": 1,
                  "Stages": [
                    { },
                    { "baseColorMap": "/vehicles/pickup/body_d.png" }
                  ]
                }
                """);
        MaterialRenderPlan plan = MaterialRenderPlanner.plan(def);
        assertFalse(plan.hasTexture());
    }

    @Test
    void stageFactorFallsBackToMaterialFactor() {
        MaterialDefinition def = def("""
                {
                  "mapTo": "body",
                  "baseColorFactor": [1.0, 0.5, 0.25, 1.0],
                  "Stages": [ { "baseColorMap": "/vehicles/pickup/body_d.png" } ]
                }
                """);
        MaterialRenderPlan plan = MaterialRenderPlanner.plan(def);
        assertTrue(plan.hasTexture());
        assertEquals(new RgbaColor(1.0f, 0.5f, 0.25f, 1.0f), plan.colorFactor());
    }

    @Test
    void noUsableStageIsColourOnlyWithMaterialFactor() {
        MaterialDefinition def = def("""
                {
                  "mapTo": "body",
                  "diffuseColor": "0.2 0.3 0.4"
                }
                """);
        MaterialRenderPlan plan = MaterialRenderPlanner.plan(def);
        assertFalse(plan.hasTexture());
        assertEquals(new RgbaColor(0.2f, 0.3f, 0.4f, 1.0f), plan.colorFactor());
    }

    @Test
    void emptyBaseColorMapStringIsTreatedAsAbsent() {
        MaterialDefinition def = def("""
                {
                  "mapTo": "body",
                  "Stages": [ { "baseColorMap": "" } ]
                }
                """);
        MaterialRenderPlan plan = MaterialRenderPlanner.plan(def);
        assertFalse(plan.hasTexture());
        assertEquals(RgbaColor.WHITE, plan.colorFactor());
    }

    @Test
    void zeroActiveLayersIsColourOnlyEvenWithDiffuseStage() {
        // activeLayers <= 0 means no active texture stage at all; the diffuse
        // stage must not be forced in via Math.max(1, activeLayers).
        MaterialDefinition def = def("""
                {
                  "mapTo": "body",
                  "activeLayers": 0,
                  "Stages": [ { "baseColorMap": "/vehicles/pickup/body_d.png" } ]
                }
                """);
        MaterialRenderPlan plan = MaterialRenderPlanner.plan(def);
        assertFalse(plan.hasTexture());
        assertEquals(RgbaColor.WHITE, plan.colorFactor());
    }

    @Test
    void negativeActiveLayersIsColourOnlyWithMaterialFactor() {
        MaterialDefinition def = def("""
                {
                  "mapTo": "body",
                  "activeLayers": -1,
                  "baseColorFactor": [0.9, 0.8, 0.7, 1.0],
                  "Stages": [ { "baseColorMap": "/vehicles/pickup/body_d.png" } ]
                }
                """);
        MaterialRenderPlan plan = MaterialRenderPlanner.plan(def);
        assertFalse(plan.hasTexture());
        assertEquals(new RgbaColor(0.9f, 0.8f, 0.7f, 1.0f), plan.colorFactor());
    }
}
