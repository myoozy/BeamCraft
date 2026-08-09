package me.mzy.beamcraft.client.material;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Unit tests for the deterministic opaque-diffuse stage selection and the
 * opaque/cutout/translucent classification in {@link MaterialRenderPlanner}.
 * Backend-neutral; no GL, no Minecraft renderer.
 */

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

    // ------------------------------------------------------------------
    // Render-mode classification (opaque / cutout / translucent)
    // ------------------------------------------------------------------

    @Test
    void explicitTranslucentFlagYieldsTranslucentPlanWithOpacityAndBlendOp() {
        MaterialDefinition def = def("""
                {
                  "mapTo": "glass",
                  "translucent": true,
                  "translucentBlendOp": "Additive",
                  "Stages": [ { "baseColorMap": "/vehicles/pickup/glass_d.png", "opacityMap": "/vehicles/pickup/glass_o.png" } ]
                }
                """);
        MaterialRenderPlan plan = MaterialRenderPlanner.plan(def);
        assertEquals(MaterialRenderPlan.RenderMode.TRANSLUCENT, plan.mode());
        assertTrue(plan.hasTexture());
        assertTrue(plan.hasOpacity());
        assertEquals("/vehicles/pickup/glass_o.png", plan.opacityPath());
        assertEquals("Additive", plan.blendOp());
    }

    @Test
    void alphaRefYieldsCutoutPlanWithItsOwnThreshold() {
        MaterialDefinition def = def("""
                {
                  "mapTo": "window",
                  "alphaRef": 0.5,
                  "Stages": [ { "baseColorMap": "/vehicles/pickup/window_d.png", "opacityMap": "/vehicles/pickup/window_o.png" } ]
                }
                """);
        MaterialRenderPlan plan = MaterialRenderPlanner.plan(def);
        assertEquals(MaterialRenderPlan.RenderMode.CUTOUT, plan.mode());
        assertEquals(0.5f, plan.alphaRef());
        assertTrue(plan.hasOpacity());
        assertEquals("/vehicles/pickup/window_o.png", plan.opacityPath());
    }

    @Test
    void alphaRefStaysCutoutNotBlendedForOpaqueAlpha() {
        // A stray alphaRef on a fully-opaque paint material must classify as
        // CUTOUT (alpha-tested, drawn in the opaque pass) — never TRANSLUCENT.
        MaterialDefinition def = def("""
                {
                  "mapTo": "paint",
                  "alphaRef": 0.5,
                  "baseColorFactor": [0.9, 0.8, 0.7, 1.0],
                  "Stages": [ { "baseColorMap": "/vehicles/pickup/body_d.png" } ]
                }
                """);
        MaterialRenderPlan plan = MaterialRenderPlanner.plan(def);
        assertEquals(MaterialRenderPlan.RenderMode.CUTOUT, plan.mode());
        assertTrue(plan.hasTexture());
    }

    @Test
    void translucentFlagWinsOverAlphaRef() {
        // A material explicitly translucent must blend, not alpha-test, even when
        // it also carries an alphaRef.
        MaterialDefinition def = def("""
                {
                  "mapTo": "glass",
                  "translucent": true,
                  "alphaRef": 0.5,
                  "Stages": [ { "baseColorMap": "/vehicles/pickup/glass_d.png" } ]
                }
                """);
        MaterialRenderPlan plan = MaterialRenderPlanner.plan(def);
        assertEquals(MaterialRenderPlan.RenderMode.TRANSLUCENT, plan.mode());
    }

    @Test
    void effectiveAlphaBelowOneYieldsTranslucent() {
        // A material that declares transparency purely via a sub-1 factor alpha
        // (no translucent flag) is classified translucent: this is the diffuse
        // alpha path.
        MaterialDefinition def = def("""
                {
                  "mapTo": "glass",
                  "diffuseColor": [0.5, 0.6, 0.7, 0.4],
                  "Stages": [ { "baseColorMap": "/vehicles/pickup/glass_d.png" } ]
                }
                """);
        MaterialRenderPlan plan = MaterialRenderPlanner.plan(def);
        assertEquals(MaterialRenderPlan.RenderMode.TRANSLUCENT, plan.mode());
        assertEquals(0.4f, plan.colorFactor().a());
    }

    @Test
    void fullyOpaqueBodyPaintStaysOpaque() {
        // The critical anti-regression: ordinary opaque body paint must stay in
        // the opaque pass. Alpha exactly 1, no flags -> OPAQUE.
        MaterialDefinition def = def("""
                {
                  "mapTo": "paint",
                  "Stages": [ { "baseColorMap": "/vehicles/pickup/body_d.png" } ]
                }
                """);
        MaterialRenderPlan plan = MaterialRenderPlanner.plan(def);
        assertEquals(MaterialRenderPlan.RenderMode.OPAQUE, plan.mode());
        assertTrue(plan.hasTexture());
        assertFalse(plan.hasOpacity());
    }

    @Test
    void cutoutWithoutDiffuseDegradesToColourOnlyOpaque() {
        // A cutout material with no usable diffuse stage must not disappear the
        // sub-mesh; it degrades to the current opaque colour-only behaviour.
        MaterialDefinition def = def("""
                {
                  "mapTo": "weird",
                  "alphaRef": 0.5,
                  "baseColorFactor": [0.9, 0.8, 0.7, 1.0]
                }
                """);
        MaterialRenderPlan plan = MaterialRenderPlanner.plan(def);
        assertEquals(MaterialRenderPlan.RenderMode.OPAQUE, plan.mode());
        assertFalse(plan.hasTexture());
        assertEquals(new RgbaColor(0.9f, 0.8f, 0.7f, 1.0f), plan.colorFactor());
    }

    @Test
    void translucentWithoutDiffuseIsColourOnlyTranslucent() {
        MaterialDefinition def = def("""
                {
                  "mapTo": "glass",
                  "translucent": true,
                  "diffuseColor": [0.1, 0.2, 0.3, 0.6]
                }
                """);
        MaterialRenderPlan plan = MaterialRenderPlanner.plan(def);
        assertEquals(MaterialRenderPlan.RenderMode.TRANSLUCENT, plan.mode());
        assertFalse(plan.hasTexture());
        assertEquals(0.6f, plan.colorFactor().a());
    }

    @Test
    void winningStageOpacityMapIsSelected() {
        // The opacity map of the winning diffuse stage is selected; a later
        // stage's opacity map must not leak in.
        MaterialDefinition def = def("""
                {
                  "mapTo": "glass",
                  "translucent": true,
                  "activeLayers": 2,
                  "Stages": [
                    { "baseColorMap": "/vehicles/pickup/glass_d.png", "opacityMap": "/vehicles/pickup/glass_o.png" },
                    { "baseColorMap": "/vehicles/pickup/glass_d2.png", "opacityMap": "/vehicles/pickup/glass_o2.png" }
                  ]
                }
                """);
        MaterialRenderPlan plan = MaterialRenderPlanner.plan(def);
        assertEquals("/vehicles/pickup/glass_d.png", plan.diffusePath());
        assertEquals("/vehicles/pickup/glass_o.png", plan.opacityPath());
    }

    // ------------------------------------------------------------------
    // opacityFactor (stage > material precedence, clamping, classification)
    // ------------------------------------------------------------------

    @Test
    void stageOpacityFactorTakesPrecedenceOverMaterialOpacityFactor() {
        // Both levels declare a factor; the selected stage's value wins.
        MaterialDefinition def = def("""
                {
                  "mapTo": "glass",
                  "opacityFactor": 0.9,
                  "Stages": [ { "baseColorMap": "/vehicles/x/glass_d.png", "opacityFactor": 0.25 } ]
                }
                """);
        MaterialRenderPlan plan = MaterialRenderPlanner.plan(def);
        assertEquals(MaterialRenderPlan.RenderMode.TRANSLUCENT, plan.mode());
        assertEquals(0.25f, plan.colorFactor().a(), 1e-6f);
    }

    @Test
    void materialOpacityFactorAppliesWhenStageHasNone() {
        MaterialDefinition def = def("""
                {
                  "mapTo": "glass",
                  "opacityFactor": 0.5,
                  "Stages": [ { "baseColorMap": "/vehicles/x/glass_d.png" } ]
                }
                """);
        MaterialRenderPlan plan = MaterialRenderPlanner.plan(def);
        assertEquals(MaterialRenderPlan.RenderMode.TRANSLUCENT, plan.mode());
        assertEquals(0.5f, plan.colorFactor().a(), 1e-6f);
    }

    @Test
    void explicitZeroOpacityFactorIsDistinctFromAbsent() {
        // opacityFactor: 0 must zero the alpha and classify translucent, never
        // be folded into "not declared" (which would leave it opaque).
        MaterialDefinition def = def("""
                {
                  "mapTo": "glass",
                  "opacityFactor": 0.0,
                  "Stages": [ { "baseColorMap": "/vehicles/x/glass_d.png" } ]
                }
                """);
        MaterialRenderPlan plan = MaterialRenderPlanner.plan(def);
        assertEquals(MaterialRenderPlan.RenderMode.TRANSLUCENT, plan.mode());
        assertEquals(0.0f, plan.colorFactor().a(), 1e-6f);
    }

    @Test
    void opacityFactorAboveOneIsClampedToOpaque() {
        // 1.5 must clamp to 1.0, not push a fully opaque factor above 1 (or
        // classify it translucent).
        MaterialDefinition def = def("""
                {
                  "mapTo": "glass",
                  "opacityFactor": 1.5,
                  "Stages": [ { "baseColorMap": "/vehicles/x/glass_d.png" } ]
                }
                """);
        MaterialRenderPlan plan = MaterialRenderPlanner.plan(def);
        assertEquals(MaterialRenderPlan.RenderMode.OPAQUE, plan.mode());
        assertEquals(1.0f, plan.colorFactor().a(), 1e-6f);
    }

    @Test
    void negativeOpacityFactorIsClampedToZero() {
        MaterialDefinition def = def("""
                {
                  "mapTo": "glass",
                  "opacityFactor": -0.5,
                  "Stages": [ { "baseColorMap": "/vehicles/x/glass_d.png" } ]
                }
                """);
        MaterialRenderPlan plan = MaterialRenderPlanner.plan(def);
        assertEquals(MaterialRenderPlan.RenderMode.TRANSLUCENT, plan.mode());
        assertEquals(0.0f, plan.colorFactor().a(), 1e-6f);
    }

    @Test
    void effectiveOpacityCombinesFactorAlphaAndOpacityFactor() {
        // Both the factor alpha and the opacityFactor contribute; the product
        // drives classification.
        MaterialDefinition def = def("""
                {
                  "mapTo": "glass",
                  "Stages": [ { "baseColorMap": "/vehicles/x/glass_d.png",
                                "baseColorFactor": [1.0, 1.0, 1.0, 0.5],
                                "opacityFactor": 0.4 } ]
                }
                """);
        MaterialRenderPlan plan = MaterialRenderPlanner.plan(def);
        assertEquals(MaterialRenderPlan.RenderMode.TRANSLUCENT, plan.mode());
        assertEquals(0.2f, plan.colorFactor().a(), 1e-6f);
    }

    @Test
    void interiorGlassWithDedicatedInteriorOpacityMapGetsNoFallback() {
        // pickup_glass_int declares a dedicated interior opacity map and no
        // opacityFactor; the fallback must NOT apply, leaving the factor opaque
        // (current pickup behaviour is correct and must be preserved).
        MaterialDefinition def = def("""
                {
                  "mapTo": "pickup_glass_int",
                  "Stages": [ { "baseColorMap": "/vehicles/pickup/pickup_glass_int_d.png",
                                "opacityMap": "/vehicles/pickup/pickup_glass_int_o.png" } ]
                }
                """);
        MaterialRenderPlan plan = MaterialRenderPlanner.plan(def);
        // OPAQUE plans drop the opacity path (opacity is only composed for
        // CUTOUT/TRANSLUCENT), so the meaningful assertions are the unchanged
        // pass and the opaque factor.
        assertEquals(MaterialRenderPlan.RenderMode.OPAQUE, plan.mode());
        assertEquals(1.0f, plan.colorFactor().a(), 1e-6f);
        assertFalse(plan.hasOpacity());
    }

    @Test
    void interiorGlassWithoutFactorAndSharedOpacityMapGetsFallback() {
        // sunburst2_glass_int reuses the exterior opacity map and declares no
        // opacityFactor; the 0.1 fallback applies and makes it translucent.
        MaterialDefinition def = def("""
                {
                  "mapTo": "sunburst2_glass_int",
                  "Stages": [ { "baseColorMap": "/vehicles/sunburst2/sunburst2_glass_int_d.png",
                                "opacityMap": "/vehicles/sunburst2/sunburst2_glass_o.png" } ]
                }
                """);
        MaterialRenderPlan plan = MaterialRenderPlanner.plan(def);
        assertEquals(MaterialRenderPlan.RenderMode.TRANSLUCENT, plan.mode());
        assertEquals(InteriorGlassOpacityFallback.DEFAULT_OPACITY_FACTOR, plan.colorFactor().a(), 1e-6f);
        assertTrue(plan.hasOpacity());
    }

    @Test
    void explicitOpacityFactorPreventsInteriorGlassFallback() {
        // An explicit factor on an interior-glass material must override the 0.1
        // compatibility default.
        MaterialDefinition def = def("""
                {
                  "mapTo": "sunburst2_glass_int",
                  "opacityFactor": 0.5,
                  "Stages": [ { "baseColorMap": "/vehicles/sunburst2/sunburst2_glass_int_d.png",
                                "opacityMap": "/vehicles/sunburst2/sunburst2_glass_o.png" } ]
                }
                """);
        MaterialRenderPlan plan = MaterialRenderPlanner.plan(def);
        assertEquals(MaterialRenderPlan.RenderMode.TRANSLUCENT, plan.mode());
        assertEquals(0.5f, plan.colorFactor().a(), 1e-6f);
    }

    @Test
    void exteriorGlassAndPaintAreUnaffectedByFallback() {
        // Exterior glass and body paint never match the interior-glass suffix,
        // so the fallback must not touch them.
        MaterialDefinition glass = def("""
                {
                  "mapTo": "sunburst2_glass",
                  "Stages": [ { "baseColorMap": "/vehicles/sunburst2/sunburst2_glass_d.png" } ]
                }
                """);
        MaterialRenderPlan glassPlan = MaterialRenderPlanner.plan(glass);
        assertEquals(MaterialRenderPlan.RenderMode.OPAQUE, glassPlan.mode());
        assertEquals(1.0f, glassPlan.colorFactor().a(), 1e-6f);

        MaterialDefinition paint = def("""
                {
                  "mapTo": "body",
                  "Stages": [ { "baseColorMap": "/vehicles/sunburst2/body_d.png" } ]
                }
                """);
        MaterialRenderPlan paintPlan = MaterialRenderPlanner.plan(paint);
        assertEquals(MaterialRenderPlan.RenderMode.OPAQUE, paintPlan.mode());
        assertEquals(1.0f, paintPlan.colorFactor().a(), 1e-6f);
    }

    // ------------------------------------------------------------------
    // Anti-regression: opacityFactor must not demote non-glass opaque ranges
    // ------------------------------------------------------------------

    @Test
    void opacityFactorZeroOnOpaqueInteriorDoesNotDemoteToTranslucent() {
        // THE Sunburst regression: sunburst2_interior declares opacityFactor 0 on
        // its diffuse stage but is not glass and not translucent. BeamNG (and the
        // pre-opacityFactor renderer) draws the interior shell opaque; honouring
        // the zero makes every interior sub-mesh vanish.
        MaterialDefinition def = def("""
                {
                  "mapTo": "sunburst2_interior",
                  "Stages": [ { "baseColorMap": "/vehicles/sunburst2/sunburst2_interior_b.color.png",
                                "opacityFactor": 0.0 } ]
                }
                """);
        MaterialRenderPlan plan = MaterialRenderPlanner.plan(def);
        assertEquals(MaterialRenderPlan.RenderMode.OPAQUE, plan.mode());
        assertEquals(1.0f, plan.colorFactor().a(), 1e-6f);
        assertTrue(plan.hasTexture());
        assertFalse(plan.hasOpacity());
    }

    @Test
    void opacityFactorOnBodyPaintIsIgnored() {
        // A stray opacityFactor on an opaque non-glass paint must not move it out
        // of the opaque pass.
        MaterialDefinition def = def("""
                {
                  "mapTo": "body",
                  "opacityFactor": 0.3,
                  "Stages": [ { "baseColorMap": "/vehicles/sunburst2/body_d.png" } ]
                }
                """);
        MaterialRenderPlan plan = MaterialRenderPlanner.plan(def);
        assertEquals(MaterialRenderPlan.RenderMode.OPAQUE, plan.mode());
        assertEquals(1.0f, plan.colorFactor().a(), 1e-6f);
    }

    @Test
    void explicitTranslucentFlagHonoursOpacityFactorEvenWithoutGlassName() {
        // A non-glass material that is explicitly translucent still scales its
        // alpha by opacityFactor (the "on"-state lamp/emissive family).
        MaterialDefinition def = def("""
                {
                  "mapTo": "sunburst2_lights_on",
                  "translucent": true,
                  "opacityFactor": 0.5,
                  "Stages": [ { "baseColorMap": "/vehicles/sunburst2/sunburst2_lights_on_d.png" } ]
                }
                """);
        MaterialRenderPlan plan = MaterialRenderPlanner.plan(def);
        assertEquals(MaterialRenderPlan.RenderMode.TRANSLUCENT, plan.mode());
        assertEquals(0.5f, plan.colorFactor().a(), 1e-6f);
    }

    @Test
    void honorsOpacityFactorIsFalseForOpaqueNonGlassOnly() {
        // Pins the gate itself: glass names and explicit translucent flags are
        // honoured; opaque non-glass materials are not.
        assertFalse(MaterialRenderPlanner.honorsOpacityFactor(def("""
                {"mapTo": "sunburst2_interior", "Stages": [ {"baseColorMap": "/x/d.png", "opacityFactor": 0.0} ]}
                """)));
        assertTrue(MaterialRenderPlanner.honorsOpacityFactor(def("""
                {"mapTo": "sunburst2_glass", "Stages": [ {"baseColorMap": "/x/d.png"} ]}
                """)));
        assertTrue(MaterialRenderPlanner.honorsOpacityFactor(def("""
                {"mapTo": "sunburst2_glass_int", "Stages": [ {"baseColorMap": "/x/d.png"} ]}
                """)));
        assertTrue(MaterialRenderPlanner.honorsOpacityFactor(def("""
                {"mapTo": "lights_on", "translucent": true, "Stages": [ {"baseColorMap": "/x/d.png"} ]}
                """)));
    }
}
