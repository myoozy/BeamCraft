package me.mzy.beamcraft.client.render;

import com.google.gson.JsonParser;
import me.mzy.beamcraft.client.material.MaterialDefinition;
import me.mzy.beamcraft.client.material.MaterialRenderPlan;
import me.mzy.beamcraft.client.material.MaterialRenderPlanner;
import me.mzy.beamcraft.client.physics.FlexbodyContainer;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class DeformMaterialSelectionTest {

    @Test
    void triggeredGroupSelectsDamagedMaterialOnlyForOwningFlexbody() {
        FlexbodyContainer flex = flexbody(
                "windshield_break", "bx_glass", "bx_windshield_dmg");

        assertEquals("bx_windshield_dmg", PhysicsVehicleRenderer.selectDeformMaterialName(
                flex, 0, "bx_glass", material("bx_glass", "bx_glass"), true));
        assertEquals("bx_glass", PhysicsVehicleRenderer.selectDeformMaterialName(
                flex, 0, "bx_glass", material("bx_glass", "bx_glass"), false));
        assertEquals("body", PhysicsVehicleRenderer.selectDeformMaterialName(
                flex, 0, "body", material("body", "body"), true));
    }

    @Test
    void resolvedMapToCanMatchBaseBehindADaeAlias() {
        FlexbodyContainer flex = flexbody(
                "lamp_break", "bx_lights", "bx_lights_dmg");
        MaterialDefinition resolvedAlias = material("bx_lights", "bx_lights");

        assertEquals("bx_lights_dmg", PhysicsVehicleRenderer.selectDeformMaterialName(
                flex, 0, "bx_lowbeam", resolvedAlias, true));
    }

    @Test
    void bxDynamicCrackOverlayStaysInvisibleWithoutCubemapSupport() {
        MaterialDefinition invisible = MaterialDefinition.fromJson(
                "glass_invisible", JsonParser.parseString("""
                        {
                          "name":"glass_invisible",
                          "mapTo":"glass_invisible",
                          "Stages":[{"opacityFactor":0}],
                          "translucent":true,
                          "translucentBlendOp":"PreMulAlpha"
                        }
                        """).getAsJsonObject(), "test");
        MaterialRenderPlan invisiblePlan = MaterialRenderPlanner.plan(invisible);

        assertTrue(PhysicsVehicleRenderer.shouldKeepInvisibleGlassFallback(
                "glass_mirror", invisible));
        assertEquals(MaterialRenderPlan.RenderMode.TRANSLUCENT, invisiblePlan.mode());
        assertEquals(0.0f, invisiblePlan.colorFactor().a(), 0.0f);
        assertFalse(PhysicsVehicleRenderer.shouldKeepInvisibleGlassFallback(
                "bx_glass_dmg", invisible));
        assertFalse(PhysicsVehicleRenderer.shouldKeepInvisibleGlassFallback(
                "glass_mirror", material("mirror", "mirror")));
    }

    private static FlexbodyContainer flexbody(String group, String base, String damaged) {
        FlexbodyContainer flex = new FlexbodyContainer();
        flex.registerFlexbody("mesh", "bx", List.of("body"),
                group, base, damaged,
                0, 0, 0, 0, 0, 0, 1, 1, 1, 1, null);
        return flex;
    }

    private static MaterialDefinition material(String name, String mapTo) {
        return MaterialDefinition.fromJson(name, JsonParser.parseString("""
                {"name":"%s", "mapTo":"%s", "Stages":[]}
                """.formatted(name, mapTo)).getAsJsonObject(), "test");
    }
}
