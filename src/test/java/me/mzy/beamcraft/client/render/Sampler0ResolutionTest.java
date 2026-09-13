package me.mzy.beamcraft.client.render;

import com.google.gson.JsonParser;
import me.mzy.beamcraft.client.material.MaterialDefinition;
import me.mzy.beamcraft.client.material.MaterialRenderPlan;
import me.mzy.beamcraft.client.material.RgbaColor;
import org.junit.jupiter.api.Test;
import org.lwjgl.opengl.GL11;

import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Pins the per-sub-mesh Sampler0 decision for translucent/cutout plans with an
 * opacity map, and the BeamNG blend-op translation. Pure logic; no GL context
 * and no Minecraft renderer required.
 */
class Sampler0ResolutionTest {

    @Test
    void maskOnlyPlanComposesOverWhite() {
        // A flat-colour grille material: no colour map, only the mask. It still has to
        // reach the composed path, or the cutout shader gets no alpha to test.
        MaterialRenderPlan plan = MaterialRenderPlan.cutoutMaskOnly(
                "/vehicles/common/grille_hex_o.data.png", RgbaColor.WHITE, 86f / 255f);

        assertFalse(plan.hasTexture());
        assertTrue(plan.hasOpacity());
        assertTrue(PhysicsVehicleRenderer.composedAvailable(plan, false, true));
    }

    @Test
    void maskOnlyPlanStillNeedsItsMaskResolved() {
        MaterialRenderPlan plan = MaterialRenderPlan.cutoutMaskOnly(
                "/vehicles/common/grille_hex_o.data.png", RgbaColor.WHITE, 86f / 255f);

        assertFalse(PhysicsVehicleRenderer.composedAvailable(plan, false, false));
    }

    @Test
    void texturedPlanStillNeedsItsDiffuseToCompose() {
        MaterialRenderPlan plan = MaterialRenderPlan.translucent(
                "/vehicles/pickup/glass_d.png", "/vehicles/pickup/glass_o.png", RgbaColor.WHITE, null);

        assertFalse(PhysicsVehicleRenderer.composedAvailable(plan, false, true),
                "a missing diffuse must keep falling back rather than sample the mask as colour");
    }

    @Test
    void composedTextureUsedWhenDiffuseAndOpacityBothResolved() {
        MaterialRenderPlan plan = MaterialRenderPlan.translucent(
                "/vehicles/pickup/glass_d.png", "/vehicles/pickup/glass_o.png", RgbaColor.WHITE, null);
        AtomicInteger composed = new AtomicInteger();
        AtomicInteger diffuse = new AtomicInteger();
        AtomicInteger white = new AtomicInteger();

        int id = PhysicsVehicleRenderer.resolveSampler0Texture(
                plan, true, true,
                () -> {
                    composed.incrementAndGet();
                    return 7;
                },
                () -> {
                    diffuse.incrementAndGet();
                    return 3;
                },
                () -> {
                    white.incrementAndGet();
                    return 0;
                });

        assertEquals(7, id, "a resolvable opacity map must bind the composed texture");
        assertEquals(1, composed.get());
        assertEquals(0, diffuse.get());
        assertEquals(0, white.get());
    }

    @Test
    void missingOpacityDegradesToDiffuseBakedAlpha() {
        // Opacity map referenced but unresolved: deterministic fallback to the
        // diffuse texture alone — never the white texture, never a vanished mesh.
        MaterialRenderPlan plan = MaterialRenderPlan.translucent(
                "/vehicles/pickup/glass_d.png", "/vehicles/pickup/glass_o.png", RgbaColor.WHITE, null);
        AtomicInteger composed = new AtomicInteger();

        int id = PhysicsVehicleRenderer.resolveSampler0Texture(
                plan, true, false,
                () -> {
                    composed.incrementAndGet();
                    return 7;
                },
                () -> 3,
                () -> 0);

        assertEquals(3, id);
        assertEquals(0, composed.get(), "the composed upload must not run when the opacity map is missing");
    }

    @Test
    void noOpacityMeansDiffuseOnlyPath() {
        MaterialRenderPlan plan = MaterialRenderPlan.textured(
                "/vehicles/pickup/body_d.png", RgbaColor.WHITE);

        int id = PhysicsVehicleRenderer.resolveSampler0Texture(
                plan, true, true,
                () -> 7, () -> 3, () -> 0);

        assertEquals(3, id, "a plan without an opacity map must not compose");
    }

    @Test
    void unresolvedDiffuseStillFallsBackToWhite() {
        MaterialRenderPlan plan = MaterialRenderPlan.translucent(
                "/vehicles/pickup/glass_d.png", "/vehicles/pickup/glass_o.png", RgbaColor.WHITE, null);

        int id = PhysicsVehicleRenderer.resolveSampler0Texture(
                plan, false, false,
                () -> 7, () -> 3, () -> 0);

        assertEquals(0, id, "an unresolved diffuse must still bind the white fallback");
    }

    @Test
    void defaultAndNoneBlendOpsUseNormalAlpha() {
        assertArrayEquals(new int[]{GL11.GL_SRC_ALPHA, GL11.GL_ONE_MINUS_SRC_ALPHA}, PhysicsVehicleRenderer.blendFuncFor(null));
        assertArrayEquals(new int[]{GL11.GL_SRC_ALPHA, GL11.GL_ONE_MINUS_SRC_ALPHA}, PhysicsVehicleRenderer.blendFuncFor("None"));
        assertArrayEquals(new int[]{GL11.GL_SRC_ALPHA, GL11.GL_ONE_MINUS_SRC_ALPHA}, PhysicsVehicleRenderer.blendFuncFor("none"));
    }

    @Test
    void additiveBlendOpUsesAdditiveBlending() {
        assertArrayEquals(new int[]{GL11.GL_SRC_ALPHA, GL11.GL_ONE}, PhysicsVehicleRenderer.blendFuncFor("Additive"));
        assertArrayEquals(new int[]{GL11.GL_SRC_ALPHA, GL11.GL_ONE}, PhysicsVehicleRenderer.blendFuncFor("additive"));
    }

    @Test
    void unknownBlendOpsFallBackToNormalAlpha() {
        // Only "Additive" and "PreMulAlpha" are handled specially; anything unrecognised
        // must never be guessed into an exotic blend mode. The stock library's remaining
        // ops are "Add", "Sub" and "AddAlpha", four materials between them.
        assertArrayEquals(new int[]{GL11.GL_SRC_ALPHA, GL11.GL_ONE_MINUS_SRC_ALPHA}, PhysicsVehicleRenderer.blendFuncFor("Multiply"));
        assertArrayEquals(new int[]{GL11.GL_SRC_ALPHA, GL11.GL_ONE_MINUS_SRC_ALPHA}, PhysicsVehicleRenderer.blendFuncFor("  "));
        assertArrayEquals(new int[]{GL11.GL_SRC_ALPHA, GL11.GL_ONE_MINUS_SRC_ALPHA}, PhysicsVehicleRenderer.blendFuncFor("AddAlpha"));
    }

    @Test
    void preMulAlphaUsesPremultipliedBlending() {
        // Premultiplied colour contributes at full strength and only the destination is
        // attenuated. isPremultipliedBlend drives the composition as well as this pair,
        // because blending this way over a non-premultiplied texture would make a mask's
        // soft edges glow.
        assertArrayEquals(new int[]{GL11.GL_ONE, GL11.GL_ONE_MINUS_SRC_ALPHA},
                PhysicsVehicleRenderer.blendFuncFor("PreMulAlpha"));
        assertArrayEquals(new int[]{GL11.GL_ONE, GL11.GL_ONE_MINUS_SRC_ALPHA},
                PhysicsVehicleRenderer.blendFuncFor("  premulalpha  "),
                "the op is trimmed and matched case-insensitively");

        assertTrue(PhysicsVehicleRenderer.isPremultipliedBlend("PreMulAlpha"));
        assertFalse(PhysicsVehicleRenderer.isPremultipliedBlend("None"));
        assertFalse(PhysicsVehicleRenderer.isPremultipliedBlend(null));
    }

    @Test
    void declaredDoubleSidedIsHonouredWithoutTheGlassHeuristic() {
        // grille_hex is a thin shell declaring doubleSided. The glass heuristic returns
        // false for a non-glass name, so the flag is what has to make this work.
        MaterialDefinition grille = MaterialDefinition.fromJson("grille_hex",
                JsonParser.parseString("""
                        {"name":"grille_hex","mapTo":"grille_hex","doubleSided":true,
                         "Stages":[{"baseColorFactor":[0.14,0.14,0.14,1],"opacityMap":"grille_hex_o.data.png"}]}
                        """).getAsJsonObject(), "test");
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("grille_hex", Set.of(), grille),
                "the heuristic alone does not cover a grille");

        assertTrue(PhysicsVehicleRenderer.isDoubleSided("grille_hex", Set.of(), grille));
    }

    @Test
    void aMaterialWithoutTheFlagStillFallsBackToTheHeuristic() {
        MaterialDefinition plain = MaterialDefinition.fromJson("plain",
                JsonParser.parseString("{\"name\":\"plain\",\"mapTo\":\"plain\"}").getAsJsonObject(), "test");

        assertFalse(PhysicsVehicleRenderer.isDoubleSided("plain", Set.of(), plain));
        assertFalse(PhysicsVehicleRenderer.isDoubleSided("plain", Set.of(), null));
    }
}
