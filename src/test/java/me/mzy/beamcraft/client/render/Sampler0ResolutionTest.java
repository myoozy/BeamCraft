package me.mzy.beamcraft.client.render;

import me.mzy.beamcraft.client.material.MaterialRenderPlan;
import me.mzy.beamcraft.client.material.RgbaColor;
import org.junit.jupiter.api.Test;
import org.lwjgl.opengl.GL11;

import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * Pins the per-sub-mesh Sampler0 decision for translucent/cutout plans with an
 * opacity map, and the BeamNG blend-op translation. Pure logic; no GL context
 * and no Minecraft renderer required.
 */
class Sampler0ResolutionTest {

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
        // Only "Additive" is handled specially; anything unrecognised must never
        // be guessed into an exotic blend mode.
        assertArrayEquals(new int[]{GL11.GL_SRC_ALPHA, GL11.GL_ONE_MINUS_SRC_ALPHA}, PhysicsVehicleRenderer.blendFuncFor("Multiply"));
        assertArrayEquals(new int[]{GL11.GL_SRC_ALPHA, GL11.GL_ONE_MINUS_SRC_ALPHA}, PhysicsVehicleRenderer.blendFuncFor("  "));
    }
}
