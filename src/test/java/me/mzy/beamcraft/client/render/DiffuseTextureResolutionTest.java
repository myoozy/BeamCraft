package me.mzy.beamcraft.client.render;

import me.mzy.beamcraft.client.material.MaterialRenderPlan;
import me.mzy.beamcraft.client.material.RgbaColor;
import org.junit.jupiter.api.Test;

import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * Pins the per-sub-mesh Sampler0 binding contract behind the Iris compatibility
 * fix: the renderer only ever binds a diffuse texture that actually resolved,
 * and every other case (no texture, or an unresolved texture) falls back to the
 * uploader's valid white 1x1 texture. In particular this guarantees the renderer
 * never binds the (nonexistent) {@code beamcraft:textures/entity/vehicle_default.png}
 * placeholder that caused a FileNotFoundException and GL_INVALID_OPERATION under
 * Iris's shadow pass. Pure logic; no GL or Minecraft renderer required.
 */
class DiffuseTextureResolutionTest {

    @Test
    void texturedAndResolvedUsesTheUploadedTexture() {
        AtomicInteger uploadCalls = new AtomicInteger();
        AtomicInteger whiteCalls = new AtomicInteger();
        MaterialRenderPlan plan = MaterialRenderPlan.textured("/vehicles/pickup/body_d.png", RgbaColor.WHITE);

        int textureId = PhysicsVehicleRenderer.resolveDiffuseTexture(
                plan, true,
                () -> {
                    uploadCalls.incrementAndGet();
                    return 42;
                },
                () -> {
                    whiteCalls.incrementAndGet();
                    return 0;
                });

        assertEquals(42, textureId, "resolved diffuse texture should be bound");
        assertEquals(1, uploadCalls.get(), "upload path should be used exactly once");
        assertEquals(0, whiteCalls.get(), "white fallback must not be used for a resolved texture");
    }

    @Test
    void texturedButUnresolvedFallsBackToWhite() {
        AtomicInteger uploadCalls = new AtomicInteger();
        MaterialRenderPlan plan = MaterialRenderPlan.textured("/vehicles/pickup/body_d.png", RgbaColor.WHITE);

        int textureId = PhysicsVehicleRenderer.resolveDiffuseTexture(
                plan, false,
                () -> {
                    uploadCalls.incrementAndGet();
                    return 42;
                },
                () -> 0);

        assertEquals(0, textureId, "unresolved texture must fall back to the white texture");
        assertEquals(0, uploadCalls.get(), "upload must not run for an unresolved texture");
    }

    @Test
    void colourOnlyAlwaysBindsWhiteEvenWhenMarkedResolved() {
        AtomicInteger uploadCalls = new AtomicInteger();
        MaterialRenderPlan plan = MaterialRenderPlan.colorOnly(new RgbaColor(0.5f, 0.6f, 0.7f, 1f));

        int textureId = PhysicsVehicleRenderer.resolveDiffuseTexture(
                plan, true,
                () -> {
                    uploadCalls.incrementAndGet();
                    return 42;
                },
                () -> 0);

        assertEquals(0, textureId, "a colour-only plan must bind the white texture");
        assertEquals(0, uploadCalls.get(), "no texture upload may run for a colour-only plan");
    }

    @Test
    void colourOnlyFallbackAlsoAppliesWhenPlanHasNoTexture() {
        // Double-check the degenerate path: a plan that claims a texture path but
        // resolves nothing still lands on white, never on a missing texture.
        MaterialRenderPlan plan = MaterialRenderPlan.textured("", RgbaColor.WHITE);
        int textureId = PhysicsVehicleRenderer.resolveDiffuseTexture(plan, true, () -> 42, () -> 0);
        assertEquals(0, textureId, "empty-path textured plan is colour-only and must bind white");
    }
}
