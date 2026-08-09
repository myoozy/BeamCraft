package me.mzy.beamcraft.client.material;

/**
 * Backend-neutral plan for rendering one opaque diffuse stage.
 *
 * <p>Produced by {@link MaterialRenderPlanner} from a resolved
 * {@link MaterialDefinition}. The plan deliberately carries no GL or Vulkan
 * types: {@link #diffusePath} is the logical BeamNG texture path that a renderer
 * backend resolves (via {@link MaterialLibrary#resolveTexture}) and uploads, and
 * {@link #colorFactor} is the RGBA multiplier to apply in the shader. This keeps
 * the material model usable by any future backend.
 *
 * <p>When {@link #hasTexture()} is false the renderer renders the sub-mesh
 * colour-only (with {@link #colorFactor()}), which is also the documented
 * deterministic fallback when no material or no usable diffuse stage exists.
 */
public final class MaterialRenderPlan {

    private final boolean hasTexture;
    private final String diffusePath;
    private final RgbaColor colorFactor;

    private MaterialRenderPlan(boolean hasTexture, String diffusePath, RgbaColor colorFactor) {
        this.hasTexture = hasTexture;
        this.diffusePath = diffusePath;
        this.colorFactor = colorFactor != null ? colorFactor : RgbaColor.WHITE;
    }

    /** Colour-only plan: no texture, diffuse multiplied by {@code factor}. */
    public static MaterialRenderPlan colorOnly(RgbaColor factor) {
        return new MaterialRenderPlan(false, null, factor);
    }

    /** Textured plan sampling the logical diffuse path, multiplied by {@code factor}. */
    public static MaterialRenderPlan textured(String diffusePath, RgbaColor factor) {
        if (diffusePath == null || diffusePath.isEmpty()) {
            return colorOnly(factor);
        }
        return new MaterialRenderPlan(true, diffusePath, factor);
    }

    /** True when a diffuse texture should be sampled; false renders colour-only. */
    public boolean hasTexture() {
        return hasTexture;
    }

    /** Logical BeamNG texture path; null when {@link #hasTexture()} is false. */
    public String diffusePath() {
        return diffusePath;
    }

    /** Immutable RGBA colour multiplier (never null; defaults to white). */
    public RgbaColor colorFactor() {
        return colorFactor;
    }

    @Override
    public String toString() {
        return "MaterialRenderPlan[texture=" + (hasTexture ? diffusePath : "none")
                + ", color=" + colorFactor + ']';
    }
}
