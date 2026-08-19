package me.mzy.beamcraft.client.material;

/**
 * Backend-neutral plan for rendering one diffuse stage of a resolved material.
 *
 * <p>Produced by {@link MaterialRenderPlanner} from a resolved
 * {@link MaterialDefinition}. The plan deliberately carries no GL or Vulkan
 * types: {@link #diffusePath()} is the logical BeamNG texture path that a
 * renderer backend resolves (via {@link MaterialLibrary#resolveTexture}) and
 * uploads, {@link #colorFactor()} is the RGBA multiplier to apply in the
 * shader, and {@link #mode()} decides which render pass the sub-mesh belongs
 * to. This keeps the material model usable by any future backend.
 *
 * <p><b>Render modes</b>:
 * <ul>
 *   <li>{@link RenderMode#OPAQUE} — the historical default. Drawn in the
 *       opaque pass with the vanilla {@code entity_cutout} shader, depth
 *       writes on, culling on, no blending.</li>
 *   <li>{@link RenderMode#CUTOUT} — alpha-tested geometry (BeamNG
 *       {@code alphaRef > 0}). Drawn in the same opaque pass but with an alpha
 *       map composed into the diffuse so the shader's built-in alpha discard
 *       clips low-alpha pixels. This is intentionally <em>not</em> blending:
 *       depth writes stay on and no blend is enabled, so opaque body paint
 *       can never become see-through from a stray {@code alphaRef}.</li>
 *   <li>{@link RenderMode#TRANSLUCENT} — true transparency (BeamNG
 *       {@code translucent: true}, or an effective alpha below 1). Drawn last,
 *       back-to-front, with blending and depth writes off. Back-face culling is
 *       decided per range by the renderer against the vehicle's mesh provenance:
 *       paired window glass (an exterior {@code glass} plus its interior
 *       {@code glass_int} shell) keeps culling on so each shell draws exactly
 *       once and translucent layers never stack white, while single-shell lamp
 *       lenses/covers draw double-sided so they never vanish from behind.</li>
 * </ul>
 *
 * <p>When {@link #hasTexture()} is false the renderer renders the sub-mesh
 * colour-only (with {@link #colorFactor()}), which is also the documented
 * deterministic fallback when no material or no usable diffuse stage exists.
 */
public final class MaterialRenderPlan {

    /**
     * Render-pass classification for a sub-mesh. The enum is ordered so that
     * ordinary opaque materials never accidentally blend: only an explicit
     * {@code translucent} flag, an explicit {@code alphaRef > 0} cutout, or a
     * genuinely sub-1 effective alpha select a non-opaque mode.
     */
    public enum RenderMode {
        OPAQUE,
        CUTOUT,
        TRANSLUCENT
    }

    private final RenderMode mode;
    private final boolean hasTexture;
    private final String diffusePath;
    private final String opacityPath;
    private final RgbaColor colorFactor;
    private final float alphaRef;
    private final String blendOp;

    private MaterialRenderPlan(RenderMode mode, boolean hasTexture, String diffusePath, String opacityPath,
                               RgbaColor colorFactor, float alphaRef, String blendOp) {
        this.mode = mode != null ? mode : RenderMode.OPAQUE;
        this.hasTexture = hasTexture;
        this.diffusePath = diffusePath;
        this.opacityPath = opacityPath;
        this.colorFactor = colorFactor != null ? colorFactor : RgbaColor.WHITE;
        this.alphaRef = alphaRef;
        this.blendOp = blendOp;
    }

    /** Opaque colour-only plan: no texture, diffuse multiplied by {@code factor}. */
    public static MaterialRenderPlan colorOnly(RgbaColor factor) {
        return new MaterialRenderPlan(RenderMode.OPAQUE, false, null, null, factor, 0f, null);
    }

    /** Opaque textured plan sampling the logical diffuse path, multiplied by {@code factor}. */
    public static MaterialRenderPlan textured(String diffusePath, RgbaColor factor) {
        if (diffusePath == null || diffusePath.isEmpty()) {
            return colorOnly(factor);
        }
        return new MaterialRenderPlan(RenderMode.OPAQUE, true, diffusePath, null, factor, 0f, null);
    }

    /**
     * Cutout plan: sampled in the opaque pass, but the diffuse alpha (optionally
     * the composed opacity map) is clipped by the shader's built-in alpha test.
     * {@code alphaRef} is the BeamNG threshold that selected this mode; it is
     * retained for diagnostics/tests — the vanilla shader applies its own cutoff.
     */
    public static MaterialRenderPlan cutout(String diffusePath, String opacityPath, RgbaColor factor, float alphaRef) {
        if (diffusePath == null || diffusePath.isEmpty()) {
            return colorOnly(factor);
        }
        return new MaterialRenderPlan(RenderMode.CUTOUT, true, diffusePath, normalizedOpacity(opacityPath),
                factor, alphaRef, null);
    }

    /**
     * Translucent plan: blended in the translucent pass. {@code opacityPath} is
     * optional — without one the diffuse's own baked alpha (and the factor's
     * alpha) drives blending. {@code blendOp} is the BeamNG
     * {@code translucentBlendOp} ("None" or "Additive"); null/unknown values
     * fall back to normal alpha blending.
     */
    public static MaterialRenderPlan translucent(String diffusePath, String opacityPath, RgbaColor factor,
                                                 String blendOp) {
        if (diffusePath == null || diffusePath.isEmpty()) {
            return new MaterialRenderPlan(RenderMode.TRANSLUCENT, false, null, null, factor, 0f, blendOp);
        }
        return new MaterialRenderPlan(RenderMode.TRANSLUCENT, true, diffusePath, normalizedOpacity(opacityPath),
                factor, 0f, blendOp);
    }

    private static String normalizedOpacity(String opacityPath) {
        return (opacityPath == null || opacityPath.isEmpty()) ? null : opacityPath;
    }

    /** Which render pass this sub-mesh belongs to. Never null. */
    public RenderMode mode() {
        return mode;
    }

    /** True when a diffuse texture should be sampled; false renders colour-only. */
    public boolean hasTexture() {
        return hasTexture;
    }

    /** Logical BeamNG diffuse texture path; null when {@link #hasTexture()} is false. */
    public String diffusePath() {
        return diffusePath;
    }

    /** True when an opacity map should be composed into the diffuse alpha. */
    public boolean hasOpacity() {
        return opacityPath != null;
    }

    /** Logical BeamNG opacity map path, or null when there is none. */
    public String opacityPath() {
        return opacityPath;
    }

    /**
     * The BeamNG {@code alphaRef} threshold that classified this plan as
     * {@link RenderMode#CUTOUT}; 0 for other modes. The vanilla shader applies
     * its own (hardcoded) alpha cutoff, so this value is diagnostic rather than
     * a shader uniform.
     */
    public float alphaRef() {
        return alphaRef;
    }

    /** The BeamNG {@code translucentBlendOp} string ("None", "Additive"), or null. */
    public String blendOp() {
        return blendOp;
    }

    /** Immutable RGBA colour multiplier (never null; defaults to white). */
    public RgbaColor colorFactor() {
        return colorFactor;
    }

    @Override
    public String toString() {
        return "MaterialRenderPlan[" + mode + ", texture=" + (hasTexture ? diffusePath : "none")
                + (hasOpacity() ? ", opacity=" + opacityPath : "")
                + ", color=" + colorFactor + ", alphaRef=" + alphaRef
                + ", blendOp=" + blendOp + ']';
    }
}
