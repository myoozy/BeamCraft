package me.mzy.beamcraft.client.material;

import java.util.List;

/**
 * Selects the diffuse stage and render mode to draw for a resolved material.
 *
 * <p><b>Deterministic stage selection</b> (documented, unit-tested): stages are
 * examined in index order, but only the first {@code activeLayers} stages are
 * candidates (BeamNG emits empty placeholder stages in multi-layer materials);
 * the first candidate carrying a non-empty {@code baseColorMap}/{@code colorMap}
 * wins. Its colour factor is {@code stage.baseColorFactor}, falling back to the
 * material-level {@code baseColorFactor}, then to white. Its {@code opacityMap}
 * is the opacity source (when present). If no candidate has a diffuse map, the
 * plan is colour-only with the material factor (white default). A
 * null/unparseable material degrades to a colour-only white plan so a broken
 * material can never prevent a vehicle from rendering.
 *
 * <p><b>Deterministic render-mode classification</b> (documented, unit-tested):
 * <ol>
 *   <li>An explicit {@code translucent: true} flag wins → {@code TRANSLUCENT}.
 *       This is the authoritative BeamNG "draw in the transparent pass" flag.</li>
 *   <li>Else an {@code alphaRef > 0} → {@code CUTOUT}. The alpha test runs in the
 *       opaque pass and is distinct from blending, so body paint carrying a
 *       stray {@code alphaRef} stays opaque-looking.</li>
 *   <li>Else an effective alpha below 1 (the selected colour factor's alpha)
 *       → {@code TRANSLUCENT}. This is the "diffuse alpha" case: a material that
 *       declares transparency purely via a 4-component factor with no
 *       {@code translucent} flag. Fully opaque factors (alpha == 1) can never
 *       reach this branch.</li>
 *   <li>Otherwise {@code OPAQUE}, preserving the historical behaviour.</li>
 * </ol>
 *
 * <p><b>Opacity factor</b> (documented, unit-tested): the selected colour
 * factor's alpha is multiplied by the effective {@code opacityFactor} before
 * classification. The effective factor is the selected stage's value, falling
 * back to the material-level value, then to the interior-glass compatibility
 * fallback (see {@link InteriorGlassOpacityFallback}). The product is clamped to
 * [0,1].
 *
 * <p>The factor is only honoured for ranges that already belong in the alpha
 * world — materials whose {@code mapTo}/{@code name} identify glass, or that are
 * explicitly {@code translucent}. An opaque body/interior material carrying a
 * stray {@code opacityFactor} is never demoted into the translucent pass by it:
 * {@code sunburst2_interior} declares {@code opacityFactor: 0} (its diffuse is a
 * full-opacity interior shell), and the previous behaviour — and BeamNG itself —
 * renders that interior opaque. Honouring the zero there is the regression that
 * makes the entire Sunburst interior vanish; see {@link #honorsOpacityFactor}.
 */
public final class MaterialRenderPlanner {

    private MaterialRenderPlanner() {
    }

    /**
     * @param material the material resolved for the sub-mesh's DAE name, or null
     *                 when no material matched
     * @return a backend-neutral render plan; never null
     */
    public static MaterialRenderPlan plan(MaterialDefinition material) {
        if (material == null) {
            return MaterialRenderPlan.colorOnly(RgbaColor.WHITE);
        }
        RgbaColor fallbackFactor = material.baseColorFactor != null
                ? material.baseColorFactor
                : RgbaColor.WHITE;

        String diffusePath = null;
        String opacityPath = null;
        RgbaColor baseFactor = fallbackFactor;
        Float stageOpacityFactor = null;
        if (material.activeLayers > 0) {
            List<MaterialStage> stages = material.stages;
            int limit = Math.min(material.activeLayers, stages.size());
            for (int i = 0; i < limit; i++) {
                MaterialStage stage = stages.get(i);
                if (stage == null || stage.baseColorMap == null || stage.baseColorMap.isEmpty()) {
                    continue;
                }
                diffusePath = stage.baseColorMap;
                String opacity = stage.opacityMap;
                opacityPath = (opacity != null && !opacity.isEmpty()) ? opacity : null;
                baseFactor = stage.baseColorFactor != null ? stage.baseColorFactor : fallbackFactor;
                stageOpacityFactor = stage.opacityFactor;
                break;
            }
        }
        Float explicitOpacity = stageOpacityFactor != null ? stageOpacityFactor : material.opacityFactor;
        Float opacityFactor = InteriorGlassOpacityFallback.resolve(explicitOpacity,
                material.mapTo, material.name, opacityPath);
        if (opacityFactor != null && !honorsOpacityFactor(material)) {
            // Never let a scalar opacityFactor demote an opaque body/interior
            // range into the translucent pass (e.g. sunburst2_interior declares
            // opacityFactor 0 yet must stay opaque). Glass materials and explicit
            // translucent flags still honour it; see honorOpacityFactor().
            opacityFactor = null;
        }
        RgbaColor factor = applyOpacityFactor(baseFactor, opacityFactor);
        return classify(material, diffusePath, opacityPath, factor);
    }

    /**
     * Whether {@code opacityFactor} is meaningful for a material. BeamNG uses
     * {@code opacityFactor} to scale alpha in the transparent pass, so it is
     * honoured for glass materials and materials that are explicitly
     * {@code translucent}. Non-glass, non-translucent materials (body paint, the
     * Sunburst interior shell) carry legacy {@code opacityFactor} values that the
     * engine ignores in its opaque pass; demoting them to translucent by that
     * scalar is exactly the interior-vanish regression.
     */
    static boolean honorsOpacityFactor(MaterialDefinition material) {
        if (material == null) {
            return false;
        }
        return material.translucent || containsGlass(material.mapTo) || containsGlass(material.name);
    }

    /** True when {@code s} is a non-empty string mentioning {@code glass}, case-insensitive. */
    static boolean containsGlass(String s) {
        return s != null && !s.isEmpty() && s.toLowerCase(java.util.Locale.ROOT).contains("glass");
    }

    /**
     * Multiplies {@code color}'s alpha by {@code opacityFactor} (clamped to
     * [0,1]), keeping RGB unchanged. Null factor or colour passes through
     * unchanged, so materials without an opacity factor are never altered.
     */
    static RgbaColor applyOpacityFactor(RgbaColor color, Float opacityFactor) {
        if (color == null || opacityFactor == null) {
            return color;
        }
        float clampedFactor = Math.max(0f, Math.min(1f, opacityFactor));
        float alpha = Math.max(0f, Math.min(1f, color.a() * clampedFactor));
        return new RgbaColor(color.r(), color.g(), color.b(), alpha);
    }

    /**
     * Pure classification, separated from stage selection so it is unit-testable
     * in isolation. Never null.
     */
    static MaterialRenderPlan classify(MaterialDefinition material, String diffusePath, String opacityPath,
                                       RgbaColor factor) {
        if (material.translucent) {
            return MaterialRenderPlan.translucent(diffusePath, opacityPath, factor, material.translucentBlendOp);
        }
        if (material.alphaRef > 0f) {
            return diffusePath != null
                    ? MaterialRenderPlan.cutout(diffusePath, opacityPath, factor, material.alphaRef)
                    : MaterialRenderPlan.colorOnly(factor);
        }
        if (factor.a() < 1.0f) {
            return MaterialRenderPlan.translucent(diffusePath, opacityPath, factor, material.translucentBlendOp);
        }
        return diffusePath != null
                ? MaterialRenderPlan.textured(diffusePath, factor)
                : MaterialRenderPlan.colorOnly(factor);
    }
}
