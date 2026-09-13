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
 * When no diffuse stage exists, the first stage-level opacity factor still
 * applies to the colour-only fallback; this is how BeamNG's
 * {@code glass_invisible} suppresses its geometry. A null/unparseable material
 * degrades to a colour-only white plan so a broken material can never prevent
 * a vehicle from rendering.
 *
 * <p><b>Deterministic render-mode classification</b> (documented, unit-tested):
 * <ol>
 *   <li>Mirror materials ({@code mapTo}/{@code name} mentioning {@code mirror},
 *       e.g. BeamNG's {@code mirror}/{@code mirror_CE}/{@code mirror_CX}/
 *       {@code mirror_F}/{@code glass_mirror}) are forced to {@code OPAQUE}
 *       (or {@code CUTOUT} when {@code alphaRef > 0}) before any other rule.
 *       BeamNG declares these {@code translucent: true} with
 *       {@code dynamicCubemap: true} (cubemap reflections, out of scope here);
 *       without PBR they must render as an opaque diffuse/colour fallback, never
 *       as ordinary see-through glass.</li>
 *   <li>Else an explicit {@code translucent: true} flag whose material is
 *       <em>effectively opaque</em> (blend op not additive, no opacity map, a
 *       fully-opaque effective factor, and not a see-through lamp cover/lens/
 *       glass) → {@code OPAQUE}. BeamNG routes opaque lamp-housing/reflector
 *       surfaces (e.g. {@code covet_lights}/{@code bx_lights}/{@code pickup_lights}
 *       and every glowMap alias onto them) through the transparent pass with
 *       {@code translucentBlendOp: "None"} and no {@code opacityMap}; with a
 *       factor alpha of 1 they paint exactly like the opaque surfaces they are.
 *       Classifying them {@code TRANSLUCENT} puts them in the back-to-front pass,
 *       where a coplanar translucent lamp lens can be sorted <em>before</em> them
 *       and then painted over — the Covet/BX lamp-cover disappearance. See-through
 *       lamp covers/lenses/glass (names mentioning {@code glass}/{@code windshield}/
 *       {@code lens}/{@code lightglass}/{@code cover}, e.g. the common
 *       {@code roundlight_cover}/{@code squarelight_cover} lamp lenses) are exempt:
 *       their baked diffuse alpha is the transparency, so they stay
 *       {@code TRANSLUCENT} (see {@link #isEffectivelyOpaqueTranslucent}).</li>
 *   <li>Else an explicit {@code translucent: true} flag → {@code TRANSLUCENT}.
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
        Float colorOnlyStageOpacityFactor = null;
        // First mask and colour factor declared by any stage, kept for the
        // no-colour-map case below.
        String colorOnlyOpacityPath = null;
        RgbaColor colorOnlyBaseFactor = null;
        if (material.activeLayers > 0) {
            List<MaterialStage> stages = material.stages;
            int limit = Math.min(material.activeLayers, stages.size());
            for (int i = 0; i < limit; i++) {
                MaterialStage stage = stages.get(i);
                if (colorOnlyStageOpacityFactor == null && stage != null && stage.opacityFactor != null) {
                    colorOnlyStageOpacityFactor = stage.opacityFactor;
                }
                if (stage == null) {
                    continue;
                }
                // Noted before the colour-map test: BeamNG's flat-colour materials keep
                // their holes in a stage that carries only a baseColorFactor plus an
                // opacityMap, and skipping such a stage dropped the mask along with the
                // colour map it was looking for.
                if (colorOnlyOpacityPath == null && stage.opacityMap != null && !stage.opacityMap.isEmpty()) {
                    colorOnlyOpacityPath = stage.opacityMap;
                }
                // The flat colour lives here too: grille_hex declares its 0.14 on the
                // stage, and losing it would paint the part white rather than dark.
                if (colorOnlyBaseFactor == null && stage.baseColorFactor != null) {
                    colorOnlyBaseFactor = stage.baseColorFactor;
                }
                if (stage.baseColorMap == null || stage.baseColorMap.isEmpty()) {
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
        if (diffusePath == null) {
            stageOpacityFactor = colorOnlyStageOpacityFactor;
            // A material with no colour map is not thereby a material with no mask, and
            // its stage factor is still its colour.
            opacityPath = colorOnlyOpacityPath;
            if (colorOnlyBaseFactor != null) {
                baseFactor = colorOnlyBaseFactor;
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
        RgbaColor factor = applyOpacityFactor(baseFactor, opacityFactor,
                MaterialDefinition.isPremultipliedBlend(material.translucentBlendOp));
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
     * Whether {@code material} is a BeamNG mirror surface. Mirrors are identified
     * by their material family name ({@code mapTo}/{@code name} mentioning
     * {@code mirror}): the shared mirror materials are {@code mirror},
     * {@code mirror_CE}, {@code mirror_CX}, {@code mirror_F} and
     * {@code glass_mirror}, and the raw DAE meshes that use them carry the same
     * names ({@code mirror_CE_}, {@code mirror_cx}, … across the available
     * vehicles). They declare {@code translucent: true} plus
     * {@code dynamicCubemap: true} for reflections, which without PBR must not be
     * rendered as ordinary see-through glass — {@link #classify} forces them into
     * the opaque fallback.
     */
    static boolean isMirrorMaterial(MaterialDefinition material) {
        if (material == null) {
            return false;
        }
        return containsMirror(material.mapTo) || containsMirror(material.name);
    }

    /** True when {@code s} is a non-empty string mentioning {@code mirror}, case-insensitive. */
    private static boolean containsMirror(String s) {
        return s != null && !s.isEmpty() && s.toLowerCase(java.util.Locale.ROOT).contains("mirror");
    }

    /**
     * Whether an explicitly-{@code translucent} material is <em>effectively
     * opaque</em> and must therefore render in the opaque pass. BeamNG declares
     * lamp-housing/reflector surfaces {@code translucent: true} with
     * {@code translucentBlendOp: "None"} and no {@code opacityMap}: the flag only
     * moves them into the transparent pass for correct sorting against glass,
     * while their diffuse (an opaque BC1/BC3 colour map, alpha 255) and a
     * fully-opaque factor make them paint exactly like opaque surfaces. Drawn as
     * {@code TRANSLUCENT} they carry alpha 1 into the back-to-front pass, where a
     * coplanar translucent lamp lens can sort before them and be painted over —
     * the Covet/BX lamp-cover disappearance.
     *
     * <p>This rule is deliberately narrow: it requires the plain BeamNG
     * {@code "None"} blend, no opacity map, an effective factor alpha of 1,
     * <em>and</em> that the material is not a see-through lamp cover/lens/glass
     * (see {@link #isSeeThroughLampCover}). The last gate is the baked-alpha
     * guard: the shared lamp-lens covers (e.g. common {@code roundlight_cover}/
     * {@code squarelight_cover} and their skins) are {@code "None"}-blend
     * translucent materials with no opacity map whose diffuse carries the baked
     * alpha that makes the lens read as glass — promoting them to {@code OPAQUE}
     * paints their transparent regions opaque. Genuinely opaque surfaces —
     * PreMulAlpha glass with an opacity map, additive emissive sheets, materials
     * that omit the blend op, sub-1 factors, and any non-lens "None" material
     * (housings, body paint, chrome) — are untouched.
     *
     * @param material    the resolved material (may be null)
     * @param opacityPath the selected stage's opacity map, or null
     * @param factor      the effective (opacity-adjusted) colour factor, never null
     */
    static boolean isEffectivelyOpaqueTranslucent(MaterialDefinition material, String opacityPath, RgbaColor factor) {
        if (material == null || !material.translucent) {
            return false;
        }
        // Only the plain BeamNG "None" blend declares the opaque-through-the-
        // transparent-pass housing pattern. PreMulAlpha glass, additive emissive
        // sheets and every material that omits the blend op stay translucent.
        if (material.translucentBlendOp == null
                || !material.translucentBlendOp.trim().equalsIgnoreCase("None")) {
            return false;
        }
        if (opacityPath != null) {
            return false; // an opacity map makes the transparency real
        }
        // A see-through lamp cover/lens/glass must not be promoted: its baked
        // diffuse alpha is the transparency (e.g. common roundlight_cover). Only
        // non-lens "None" surfaces (housings, body paint, chrome) are opaque.
        if (isSeeThroughLampCover(material)) {
            return false;
        }
        return factor != null && factor.a() >= 1.0f;
    }

    /**
     * Whether {@code material} is a see-through lamp cover/lens/glass — the set
     * of "None"-blend translucent materials whose baked diffuse alpha is the
     * transparency and that {@link #isEffectivelyOpaqueTranslucent} must never
     * promote to the opaque pass. A material qualifies when its {@code mapTo} or
     * {@code name} mentions one of {@link #SEE_THROUGH_LAMP_MARKERS}. Lamp
     * housings/reflectors ({@code *_lights}, {@code roundlight}/{@code squarelight})
     * and non-lens surfaces (body paint, chrome, engines) carry none of those
     * markers and stay eligible for the opaque promotion.
     */
    static boolean isSeeThroughLampCover(MaterialDefinition material) {
        if (material == null) {
            return false;
        }
        return containsSeeThroughLampMarker(material.mapTo) || containsSeeThroughLampMarker(material.name);
    }

    /** True when a non-empty string mentions a see-through lamp-cover marker, case-insensitive. */
    private static boolean containsSeeThroughLampMarker(String s) {
        if (s == null || s.isEmpty()) {
            return false;
        }
        String n = s.toLowerCase(java.util.Locale.ROOT);
        for (String marker : SEE_THROUGH_LAMP_MARKERS) {
            if (n.contains(marker)) {
                return true;
            }
        }
        return false;
    }

    /**
     * See-through lamp cover/lens/glass markers seen in the actual bundled
     * materials (covet/bx/etki/pickup/citybus/common). "None"-blend translucent
     * materials named with one of these carry their transparency in the baked
     * diffuse alpha (the common {@code roundlight_cover}/{@code squarelight_cover}
     * lamp lenses and skins) and must stay in the translucent pass; the opaque
     * promotion is reserved for housings/body/chrome, which carry none of them.
     */
    private static final String[] SEE_THROUGH_LAMP_MARKERS = {
            "glass", "windshield", "lens", "lightglass", "cover"
    };

    /**
     * Multiplies {@code color}'s alpha by {@code opacityFactor} (clamped to
     * [0,1]), keeping RGB unchanged. Null factor or colour passes through
     * unchanged, so materials without an opacity factor are never altered.
     */
    static RgbaColor applyOpacityFactor(RgbaColor color, Float opacityFactor, boolean premultiplied) {
        if (color == null || opacityFactor == null) {
            return color;
        }
        float clampedFactor = Math.max(0f, Math.min(1f, opacityFactor));
        float alpha = Math.max(0f, Math.min(1f, color.a() * clampedFactor));
        if (!premultiplied) {
            // Normal alpha blending weights the rgb by this alpha itself.
            return new RgbaColor(color.r(), color.g(), color.b(), alpha);
        }
        // A premultiplied blend weights the rgb by nothing, so the factor has to scale rgb
        // too: for premultiplied output rgb must equal colour x alpha, which is why the
        // channels move together. Without this, BeamNG's invisible shells — an
        // opacityFactor of 0 with PreMulAlpha, e.g. glass_invisible on the shattered
        // windshield mesh — contribute their full rgb and stop being invisible.
        return new RgbaColor(color.r() * alpha, color.g() * alpha, color.b() * alpha, alpha);
    }

    /**
     * Pure classification, separated from stage selection so it is unit-testable
     * in isolation. Never null.
     */
    static MaterialRenderPlan classify(MaterialDefinition material, String diffusePath, String opacityPath,
                                       RgbaColor factor) {
        if (isMirrorMaterial(material)) {
            // Mirror reflective surfaces must not enter the see-through translucent
            // pass (BeamNG flags them translucent:true for a cubemap reflection that
            // is out of scope). Render them as an opaque diffuse/colour fallback,
            // honouring an explicit alpha cutout when the asset asks for one.
            if (material.alphaRef > 0f) {
                return diffusePath != null
                        ? MaterialRenderPlan.cutout(diffusePath, opacityPath, factor, material.alphaRef)
                        : MaterialRenderPlan.colorOnly(factor);
            }
            return diffusePath != null
                    ? MaterialRenderPlan.textured(diffusePath, factor)
                    : MaterialRenderPlan.colorOnly(factor);
        }
        if (material.translucent && isEffectivelyOpaqueTranslucent(material, opacityPath, factor)) {
            // BeamNG's effectively-opaque lamp housings must not enter the
            // see-through translucent pass: as translucent they sort by centroid
            // with the coplanar lamp lens and can be drawn after (over) it,
            // hiding the cover at certain angles. Draw them in the opaque pass
            // so they always write depth; the coplanar single-shell lens then
            // draws over them with LEQUAL (see the renderer's
            // isDoubleSidedLampLens path).
            return diffusePath != null
                    ? MaterialRenderPlan.textured(diffusePath, factor)
                    : MaterialRenderPlan.colorOnly(factor);
        }
        if (material.translucent) {
            return MaterialRenderPlan.translucent(diffusePath, opacityPath, factor, material.translucentBlendOp);
        }
        if (material.alphaRef > 0f) {
            if (diffusePath != null) {
                return MaterialRenderPlan.cutout(diffusePath, opacityPath, factor, material.alphaRef);
            }
            // No colour map at all: a flat baseColorFactor plus a coverage mask. The mask
            // still has to reach the alpha test, so it is composed over white and the
            // factor tints it — dropping to colorOnly is what painted grilles solid.
            // A declared cutout with no colour map: a flat colour plus a coverage mask.
            // Interior glass stays excluded — its shell must not be cut open (see
            // isInteriorGlassMaterial).
            if (opacityPath != null && !isInteriorGlassMaterial(material)) {
                return MaterialRenderPlan.cutoutMaskOnly(opacityPath, factor, material.alphaRef);
            }
            return MaterialRenderPlan.colorOnly(factor);
        }
        if (factor.a() < 1.0f) {
            return MaterialRenderPlan.translucent(diffusePath, opacityPath, factor, material.translucentBlendOp);
        }
        // An opacity map with neither an alphaRef nor a translucent flag is deliberately
        // left alone. BeamNG does not declare those as coverage, and the same _o.data slot
        // holds maps that are not coverage at all: the Sunburst2 body's is 25% zero, 47%
        // low and 28% full with no clean separation, so reading it as alpha turned most of
        // the body transparent. A cutout therefore requires the asset to say so through
        // alphaRef, which the stock grille materials do (grille_hex: 86, grille: 127).
        return diffusePath != null
                ? MaterialRenderPlan.textured(diffusePath, factor)
                : MaterialRenderPlan.colorOnly(factor);
    }

    /**
     * Interior glass is the one mask-carrying material that must stay opaque. It is
     * not flagged translucent, and cutting its shell open would punch holes in the
     * cabin; the see-through look comes from the exterior glass instead. Exterior
     * glass, lamp covers and paint are flagged translucent or handled by an earlier
     * branch, so they never reach the coverage rule above.
     *
     * <p>The classification is BeamNG's own {@code *_glass_int} naming, reused from
     * {@link InteriorGlassOpacityFallback} rather than restated here. A material that
     * ships a mask and is genuinely meant to be clipped under an unusual name would
     * need that classification widened.
     */
    private static boolean isInteriorGlassMaterial(MaterialDefinition material) {
        return InteriorGlassOpacityFallback.isInteriorGlass(material.mapTo)
                || InteriorGlassOpacityFallback.isInteriorGlass(material.name);
    }
}
