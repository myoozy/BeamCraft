package me.mzy.beamcraft.client.material;

import java.util.List;

/**
 * Selects the opaque diffuse stage to render for a resolved material.
 *
 * <p><b>Deterministic selection rule</b> (documented, unit-tested): stages are
 * examined in index order, but only the first {@code activeLayers} stages are
 * candidates (BeamNG emits empty placeholder stages in multi-layer materials);
 * the first candidate carrying a non-empty {@code baseColorMap}/{@code colorMap}
 * wins. Its colour factor is {@code stage.baseColorFactor}, falling back to the
 * material-level {@code baseColorFactor}, then to white. If no candidate has a
 * diffuse map, the plan is colour-only with the material factor (white default).
 * A null/unparseable material degrades to a colour-only white plan so a broken
 * material can never prevent a vehicle from rendering.
 *
 * <p>Deliberately deferred to later stages (not part of this plan): opacity
 * maps, translucent flags, PBR channels, and multi-stage blending.
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
        if (material.activeLayers <= 0) {
            // No active texture stage; colour-only with the material factor.
            return MaterialRenderPlan.colorOnly(fallbackFactor);
        }
        List<MaterialStage> stages = material.stages;
        int limit = Math.min(material.activeLayers, stages.size());
        for (int i = 0; i < limit; i++) {
            MaterialStage stage = stages.get(i);
            if (stage == null || stage.baseColorMap == null || stage.baseColorMap.isEmpty()) {
                continue;
            }
            RgbaColor factor = stage.baseColorFactor != null ? stage.baseColorFactor : fallbackFactor;
            return MaterialRenderPlan.textured(stage.baseColorMap, factor);
        }
        return MaterialRenderPlan.colorOnly(fallbackFactor);
    }
}
