package me.mzy.beamcraft.client.material;

/**
 * Immutable descriptor for a single BeamNG material stage (one entry of a
 * material's {@code Stages} array). A stage exists even when it carries no
 * diffuse/opacity maps (BeamNG emits empty stages in multi-stage materials),
 * so callers can rely on {@link MaterialDefinition#stages} preserving the
 * stage count.
 */
public final class MaterialStage {

    /** 0-based position inside the {@code Stages} array. */
    public final int index;

    /** Diffuse map (logical BeamNG path), from {@code baseColorMap} or legacy {@code colorMap}. May be null. */
    public final String baseColorMap;

    /** Opacity map (logical BeamNG path), from {@code opacityMap}. May be null. */
    public final String opacityMap;

    /**
     * Stage color factor from {@code baseColorFactor} or legacy {@code diffuseColor}
     * declared inside the stage, as an immutable RGBA value; null when absent.
     */
    public final RgbaColor baseColorFactor;

    /**
     * Stage-level {@code opacityFactor} (a scalar multiplier for the diffuse
     * alpha), or null when absent. An explicit 0 is preserved (never folded into
     * null), so callers can distinguish "not declared" from "declared fully
     * transparent". The planner treats a non-null stage value as authoritative
     * for the selected stage, ahead of the material-level value.
     */
    public final Float opacityFactor;

    public MaterialStage(int index, String baseColorMap, String opacityMap, RgbaColor baseColorFactor,
                         Float opacityFactor) {
        this.index = index;
        this.baseColorMap = baseColorMap;
        this.opacityMap = opacityMap;
        this.baseColorFactor = baseColorFactor;
        this.opacityFactor = opacityFactor;
    }
}
