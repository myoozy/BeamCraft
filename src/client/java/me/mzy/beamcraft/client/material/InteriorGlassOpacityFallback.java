package me.mzy.beamcraft.client.material;

import java.util.Locale;

/**
 * Pure, backend-neutral decision for the <em>compatibility</em> opacity-factor
 * fallback applied to interior glass.
 *
 * <p>BeamNG material docs describe {@code opacityFactor} for interior glass and
 * use very small values (the docs example is 0.062; common assets like
 * {@code generic_glass_int} use 0.1). Several legacy interiors
 * (sunburst2/roamer/covet/etki) declare no {@code opacityFactor} at all and
 * reuse the non-{@code _int} exterior opacity map, so their interior glass
 * ends up as opaque as the exterior glass and washes already-rendered world
 * geometry white when viewed from inside the cab. The official modern materials
 * (e.g. {@code citybus_glass_int=0.481}, {@code van_glass_int=0.342}) carry an
 * explicit factor; modern pickup uses a dedicated interior opacity map
 * ({@code pickup_glass_int_o}) and no factor.
 *
 * <p><b>Rules</b> (deterministic, unit-tested):
 * <ol>
 *   <li>An explicit {@code opacityFactor} (stage-level already precedence-resolved,
 *       or material-level) is returned as-is and never overridden — including an
 *       explicit 0, which is deliberately distinct from absent.</li>
 *   <li>Else, only when {@code mapTo} or {@code name} identifies interior glass
 *       (case-insensitive {@code *_glass_int} suffix) does the fallback apply.</li>
 *   <li>Within interior glass, a <em>dedicated</em> interior opacity map (the
 *       opacity path itself contains {@code glass_int} or {@code _int}, e.g.
 *       {@code pickup_glass_int_o}) means the material already expresses its own
 *       interior alpha, so no fallback is applied.</li>
 *   <li>Otherwise (shared exterior opacity map, or no opacity map) the
 *       conservative {@link #DEFAULT_OPACITY_FACTOR} (0.1) is returned.</li>
 * </ol>
 *
 * <p>Exterior glass, lamp covers and body paint never match rule 2, so they are
 * untouched. This helper only decides the scalar; the caller multiplies it into
 * the diffuse alpha and lets classification decide the pass. It never modifies
 * blend modes, texture pixels, or PBR channels.
 */
public final class InteriorGlassOpacityFallback {

    /** Conservative default interior-glass opacity when nothing else declares it. */
    public static final float DEFAULT_OPACITY_FACTOR = 0.1f;

    private static final String INTERIOR_GLASS_SUFFIX = "_glass_int";

    private InteriorGlassOpacityFallback() {
    }

    /**
     * Resolves the effective opacity factor for one material/stage.
     *
     * @param explicitOpacityFactor the stage-level factor (already preferred over
     *                              the material-level one) or null when neither
     *                              declared one
     * @param mapTo                 the material's {@code mapTo} (may be null)
     * @param name                  the material's name (may be null)
     * @param opacityPath           the selected opacity map path, or null
     * @return the factor to multiply into the diffuse alpha, or null when no
     *         factor applies (fully opaque default)
     */
    public static Float resolve(Float explicitOpacityFactor, String mapTo, String name, String opacityPath) {
        if (explicitOpacityFactor != null) {
            return explicitOpacityFactor; // explicit wins, including an explicit 0
        }
        if (!isInteriorGlass(mapTo) && !isInteriorGlass(name)) {
            return null;
        }
        if (isDedicatedInteriorOpacityMap(opacityPath)) {
            return null;
        }
        return DEFAULT_OPACITY_FACTOR;
    }

    /** True when {@code s} is a non-empty name ending in {@code *_glass_int}, case-insensitive. */
    static boolean isInteriorGlass(String s) {
        return s != null && s.toLowerCase(Locale.ROOT).endsWith(INTERIOR_GLASS_SUFFIX);
    }

    /**
     * True when the opacity path is a <em>dedicated</em> interior opacity map,
     * i.e. it identifies the interior in its own name ({@code glass_int} or
     * {@code _int}). {@code pickup_glass_int_o} qualifies; a shared exterior map
     * like {@code sunburst2_glass_o} does not. Null/absent paths are never
     * dedicated.
     */
    static boolean isDedicatedInteriorOpacityMap(String opacityPath) {
        if (opacityPath == null) {
            return false;
        }
        String lower = opacityPath.toLowerCase(Locale.ROOT);
        return lower.contains("glass_int") || lower.contains("_int");
    }
}
