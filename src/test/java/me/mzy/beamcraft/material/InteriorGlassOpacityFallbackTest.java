package me.mzy.beamcraft.client.material;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Unit tests for {@link InteriorGlassOpacityFallback}, the pure compatibility
 * decision for interior-glass opacity factors. Backend-neutral; no GL, no
 * Minecraft renderer.
 */
class InteriorGlassOpacityFallbackTest {

    @Test
    void explicitFactorWinsIncludingZero() {
        // An explicit stage/material factor (already precedence-resolved by the
        // caller) is authoritative and never replaced by the fallback — 0 is
        // deliberately distinct from absent.
        assertEquals(0.5f, InteriorGlassOpacityFallback.resolve(0.5f, "sunburst2_glass_int",
                null, "/vehicles/sunburst2/sunburst2_glass_o.png"));
        assertEquals(0.0f, InteriorGlassOpacityFallback.resolve(0.0f, "sunburst2_glass_int",
                null, "/vehicles/sunburst2/sunburst2_glass_o.png"));
    }

    @Test
    void mapToMatchingInteriorGlassSuffixAppliesFallback() {
        assertEquals(InteriorGlassOpacityFallback.DEFAULT_OPACITY_FACTOR,
                InteriorGlassOpacityFallback.resolve(null, "sunburst2_glass_int", "whatever",
                        "/vehicles/sunburst2/sunburst2_glass_o.png"));
    }

    @Test
    void nameMatchingInteriorGlassSuffixAppliesFallback() {
        // The interior-glass identity may come from the material name alone.
        assertEquals(InteriorGlassOpacityFallback.DEFAULT_OPACITY_FACTOR,
                InteriorGlassOpacityFallback.resolve(null, "something_else", "sunburst2_glass_int", null));
    }

    @Test
    void suffixMatchingIsCaseInsensitive() {
        assertEquals(InteriorGlassOpacityFallback.DEFAULT_OPACITY_FACTOR,
                InteriorGlassOpacityFallback.resolve(null, "Sunburst2_GLASS_INT", null, null));
    }

    @Test
    void dedicatedInteriorOpacityMapExcludesFallback() {
        // pickup_glass_int_o identifies the interior in the opacity path itself,
        // so the material already expresses its own interior alpha.
        assertNull(InteriorGlassOpacityFallback.resolve(null, "pickup_glass_int", null,
                "/vehicles/pickup/pickup_glass_int_o.png"));
    }

    @Test
    void sharedExteriorOpacityMapStillGetsFallback() {
        // sunburst2_glass_int reuses the exterior sunburst2_glass_o map, which
        // does not identify the interior.
        assertEquals(InteriorGlassOpacityFallback.DEFAULT_OPACITY_FACTOR,
                InteriorGlassOpacityFallback.resolve(null, "sunburst2_glass_int", null,
                        "/vehicles/sunburst2/sunburst2_glass_o.png"));
    }

    @Test
    void missingOpacityMapStillGetsFallback() {
        // No opacity map at all is never "dedicated", so the fallback applies.
        assertEquals(InteriorGlassOpacityFallback.DEFAULT_OPACITY_FACTOR,
                InteriorGlassOpacityFallback.resolve(null, "sunburst2_glass_int", null, null));
    }

    @Test
    void nonInteriorGlassIsUnaffected() {
        // Exterior glass, lamp covers and body paint never match the suffix.
        assertNull(InteriorGlassOpacityFallback.resolve(null, "sunburst2_glass", null, null));
        assertNull(InteriorGlassOpacityFallback.resolve(null, "pickup_headlightglass", null, null));
        assertNull(InteriorGlassOpacityFallback.resolve(null, "body", null, null));
    }

    @Test
    void interiorLikeNameWithoutSuffixDoesNotMatch() {
        // "glass_interior" does not end in the *_glass_int suffix.
        assertNull(InteriorGlassOpacityFallback.resolve(null, "sunburst2_glass_interior", null, null));
    }

    @Test
    void isDedicatedInteriorOpacityMapRecognisesGlassIntAndInt() {
        assertTrue(InteriorGlassOpacityFallback.isDedicatedInteriorOpacityMap("/vehicles/pickup/pickup_glass_int_o.png"));
        assertTrue(InteriorGlassOpacityFallback.isDedicatedInteriorOpacityMap("/vehicles/x/x_int_o.png"));
        assertFalse(InteriorGlassOpacityFallback.isDedicatedInteriorOpacityMap("/vehicles/sunburst2/sunburst2_glass_o.png"));
        assertFalse(InteriorGlassOpacityFallback.isDedicatedInteriorOpacityMap(null));
    }
}
