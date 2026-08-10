package me.mzy.beamcraft.client.render;

import com.google.gson.JsonParser;
import me.mzy.beamcraft.client.material.MaterialDefinition;
import org.junit.jupiter.api.Test;

import java.util.HashSet;
import java.util.Locale;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Unit tests for {@link PhysicsVehicleRenderer#isDoubleSidedTranslucentGlass},
 * the per-range translucent culling policy. A translucent range draws double-sided
 * only when it is actually a glass/lens/windshield material (raw DAE name or the
 * resolved material mentions {@code glass}/{@code windshield}/{@code lens})
 * <em>and</em> has no paired {@code *_int} opposite shell in the same vehicle.
 * Everything else — paired window shells, decals, emissive/additive sheets, lamp
 * housings, screens — keeps default back-face culling. The names below are the
 * actual raw DAE material names and resolved definitions found in the bundled
 * Sunburst / Pickup / Covet / Etki / BX / Citybus / common archives. Pure logic;
 * no GL, no Minecraft renderer.
 */
class TranslucentCullPolicyTest {

    private static Set<String> mesh(String... names) {
        Set<String> set = new HashSet<>();
        for (String n : names) {
            set.add(n.toLowerCase(Locale.ROOT));
        }
        return set;
    }

    /** A resolved material definition carrying only {@code mapTo}; enough for the semantic guard. */
    private static MaterialDefinition def(String mapTo) {
        return MaterialDefinition.fromJson(mapTo,
                JsonParser.parseString("{\"mapTo\": \"" + mapTo + "\"}").getAsJsonObject(), "test");
    }

    @Test
    void pairedWindowGlassKeepsCulling() {
        // Exterior window shells pair with their interior *_glass_int shell in the
        // same vehicle's DAE, so both draw culled (each shell once, outward only).
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("sunburst2_glass",
                mesh("sunburst2_glass", "sunburst2_glass_int"), def("sunburst2_glass")));
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("sunburst2_glass_int",
                mesh("sunburst2_glass", "sunburst2_glass_int"), def("sunburst2_glass_int")));
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("pickup_glass",
                mesh("pickup_glass", "pickup_glass_int"), def("pickup_glass")));
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("pickup_glass_int",
                mesh("pickup_glass", "pickup_glass_int"), def("pickup_glass_int")));
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("citybus_glass",
                mesh("citybus_glass", "citybus_glass_int"), def("citybus_glass")));
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("covet_glass",
                mesh("covet_glass", "covet_glass_int"), def("covet_glass")));
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("etki_glass",
                mesh("etki_glass", "etki_glass_int"), def("etki_glass")));
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("bx_glass",
                mesh("bx_glass", "bx_glass_int"), def("bx_glass")));
    }

    @Test
    void singleShellLampLensesDrawDoubleSided() {
        // Headlight/taillight/signal/fog covers are single shells (no *_int
        // sibling in the vehicle) and must draw double-sided so they never vanish
        // from behind. Their resolved material is the vehicle's glass material
        // (the JBeam glowMap alias target), which still carries "glass".
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("sunburst2_headlightglass",
                mesh("sunburst2_glass", "sunburst2_glass_int"), def("sunburst2_glass")),
                "a headlight lens is a single shell and must be double-sided");
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("sunburst2_signalglass_L",
                mesh("sunburst2_glass", "sunburst2_glass_int"), def("sunburst2_glass")));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("sunburst2_signalglass_R",
                mesh("sunburst2_glass", "sunburst2_glass_int"), def("sunburst2_glass")));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("sunburst2_taillightglass",
                mesh("sunburst2_glass", "sunburst2_glass_int"), def("sunburst2_glass")));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("sunburst2_foglightglass",
                mesh("sunburst2_glass", "sunburst2_glass_int"), def("sunburst2_glass")));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("sunburst2_reverselightglass",
                mesh("sunburst2_glass", "sunburst2_glass_int"), def("sunburst2_glass")));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("pickup_headlightglass",
                mesh("pickup_glass", "pickup_glass_int"), def("pickup_lightglass")));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("pickup_lowbeamglass",
                mesh("pickup_glass", "pickup_glass_int"), def("pickup_lightglass")));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("pickup_highbeamglass",
                mesh("pickup_glass", "pickup_glass_int"), def("pickup_lightglass")));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("pickup_taillightglass_L",
                mesh("pickup_glass", "pickup_glass_int"), def("pickup_lightglass")));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("pickup_signalglass_R",
                mesh("pickup_glass", "pickup_glass_int"), def("pickup_lightglass")));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("pickup_reverselightglass",
                mesh("pickup_glass", "pickup_glass_int"), def("pickup_lightglass")));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("pickup_foglightglass",
                mesh("pickup_glass", "pickup_glass_int"), def("pickup_lightglass")));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("pickup_runninglightglass",
                mesh("pickup_glass", "pickup_glass_int"), def("pickup_lightglass")));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("pickup_runningsignalglass_L",
                mesh("pickup_glass", "pickup_glass_int"), def("pickup_lightglass")));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("pickup_sealedbeam_foglightglass",
                mesh("pickup_glass", "pickup_glass_int"), def("sealedbeam_glass")));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("covet_headlightglass",
                mesh("covet_glass", "covet_glass_int"), def("covet_glass")));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("covet_signalglass_L_us",
                mesh("covet_glass", "covet_glass_int"), def("covet_glass")));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("etki_markersignalglass_R",
                mesh("etki_glass", "etki_glass_int"), def("etki_glass")));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("bx_brakelightglass",
                mesh("bx_glass", "bx_glass_int"), def("bx_glass")));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("bx_chmslglass",
                mesh("bx_glass", "bx_glass_int"), def("bx_glass")));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("bx_markerglass_L",
                mesh("bx_glass", "bx_glass_int"), def("bx_glass")));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("citybus_signglass",
                mesh("citybus_glass", "citybus_glass_int"), def("citybus_signglass")));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("citybus_gauges_glass",
                mesh("citybus_glass", "citybus_glass_int"), def("citybus_gauges_glass")));
    }

    @Test
    void sealedbeamGlassIsASingleShellLampLens() {
        // sealedbeam_glass is the single-shell sealed-beam headlamp lens used
        // directly by the covet/etki/bx DAEs. It ends in a bare "_glass" which a
        // suffix heuristic would misread as paired window glass; the data-driven
        // sibling check sees no sealedbeam_glass_int and keeps it double-sided.
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("sealedbeam_glass",
                mesh("sealedbeam_glass", "sealedbeam"), def("sealedbeam_glass")));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("sealedbeam_glass_amber",
                mesh("sealedbeam_glass_amber", "sealedbeam"), def("sealedbeam_glass_amber")));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("generic_halogen_glass_highbeam",
                mesh("generic_halogen_glass_highbeam", "generic_halogen_highbeam"),
                def("generic_halogen_glass")));
        // But the same name IS a paired window shell when a sibling exists.
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("sealedbeam_glass",
                mesh("sealedbeam_glass", "sealedbeam_glass_int"), def("sealedbeam_glass")));
    }

    @Test
    void windshieldWithoutGlassSubstringIsStillDoubleSided() {
        // sunburst2_windshield is a single-shell window-glass mesh whose raw DAE
        // name does not spell "glass" (it carries "windshield") and whose resolved
        // material name is likewise glass-free. The semantic guard must still
        // recognise it as glass so it never vanishes from behind.
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("sunburst2_windshield",
                mesh("sunburst2_windshield", "sunburst2_glass", "sunburst2_glass_int"),
                def("sunburst2_windshield")));
    }

    @Test
    void caseInsensitiveOnRawDaeName() {
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("Sunburst2_HEADLIGHTGLASS",
                mesh("sunburst2_glass", "sunburst2_glass_int"), def("sunburst2_glass")));
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("SUNBURST2_GLASS",
                mesh("SUNBURST2_GLASS", "SUNBURST2_GLASS_INT"), def("sunburst2_glass")));
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("SUNBURST2_GLASS_INT",
                mesh("SUNBURST2_GLASS", "SUNBURST2_GLASS_INT"), def("sunburst2_glass_int")));
    }

    @Test
    void aliasTargetMustNotBeUsedForThePairingDecision() {
        // The Sunburst lamp covers resolve (via glowMap) to the sunburst2_glass
        // plan, which is a paired window-glass shell. If the pairing check keyed
        // on the resolved name instead of the raw DAE identity, the single-shell
        // lamp covers would be culled and vanish from behind. The mesh set is
        // still the raw provenance: sunburst2_glass has its _int shell, the
        // headlight cover does not.
        Set<String> sunburst = mesh("sunburst2_glass", "sunburst2_glass_int", "sunburst2_headlightglass");
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("sunburst2_headlightglass",
                sunburst, def("sunburst2_glass")));
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("sunburst2_glass",
                sunburst, def("sunburst2_glass")),
                "the aliased target alone must not decide the pairing");
    }

    @Test
    void rawNameWithoutGlassMarkerResolvesToGlassMaterialAndDrawsDoubleSided() {
        // A lamp lens whose raw DAE name carries no glass marker still resolves
        // (via the JBeam glowMap alias) to a glass-named material. The resolved
        // material provenance — not a loose universal fallback — proves it is
        // glass, so the single shell stays double-sided.
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("pickup_lightglass",
                mesh("pickup_lightglass"), def("pickup_lightglass")));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("generic_halogen_highbeam_cover",
                mesh("generic_halogen_highbeam_cover", "generic_halogen_glass_highbeam"),
                def("generic_halogen_glass")));
    }

    @Test
    void singleShellWindowGlassWithoutIntSiblingDrawsDoubleSided() {
        // van_glass (pickup's shared rear/side glass) and generic_glass_int have no
        // opposite shell in the same vehicle's DAE; drawing them double-sided is
        // safe (single layer, no white-wash stacking) and keeps them visible from
        // both sides instead of vanishing from the culled direction.
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("van_glass",
                mesh("van_glass", "pickup_glass", "pickup_glass_int"), def("van_glass")));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("generic_glass_int",
                mesh("generic_glass_int", "sunburst2_glass", "sunburst2_glass_int"),
                def("generic_glass_int")));
    }

    @Test
    void nonGlassTranslucentMaterialsKeepDefaultCulling() {
        // THE anti-regression: non-glass translucent ranges must NOT be drawn
        // double-sided just because they have no *_int sibling. The lamp housing
        // sheets (pickup_lights + every raw name that resolves to it), the gauge
        // decal, the warning indicators and the gear indicator all stay culled.
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("pickup_lights",
                mesh("pickup_lights", "pickup_headlight"), def("pickup_lights")),
                "a lamp housing sheet is not glass and keeps back-face culling");
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("pickup_headlight",
                mesh("pickup_lights", "pickup_headlight"), def("pickup_lights")),
                "a lamp housing that resolves to pickup_lights keeps culling");
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("decals_gauges",
                mesh("decals_gauges"), def("decals_gauges")),
                "a gauge-cluster decal keeps default culling");
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("sunburst2_lights",
                mesh("sunburst2_lights", "sunburst2_glass", "sunburst2_glass_int"),
                def("sunburst2_lights")));
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("etki_gearindicator",
                mesh("etki_gearindicator"), def("etki_gearindicator")));
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("bx_gauges",
                mesh("bx_gauges"), def("bx_gauges")));
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("neon_tube",
                mesh("neon_tube", "neon_tube_glow"), def("neon_tube")));
        // Warning indicators are non-glass translucent sheets on the dash.
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("checkengine",
                mesh("checkengine"), def("checkengine")));
    }

    @Test
    void doubleSidedLampLensesGetEqualDepthPolicy() {
        // The Covet/BX lamp covers are single-shell double-sided lenses coplanar
        // with their (now opaque) housing reflector. They must draw with LEQUAL
        // so the cover passes depth at the coplanar housing depth instead of
        // being depth-rejected by GL_LESS.
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedLampLens("covet_headlightglass",
                mesh("covet_glass", "covet_glass_int"), def("covet_glass")));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedLampLens("covet_taillightglass",
                mesh("covet_glass", "covet_glass_int"), def("covet_glass")));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedLampLens("covet_signalglass_L_us",
                mesh("covet_glass", "covet_glass_int"), def("covet_glass")));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedLampLens("bx_headlightglass",
                mesh("bx_glass", "bx_glass_int"), def("bx_glass")));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedLampLens("bx_taillightglass",
                mesh("bx_glass", "bx_glass_int"), def("bx_glass")));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedLampLens("bx_signalglass_L",
                mesh("bx_glass", "bx_glass_int"), def("bx_glass")));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedLampLens("bx_brakelightglass",
                mesh("bx_glass", "bx_glass_int"), def("bx_glass")));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedLampLens("bx_chmslglass",
                mesh("bx_glass", "bx_glass_int"), def("bx_glass")));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedLampLens("bx_parkinglightglass",
                mesh("bx_glass", "bx_glass_int"), def("bx_glass")));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedLampLens("bx_markerglass_R",
                mesh("bx_glass", "bx_glass_int"), def("bx_glass")));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedLampLens("sealedbeam_glass",
                mesh("sealedbeam_glass", "sealedbeam"), def("sealedbeam_glass")));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedLampLens("sealedbeam_glass_amber",
                mesh("sealedbeam_glass_amber", "sealedbeam"), def("sealedbeam_glass_amber")));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedLampLens("generic_halogen_glass_highbeam",
                mesh("generic_halogen_glass_highbeam", "generic_halogen_highbeam"),
                def("generic_halogen_glass")));
    }

    @Test
    void nonLampTranslucentRangesKeepLessDepth() {
        // Paired window shells are culled, not double-sided -> never LEQUAL.
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedLampLens("covet_glass",
                mesh("covet_glass", "covet_glass_int"), def("covet_glass")));
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedLampLens("bx_glass_int",
                mesh("bx_glass", "bx_glass_int"), def("bx_glass_int")));
        // Lamp housings are not glass -> never LEQUAL even though their material
        // name mentions lights.
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedLampLens("covet_lights",
                mesh("covet_lights", "covet_headlightglass"), def("covet_lights")));
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedLampLens("bx_lights",
                mesh("bx_lights", "bx_headlightglass"), def("bx_lights")));
        // The windshield is double-sided single-shell glass but is a window, not
        // a lamp lens: it keeps GL_LESS (it is not coplanar with an opaque
        // housing).
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedLampLens("sunburst2_windshield",
                mesh("sunburst2_windshield", "sunburst2_glass", "sunburst2_glass_int"),
                def("sunburst2_windshield")));
        // Single-shell window glass (van_glass) and the invisible damage glass
        // are double-sided but not lamp lenses.
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedLampLens("van_glass",
                mesh("van_glass", "pickup_glass", "pickup_glass_int"), def("van_glass")));
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedLampLens("glass_invisible",
                mesh("glass_invisible"), def("glass_invisible")));
        // Non-glass translucent decals keep GL_LESS.
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedLampLens("decals_gauges",
                mesh("decals_gauges"), def("decals_gauges")));
    }

    @Test
    void lampLensMarkerCoversTheAffectedVehiclesOnly() {
        assertTrue(PhysicsVehicleRenderer.isLampLensCover("covet_headlightglass", def("covet_glass")));
        assertTrue(PhysicsVehicleRenderer.isLampLensCover("bx_taillightglass", def("bx_glass")));
        assertTrue(PhysicsVehicleRenderer.isLampLensCover("bx_markerglass_L", def("bx_glass")));
        assertTrue(PhysicsVehicleRenderer.isLampLensCover("etki_markersignalglass_R", def("etki_glass")));
        assertTrue(PhysicsVehicleRenderer.isLampLensCover("citybus_signglass", def("citybus_signglass")));
        assertTrue(PhysicsVehicleRenderer.isLampLensCover("pickup_lowbeamglass", def("pickup_lightglass")));
        assertTrue(PhysicsVehicleRenderer.isLampLensCover("pickup_runningsignalglass_L", def("pickup_lightglass")));
        assertTrue(PhysicsVehicleRenderer.isLampLensCover("generic_halogen_highbeam_cover", def("generic_halogen_glass")));
        // Resolved-material provenance: a raw name without a marker that resolves
        // to a light-glass material is still a lamp lens.
        assertTrue(PhysicsVehicleRenderer.isLampLensCover("some_mesh", def("pickup_lightglass")));
        // Window glass, windshields, invisible glass and null are never lamp lenses.
        assertFalse(PhysicsVehicleRenderer.isLampLensCover("sunburst2_windshield", def("sunburst2_windshield")));
        assertFalse(PhysicsVehicleRenderer.isLampLensCover("glass_invisible", def("glass_invisible")));
        assertFalse(PhysicsVehicleRenderer.isLampLensCover("van_glass", def("van_glass")));
        assertFalse(PhysicsVehicleRenderer.isLampLensCover("covet_glass", def("covet_glass")));
        assertFalse(PhysicsVehicleRenderer.isLampLensCover(null, null));
    }

    @Test
    void nullOrUnknownMaterialIsNeverDoubleSided() {
        // A null raw name is never double-sided; an unresolved material (no
        // definition) cannot be proven glass, so it keeps default culling.
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass(null,
                mesh("sunburst2_lights"), def("sunburst2_lights")));
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("unknown_sheet",
                mesh("unknown_sheet"), null));
    }
}
