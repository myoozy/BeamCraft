package me.mzy.beamcraft.client.render;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Unit tests for {@link PhysicsVehicleRenderer#isDoubleSidedTranslucentGlass},
 * the per-range translucent culling policy. Window glass ships as paired shells
 * ({@code *_glass} + {@code *_glass_int}) and keeps back-face culling; lamp
 * lenses and covers are single-shell and must draw double-sided. The decision
 * keys on the raw DAE material identity, never the aliased target definition —
 * Sunburst lamp covers alias to {@code sunburst2_glass}, which must NOT cull
 * them. Pure logic; no GL, no Minecraft renderer.
 */
class TranslucentCullPolicyTest {

    @Test
    void pairedWindowGlassKeepsCulling() {
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("sunburst2_glass"),
                "exterior window glass is paired and culls back-faces");
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("sunburst2_glass_int"),
                "interior window glass is the paired opposite shell and culls");
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("sunburst2_glass_dmg"));
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("sunburst2_glass_on"));
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("sunburst2_glass_on_intense"));
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("pickup_glass"));
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("pickup_glass_int"));
    }

    @Test
    void singleShellLampLensesDrawDoubleSided() {
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("sunburst2_headlightglass"),
                "a headlight lens is a single shell and must be double-sided");
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("sunburst2_signalglass_L"));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("sunburst2_signalglass_R"));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("sunburst2_taillightglass"));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("sunburst2_foglightglass"));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("sunburst2_reverselightglass"));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("pickup_headlightglass"));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("pickup_lowbeamglass"));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("pickup_highbeamglass"));
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("pickup_sealedbeam_foglightglass"));
    }

    @Test
    void caseInsensitiveOnRawDaeName() {
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("Sunburst2_HEADLIGHTGLASS"));
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("SUNBURST2_GLASS_INT"));
    }

    @Test
    void aliasTargetMustNotBeUsedForTheDecision() {
        // The Sunburst lamp covers resolve (via glowMap) to the sunburst2_glass
        // plan, which is a paired window-glass family member. If the policy keyed
        // on the aliased target instead of the raw DAE identity, the single-shell
        // lamp covers would be culled and vanish from behind.
        assertTrue(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("sunburst2_headlightglass"));
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("sunburst2_glass"),
                "the aliased target alone must not decide the policy");
    }

    @Test
    void nonGlassTranslucentNeverCallsThePolicy() {
        // Body/interior/mechanical ranges contain no "glass" and keep the
        // default culling; the policy is only consulted for translucent ranges.
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("sunburst2_interior"));
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("sunburst2_lights"));
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass("pickup_interior"));
        assertFalse(PhysicsVehicleRenderer.isDoubleSidedTranslucentGlass(null));
    }
}
