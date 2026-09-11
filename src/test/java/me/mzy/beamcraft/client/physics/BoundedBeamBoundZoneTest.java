package me.mzy.beamcraft.client.physics;

import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * Regression coverage for the BeamNG-compatible bounded-beam limit behaviour.
 *
 * <p>Past a short/long bound the ordinary spring/damping ramp to the limit
 * properties over {@code boundZone} meters of penetration: the limit spring and
 * damping contributions are scaled by a clamped {@code penetration / boundZone}
 * factor, so the force is continuous at the boundary and fully authored once the
 * penetration reaches the zone. {@code beamLimitDampRebound} selects the limit
 * damping while the beam lengthens and falls back to {@code beamLimitDamp}.
 *
 * <p>Nodes are massless and the beams are given an unbounded yield surface, so a
 * single solve leaves geometry and velocities untouched and
 * {@code nodes.forceX[0]} is exactly the axial beam force for the configured
 * separation and relative velocity.
 */
class BoundedBeamBoundZoneTest {

    private static final float DT = 1.0e-3f;
    private static final float RELAXATION = 1.0f;
    private static final float EPS = 1.0e-2f;

    private static final float SPRING = 100.0f;
    private static final float NO_DAMP = 0.0f;

    @Test
    void forceIsContinuousAtTheShortBoundBoundary() {
        float atBoundary = axialForce(0.20f, 0.0f, NO_DAMP,
                0.8f, 1.0f, -1.0f, -1.0f, 1000.0f, 50.0f, 70.0f, 0.1f);

        float justInside = axialForce(0.20f - 1.0e-5f, 0.0f, NO_DAMP,
                0.8f, 1.0f, -1.0f, -1.0f, 1000.0f, 50.0f, 70.0f, 0.1f);

        assertEquals(-80.0f, atBoundary, EPS,
                "at the bound only the ordinary spring may act");
        assertEquals(atBoundary, justInside, 1.0e-3f,
                "entering the bound zone must not step the force");
    }

    @Test
    void halfwayThroughTheZoneBlendsOrdinaryAndLimitSpring() {
        // shortBoundary = 1 * (1 - 0.8) = 0.2, penetration = 0.05, boundZone = 0.1.
        float force = axialForce(0.15f, 0.0f, NO_DAMP,
                0.8f, 1.0f, -1.0f, -1.0f, 1000.0f, 50.0f, 70.0f, 0.1f);

        // 100 * (0.15 - 1) + 0.5 * 1000 * (0.15 - 0.2) = -85 - 25.
        assertEquals(-110.0f, force, EPS);
    }

    @Test
    void penetrationBeyondTheZoneAppliesTheFullAuthoredLimit() {
        // penetration = 0.1 >= boundZone = 0.05, so the blend saturates at 1.
        float force = axialForce(0.10f, 0.0f, NO_DAMP,
                0.8f, 1.0f, -1.0f, -1.0f, 1000.0f, 50.0f, 70.0f, 0.05f);

        // 100 * (0.1 - 1) + 1000 * (0.1 - 0.2) = -90 - 100.
        assertEquals(-190.0f, force, EPS);
    }

    @Test
    void nonPositiveBoundZoneTransitionsImmediately() {
        float atBoundary = axialForce(0.20f, 0.0f, NO_DAMP,
                0.8f, 1.0f, -1.0f, -1.0f, 1000.0f, 50.0f, 70.0f, 0.0f);
        float inside = axialForce(0.19f, 0.0f, NO_DAMP,
                0.8f, 1.0f, -1.0f, -1.0f, 1000.0f, 50.0f, 70.0f, -3.0f);

        assertEquals(-80.0f, atBoundary, EPS, "no penetration means no limit force");
        // 100 * (0.19 - 1) + 1000 * (0.19 - 0.2) = -81 - 10.
        assertEquals(-91.0f, inside, EPS);
    }

    @Test
    void limitReboundDampingIsSelectedByAxialVelocitySign() {
        // Blend saturates at 1; only the damping channel differs between the runs.
        float lengthening = axialForce(0.10f, 2.0f, NO_DAMP,
                0.8f, 1.0f, -1.0f, -1.0f, 1000.0f, 50.0f, 70.0f, 0.05f);
        float shortening = axialForce(0.10f, -2.0f, NO_DAMP,
                0.8f, 1.0f, -1.0f, -1.0f, 1000.0f, 50.0f, 70.0f, 0.05f);

        assertEquals(-190.0f + 2.0f * 70.0f, lengthening, EPS,
                "a lengthening beam uses beamLimitDampRebound");
        assertEquals(-190.0f - 2.0f * 50.0f, shortening, EPS,
                "a shortening beam uses beamLimitDamp");
    }

    @Test
    void dampingBlendsFromTheOrdinaryChannelToTheSelectedLimit() {
        // boundZone = 0.2, penetration = 0.1, blend = 0.5; ordinary damp = 10.
        float lengthening = axialForce(0.10f, 2.0f, 10.0f,
                0.8f, 1.0f, -1.0f, -1.0f, 1000.0f, 50.0f, 90.0f, 0.2f);
        float shortening = axialForce(0.10f, -2.0f, 10.0f,
                0.8f, 1.0f, -1.0f, -1.0f, 1000.0f, 50.0f, 90.0f, 0.2f);

        // Spring at the halfway blend: 100 * (0.1 - 1) + 0.5 * 1000 * (0.1 - 0.2) = -140.
        // Halfway damping: 10 + 0.5 * (90 - 10) = 50, and 10 + 0.5 * (50 - 10) = 30.
        assertEquals(-140.0f + 2.0f * 50.0f, lengthening, EPS);
        assertEquals(-140.0f - 2.0f * 30.0f, shortening, EPS);
    }

    @Test
    void unspecifiedLimitReboundDampingFallsBackToLimitDamp() {
        float lengthening = axialForce(0.10f, 2.0f, NO_DAMP,
                0.8f, 1.0f, -1.0f, -1.0f, 1000.0f, 50.0f, -1.0f, 0.05f);
        float shortening = axialForce(0.10f, -2.0f, NO_DAMP,
                0.8f, 1.0f, -1.0f, -1.0f, 1000.0f, 50.0f, -1.0f, 0.05f);

        assertEquals(-190.0f + 2.0f * 50.0f, lengthening, EPS,
                "an unauthored beamLimitDampRebound falls back to beamLimitDamp");
        assertEquals(-190.0f - 2.0f * 50.0f, shortening, EPS);
    }

    @Test
    void longBoundRampsOverTheSameZone() {
        // longBound = 0.2 => longBoundary = 1.2, penetration = 0.05, blend = 0.5.
        float force = axialForce(1.25f, 0.0f, NO_DAMP,
                1.0f, 0.2f, -1.0f, -1.0f, 1000.0f, 50.0f, 70.0f, 0.1f);

        // 100 * (1.25 - 1) + 0.5 * 1000 * (1.25 - 1.2) = 25 + 25.
        assertEquals(50.0f, force, EPS);
    }

    @Test
    void shortBoundRangeOverridesTheRatioBound() {
        // shortBoundRange = 0.5 => shortBoundary = 0.5, penetration = 0.1, blend = 0.5.
        float force = axialForce(0.40f, 0.0f, NO_DAMP,
                0.8f, 1.0f, 0.5f, -1.0f, 1000.0f, 50.0f, 70.0f, 0.2f);

        // 100 * (0.4 - 1) + 0.5 * 1000 * (0.4 - 0.5) = -60 - 50.
        assertEquals(-110.0f, force, EPS);
    }

    /**
     * Builds a two-node bounded beam, positions it at {@code dist}, applies an
     * axial relative velocity and returns the resulting axial force on node 0.
     */
    private static float axialForce(float dist, float relVel, float damp,
                                    float shortBound, float longBound,
                                    float shortBoundRange, float longBoundRange,
                                    float limitSpring, float limitDamp,
                                    float limitDampRebound, float boundZone) {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        vehicle.addNode(node("a", 0.0f));
        vehicle.addNode(node("b", 1.0f));
        vehicle.addBeam(new PhysicsSpecs.BeamSpec(
                BeamContainer.BEAM_BOUNDED, "a", "b", null,
                List.of(), Float.POSITIVE_INFINITY,
                List.of(), 0, false,
                SPRING, damp, -1.0f, Float.MAX_VALUE, Float.MAX_VALUE,
                1.0f, 0.0f, false, 0.0f,
                shortBound, longBound, shortBoundRange, longBoundRange, boundZone,
                limitSpring, limitDamp, limitDampRebound,
                -1.0f, -1.0f, -1.0f, -1.0f, -1.0f,
                0.0f, 0.0f, 0.0f, Float.MAX_VALUE, null));

        // The beam's rest length is the add-time separation of 1 m; move the nodes
        // afterwards so the solve sees the requested axial strain.
        vehicle.nodes.posX[1] = dist;
        vehicle.nodes.velX[0] = 0.0f;
        vehicle.nodes.velX[1] = relVel;
        vehicle.solveInternalForces(DT, RELAXATION);
        return vehicle.nodes.forceX[0];
    }

    private static PhysicsSpecs.NodeSpec node(String name, float x) {
        // Massless, so the solve neither integrates the nodes nor adds gravity.
        return new PhysicsSpecs.NodeSpec(name, x, 0.0f, 0.0f,
                0.0f, 1.0f, 1.0f, 0, false, false, List.of());
    }
}
