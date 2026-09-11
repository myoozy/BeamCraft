package me.mzy.beamcraft.client.physics;

import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * Regression coverage for the BeamNG-compatible bounded-beam limit behaviour.
 *
 * <p>Past a short/long bound the ordinary spring/damping ramp to the limit
 * properties over {@code boundZone} meters of penetration. Spring coefficients
 * are treated as tangent stiffnesses and integrated across the transition, so
 * both force and stiffness remain continuous and the ordinary and limit springs
 * are never added in parallel. {@code beamLimitDampRebound} selects the limit
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

        // Boundary force = -80. Integrated force through half the zone is
        // 100*0.05 + 0.5*(1000-100)*0.05^2/0.1 = 16.25.
        assertEquals(-96.25f, force, EPS);
    }

    @Test
    void penetrationBeyondTheZoneAppliesTheFullAuthoredLimit() {
        // penetration = 0.1 >= boundZone = 0.05, so the blend saturates at 1.
        float force = axialForce(0.10f, 0.0f, NO_DAMP,
                0.8f, 1.0f, -1.0f, -1.0f, 1000.0f, 50.0f, 70.0f, 0.05f);

        // Boundary force = -80. The 0.05 m transition contributes 27.5 N and
        // the remaining 0.05 m at the limit stiffness contributes 50 N.
        assertEquals(-157.5f, force, EPS);
    }

    @Test
    void nonPositiveBoundZoneTransitionsImmediately() {
        float atBoundary = axialForce(0.20f, 0.0f, NO_DAMP,
                0.8f, 1.0f, -1.0f, -1.0f, 1000.0f, 50.0f, 70.0f, 0.0f);
        float inside = axialForce(0.19f, 0.0f, NO_DAMP,
                0.8f, 1.0f, -1.0f, -1.0f, 1000.0f, 50.0f, 70.0f, -3.0f);

        assertEquals(-80.0f, atBoundary, EPS, "no penetration means no limit force");
        // The force stays continuous at -80 and immediately takes the 1000 N/m
        // limit slope for the 0.01 m penetration.
        assertEquals(-90.0f, inside, EPS);
    }

    @Test
    void limitReboundDampingIsSelectedByAxialVelocitySign() {
        // Blend saturates at 1; only the damping channel differs between the runs.
        float lengthening = axialForce(0.10f, 2.0f, NO_DAMP,
                0.8f, 1.0f, -1.0f, -1.0f, 1000.0f, 50.0f, 70.0f, 0.05f);
        float shortening = axialForce(0.10f, -2.0f, NO_DAMP,
                0.8f, 1.0f, -1.0f, -1.0f, 1000.0f, 50.0f, 70.0f, 0.05f);

        assertEquals(-157.5f + 2.0f * 70.0f, lengthening, EPS,
                "a lengthening beam uses beamLimitDampRebound");
        assertEquals(-157.5f - 2.0f * 50.0f, shortening, EPS,
                "a shortening beam uses beamLimitDamp");
    }

    @Test
    void dampingBlendsFromTheOrdinaryChannelToTheSelectedLimit() {
        // boundZone = 0.2, penetration = 0.1, blend = 0.5; ordinary damp = 10.
        float lengthening = axialForce(0.10f, 2.0f, 10.0f,
                0.8f, 1.0f, -1.0f, -1.0f, 1000.0f, 50.0f, 90.0f, 0.2f);
        float shortening = axialForce(0.10f, -2.0f, 10.0f,
                0.8f, 1.0f, -1.0f, -1.0f, 1000.0f, 50.0f, 90.0f, 0.2f);

        // Spring at the halfway blend: boundary force -80 minus the integrated
        // transition force 100*0.1 + 0.5*(1000-100)*0.1^2/0.2 = 32.5.
        // Halfway damping: 10 + 0.5 * (90 - 10) = 50, and 10 + 0.5 * (50 - 10) = 30.
        assertEquals(-112.5f + 2.0f * 50.0f, lengthening, EPS);
        assertEquals(-112.5f - 2.0f * 30.0f, shortening, EPS);
    }

    @Test
    void unspecifiedLimitReboundDampingFallsBackToLimitDamp() {
        float lengthening = axialForce(0.10f, 2.0f, NO_DAMP,
                0.8f, 1.0f, -1.0f, -1.0f, 1000.0f, 50.0f, -1.0f, 0.05f);
        float shortening = axialForce(0.10f, -2.0f, NO_DAMP,
                0.8f, 1.0f, -1.0f, -1.0f, 1000.0f, 50.0f, -1.0f, 0.05f);

        assertEquals(-157.5f + 2.0f * 50.0f, lengthening, EPS,
                "an unauthored beamLimitDampRebound falls back to beamLimitDamp");
        assertEquals(-157.5f - 2.0f * 50.0f, shortening, EPS);
    }

    @Test
    void longBoundRampsOverTheSameZone() {
        // longBound = 0.2 => longBoundary = 1.2, penetration = 0.05, blend = 0.5.
        float force = axialForce(1.25f, 0.0f, NO_DAMP,
                1.0f, 0.2f, -1.0f, -1.0f, 1000.0f, 50.0f, 70.0f, 0.1f);

        // Boundary force 20 plus 16.25 N integrated through half the zone.
        assertEquals(36.25f, force, EPS);
    }

    @Test
    void shortBoundRangeOverridesTheRatioBound() {
        // shortBoundRange = 0.5 => shortBoundary = 0.5, penetration = 0.1, blend = 0.5.
        float force = axialForce(0.40f, 0.0f, NO_DAMP,
                0.8f, 1.0f, 0.5f, -1.0f, 1000.0f, 50.0f, 70.0f, 0.2f);

        // Boundary force -50 minus 32.5 N integrated through half the zone.
        assertEquals(-82.5f, force, EPS);
    }

    @Test
    void springSlopeDoesNotDoubleNearTheEndOfTheZone() {
        float beforeEnd = axialForce(0.101f, 0.0f, NO_DAMP,
                0.8f, 1.0f, -1.0f, -1.0f, 1000.0f, 50.0f, 70.0f, 0.1f);
        float atEnd = axialForce(0.10f, 0.0f, NO_DAMP,
                0.8f, 1.0f, -1.0f, -1.0f, 1000.0f, 50.0f, 70.0f, 0.1f);

        float numericalSlope = (beforeEnd - atEnd) / 0.001f;
        assertEquals(995.5f, numericalSlope, 1.0f,
                "the transition slope should approach beamLimitSpring, not twice that value");
    }

    @Test
    void stabilityLimiterBudgetsTheLargerSpringInsteadOfTheirSum() {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        vehicle.addNode(TestBeamBuilder.massNode("a", 0.0f, 0.0f, 0.0f, 5.0f));
        vehicle.addNode(TestBeamBuilder.massNode("b", 1.0f, 0.0f, 0.0f, 5.0f));
        vehicle.addBeam(TestBeamBuilder.bounded().spring(30_000_000.0f).build());
        vehicle.boundedBeams.limitSpring[0] = 30_000_000.0f;

        // At 2000 Hz with the 0.9 safety fraction this pair admits 36 MN/m.
        // Each alternative spring is safe by itself; treating them as parallel
        // would incorrectly register 60 MN/m and soften both coefficients.
        vehicle.finalizePhysicsSetup();

        assertEquals(30_000_000.0f, vehicle.boundedBeams.spring[0], 1.0f);
        assertEquals(30_000_000.0f, vehicle.boundedBeams.limitSpring[0], 1.0f);
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
