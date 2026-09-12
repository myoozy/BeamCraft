package me.mzy.beamcraft.client.physics;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Pins the semi-implicit Euler stability bound a two-node beam actually has, because
 * the limiter's per-node budget prefactor is derived from it and the single-DOF
 * number is easy to reach for by mistake.
 *
 * <p>Two equal node masses {@code m} joined by one spring obey, for the relative
 * coordinate {@code r = x1 - x2},
 *
 * <pre>
 *   m x1'' = -k r,  m x2'' = +k r   =>   r'' = -(2k/m) r
 * </pre>
 *
 * <p>so the oscillator frequency is {@code w^2 = 2k/m}, not {@code k/m}. The
 * semi-implicit amplification matrix for {@code r'' = -w^2 r} is
 * {@code A = [[1 - w^2 dt^2, dt], [-w^2 dt, 1]]}, whose determinant is identically 1
 * and whose trace is {@code 2 - w^2 dt^2}. Stability is {@code |trace| <= 2}, i.e.
 * {@code w^2 dt^2 <= 4}, which substitutes to
 *
 * <pre>
 *   k dt^2 &lt;= 2 m
 * </pre>
 *
 * <p>Exactly half the single-DOF {@code k dt^2 <= 4 m}, because a two-node relative
 * mode carries the reduced mass {@code m/2}. Using each node's own mass in a local
 * budget therefore wants a prefactor of 2 — which is what
 * {@code DirectionalStabilityLimiter.BUDGET_PREFACTOR} uses — and a prefactor of 4 is
 * not "the analytic margin", it is past the boundary.
 *
 * <p>These cases deliberately skip {@code finalizePhysicsSetup}, so the stability
 * limiter never runs and the raw integrator is what is under test.
 */
class TwoNodeStabilityBoundTest {

    private static final float DT = 1.0f / PhysicsWorld.invPhysicsDT;
    private static final float MASS = 1.0f;
    private static final float REST_SEPARATION = 1.0f;

    @Test
    void relativeModeIsBoundedBelowTwiceTheNodeMass() {
        assertTrue(bounded(1.8f), "k dt^2 = 1.8 m is inside the k dt^2 <= 2 m bound");
    }

    @Test
    void relativeModeGrowsJustAboveTwiceTheNodeMass() {
        assertFalse(bounded(2.2f), "k dt^2 = 2.2 m is past the k dt^2 <= 2 m bound");
    }

    @Test
    void singleDofMarginDoesNotRescueTheRelativeMode() {
        // k dt^2 = 3.6 m would be comfortably stable if the bound were the single-DOF
        // k dt^2 <= 4 m. It is not, and this case is what catches a prefactor of 4.
        assertFalse(bounded(3.6f), "k dt^2 = 3.6 m is still past the two-node bound");
    }

    /**
     * Integrates two equal-mass nodes joined by one axial spring for a fixed number of
     * sub-steps and reports whether the relative amplitude stayed bounded.
     *
     * @param kOverMassDtSq the spring in units of {@code m / dt^2}
     */
    private static boolean bounded(float kOverMassDtSq) {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        vehicle.addNode(TestBeamBuilder.massNode("a", 0.0f, 0.0f, 0.0f, MASS));
        vehicle.addNode(TestBeamBuilder.massNode("b", 1.0f, 0.0f, 0.0f, MASS));
        float spring = kOverMassDtSq * MASS / (DT * DT);
        vehicle.addBeam(TestBeamBuilder.normal().spring(spring).build());

        // Strain the beam so the relative mode has something to oscillate with. The
        // rest length is whatever the nodes spanned at add time, so the perturbation
        // has to be small: a large one would drive the node speed past the solver's
        // MAX_NODE_SPEED sanitiser and mask the integrator's own behaviour.
        float strain = 0.002f;
        vehicle.nodes.posX[0] -= strain * 0.5f;
        vehicle.nodes.posX[1] += strain * 0.5f;

        float worst = 0.0f;
        for (int step = 0; step < 4_000; step++) {
            vehicle.solveInternalForces(DT, 1.0f);
            float amplitude = Math.abs(separation(vehicle) - REST_SEPARATION);
            if (!Float.isFinite(amplitude)) return false;
            worst = Math.max(worst, amplitude);
        }
        // Bounded means the oscillation never runs away; the marginal case neither
        // grows nor decays, so anything past a modest multiple counts as growth.
        return worst < strain * 8.0f;
    }

    /** Absolute node separation; the signed difference flips sign with the axis. */
    private static float separation(SoftBodyVehicle vehicle) {
        return Math.abs(vehicle.nodes.posX[0] - vehicle.nodes.posX[1]);
    }
}
