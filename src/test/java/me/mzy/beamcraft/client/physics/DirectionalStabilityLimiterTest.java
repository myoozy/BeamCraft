package me.mzy.beamcraft.client.physics;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class DirectionalStabilityLimiterTest {
    private static final float INV_DT = 10.0f;
    private static final float SAFETY = 0.90f;

    @Test
    void clipsTheOutlierInsteadOfSofteningEveryCollinearBeam() {
        DirectionalStabilityLimiter limiter = limiter(3);
        int huge = limiter.addTwoNode(0, 1, 1, 0, 0, 1_000, 0);
        int ordinary = limiter.addTwoNode(0, 2, 1, 0, 0, 10, 0);

        limiter.solve();

        assertTrue(limiter.scale(huge) < 0.18f);
        assertEquals(1.0f, limiter.scale(ordinary), 1.0e-6f);
    }

    @Test
    void preservesIndependentOrthogonalDirections() {
        DirectionalStabilityLimiter limiter = limiter(3);
        int x = limiter.addTwoNode(0, 1, 1, 0, 0, 170, 0);
        int y = limiter.addTwoNode(0, 2, 0, 1, 0, 170, 0);

        limiter.solve();

        assertEquals(1.0f, limiter.scale(x), 1.0e-6f);
        assertEquals(1.0f, limiter.scale(y), 1.0e-6f);
    }

    @Test
    void dampingConsumesTheSameDiscreteTimeStabilityBudgetAsSpring() {
        DirectionalStabilityLimiter limiter = limiter(2);
        // q = k + 2*c/dt = 180 + 2*10*9 = 360; node budget is 180.
        int constraint = limiter.addTwoNode(0, 1, 1, 0, 0, 180, 9);

        limiter.solve();

        assertEquals(0.5f, limiter.scale(constraint), 1.0e-4f);
        DirectionalStabilityLimiter.CoefficientAllocation allocation =
                limiter.allocation(constraint, 180, 9, 9);
        assertEquals(1.0f, allocation.stiffnessScale(), 1.0e-6f);
        assertEquals(0.0f, allocation.dampingScale(), 1.0e-6f);
    }

    @Test
    void couplerLikeConstraintReservesSafeDampingBeforeSpring() {
        float[] masses = {5.0f, 5.0f};
        DirectionalStabilityLimiter limiter =
                new DirectionalStabilityLimiter(2, masses, 2_000.0f, SAFETY);
        double dampingCeiling = 4_750.0;
        int constraint = limiter.addTwoNode(
                0, 1, 1, 0, 0, 1.0e9, dampingCeiling);

        limiter.solve();
        DirectionalStabilityLimiter.CoefficientAllocation allocation =
                limiter.allocation(constraint, 1.0e9, 1.0e7, dampingCeiling, true);

        assertTrue(1.0e9 * allocation.stiffnessScale() > 15_000_000.0);
        assertEquals(dampingCeiling, 1.0e7 * allocation.dampingScale(), 0.1);
    }

    @Test
    void initiallyCoincidentCouplerGetsAnIsotropicBudgetInsteadOfZeroStiffness() {
        float[] masses = {7.5f, 7.5f};
        DirectionalStabilityLimiter limiter =
                new DirectionalStabilityLimiter(2, masses, 2_000.0f, SAFETY);
        double dampingCeiling = 7_125.0;
        int constraint = limiter.addIsotropicTwoNode(0, 1, 1.0e9, dampingCeiling);

        limiter.solve();
        DirectionalStabilityLimiter.CoefficientAllocation allocation =
                limiter.allocation(constraint, 1.0e9, 1.0e7, dampingCeiling, true);

        assertTrue(allocation.stiffnessScale() > 0.0f);
        assertTrue(allocation.dampingScale() > 0.0f);
        assertEquals(dampingCeiling, 1.0e7 * allocation.dampingScale(), 0.1);
    }

    @Test
    void limitsThreeNodeConstraintsUsingEveryGradient() {
        DirectionalStabilityLimiter limiter = limiter(3);
        int constraint = limiter.addThreeNode(
                0, 2, 0, 0,
                1, -1, 0, 0,
                2, -1, 0, 0,
                100, 0);

        limiter.solve();

        assertTrue(limiter.scale(constraint) < 0.46f);
    }

    @Test
    void floatMaxSpringIsCappedInsteadOfBeingZeroed() {
        DirectionalStabilityLimiter limiter = limiter(2);
        int constraint = limiter.addTwoNode(0, 1, 1, 0, 0, Float.MAX_VALUE, 0);

        limiter.solve();

        float scale = limiter.scale(constraint);
        assertTrue(Float.isFinite(scale));
        assertTrue(scale > 0.0f);
        assertEquals(180.0f, Float.MAX_VALUE * scale, 0.1f);
    }

    private static DirectionalStabilityLimiter limiter(int nodeCount) {
        float[] masses = new float[nodeCount];
        java.util.Arrays.fill(masses, 1.0f);
        return new DirectionalStabilityLimiter(nodeCount, masses, INV_DT, SAFETY);
    }
}
