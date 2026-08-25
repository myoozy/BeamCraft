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
    void returnsAbsoluteCeilingsForCombinedSpringAndDampingBudget() {
        DirectionalStabilityLimiter limiter = limiter(2);
        int constraint = limiter.addTwoNode(0, 1, 1, 0, 0, 180, 9);

        limiter.solve();

        assertEquals(0.5f, limiter.scale(constraint), 1.0e-4f);
        DirectionalStabilityLimiter.CoefficientCeilings ceilings =
                limiter.ceilings(constraint, 180, 9, 9);
        assertEquals(90.0f, ceilings.maxStiffness(), 1.0e-3f);
        assertEquals(4.5f, ceilings.maxDamping(), 1.0e-3f);
    }

    @Test
    void highStiffnessConstraintIsClippedWithoutDampingInput() {
        float[] masses = {5.0f, 5.0f};
        DirectionalStabilityLimiter limiter =
                new DirectionalStabilityLimiter(2, masses, 2_000.0f, SAFETY);
        int constraint = limiter.addTwoNode(
                0, 1, 1, 0, 0, 1.0e9, 0);

        limiter.solve();

        assertEquals(36_000_000.0, 1.0e9 * limiter.scale(constraint), 10.0);
    }

    @Test
    void initiallyCoincidentConstraintGetsAnIsotropicBudgetInsteadOfZeroStiffness() {
        float[] masses = {7.5f, 7.5f};
        DirectionalStabilityLimiter limiter =
                new DirectionalStabilityLimiter(2, masses, 2_000.0f, SAFETY);
        int constraint = limiter.addIsotropicTwoNode(0, 1, 1.0e9, 0);

        limiter.solve();

        assertEquals(54_000_000.0, 1.0e9 * limiter.scale(constraint), 1_000.0);
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
