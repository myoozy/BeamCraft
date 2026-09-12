package me.mzy.beamcraft.client.physics;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class DirectionalStabilityLimiterTest {
    private static final float INV_DT = 10.0f;
    private static final float SAFETY = 0.90f;

    /**
     * The reduction is weighted by how much each constraint fills the node: the larger
     * the contribution, the larger the cut.
     *
     * <p>Both simpler policies are wrong in opposite directions. Cutting only the
     * largest starves dampers — a damper's {@code k + 2c/dt} is always the largest on
     * its node, so it paid the whole overage (the ETK800 rear shock came back at 55% of
     * its authored rebound coefficient). Scaling everything by one common factor wrecks
     * whatever merely shares the node: the suspension spring's {@code q = k} is 0.03%
     * of its node's budget, so shrinking it relieves nothing while it carries the
     * corner.
     */
    @Test
    void weightsTheReductionByHowMuchEachConstraintFillsTheNode() {
        DirectionalStabilityLimiter limiter = limiter(3);
        int huge = limiter.addTwoNode(0, 1, 1, 0, 0, 1_000, 0);
        int ordinary = limiter.addTwoNode(0, 2, 1, 0, 0, 10, 0);

        limiter.solve();

        // budget 2 * 1 * 10^2 * 0.90 = 180 against 1010 of collinear stiffness
        assertTrue(limiter.scale(huge) < limiter.scale(ordinary),
                "the larger contributor gives up more");
        assertTrue(1_000 * limiter.scale(huge) + 10 * limiter.scale(ordinary) <= 180.0f * 1.001f,
                "the node lands under its budget");
    }

    /**
     * A constraint too small to fill the node is left essentially untouched, however
     * far over the node is — with no threshold deciding that, only its own share.
     *
     * <p>This is the regression guard for the suspension springs. On the ETK800 the
     * front spring was 0.03% of its node's budget, yet an unqualified shared factor
     * took it from 30000 to 11774 and dropped the car onto the ground.
     */
    @Test
    void leavesConstraintsWhoseShareCannotRelieveTheBudgetAlone() {
        DirectionalStabilityLimiter limiter = limiter(3);
        int huge = limiter.addTwoNode(0, 1, 1, 0, 0, 1_000, 0);
        // 0.5 against the 180 budget is 0.28% of the node. Shrinking it would buy
        // essentially nothing, so its share drives its reduction to essentially zero.
        int springLike = limiter.addTwoNode(0, 2, 1, 0, 0, 0.5, 0);

        limiter.solve();

        assertTrue(limiter.scale(springLike) > 0.999f,
                "a 0.28% share cannot relieve the budget, so the stiffness stays");
        assertTrue(limiter.scale(huge) < 0.2f, "the node still has to come under budget");
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
