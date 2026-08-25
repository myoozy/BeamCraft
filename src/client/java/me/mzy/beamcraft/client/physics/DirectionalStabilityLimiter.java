package me.mzy.beamcraft.client.physics;

import me.mzy.beamcraft.utility.Utility;

import java.util.ArrayList;
import java.util.List;

/**
 * Limits scalar spring/damper constraints for the semi-implicit Euler step.
 *
 * <p>For one mode the stability boundary is {@code k dt^2 + 2 c dt < 4 m}.
 * We therefore limit the combined coefficient {@code q = k + 2 c / dt}.
 * Each constraint contributes {@code q * g * g^T} to every participating
 * node's directional matrix. Keeping every node matrix below
 * {@code 2 m / dt^2} is a conservative sufficient bound for the coupled
 * system. When a direction is overloaded, the largest contributor in that
 * direction is reduced first, so ordinary beams sharing the node retain their
 * stiffness whenever possible.</p>
 */
final class DirectionalStabilityLimiter {
    private static final double EPS = 1.0e-9;
    private static final int MAX_SWEEPS = 128;

    private final float[] nodeMass;
    private final double invDt;
    private final double safetyFraction;
    private final List<Constraint> constraints = new ArrayList<>();
    private final List<List<Integer>> incident;

    DirectionalStabilityLimiter(int nodeCount, float[] nodeMass, float invDt, float safetyFraction) {
        this.nodeMass = nodeMass;
        this.invDt = invDt;
        this.safetyFraction = safetyFraction;
        this.incident = new ArrayList<>(nodeCount);
        for (int i = 0; i < nodeCount; i++) incident.add(new ArrayList<>());
    }

    int addTwoNode(int n1, int n2, double gx, double gy, double gz,
                   double stiffness, double damping) {
        return addConstraint(
                new int[]{n1, n2},
                new double[][]{{-gx, -gy, -gz}, {gx, gy, gz}},
                stiffness, damping, false);
    }

    int addIsotropicTwoNode(int n1, int n2, double stiffness, double damping) {
        return addConstraint(
                new int[]{n1, n2},
                new double[][]{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
                stiffness, damping, true);
    }

    int addThreeNode(int n1, double g1x, double g1y, double g1z,
                     int n2, double g2x, double g2y, double g2z,
                     int n3, double g3x, double g3y, double g3z,
                     double stiffness, double damping) {
        return addConstraint(
                new int[]{n1, n2, n3},
                new double[][]{
                        {g1x, g1y, g1z},
                        {g2x, g2y, g2z},
                        {g3x, g3y, g3z}},
                stiffness, damping, false);
    }

    private int addConstraint(int[] nodeIds, double[][] gradients,
                              double stiffness, double damping, boolean isotropic) {
        double k = Math.max(0.0, stiffness);
        double c = Math.max(0.0, damping);
        double q = k + 2.0 * invDt * c;
        if (!Double.isFinite(q)) q = Float.MAX_VALUE;

        int id = constraints.size();
        constraints.add(new Constraint(nodeIds, gradients, q, isotropic));
        for (int nodeId : nodeIds) incident.get(nodeId).add(id);
        return id;
    }

    void solve() {
        for (int sweep = 0; sweep < MAX_SWEEPS; sweep++) {
            boolean changed = false;
            for (int node = 0; node < incident.size(); node++) {
                List<Integer> nodeConstraints = incident.get(node);
                if (nodeConstraints.isEmpty() || nodeMass[node] <= PhysicsWorld.KINDA_SMALL_NUMBER) continue;

                int remainingAdjustments = nodeConstraints.size() + 1;
                while (remainingAdjustments-- > 0) {
                    Utility.SymmetricEigenpair3 mode = largestMode(node, nodeConstraints);
                    double budget = 2.0 * nodeMass[node] * invDt * invDt * safetyFraction;
                    if (mode.value() <= budget * (1.0 + 1.0e-7)) break;

                    int worstId = -1;
                    double worstContribution = 0.0;
                    for (int constraintId : nodeConstraints) {
                        Constraint constraint = constraints.get(constraintId);
                        if (constraint.scale <= 0.0 || constraint.q <= 0.0) continue;
                        double contribution;
                        if (constraint.isotropic) {
                            contribution = constraint.scale * constraint.q;
                        } else {
                            double[] g = constraint.gradientAt(node);
                            double projection = g[0] * mode.x() + g[1] * mode.y() + g[2] * mode.z();
                            contribution = constraint.scale * constraint.q * projection * projection;
                        }
                        if (contribution > worstContribution) {
                            worstContribution = contribution;
                            worstId = constraintId;
                        }
                    }

                    if (worstId < 0 || worstContribution <= EPS) break;
                    Constraint worst = constraints.get(worstId);
                    // Multiplying by the modal budget ratio avoids catastrophic
                    // cancellation for values such as Float.MAX_VALUE.
                    worst.scale *= (budget / mode.value()) * (1.0 - 1.0e-7);
                    changed = true;
                }
            }
            if (!changed) return;
        }

        // Numerical/pathological fallback. This is only reached if outlier-first
        // clipping did not converge after many complete sweeps.
        for (int sweep = 0; sweep < 8; sweep++) {
            boolean changed = false;
            for (int node = 0; node < incident.size(); node++) {
                List<Integer> nodeConstraints = incident.get(node);
                if (nodeConstraints.isEmpty() || nodeMass[node] <= PhysicsWorld.KINDA_SMALL_NUMBER) continue;
                Utility.SymmetricEigenpair3 mode = largestMode(node, nodeConstraints);
                double budget = 2.0 * nodeMass[node] * invDt * invDt * safetyFraction;
                if (mode.value() <= budget) continue;
                double factor = budget / mode.value();
                for (int id : nodeConstraints) constraints.get(id).scale *= factor;
                changed = true;
            }
            if (!changed) break;
        }
    }

    float scale(int constraintId) {
        return (float) Math.clamp(constraints.get(constraintId).scale, 0.0, 1.0);
    }

    CoefficientCeilings ceilings(int constraintId, double stiffness, double damping,
                                 double dampingCeiling) {
        Constraint constraint = constraints.get(constraintId);
        double coefficientScale = Math.clamp(constraint.scale, 0.0, 1.0);
        double k = Math.max(0.0, stiffness);
        double c = Math.min(Math.max(0.0, damping), Math.max(0.0, dampingCeiling));
        return new CoefficientCeilings(
                (float) Math.min(Float.MAX_VALUE, k * coefficientScale),
                (float) Math.min(Float.MAX_VALUE, c * coefficientScale));
    }

    private Utility.SymmetricEigenpair3 largestMode(int node, List<Integer> nodeConstraints) {
        double a00 = 0.0, a01 = 0.0, a02 = 0.0;
        double a11 = 0.0, a12 = 0.0, a22 = 0.0;
        for (int id : nodeConstraints) {
            Constraint constraint = constraints.get(id);
            double coefficient = constraint.scale * constraint.q;
            if (coefficient <= 0.0) continue;
            if (constraint.isotropic) {
                a00 += coefficient;
                a11 += coefficient;
                a22 += coefficient;
                continue;
            }
            double[] g = constraint.gradientAt(node);
            double x = g[0], y = g[1], z = g[2];
            a00 += coefficient * x * x;
            a01 += coefficient * x * y;
            a02 += coefficient * x * z;
            a11 += coefficient * y * y;
            a12 += coefficient * y * z;
            a22 += coefficient * z * z;
        }

        return Utility.dominantEigenpairSym3x3(a00, a01, a02, a11, a12, a22);
    }

    private static final class Constraint {
        final int[] nodeIds;
        final double[][] gradients;
        final double q;
        final boolean isotropic;
        double scale = 1.0;

        Constraint(int[] nodeIds, double[][] gradients, double q, boolean isotropic) {
            this.nodeIds = nodeIds;
            this.gradients = gradients;
            this.q = q;
            this.isotropic = isotropic;
        }

        double[] gradientAt(int nodeId) {
            for (int i = 0; i < nodeIds.length; i++) {
                if (nodeIds[i] == nodeId) return gradients[i];
            }
            throw new IllegalArgumentException("Node is not part of constraint");
        }
    }

    record CoefficientCeilings(float maxStiffness, float maxDamping) {}

}
