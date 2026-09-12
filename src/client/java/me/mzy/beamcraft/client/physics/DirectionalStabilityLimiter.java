package me.mzy.beamcraft.client.physics;

import me.mzy.beamcraft.utility.Utility;

import java.util.ArrayList;
import java.util.List;

/**
 * Limits scalar spring/damper constraints for the semi-implicit Euler step.
 *
 * <p>For one mode the stability boundary is {@code k dt^2 + 2 c dt < 4 m_generalised}.
 * A two-node beam's relative mode has half the per-node mass as its generalised
 * mass, so written against each node's own mass the bound is
 * {@code k dt^2 + 2 c dt < 2 m_node}, which is what {@link #BUDGET_PREFACTOR}
 * encodes. We therefore limit the combined coefficient {@code q = k + 2 c / dt}.
 * Each constraint contributes {@code q * g * g^T} to every participating node's
 * directional matrix, and an overloaded node scales its constraints down in
 * proportion to how much each one fills it — see {@link #solve()} for why both
 * cutting only the largest and scaling everything uniformly are wrong.</p>
 */
final class DirectionalStabilityLimiter {
    private static final double EPS = 1.0e-9;
    private static final int MAX_SWEEPS = 128;

    /**
     * Prefactor on the per-node directional budget {@code PREFACTOR * m / dt^2 * safetyFraction}.
     *
     * <p>A two-node beam's relative mode carries the reduced mass {@code m/2}, so
     * written against each node's own mass the exact two-node boundary is
     * {@code k dt^2 + 2 c dt <= 2 m} — half the single-DOF {@code <= 4 m} because the
     * relative mode sees half the mass. See {@code TwoNodeStabilityBoundTest} for the
     * numerical check. A prefactor of 2 is therefore the analytic bound for this
     * budget, not half of a larger one, and {@code safetyFraction} 0.90 leaves only
     * 10% of margin.
     *
     * <p>This is not a tuning knob. A prefactor of 4 is the single-DOF number applied
     * to the wrong mass and destabilises both the wheels and the body.
     */
    private static final double BUDGET_PREFACTOR = 2.0;

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

    /**
     * Brings every node under its directional budget.
     *
     * <p>Every constraint that fills the node is scaled, and the amount it gives up is
     * weighted by how much of the node it accounts for — the bigger the contribution,
     * the bigger the reduction. Two simpler policies were tried first and both are
     * wrong in opposite directions:
     *
     * <ul>
     *   <li>Cutting only the largest contributor starves dampers. A damper's
     *       {@code q = k + 2c/dt} is an order of magnitude above the shell beams sharing
     *       its node, so it is always the largest and always pays the whole overage: on
     *       the stock ETK800 the rear shock came back at 55% of its authored rebound
     *       coefficient while every surrounding beam kept 100%.</li>
     *   <li>Scaling every constraint by one common factor wrecks anything that merely
     *       shares the node. The suspension spring's {@code q = k = 3e4} is 0.03% of a
     *       {@code 3e7} node budget, so scaling it relieves nothing, but it carries the
     *       corner — one pass took the front springs from 30000 to 11774 and put the car
     *       on the ground.</li>
     * </ul>
     *
     * <p>Weighting by share needs no threshold to avoid the second failure: a
     * constraint's protection is its own share going to zero. This is the property the
     * tests pin, so no magic cut-off can creep back in.
     *
     * <p>Sweeps repeat because a constraint belongs to two nodes, so reducing one
     * node's share moves the other's.
     */
    void solve() {
        for (int sweep = 0; sweep < MAX_SWEEPS; sweep++) {
            boolean changed = false;
            for (int node = 0; node < incident.size(); node++) {
                List<Integer> nodeConstraints = incident.get(node);
                if (nodeConstraints.isEmpty() || nodeMass[node] <= PhysicsWorld.KINDA_SMALL_NUMBER) continue;

                Utility.SymmetricEigenpair3 mode = largestMode(node, nodeConstraints);
                double budget = BUDGET_PREFACTOR * nodeMass[node] * invDt * invDt * safetyFraction;

                // The dominant eigenvalue is the sum of the per-constraint contributions
                // along the eigenvector, so summing them here both checks the budget and
                // gives the shares the weighting needs, with no mismatch between the two.
                double total = 0.0;
                double sumSquares = 0.0;
                for (int id : nodeConstraints) {
                    double c = contribution(id, node, mode);
                    total += c;
                    sumSquares += c * c;
                }
                if (total <= budget * (1.0 + 1.0e-7)) continue;
                if (total <= EPS || sumSquares <= EPS) continue;

                // Each constraint is multiplied by (1 - share) + share * base, a convex
                // combination that gives a constraint filling the whole node `base` and
                // leaves a negligible one at 1. `base` is the factor that brings the node
                // exactly onto its budget when a single constraint dominates, and is
                // written as (1 - q) + r * q rather than 1 - excess because the latter
                // cancels to zero for a pathological stiffness such as Float.MAX_VALUE:
                // the reduction there is ~1e-37 relative, well under double epsilon, and
                // a naive 1 - x would round it away and zero the beam outright.
                double r = budget / total;
                double q = total * total / sumSquares;
                double base = Math.max(0.0, (1.0 - q) + r * q);
                for (int id : nodeConstraints) {
                    double c = contribution(id, node, mode);
                    if (c <= 0.0) continue;
                    double share = c / total;
                    constraints.get(id).scale *= (1.0 - share) + share * base;
                }
                changed = true;
            }
            if (!changed) return;
        }

        // Fallback for the case where the exempt constraints alone fill the budget, so
        // no amount of scaling the rest can bring the node under it. Rare by
        // construction, and it moves the node out of the pathological configuration.
        for (int sweep = 0; sweep < 8; sweep++) {
            boolean changed = false;
            for (int node = 0; node < incident.size(); node++) {
                List<Integer> nodeConstraints = incident.get(node);
                if (nodeConstraints.isEmpty() || nodeMass[node] <= PhysicsWorld.KINDA_SMALL_NUMBER) continue;
                Utility.SymmetricEigenpair3 mode = largestMode(node, nodeConstraints);
                double budget = BUDGET_PREFACTOR * nodeMass[node] * invDt * invDt * safetyFraction;
                if (mode.value() <= budget || mode.value() <= EPS) continue;
                double factor = budget / mode.value();
                for (int id : nodeConstraints) constraints.get(id).scale *= factor;
                changed = true;
            }
            if (!changed) break;
        }
    }

    /**
     * How much of {@code mode} this constraint accounts for at {@code node}, i.e. its
     * share of the node's directional stiffness along the dominant eigenvector.
     */
    private double contribution(int constraintId, int node, Utility.SymmetricEigenpair3 mode) {
        Constraint constraint = constraints.get(constraintId);
        if (constraint.scale <= 0.0 || constraint.q <= 0.0) return 0.0;
        if (constraint.isotropic) return constraint.scale * constraint.q;
        double[] g = constraint.gradientAt(node);
        double projection = g[0] * mode.x() + g[1] * mode.y() + g[2] * mode.z();
        return constraint.scale * constraint.q * projection * projection;
    }

    float scale(int constraintId) {
        return (float) Math.clamp(constraints.get(constraintId).scale, 0.0, 1.0);
    }

    /**
     * Largest damping coefficient this constraint may carry, independent of the
     * value that was registered: the cutoff-aware budget times this constraint's
     * share of the node budget. A value that was authored below the budget may be
     * raised up to here by an actuator (for example an adaptive damper mode)
     * without re-running the limiter.
     */
    float maxDampingCeiling(int constraintId, double dampingCeiling) {
        Constraint constraint = constraints.get(constraintId);
        double coefficientScale = Math.clamp(constraint.scale, 0.0, 1.0);
        return (float) Math.min(Float.MAX_VALUE,
                Math.max(0.0, dampingCeiling) * coefficientScale);
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

    /**
     * Budget pressure one node is under after {@link #solve()}, one entry per
     * incident constraint, ordered by how much of the node's largest directional
     * mode that constraint accounts for. Diagnostic only — it must run after
     * {@link #solve()}, which is what fixes the {@code scale} values reported.
     */
    List<NodePressure> pressure(int node) {
        List<Integer> nodeConstraints = incident.get(node);
        Utility.SymmetricEigenpair3 mode = largestMode(node, nodeConstraints);
        double budget = BUDGET_PREFACTOR * nodeMass[node] * invDt * invDt * safetyFraction;
        List<NodePressure> result = new ArrayList<>(nodeConstraints.size());
        for (int id : nodeConstraints) {
            Constraint constraint = constraints.get(id);
            double contribution;
            if (constraint.isotropic) {
                contribution = constraint.scale * constraint.q;
            } else {
                double[] g = constraint.gradientAt(node);
                double projection = g[0] * mode.x() + g[1] * mode.y() + g[2] * mode.z();
                contribution = constraint.scale * constraint.q * projection * projection;
            }
            result.add(new NodePressure(id, constraint.q, constraint.scale,
                    contribution, budget, mode.value()));
        }
        result.sort((a, b) -> Double.compare(b.contribution(), a.contribution()));
        return result;
    }

    /** Number of registered constraints, so callers can map ids back to beam families. */
    int constraintCount() {
        return constraints.size();
    }

    /** One constraint's share of one node's directional budget after {@link #solve()}. */
    record NodePressure(int constraintId, double q, double scale, double contribution,
                        double budget, double largestMode) {
        /** Share of the largest directional mode this constraint accounts for, 0..1. */
        double contributionShare() {
            return largestMode <= 0.0 ? 0.0 : contribution / largestMode;
        }

        /** How far the node is over its budget; 1.0 means exactly at the limit. */
        double pressureRatio() {
            return budget <= 0.0 ? Double.POSITIVE_INFINITY : largestMode / budget;
        }
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
