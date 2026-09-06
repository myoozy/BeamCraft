package me.mzy.beamcraft.client.physics;

import me.mzy.beamcraft.utility.Utility;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Two-node, impulse-based coupler constraints.
 *
 * <p>Couplers deliberately do not participate in the beam stiffness matrix:
 * they have no spring/damping coefficients. Latching is velocity-limited by
 * the JBeam latch speed, while an attached pair cancels relative velocity.
 * The impulse required by that constraint, divided by the substep duration,
 * is compared directly with the JBeam coupler strength.</p>
 */
public final class CouplerContainer {
    public static final byte LATCHING = 0;
    public static final byte ATTACHED = 1;
    public static final byte BROKEN = 2;

    private static final int INITIAL_CAPACITY = 16;

    public int count;
    public int[] node1 = new int[INITIAL_CAPACITY];
    public int[] node2 = new int[INITIAL_CAPACITY];
    public float[] strength = new float[INITIAL_CAPACITY];
    public float[] captureRadius = new float[INITIAL_CAPACITY];
    public float[] lockRadius = new float[INITIAL_CAPACITY];
    public float[] latchSpeed = new float[INITIAL_CAPACITY];
    public String[] breakGroup = new String[INITIAL_CAPACITY];
    public byte[] state = new byte[INITIAL_CAPACITY];

    private final Map<String, List<Integer>> breakGroupToCouplers = new HashMap<>();

    public int add(PhysicsSpecs.CouplerSpec spec, int node1Index, int node2Index) {
        ensureCapacity();
        int index = count++;
        node1[index] = node1Index;
        node2[index] = node2Index;
        strength[index] = Math.max(0.0f, spec.strength());
        captureRadius[index] = Math.max(0.0f, spec.captureRadius());
        lockRadius[index] = Math.max(0.0f, spec.lockRadius());
        latchSpeed[index] = Math.max(0.0f, spec.latchSpeed());
        breakGroup[index] = normalizeGroup(spec.breakGroup());
        state[index] = LATCHING;
        if (breakGroup[index] != null) {
            breakGroupToCouplers.computeIfAbsent(breakGroup[index], ignored -> new ArrayList<>())
                    .add(index);
        }
        return index;
    }

    /** Solves each pair once after ordinary forces have updated node velocities. */
    public void solveVelocityConstraints(NodeContainer nodes, float dt) {
        if (!(dt > 0.0f) || !Float.isFinite(dt)) return;
        float invDt = 1.0f / dt;

        for (int i = 0; i < count; i++) {
            if (state[i] == BROKEN) continue;
            int n1 = node1[i];
            int n2 = node2[i];
            float m1 = nodes.mass[n1];
            float m2 = nodes.mass[n2];
            if (!(m1 > PhysicsWorld.KINDA_SMALL_NUMBER)
                    || !(m2 > PhysicsWorld.KINDA_SMALL_NUMBER)) continue;

            float dx = nodes.posX[n2] - nodes.posX[n1];
            float dy = nodes.posY[n2] - nodes.posY[n1];
            float dz = nodes.posZ[n2] - nodes.posZ[n1];
            float distanceSq = dx * dx + dy * dy + dz * dz;
            float distance = (float) Math.sqrt(Math.max(0.0f, distanceSq));
            float nx = 0.0f, ny = 0.0f, nz = 0.0f;
            if (distance > PhysicsWorld.KINDA_SMALL_NUMBER) {
                float invDistance = 1.0f / distance;
                nx = dx * invDistance;
                ny = dy * invDistance;
                nz = dz * invDistance;
            }

            float rvx = nodes.velX[n2] - nodes.velX[n1];
            float rvy = nodes.velY[n2] - nodes.velY[n1];
            float rvz = nodes.velZ[n2] - nodes.velZ[n1];
            float desiredX;
            float desiredY;
            float desiredZ;

            if (state[i] == LATCHING && distance > lockRadius[i]) {
                float closingSpeed = Math.min(latchSpeed[i], (distance - lockRadius[i]) * invDt);
                desiredX = -nx * closingSpeed;
                desiredY = -ny * closingSpeed;
                desiredZ = -nz * closingSpeed;
                // Attraction is radial while the latch is still travelling.
                float radialVelocity = rvx * nx + rvy * ny + rvz * nz;
                rvx = nx * radialVelocity;
                rvy = ny * radialVelocity;
                rvz = nz * radialVelocity;
            } else {
                state[i] = ATTACHED;
                float correctionSpeed = distance > lockRadius[i]
                        ? Math.min(latchSpeed[i], (distance - lockRadius[i]) * invDt)
                        : 0.0f;
                desiredX = -nx * correctionSpeed;
                desiredY = -ny * correctionSpeed;
                desiredZ = -nz * correctionSpeed;
            }

            float reducedMass = Utility.reducedMass(m1, m2);
            float impulseX = reducedMass * (desiredX - rvx);
            float impulseY = reducedMass * (desiredY - rvy);
            float impulseZ = reducedMass * (desiredZ - rvz);
            float impulseMagnitude = (float) Math.sqrt(
                    impulseX * impulseX + impulseY * impulseY + impulseZ * impulseZ);
            float requiredForce = impulseMagnitude * invDt;
            if (!Float.isFinite(requiredForce) || requiredForce > strength[i]) {
                state[i] = BROKEN;
                continue;
            }

            float invM1 = 1.0f / m1;
            float invM2 = 1.0f / m2;
            nodes.velX[n1] -= impulseX * invM1;
            nodes.velY[n1] -= impulseY * invM1;
            nodes.velZ[n1] -= impulseZ * invM1;
            nodes.velX[n2] += impulseX * invM2;
            nodes.velY[n2] += impulseY * invM2;
            nodes.velZ[n2] += impulseZ * invM2;
        }
    }

    public void breakByGroup(String group) {
        List<Integer> indices = breakGroupToCouplers.get(group);
        if (indices == null) return;
        for (int index : indices) state[index] = BROKEN;
    }

    public void reset() {
        for (int i = 0; i < count; i++) state[i] = LATCHING;
    }

    public void clear() {
        count = 0;
        breakGroupToCouplers.clear();
    }

    private void ensureCapacity() {
        if (count < node1.length) return;
        int newSize = node1.length * 2;
        node1 = Utility.expand(node1, newSize);
        node2 = Utility.expand(node2, newSize);
        strength = Utility.expand(strength, newSize);
        captureRadius = Utility.expand(captureRadius, newSize);
        lockRadius = Utility.expand(lockRadius, newSize);
        latchSpeed = Utility.expand(latchSpeed, newSize);
        breakGroup = java.util.Arrays.copyOf(breakGroup, newSize);
        state = java.util.Arrays.copyOf(state, newSize);
    }

    private static String normalizeGroup(String group) {
        return group == null || group.isBlank() ? null : group.trim();
    }
}
