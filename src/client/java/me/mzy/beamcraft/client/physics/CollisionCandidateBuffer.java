package me.mzy.beamcraft.client.physics;

import java.util.Arrays;

/** Reusable, task-exclusive output for parallel collision candidate generation. */
final class CollisionCandidateBuffer {
    private static final int INITIAL_CAPACITY = 256;

    SoftBodyVehicle triangleVehicle;
    SoftBodyVehicle[] nodeVehicles = new SoftBodyVehicle[INITIAL_CAPACITY];
    int[] nodeIds = new int[INITIAL_CAPACITY];
    SoftBodyVehicle[] triangleVehicles = new SoftBodyVehicle[INITIAL_CAPACITY];
    int[] triangleIds = new int[INITIAL_CAPACITY];
    int[] triangleA = new int[INITIAL_CAPACITY];
    int[] triangleB = new int[INITIAL_CAPACITY];
    int[] triangleC = new int[INITIAL_CAPACITY];
    int count;
    int dropped;
    int rawHits;
    int chunkPairTests;
    int chunkPairOverlaps;
    int chunkPairProductive;
    long finePairTests;

    void reset(SoftBodyVehicle triangleVehicle) {
        this.triangleVehicle = triangleVehicle;
        count = 0;
        dropped = 0;
        rawHits = 0;
        chunkPairTests = 0;
        chunkPairOverlaps = 0;
        chunkPairProductive = 0;
        finePairTests = 0L;
    }

    boolean add(SoftBodyVehicle nodeVehicle, int nodeId,
                SoftBodyVehicle triangleVehicle, int triangleId, int nA, int nB, int nC) {
        if (count >= SoftBodyCollisionManager.MAX_CONTACTS) {
            dropped++;
            return false;
        }
        ensureCapacity(count + 1);
        nodeVehicles[count] = nodeVehicle;
        nodeIds[count] = nodeId;
        triangleVehicles[count] = triangleVehicle;
        triangleIds[count] = triangleId;
        triangleA[count] = nA;
        triangleB[count] = nB;
        triangleC[count] = nC;
        count++;
        return true;
    }

    private void ensureCapacity(int required) {
        if (required <= nodeIds.length) return;
        int capacity = Math.min(SoftBodyCollisionManager.MAX_CONTACTS,
                Math.max(required, nodeIds.length << 1));
        nodeVehicles = Arrays.copyOf(nodeVehicles, capacity);
        nodeIds = Arrays.copyOf(nodeIds, capacity);
        triangleVehicles = Arrays.copyOf(triangleVehicles, capacity);
        triangleIds = Arrays.copyOf(triangleIds, capacity);
        triangleA = Arrays.copyOf(triangleA, capacity);
        triangleB = Arrays.copyOf(triangleB, capacity);
        triangleC = Arrays.copyOf(triangleC, capacity);
    }
}
