package me.mzy.beamcraft.client.physics;

import java.util.BitSet;

/** Persistent fat-AABB overlap state for one directed triangle/node vehicle pair. */
final class CollisionChunkPairState {
    private int meshletCount = -1;
    private int nodeChunkCount = -1;
    private int triangleCount = -1;
    private int nodeCount = -1;
    private boolean[] overlapping = new boolean[0];
    private final BitSet formalCandidates = new BitSet();
    private boolean formalCandidatesInitialized;
    private boolean candidateKeysAddressable;

    int activeOverlaps;
    int lastTests;
    int lastAdded;
    int lastRemoved;
    long lastCoarsePairsExpanded;
    long lastTightCoarsePairs;
    long lastFinePairsExpanded;
    long lastTightFinePairs;
    long lastMissingCandidates;
    long lastNarrowHits;
    long lastFineFilterNanos;

    void update(SoftBodyVehicle triangleVehicle, SoftBodyVehicle nodeVehicle) {
        CollisionChunkIndex triangles = triangleVehicle.collisionChunks;
        CollisionChunkIndex nodes = nodeVehicle.collisionChunks;
        ensureDimensions(triangles.triangleMeshletCount(), nodes.nodeChunkCount(),
                triangleVehicle.triangles.count, nodeVehicle.nodes.count);
        lastTests = 0;
        lastAdded = 0;
        lastRemoved = 0;
        lastCoarsePairsExpanded = 0L;
        lastTightCoarsePairs = 0L;
        lastFinePairsExpanded = 0L;
        lastTightFinePairs = 0L;
        lastMissingCandidates = 0L;
        lastNarrowHits = 0L;
        lastFineFilterNanos = 0L;

        for (int dirty = 0; dirty < triangles.dirtyMeshletCount(); dirty++) {
            int meshlet = triangles.dirtyMeshletAt(dirty);
            for (int nodeChunk = 0; nodeChunk < nodeChunkCount; nodeChunk++) {
                updatePair(triangleVehicle == nodeVehicle, triangles, meshlet, nodes, nodeChunk);
            }
        }
        for (int dirty = 0; dirty < nodes.dirtyNodeChunkCount(); dirty++) {
            int nodeChunk = nodes.dirtyNodeChunkAt(dirty);
            for (int meshlet = 0; meshlet < meshletCount; meshlet++) {
                if (triangles.meshletDirty(meshlet)) continue;
                updatePair(triangleVehicle == nodeVehicle, triangles, meshlet, nodes, nodeChunk);
            }
        }
    }

    boolean overlaps(int meshlet, int nodeChunk) {
        return overlapping[meshlet * nodeChunkCount + nodeChunk];
    }

    void beginFormalWindow() {
        formalCandidates.clear();
        formalCandidatesInitialized = true;
    }

    void recordFormalCandidate(int triangle, int node) {
        int key = candidateKey(triangle, node);
        if (key >= 0) formalCandidates.set(key);
    }

    private void ensureDimensions(int requiredMeshlets, int requiredNodeChunks,
                                  int requiredTriangles, int requiredNodes) {
        if (meshletCount == requiredMeshlets && nodeChunkCount == requiredNodeChunks
                && triangleCount == requiredTriangles && nodeCount == requiredNodes) return;
        meshletCount = requiredMeshlets;
        nodeChunkCount = requiredNodeChunks;
        triangleCount = requiredTriangles;
        nodeCount = requiredNodes;
        overlapping = new boolean[meshletCount * nodeChunkCount];
        activeOverlaps = 0;
        formalCandidates.clear();
        formalCandidatesInitialized = false;
        candidateKeysAddressable = (long) triangleCount * nodeCount <= Integer.MAX_VALUE;
    }

    private void updatePair(boolean self, CollisionChunkIndex triangles, int meshlet,
                            CollisionChunkIndex nodes, int nodeChunk) {
        int pair = meshlet * nodeChunkCount + nodeChunk;
        boolean next = (!self || nodes.nodeChunkHasSelfCollision(nodeChunk))
                && triangles.fatChunksOverlap(meshlet, nodes, nodeChunk);
        boolean previous = overlapping[pair];
        lastTests++;
        if (next != previous) {
            overlapping[pair] = next;
            if (next) {
                activeOverlaps++;
                lastAdded++;
            } else {
                activeOverlaps--;
                lastRemoved++;
            }
        }
        if (next) {
            lastCoarsePairsExpanded++;
            if (triangles.chunksOverlap(meshlet, nodes, nodeChunk)) {
                lastTightCoarsePairs++;
                expandShadowCandidates(self, triangles, meshlet, nodes, nodeChunk);
            }
        }
    }

    private void expandShadowCandidates(boolean self, CollisionChunkIndex triangles, int meshlet,
                                        CollisionChunkIndex nodes, int nodeChunk) {
        long started = System.nanoTime();
        SoftBodyVehicle triangleVehicle = triangles.vehicle();
        SoftBodyVehicle nodeVehicle = nodes.vehicle();
        TriangleContainer triangleData = triangleVehicle.triangles;
        for (int triangleMember = triangles.triangleMeshletStart(meshlet);
             triangleMember < triangles.triangleMeshletEnd(meshlet); triangleMember++) {
            int triangle = triangles.triangleAt(triangleMember);
            if (triangleData.broken[triangle]) continue;
            int nA = triangleData.node1[triangle];
            int nB = triangleData.node2[triangle];
            int nC = triangleData.node3[triangle];
            int trianglePart = triangleData.partId[triangle];
            for (int nodeMember = nodes.nodeChunkStart(nodeChunk);
                 nodeMember < nodes.nodeChunkEnd(nodeChunk); nodeMember++) {
                int node = nodes.nodeAt(nodeMember);
                if (self) {
                    if (!nodeVehicle.nodes.selfCollision[node] || node == nA || node == nB || node == nC) continue;
                    if (trianglePart >= 0 && trianglePart < triangleVehicle.matrixPartStride
                            && triangleVehicle.nodeInPartMatrix != null
                            && triangleVehicle.nodeInPartMatrix[
                            node * triangleVehicle.matrixPartStride + trianglePart]) continue;
                }
                lastFinePairsExpanded++;
                if (!triangles.coarseTriangleOverlapsNode(triangle, nodes, node)) continue;
                lastTightFinePairs++;
                if (!formalCandidatesInitialized) continue;
                int key = candidateKey(triangle, node);
                if (key >= 0 && !formalCandidates.get(key)) {
                    lastMissingCandidates++;
                    if (CollisionPipeline.shadowCandidateNarrowHit(
                            nodeVehicle, node, triangleVehicle, triangle)) lastNarrowHits++;
                }
            }
        }
        lastFineFilterNanos += System.nanoTime() - started;
    }

    private int candidateKey(int triangle, int node) {
        if (!candidateKeysAddressable || triangle < 0 || triangle >= triangleCount
                || node < 0 || node >= nodeCount) return -1;
        return triangle * nodeCount + node;
    }
}
