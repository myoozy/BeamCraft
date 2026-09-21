package me.mzy.beamcraft.client.physics;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

/**
 * Static collision primitive grouping plus swept bounds for one vehicle.
 *
 * <p>Nodes and triangles deliberately use independent partitions. Every
 * collidable node belongs to exactly one node chunk and every complete
 * collidable triangle belongs to exactly one triangle meshlet. A triangle may
 * therefore span several node chunks without being split or duplicated.</p>
 */
final class CollisionChunkIndex {
    static final int MAX_NODES_PER_CHUNK = 16;
    static final int MAX_TRIANGLES_PER_MESHLET = 8;
    static final double SPAN_WARNING_RATIO = 2.0;
    private static final double LARGE_TRIANGLE_MEDIAN_MULTIPLIER = 4.0;
    private static final double LARGE_TRIANGLE_VEHICLE_DIAGONAL_FRACTION = 0.5;
    private static final double SPAN_FLOOR = CollisionPipeline.SOFT_BROADPHASE_MARGIN * 2.0;

    private final SoftBodyVehicle vehicle;

    private int builtNodeCount = -1;
    private int builtTriangleCount = -1;

    private int[] nodeMembers = new int[0];
    private int[] nodeChunkStart = new int[]{0};
    private boolean[] nodeChunkHasSelfCollision = new boolean[0];
    private int[] nodeChunkPartId = new int[0];
    private int[] triangleMembers = new int[0];
    private int[] triangleMeshletStart = new int[]{0};
    private int[] triangleMeshletPartId = new int[0];
    private int oversizedTriangleCount;

    private double[] nodeMinX = new double[0], nodeMinY = new double[0], nodeMinZ = new double[0];
    private double[] nodeMaxX = new double[0], nodeMaxY = new double[0], nodeMaxZ = new double[0];
    private double[] triangleMinX = new double[0], triangleMinY = new double[0], triangleMinZ = new double[0];
    private double[] triangleMaxX = new double[0], triangleMaxY = new double[0], triangleMaxZ = new double[0];

    private double[] nodeChunkMinX = new double[0], nodeChunkMinY = new double[0], nodeChunkMinZ = new double[0];
    private double[] nodeChunkMaxX = new double[0], nodeChunkMaxY = new double[0], nodeChunkMaxZ = new double[0];
    private double[] meshletMinX = new double[0], meshletMinY = new double[0], meshletMinZ = new double[0];
    private double[] meshletMaxX = new double[0], meshletMaxY = new double[0], meshletMaxZ = new double[0];
    private double[] nodeChunkReferenceSpan = new double[0];
    private double[] meshletReferenceSpan = new double[0];

    private int stretchedNodeChunkCount;
    private int stretchedMeshletCount;
    private int regroupableNodeChunkCount;
    private int regroupableMeshletCount;
    private double maxNodeChunkSpanRatio = 1.0;
    private double maxMeshletSpanRatio = 1.0;

    // Rebuilt with the swept bounds. The first SAP prunes node chunks for a
    // meshlet. Each at-most-16-node chunk is sorted independently on all three
    // axes so an overlapping meshlet/chunk pair can use its narrowest scan.
    private long[] sortedNodeChunkKeys = new long[0];
    private double[] sortedNodeChunkPrefixMax = new double[0];
    private byte nodeChunkSweepAxis;
    private long[] sortedNodeKeys = new long[0];
    private double[] sortedNodePrefixMax = new double[0];
    private boolean nodeSapsInitialized;

    long refitNodeBoundsNanos;
    long refitNodeChunkBoundsNanos;
    long refitChunkSapNanos;
    long refitLocalSapsNanos;
    long refitLocalKeyFillNanos;
    long refitLocalSortNanos;
    long refitLocalPrefixNanos;
    long refitTriangleBoundsNanos;
    long refitMeshletBoundsNanos;

    CollisionChunkIndex(SoftBodyVehicle vehicle) {
        this.vehicle = vehicle;
    }

    void rebuild() {
        builtNodeCount = vehicle.nodes.count;
        builtTriangleCount = vehicle.triangles.count;

        Integer[] collidableNodes = collectCollidableNodes();
        List<int[]> nodeLeaves = new ArrayList<>();
        splitNodes(collidableNodes, 0, collidableNodes.length, nodeLeaves);
        nodeLeaves = refineLeavesByPart(nodeLeaves, false);
        nodeMembers = flatten(nodeLeaves);
        nodeChunkStart = starts(nodeLeaves);
        regroupableNodeChunkCount = countMultiMemberLeaves(nodeLeaves);
        nodeChunkPartId = leafPartIds(nodeLeaves, false);
        nodeChunkHasSelfCollision = new boolean[nodeLeaves.size()];
        for (int chunk = 0; chunk < nodeLeaves.size(); chunk++) {
            for (int node : nodeLeaves.get(chunk)) {
                nodeChunkHasSelfCollision[chunk] |= vehicle.nodes.selfCollision[node];
            }
        }

        Integer[] collidableTriangles = collectCollidableTriangles();
        List<int[]> meshlets = new ArrayList<>();
        oversizedTriangleCount = isolateOversizedTriangles(collidableTriangles, meshlets);
        splitTriangles(collidableTriangles, oversizedTriangleCount, collidableTriangles.length, meshlets);
        meshlets = refineLeavesByPart(meshlets, true);
        triangleMembers = flatten(meshlets);
        triangleMeshletStart = starts(meshlets);
        regroupableMeshletCount = countMultiMemberLeaves(meshlets);
        triangleMeshletPartId = leafPartIds(meshlets, true);

        int nodeCapacity = vehicle.nodes.count;
        nodeMinX = new double[nodeCapacity]; nodeMinY = new double[nodeCapacity]; nodeMinZ = new double[nodeCapacity];
        nodeMaxX = new double[nodeCapacity]; nodeMaxY = new double[nodeCapacity]; nodeMaxZ = new double[nodeCapacity];
        int triangleCapacity = vehicle.triangles.count;
        triangleMinX = new double[triangleCapacity]; triangleMinY = new double[triangleCapacity]; triangleMinZ = new double[triangleCapacity];
        triangleMaxX = new double[triangleCapacity]; triangleMaxY = new double[triangleCapacity]; triangleMaxZ = new double[triangleCapacity];

        int nodeChunkCount = nodeLeaves.size();
        nodeChunkMinX = new double[nodeChunkCount]; nodeChunkMinY = new double[nodeChunkCount]; nodeChunkMinZ = new double[nodeChunkCount];
        nodeChunkMaxX = new double[nodeChunkCount]; nodeChunkMaxY = new double[nodeChunkCount]; nodeChunkMaxZ = new double[nodeChunkCount];
        sortedNodeChunkKeys = new long[nodeChunkCount];
        sortedNodeChunkPrefixMax = new double[nodeChunkCount];
        sortedNodeKeys = new long[nodeMembers.length * 3];
        sortedNodePrefixMax = new double[nodeMembers.length * 3];
        nodeSapsInitialized = false;
        int meshletCount = meshlets.size();
        meshletMinX = new double[meshletCount]; meshletMinY = new double[meshletCount]; meshletMinZ = new double[meshletCount];
        meshletMaxX = new double[meshletCount]; meshletMaxY = new double[meshletCount]; meshletMaxZ = new double[meshletCount];
        nodeChunkReferenceSpan = buildNodeChunkReferenceSpans();
        meshletReferenceSpan = buildMeshletReferenceSpans();
    }

    void refit(double dtPredict) {
        if (builtNodeCount != vehicle.nodes.count || builtTriangleCount != vehicle.triangles.count) rebuild();

        long stageStarted = System.nanoTime();
        NodeContainer nodes = vehicle.nodes;
        double entityX = vehicle.entityX, entityY = vehicle.entityY, entityZ = vehicle.entityZ;
        for (int node = 0; node < nodes.count; node++) {
            double x = entityX + nodes.posX[node];
            double y = entityY + nodes.posY[node];
            double z = entityZ + nodes.posZ[node];
            double previousX = entityX + nodes.prevPosX[node];
            double previousY = entityY + nodes.prevPosY[node];
            double previousZ = entityZ + nodes.prevPosZ[node];
            double futureX = x + nodes.velX[node] * dtPredict;
            double futureY = y + nodes.velY[node] * dtPredict;
            double futureZ = z + nodes.velZ[node] * dtPredict;
            nodeMinX[node] = Math.min(previousX, Math.min(x, futureX));
            nodeMinY[node] = Math.min(previousY, Math.min(y, futureY));
            nodeMinZ[node] = Math.min(previousZ, Math.min(z, futureZ));
            nodeMaxX[node] = Math.max(previousX, Math.max(x, futureX));
            nodeMaxY[node] = Math.max(previousY, Math.max(y, futureY));
            nodeMaxZ[node] = Math.max(previousZ, Math.max(z, futureZ));
        }
        long stageFinished = System.nanoTime();
        refitNodeBoundsNanos = stageFinished - stageStarted;
        stageStarted = stageFinished;

        stretchedNodeChunkCount = 0;
        maxNodeChunkSpanRatio = 1.0;
        for (int chunk = 0; chunk < nodeChunkCount(); chunk++) {
            resetBounds(nodeChunkMinX, nodeChunkMinY, nodeChunkMinZ,
                    nodeChunkMaxX, nodeChunkMaxY, nodeChunkMaxZ, chunk);
            double tightMinX = Double.POSITIVE_INFINITY, tightMinY = Double.POSITIVE_INFINITY;
            double tightMinZ = Double.POSITIVE_INFINITY;
            double tightMaxX = Double.NEGATIVE_INFINITY, tightMaxY = Double.NEGATIVE_INFINITY;
            double tightMaxZ = Double.NEGATIVE_INFINITY;
            for (int member = nodeChunkStart[chunk]; member < nodeChunkStart[chunk + 1]; member++) {
                int node = nodeMembers[member];
                includeNodeBounds(nodeChunkMinX, nodeChunkMinY, nodeChunkMinZ,
                        nodeChunkMaxX, nodeChunkMaxY, nodeChunkMaxZ, chunk, node);
                tightMinX = Math.min(tightMinX, nodes.posX[node]);
                tightMinY = Math.min(tightMinY, nodes.posY[node]);
                tightMinZ = Math.min(tightMinZ, nodes.posZ[node]);
                tightMaxX = Math.max(tightMaxX, nodes.posX[node]);
                tightMaxY = Math.max(tightMaxY, nodes.posY[node]);
                tightMaxZ = Math.max(tightMaxZ, nodes.posZ[node]);
            }
            if (nodeChunkEnd(chunk) - nodeChunkStart[chunk] > 1) {
                double ratio = aabbSpan(tightMinX, tightMinY, tightMinZ, tightMaxX, tightMaxY, tightMaxZ)
                        / nodeChunkReferenceSpan[chunk];
                maxNodeChunkSpanRatio = Math.max(maxNodeChunkSpanRatio, ratio);
                if (ratio > SPAN_WARNING_RATIO) stretchedNodeChunkCount++;
            }
        }
        stageFinished = System.nanoTime();
        refitNodeChunkBoundsNanos = stageFinished - stageStarted;
        stageStarted = stageFinished;
        rebuildNodeChunkSap();
        stageFinished = System.nanoTime();
        refitChunkSapNanos = stageFinished - stageStarted;
        stageStarted = stageFinished;
        rebuildNodeSaps();
        stageFinished = System.nanoTime();
        refitLocalSapsNanos = stageFinished - stageStarted;
        stageStarted = stageFinished;

        TriangleContainer triangles = vehicle.triangles;
        double margin = CollisionPipeline.SOFT_BROADPHASE_MARGIN;
        for (int triangle = 0; triangle < triangles.count; triangle++) {
            if (!triangles.collision[triangle] || triangles.broken[triangle]) {
                triangleMinX[triangle] = triangleMinY[triangle] = triangleMinZ[triangle] = Double.POSITIVE_INFINITY;
                triangleMaxX[triangle] = triangleMaxY[triangle] = triangleMaxZ[triangle] = Double.NEGATIVE_INFINITY;
                continue;
            }
            int a = triangles.node1[triangle], b = triangles.node2[triangle], c = triangles.node3[triangle];
            triangleMinX[triangle] = Math.min(nodeMinX[a], Math.min(nodeMinX[b], nodeMinX[c])) - margin;
            triangleMinY[triangle] = Math.min(nodeMinY[a], Math.min(nodeMinY[b], nodeMinY[c])) - margin;
            triangleMinZ[triangle] = Math.min(nodeMinZ[a], Math.min(nodeMinZ[b], nodeMinZ[c])) - margin;
            triangleMaxX[triangle] = Math.max(nodeMaxX[a], Math.max(nodeMaxX[b], nodeMaxX[c])) + margin;
            triangleMaxY[triangle] = Math.max(nodeMaxY[a], Math.max(nodeMaxY[b], nodeMaxY[c])) + margin;
            triangleMaxZ[triangle] = Math.max(nodeMaxZ[a], Math.max(nodeMaxZ[b], nodeMaxZ[c])) + margin;
        }
        stageFinished = System.nanoTime();
        refitTriangleBoundsNanos = stageFinished - stageStarted;
        stageStarted = stageFinished;

        stretchedMeshletCount = 0;
        maxMeshletSpanRatio = 1.0;
        for (int meshlet = 0; meshlet < triangleMeshletCount(); meshlet++) {
            resetBounds(meshletMinX, meshletMinY, meshletMinZ,
                    meshletMaxX, meshletMaxY, meshletMaxZ, meshlet);
            double tightMinX = Double.POSITIVE_INFINITY, tightMinY = Double.POSITIVE_INFINITY;
            double tightMinZ = Double.POSITIVE_INFINITY;
            double tightMaxX = Double.NEGATIVE_INFINITY, tightMaxY = Double.NEGATIVE_INFINITY;
            double tightMaxZ = Double.NEGATIVE_INFINITY;
            for (int member = triangleMeshletStart[meshlet]; member < triangleMeshletStart[meshlet + 1]; member++) {
                int triangle = triangleMembers[member];
                if (triangleMinX[triangle] == Double.POSITIVE_INFINITY) continue;
                includeBounds(meshletMinX, meshletMinY, meshletMinZ,
                        meshletMaxX, meshletMaxY, meshletMaxZ, meshlet,
                        triangleMinX[triangle], triangleMinY[triangle], triangleMinZ[triangle],
                        triangleMaxX[triangle], triangleMaxY[triangle], triangleMaxZ[triangle]);
                int a = triangles.node1[triangle], b = triangles.node2[triangle], c = triangles.node3[triangle];
                tightMinX = Math.min(tightMinX, Math.min(nodes.posX[a], Math.min(nodes.posX[b], nodes.posX[c])));
                tightMinY = Math.min(tightMinY, Math.min(nodes.posY[a], Math.min(nodes.posY[b], nodes.posY[c])));
                tightMinZ = Math.min(tightMinZ, Math.min(nodes.posZ[a], Math.min(nodes.posZ[b], nodes.posZ[c])));
                tightMaxX = Math.max(tightMaxX, Math.max(nodes.posX[a], Math.max(nodes.posX[b], nodes.posX[c])));
                tightMaxY = Math.max(tightMaxY, Math.max(nodes.posY[a], Math.max(nodes.posY[b], nodes.posY[c])));
                tightMaxZ = Math.max(tightMaxZ, Math.max(nodes.posZ[a], Math.max(nodes.posZ[b], nodes.posZ[c])));
            }
            if (triangleMeshletEnd(meshlet) - triangleMeshletStart(meshlet) > 1
                    && tightMinX != Double.POSITIVE_INFINITY) {
                double ratio = aabbSpan(tightMinX, tightMinY, tightMinZ, tightMaxX, tightMaxY, tightMaxZ)
                        / meshletReferenceSpan[meshlet];
                maxMeshletSpanRatio = Math.max(maxMeshletSpanRatio, ratio);
                if (ratio > SPAN_WARNING_RATIO) stretchedMeshletCount++;
            }
        }
        refitMeshletBoundsNanos = System.nanoTime() - stageStarted;
    }

    int nodeChunkCount() { return nodeChunkStart.length - 1; }
    int triangleMeshletCount() { return triangleMeshletStart.length - 1; }
    int oversizedTriangleCount() { return oversizedTriangleCount; }
    int nodeChunkEnd(int chunk) { return nodeChunkStart[chunk + 1]; }
    boolean nodeChunkHasSelfCollision(int chunk) { return nodeChunkHasSelfCollision[chunk]; }
    int nodeChunkPartId(int chunk) { return nodeChunkPartId[chunk]; }
    int triangleMeshletStart(int meshlet) { return triangleMeshletStart[meshlet]; }
    int triangleMeshletEnd(int meshlet) { return triangleMeshletStart[meshlet + 1]; }
    int triangleAt(int member) { return triangleMembers[member]; }
    int triangleMeshletPartId(int meshlet) { return triangleMeshletPartId[meshlet]; }
    int regroupableNodeChunkCount() { return regroupableNodeChunkCount; }
    int regroupableMeshletCount() { return regroupableMeshletCount; }
    int stretchedNodeChunkCount() { return stretchedNodeChunkCount; }
    int stretchedMeshletCount() { return stretchedMeshletCount; }
    double maxNodeChunkSpanRatio() { return maxNodeChunkSpanRatio; }
    double maxMeshletSpanRatio() { return maxMeshletSpanRatio; }

    boolean sameKnownPart(int meshlet, CollisionChunkIndex nodes, int nodeChunk) {
        int partId = triangleMeshletPartId[meshlet];
        return partId >= 0 && partId == nodes.nodeChunkPartId[nodeChunk];
    }

    boolean chunksOverlap(int meshlet, CollisionChunkIndex nodes, int nodeChunk) {
        return overlaps(meshletMinX[meshlet], meshletMinY[meshlet], meshletMinZ[meshlet],
                meshletMaxX[meshlet], meshletMaxY[meshlet], meshletMaxZ[meshlet],
                nodes.nodeChunkMinX[nodeChunk], nodes.nodeChunkMinY[nodeChunk], nodes.nodeChunkMinZ[nodeChunk],
                nodes.nodeChunkMaxX[nodeChunk], nodes.nodeChunkMaxY[nodeChunk], nodes.nodeChunkMaxZ[nodeChunk]);
    }

    boolean meshletActive(int meshlet) {
        return meshletMinX[meshlet] != Double.POSITIVE_INFINITY;
    }

    /** First sorted node-chunk position whose prefix maximum can reach the meshlet. */
    int firstNodeChunkCandidate(int meshlet, CollisionChunkIndex nodes) {
        double targetMin = axisValue(meshletMinX[meshlet], meshletMinY[meshlet], meshletMinZ[meshlet],
                nodes.nodeChunkSweepAxis);
        return firstPrefixCandidate(nodes.sortedNodeChunkPrefixMax, 0, nodes.nodeChunkCount(), targetMin);
    }

    int sortedNodeChunkAt(CollisionChunkIndex nodes, int sortedPosition) {
        return (int) nodes.sortedNodeChunkKeys[sortedPosition];
    }

    boolean nodeChunkStartsAfterMeshlet(int meshlet, CollisionChunkIndex nodes, int sortedPosition) {
        double targetMax = axisValue(meshletMaxX[meshlet], meshletMaxY[meshlet], meshletMaxZ[meshlet],
                nodes.nodeChunkSweepAxis);
        return nodes.sortedNodeChunkKeys[sortedPosition] > sortableLimit(targetMax);
    }

    /** Axis whose meshlet query scans the fewest sorted node positions. */
    int localSweepAxis(int meshlet, int nodeChunk, CollisionChunkIndex nodes) {
        int bestAxis = 0;
        int bestSpan = Integer.MAX_VALUE;
        for (int axis = 0; axis < 3; axis++) {
            int start = nodes.axisOffset(axis) + nodes.nodeChunkStart[nodeChunk];
            int end = nodes.axisOffset(axis) + nodes.nodeChunkStart[nodeChunk + 1];
            double targetMin = axisValue(meshletMinX[meshlet], meshletMinY[meshlet], meshletMinZ[meshlet], axis);
            double targetMax = axisValue(meshletMaxX[meshlet], meshletMaxY[meshlet], meshletMaxZ[meshlet], axis);
            int first = firstPrefixCandidate(nodes.sortedNodePrefixMax, start, end, targetMin);
            int pastLast = firstKeyAfter(nodes.sortedNodeKeys, first, end, sortableLimit(targetMax));
            int span = pastLast - first;
            if (span < bestSpan) {
                bestSpan = span;
                bestAxis = axis;
            }
        }
        return bestAxis;
    }

    /** First sorted node position whose prefix maximum can reach the triangle. */
    int firstNodeCandidate(int triangle, int nodeChunk, CollisionChunkIndex nodes, int axis) {
        double targetMin = axisValue(triangleMinX[triangle], triangleMinY[triangle], triangleMinZ[triangle], axis);
        int axisOffset = nodes.axisOffset(axis);
        return firstPrefixCandidate(nodes.sortedNodePrefixMax,
                axisOffset + nodes.nodeChunkStart[nodeChunk],
                axisOffset + nodes.nodeChunkStart[nodeChunk + 1], targetMin);
    }

    int sortedNodeAt(CollisionChunkIndex nodes, int sortedPosition) {
        return (int) nodes.sortedNodeKeys[sortedPosition];
    }

    int sortedNodeEnd(int nodeChunk, CollisionChunkIndex nodes, int axis) {
        return nodes.axisOffset(axis) + nodes.nodeChunkStart[nodeChunk + 1];
    }

    boolean nodeStartsAfterTriangle(int triangle, CollisionChunkIndex nodes,
                                    int sortedPosition, int axis) {
        double targetMax = axisValue(triangleMaxX[triangle], triangleMaxY[triangle], triangleMaxZ[triangle], axis);
        return nodes.sortedNodeKeys[sortedPosition] > sortableLimit(targetMax);
    }

    boolean triangleOverlapsNode(int triangle, CollisionChunkIndex nodes, int node) {
        return overlaps(triangleMinX[triangle], triangleMinY[triangle], triangleMinZ[triangle],
                triangleMaxX[triangle], triangleMaxY[triangle], triangleMaxZ[triangle],
                nodes.nodeMinX[node], nodes.nodeMinY[node], nodes.nodeMinZ[node],
                nodes.nodeMaxX[node], nodes.nodeMaxY[node], nodes.nodeMaxZ[node]);
    }

    private Integer[] collectCollidableNodes() {
        Integer[] result = new Integer[(int) java.util.stream.IntStream.range(0, vehicle.nodes.count)
                .filter(node -> vehicle.nodes.collision[node]).count()];
        int write = 0;
        for (int node = 0; node < vehicle.nodes.count; node++) {
            if (vehicle.nodes.collision[node]) result[write++] = node;
        }
        return result;
    }

    private Integer[] collectCollidableTriangles() {
        Integer[] result = new Integer[(int) java.util.stream.IntStream.range(0, vehicle.triangles.count)
                .filter(triangle -> vehicle.triangles.collision[triangle]).count()];
        int write = 0;
        for (int triangle = 0; triangle < vehicle.triangles.count; triangle++) {
            if (vehicle.triangles.collision[triangle]) result[write++] = triangle;
        }
        return result;
    }

    private void splitNodes(Integer[] indices, int start, int end, List<int[]> leaves) {
        if (end <= start) return;
        if (end - start <= MAX_NODES_PER_CHUNK) {
            leaves.add(copy(indices, start, end));
            return;
        }
        int middle = applySahSplit(indices, start, end, false, MAX_NODES_PER_CHUNK);
        splitNodes(indices, start, middle, leaves);
        splitNodes(indices, middle, end, leaves);
    }

    /**
     * Keeps the spatial SAH partition as the outer grouping, then separates a
     * mixed leaf by part. Child bounds can therefore only shrink relative to
     * the original leaf, while detached parts cannot stretch one another's
     * chunk bounds.
     */
    private List<int[]> refineLeavesByPart(List<int[]> leaves, boolean triangles) {
        List<int[]> refined = new ArrayList<>(leaves.size());
        for (int[] leaf : leaves) {
            if (leaf.length <= 1) {
                refined.add(leaf);
                continue;
            }

            Map<Integer, List<Integer>> membersByPart = new TreeMap<>();
            for (int primitive : leaf) {
                int partId = triangles
                        ? vehicle.triangles.partId[primitive]
                        : vehicle.nodes.partId[primitive];
                membersByPart.computeIfAbsent(partId, ignored -> new ArrayList<>()).add(primitive);
            }
            if (membersByPart.size() == 1) {
                refined.add(leaf);
                continue;
            }

            for (List<Integer> partMembers : membersByPart.values()) {
                int[] child = new int[partMembers.size()];
                for (int member = 0; member < child.length; member++) {
                    child[member] = partMembers.get(member);
                }
                refined.add(child);
            }
        }
        return refined;
    }

    private int[] leafPartIds(List<int[]> leaves, boolean triangles) {
        int[] result = new int[leaves.size()];
        for (int leaf = 0; leaf < leaves.size(); leaf++) {
            int primitive = leaves.get(leaf)[0];
            result[leaf] = triangles
                    ? vehicle.triangles.partId[primitive]
                    : vehicle.nodes.partId[primitive];
        }
        return result;
    }

    private double[] buildNodeChunkReferenceSpans() {
        double[] spans = new double[nodeChunkCount()];
        NodeContainer nodes = vehicle.nodes;
        for (int chunk = 0; chunk < nodeChunkCount(); chunk++) {
            double minX = Double.POSITIVE_INFINITY, minY = Double.POSITIVE_INFINITY, minZ = Double.POSITIVE_INFINITY;
            double maxX = Double.NEGATIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY, maxZ = Double.NEGATIVE_INFINITY;
            for (int member = nodeChunkStart[chunk]; member < nodeChunkStart[chunk + 1]; member++) {
                int node = nodeMembers[member];
                minX = Math.min(minX, nodes.baseX[node]);
                minY = Math.min(minY, nodes.baseY[node]);
                minZ = Math.min(minZ, nodes.baseZ[node]);
                maxX = Math.max(maxX, nodes.baseX[node]);
                maxY = Math.max(maxY, nodes.baseY[node]);
                maxZ = Math.max(maxZ, nodes.baseZ[node]);
            }
            spans[chunk] = aabbSpan(minX, minY, minZ, maxX, maxY, maxZ);
        }
        return spans;
    }

    private double[] buildMeshletReferenceSpans() {
        double[] spans = new double[triangleMeshletCount()];
        NodeContainer nodes = vehicle.nodes;
        TriangleContainer triangles = vehicle.triangles;
        for (int meshlet = 0; meshlet < triangleMeshletCount(); meshlet++) {
            double minX = Double.POSITIVE_INFINITY, minY = Double.POSITIVE_INFINITY, minZ = Double.POSITIVE_INFINITY;
            double maxX = Double.NEGATIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY, maxZ = Double.NEGATIVE_INFINITY;
            for (int member = triangleMeshletStart[meshlet]; member < triangleMeshletStart[meshlet + 1]; member++) {
                int triangle = triangleMembers[member];
                int a = triangles.node1[triangle], b = triangles.node2[triangle], c = triangles.node3[triangle];
                minX = Math.min(minX, Math.min(nodes.baseX[a], Math.min(nodes.baseX[b], nodes.baseX[c])));
                minY = Math.min(minY, Math.min(nodes.baseY[a], Math.min(nodes.baseY[b], nodes.baseY[c])));
                minZ = Math.min(minZ, Math.min(nodes.baseZ[a], Math.min(nodes.baseZ[b], nodes.baseZ[c])));
                maxX = Math.max(maxX, Math.max(nodes.baseX[a], Math.max(nodes.baseX[b], nodes.baseX[c])));
                maxY = Math.max(maxY, Math.max(nodes.baseY[a], Math.max(nodes.baseY[b], nodes.baseY[c])));
                maxZ = Math.max(maxZ, Math.max(nodes.baseZ[a], Math.max(nodes.baseZ[b], nodes.baseZ[c])));
            }
            spans[meshlet] = aabbSpan(minX, minY, minZ, maxX, maxY, maxZ);
        }
        return spans;
    }

    private static double aabbSpan(double minX, double minY, double minZ,
                                   double maxX, double maxY, double maxZ) {
        double x = maxX - minX;
        double y = maxY - minY;
        double z = maxZ - minZ;
        return Math.max(SPAN_FLOOR, Math.sqrt(x * x + y * y + z * z));
    }

    private void rebuildNodeChunkSap() {
        int count = nodeChunkCount();
        if (count == 0) return;
        double minX = Double.POSITIVE_INFINITY, minY = Double.POSITIVE_INFINITY, minZ = Double.POSITIVE_INFINITY;
        double maxX = Double.NEGATIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY, maxZ = Double.NEGATIVE_INFINITY;
        for (int chunk = 0; chunk < count; chunk++) {
            minX = Math.min(minX, nodeChunkMinX[chunk]);
            minY = Math.min(minY, nodeChunkMinY[chunk]);
            minZ = Math.min(minZ, nodeChunkMinZ[chunk]);
            maxX = Math.max(maxX, nodeChunkMaxX[chunk]);
            maxY = Math.max(maxY, nodeChunkMaxY[chunk]);
            maxZ = Math.max(maxZ, nodeChunkMaxZ[chunk]);
        }
        nodeChunkSweepAxis = (byte) longestAxis(maxX - minX, maxY - minY, maxZ - minZ);
        for (int chunk = 0; chunk < count; chunk++) {
            sortedNodeChunkKeys[chunk] = sortKey(axisChunkMin(chunk, nodeChunkSweepAxis), chunk);
        }
        Arrays.sort(sortedNodeChunkKeys);
        double prefixMax = Double.NEGATIVE_INFINITY;
        for (int sorted = 0; sorted < count; sorted++) {
            int chunk = (int) sortedNodeChunkKeys[sorted];
            prefixMax = Math.max(prefixMax, axisChunkMax(chunk, nodeChunkSweepAxis));
            sortedNodeChunkPrefixMax[sorted] = prefixMax;
        }
    }

    private void rebuildNodeSaps() {
        long stageStarted = System.nanoTime();
        for (int axis = 0; axis < 3; axis++) {
            int axisOffset = axisOffset(axis);
            for (int chunk = 0; chunk < nodeChunkCount(); chunk++) {
                int start = axisOffset + nodeChunkStart[chunk];
                int end = axisOffset + nodeChunkStart[chunk + 1];
                for (int sorted = start; sorted < end; sorted++) {
                    int node = nodeSapsInitialized
                            ? (int) sortedNodeKeys[sorted]
                            : nodeMembers[sorted - axisOffset];
                    sortedNodeKeys[sorted] = sortKey(axisNodeMin(node, axis), node);
                }
            }
        }
        long stageFinished = System.nanoTime();
        refitLocalKeyFillNanos = stageFinished - stageStarted;
        stageStarted = stageFinished;

        for (int axis = 0; axis < 3; axis++) {
            int axisOffset = axisOffset(axis);
            for (int chunk = 0; chunk < nodeChunkCount(); chunk++) {
                int start = axisOffset + nodeChunkStart[chunk];
                int end = axisOffset + nodeChunkStart[chunk + 1];
                Arrays.sort(sortedNodeKeys, start, end);
            }
        }
        stageFinished = System.nanoTime();
        refitLocalSortNanos = stageFinished - stageStarted;
        stageStarted = stageFinished;

        for (int axis = 0; axis < 3; axis++) {
            int axisOffset = axisOffset(axis);
            for (int chunk = 0; chunk < nodeChunkCount(); chunk++) {
                int start = axisOffset + nodeChunkStart[chunk];
                int end = axisOffset + nodeChunkStart[chunk + 1];
                double prefixMax = Double.NEGATIVE_INFINITY;
                for (int sorted = start; sorted < end; sorted++) {
                    int node = (int) sortedNodeKeys[sorted];
                    prefixMax = Math.max(prefixMax, axisNodeMax(node, axis));
                    sortedNodePrefixMax[sorted] = prefixMax;
                }
            }
        }
        refitLocalPrefixNanos = System.nanoTime() - stageStarted;
        nodeSapsInitialized = true;
    }

    private void splitTriangles(Integer[] indices, int start, int end, List<int[]> leaves) {
        if (end <= start) return;
        if (end - start <= MAX_TRIANGLES_PER_MESHLET) {
            leaves.add(copy(indices, start, end));
            return;
        }
        int middle = applySahSplit(indices, start, end, true, MAX_TRIANGLES_PER_MESHLET);
        splitTriangles(indices, start, middle, leaves);
        splitTriangles(indices, middle, end, leaves);
    }

    /**
     * Applies a build-time surface-area split using complete primitive bounds.
     * Requiring each child to contain at least half a full leaf prevents SAH
     * from trading runtime overlap for a long tail of singleton chunks.
     */
    private int applySahSplit(Integer[] indices, int start, int end,
                              boolean triangles, int maxLeafSize) {
        int count = end - start;
        int minChildSize = Math.max(1, maxLeafSize >>> 1);
        Integer[] source = Arrays.copyOfRange(indices, start, end);
        Integer[] bestOrder = null;
        int bestLeftCount = -1;
        double bestCost = Double.POSITIVE_INFINITY;
        int bestImbalance = Integer.MAX_VALUE;

        double[] prefixArea = new double[count];
        double[] suffixArea = new double[count];
        for (int axis = 0; axis < 3; axis++) {
            final int sortAxis = axis;
            Integer[] order = source.clone();
            Arrays.sort(order, Comparator.comparingDouble(index -> triangles
                    ? triangleCentroidCoordinate(index, sortAxis)
                    : nodeCoordinate(index, sortAxis)));
            fillPrimitiveUnionAreas(order, prefixArea, triangles, false);
            fillPrimitiveUnionAreas(order, suffixArea, triangles, true);

            for (int leftCount = minChildSize; leftCount <= count - minChildSize; leftCount++) {
                double cost = prefixArea[leftCount - 1] * leftCount
                        + suffixArea[leftCount] * (count - leftCount);
                int imbalance = Math.abs(count - (leftCount << 1));
                if (cost < bestCost || (cost == bestCost && imbalance < bestImbalance)) {
                    bestCost = cost;
                    bestImbalance = imbalance;
                    bestLeftCount = leftCount;
                    bestOrder = order.clone();
                }
            }
        }

        if (bestOrder == null) {
            throw new IllegalStateException("No valid collision chunk SAH split for " + count + " primitives");
        }
        System.arraycopy(bestOrder, 0, indices, start, count);
        return start + bestLeftCount;
    }

    private void fillPrimitiveUnionAreas(Integer[] order, double[] areas,
                                         boolean triangles, boolean reverse) {
        double minX = Double.POSITIVE_INFINITY, minY = Double.POSITIVE_INFINITY, minZ = Double.POSITIVE_INFINITY;
        double maxX = Double.NEGATIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY, maxZ = Double.NEGATIVE_INFINITY;
        for (int step = 0; step < order.length; step++) {
            int position = reverse ? order.length - 1 - step : step;
            int primitive = order[position];
            minX = Math.min(minX, primitiveBaseMin(primitive, 0, triangles));
            minY = Math.min(minY, primitiveBaseMin(primitive, 1, triangles));
            minZ = Math.min(minZ, primitiveBaseMin(primitive, 2, triangles));
            maxX = Math.max(maxX, primitiveBaseMax(primitive, 0, triangles));
            maxY = Math.max(maxY, primitiveBaseMax(primitive, 1, triangles));
            maxZ = Math.max(maxZ, primitiveBaseMax(primitive, 2, triangles));
            double dx = maxX - minX, dy = maxY - minY, dz = maxZ - minZ;
            areas[position] = 2.0 * (dx * dy + dy * dz + dz * dx);
        }
    }

    private double primitiveBaseMin(int primitive, int axis, boolean triangles) {
        return triangles ? triangleBaseMin(primitive, axis) : nodeCoordinate(primitive, axis);
    }

    private double primitiveBaseMax(int primitive, int axis, boolean triangles) {
        return triangles ? triangleBaseMax(primitive, axis) : nodeCoordinate(primitive, axis);
    }

    /** Moves large spatial outliers to the front and emits singleton meshlets for them. */
    private int isolateOversizedTriangles(Integer[] indices, List<int[]> leaves) {
        if (indices.length == 0) return 0;
        double[] diagonals = new double[indices.length];
        double vehicleMinX = Double.POSITIVE_INFINITY, vehicleMinY = Double.POSITIVE_INFINITY, vehicleMinZ = Double.POSITIVE_INFINITY;
        double vehicleMaxX = Double.NEGATIVE_INFINITY, vehicleMaxY = Double.NEGATIVE_INFINITY, vehicleMaxZ = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < indices.length; i++) {
            int triangle = indices[i];
            double minX = triangleBaseMin(triangle, 0), minY = triangleBaseMin(triangle, 1), minZ = triangleBaseMin(triangle, 2);
            double maxX = triangleBaseMax(triangle, 0), maxY = triangleBaseMax(triangle, 1), maxZ = triangleBaseMax(triangle, 2);
            double dx = maxX - minX, dy = maxY - minY, dz = maxZ - minZ;
            diagonals[i] = Math.sqrt(dx * dx + dy * dy + dz * dz);
            vehicleMinX = Math.min(vehicleMinX, minX); vehicleMinY = Math.min(vehicleMinY, minY); vehicleMinZ = Math.min(vehicleMinZ, minZ);
            vehicleMaxX = Math.max(vehicleMaxX, maxX); vehicleMaxY = Math.max(vehicleMaxY, maxY); vehicleMaxZ = Math.max(vehicleMaxZ, maxZ);
        }
        double[] sorted = diagonals.clone();
        Arrays.sort(sorted);
        double median = sorted[sorted.length >>> 1];
        double vehicleDx = vehicleMaxX - vehicleMinX, vehicleDy = vehicleMaxY - vehicleMinY, vehicleDz = vehicleMaxZ - vehicleMinZ;
        double vehicleDiagonal = Math.sqrt(vehicleDx * vehicleDx + vehicleDy * vehicleDy + vehicleDz * vehicleDz);
        double threshold = Math.max(median * LARGE_TRIANGLE_MEDIAN_MULTIPLIER,
                vehicleDiagonal * LARGE_TRIANGLE_VEHICLE_DIAGONAL_FRACTION);

        int oversized = 0;
        for (int read = 0; read < indices.length; read++) {
            if (!(diagonals[read] > threshold)) continue;
            Integer triangle = indices[read];
            indices[read] = indices[oversized];
            indices[oversized] = triangle;
            double diagonal = diagonals[read];
            diagonals[read] = diagonals[oversized];
            diagonals[oversized] = diagonal;
            oversized++;
        }
        for (int i = 0; i < oversized; i++) leaves.add(new int[]{indices[i]});
        return oversized;
    }

    private static int longestAxis(double x, double y, double z) {
        if (x >= y && x >= z) return 0;
        return y >= z ? 1 : 2;
    }

    private double nodeCoordinate(int node, int axis) {
        return axis == 0 ? vehicle.nodes.baseX[node] : axis == 1 ? vehicle.nodes.baseY[node] : vehicle.nodes.baseZ[node];
    }

    private double triangleCentroidCoordinate(int triangle, int axis) {
        TriangleContainer triangles = vehicle.triangles;
        return (nodeCoordinate(triangles.node1[triangle], axis)
                + nodeCoordinate(triangles.node2[triangle], axis)
                + nodeCoordinate(triangles.node3[triangle], axis)) / 3.0;
    }

    private double triangleBaseMin(int triangle, int axis) {
        TriangleContainer triangles = vehicle.triangles;
        return Math.min(nodeCoordinate(triangles.node1[triangle], axis),
                Math.min(nodeCoordinate(triangles.node2[triangle], axis), nodeCoordinate(triangles.node3[triangle], axis)));
    }

    private double triangleBaseMax(int triangle, int axis) {
        TriangleContainer triangles = vehicle.triangles;
        return Math.max(nodeCoordinate(triangles.node1[triangle], axis),
                Math.max(nodeCoordinate(triangles.node2[triangle], axis), nodeCoordinate(triangles.node3[triangle], axis)));
    }

    private static int[] copy(Integer[] source, int start, int end) {
        int[] result = new int[end - start];
        for (int i = start; i < end; i++) result[i - start] = source[i];
        return result;
    }

    private static int[] flatten(List<int[]> leaves) {
        int total = leaves.stream().mapToInt(leaf -> leaf.length).sum();
        int[] result = new int[total];
        int write = 0;
        for (int[] leaf : leaves) {
            System.arraycopy(leaf, 0, result, write, leaf.length);
            write += leaf.length;
        }
        return result;
    }

    private static int[] starts(List<int[]> leaves) {
        int[] result = new int[leaves.size() + 1];
        for (int i = 0; i < leaves.size(); i++) result[i + 1] = result[i] + leaves.get(i).length;
        return result;
    }

    private static int countMultiMemberLeaves(List<int[]> leaves) {
        int count = 0;
        for (int[] leaf : leaves) {
            if (leaf.length > 1) count++;
        }
        return count;
    }

    private void includeNodeBounds(double[] minX, double[] minY, double[] minZ,
                                   double[] maxX, double[] maxY, double[] maxZ,
                                   int target, int node) {
        includeBounds(minX, minY, minZ, maxX, maxY, maxZ, target,
                nodeMinX[node], nodeMinY[node], nodeMinZ[node], nodeMaxX[node], nodeMaxY[node], nodeMaxZ[node]);
    }

    private static void resetBounds(double[] minX, double[] minY, double[] minZ,
                                    double[] maxX, double[] maxY, double[] maxZ, int target) {
        minX[target] = minY[target] = minZ[target] = Double.POSITIVE_INFINITY;
        maxX[target] = maxY[target] = maxZ[target] = Double.NEGATIVE_INFINITY;
    }

    private static void includeBounds(double[] minX, double[] minY, double[] minZ,
                                      double[] maxX, double[] maxY, double[] maxZ, int target,
                                      double sourceMinX, double sourceMinY, double sourceMinZ,
                                      double sourceMaxX, double sourceMaxY, double sourceMaxZ) {
        minX[target] = Math.min(minX[target], sourceMinX);
        minY[target] = Math.min(minY[target], sourceMinY);
        minZ[target] = Math.min(minZ[target], sourceMinZ);
        maxX[target] = Math.max(maxX[target], sourceMaxX);
        maxY[target] = Math.max(maxY[target], sourceMaxY);
        maxZ[target] = Math.max(maxZ[target], sourceMaxZ);
    }

    private static boolean overlaps(double aMinX, double aMinY, double aMinZ,
                                    double aMaxX, double aMaxY, double aMaxZ,
                                    double bMinX, double bMinY, double bMinZ,
                                    double bMaxX, double bMaxY, double bMaxZ) {
        return aMaxX >= bMinX && aMinX <= bMaxX
                && aMaxY >= bMinY && aMinY <= bMaxY
                && aMaxZ >= bMinZ && aMinZ <= bMaxZ;
    }

    private double axisChunkMin(int chunk, int axis) {
        return axisValue(nodeChunkMinX[chunk], nodeChunkMinY[chunk], nodeChunkMinZ[chunk], axis);
    }

    private double axisChunkMax(int chunk, int axis) {
        return axisValue(nodeChunkMaxX[chunk], nodeChunkMaxY[chunk], nodeChunkMaxZ[chunk], axis);
    }

    private double axisNodeMin(int node, int axis) {
        return axisValue(nodeMinX[node], nodeMinY[node], nodeMinZ[node], axis);
    }

    private double axisNodeMax(int node, int axis) {
        return axisValue(nodeMaxX[node], nodeMaxY[node], nodeMaxZ[node], axis);
    }

    private int axisOffset(int axis) {
        return axis * nodeMembers.length;
    }

    private static double axisValue(double x, double y, double z, int axis) {
        return axis == 0 ? x : axis == 1 ? y : z;
    }

    private static int firstPrefixCandidate(double[] prefixMaxima, int start, int end, double targetMin) {
        int left = start, right = end - 1, result = end;
        while (left <= right) {
            int middle = (left + right) >>> 1;
            if (prefixMaxima[middle] >= targetMin) {
                result = middle;
                right = middle - 1;
            } else {
                left = middle + 1;
            }
        }
        return result;
    }

    private static int firstKeyAfter(long[] keys, int start, int end, long limit) {
        int left = start, right = end - 1, result = end;
        while (left <= right) {
            int middle = (left + right) >>> 1;
            if (keys[middle] > limit) {
                result = middle;
                right = middle - 1;
            } else {
                left = middle + 1;
            }
        }
        return result;
    }

    private static long sortKey(double value, int original) {
        long sortable = Double.doubleToRawLongBits(value);
        if (sortable < 0) sortable = Long.MIN_VALUE - sortable;
        return (sortable & 0xFFFFFFFF00000000L) | (original & 0xFFFFFFFFL);
    }

    private static long sortableLimit(double value) {
        long sortable = Double.doubleToRawLongBits(value);
        if (sortable < 0) sortable = Long.MIN_VALUE - sortable;
        return (sortable & 0xFFFFFFFF00000000L) | 0xFFFFFFFFL;
    }
}
