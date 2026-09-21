package me.mzy.beamcraft.client.physics;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

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
    private static final double LARGE_TRIANGLE_MEDIAN_MULTIPLIER = 4.0;
    private static final double LARGE_TRIANGLE_VEHICLE_DIAGONAL_FRACTION = 0.5;

    private final SoftBodyVehicle vehicle;

    private int builtNodeCount = -1;
    private int builtTriangleCount = -1;

    private int[] nodeMembers = new int[0];
    private int[] nodeChunkStart = new int[]{0};
    private boolean[] nodeChunkHasSelfCollision = new boolean[0];
    private int[] triangleMembers = new int[0];
    private int[] triangleMeshletStart = new int[]{0};
    private int oversizedTriangleCount;

    private double[] nodeMinX = new double[0], nodeMinY = new double[0], nodeMinZ = new double[0];
    private double[] nodeMaxX = new double[0], nodeMaxY = new double[0], nodeMaxZ = new double[0];
    private double[] triangleMinX = new double[0], triangleMinY = new double[0], triangleMinZ = new double[0];
    private double[] triangleMaxX = new double[0], triangleMaxY = new double[0], triangleMaxZ = new double[0];

    private double[] nodeChunkMinX = new double[0], nodeChunkMinY = new double[0], nodeChunkMinZ = new double[0];
    private double[] nodeChunkMaxX = new double[0], nodeChunkMaxY = new double[0], nodeChunkMaxZ = new double[0];
    private double[] meshletMinX = new double[0], meshletMinY = new double[0], meshletMinZ = new double[0];
    private double[] meshletMaxX = new double[0], meshletMaxY = new double[0], meshletMaxZ = new double[0];

    // Rebuilt with the swept bounds. The first SAP prunes node chunks for a
    // meshlet. Each at-most-16-node chunk is sorted independently on all three
    // axes so an overlapping meshlet/chunk pair can use its narrowest scan.
    private long[] sortedNodeChunkKeys = new long[0];
    private double[] sortedNodeChunkPrefixMax = new double[0];
    private byte nodeChunkSweepAxis;
    private long[] sortedNodeKeys = new long[0];
    private double[] sortedNodePrefixMax = new double[0];

    CollisionChunkIndex(SoftBodyVehicle vehicle) {
        this.vehicle = vehicle;
    }

    void rebuild() {
        builtNodeCount = vehicle.nodes.count;
        builtTriangleCount = vehicle.triangles.count;

        Integer[] collidableNodes = collectCollidableNodes();
        List<int[]> nodeLeaves = new ArrayList<>();
        splitNodes(collidableNodes, 0, collidableNodes.length, nodeLeaves);
        nodeMembers = flatten(nodeLeaves);
        nodeChunkStart = starts(nodeLeaves);
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
        triangleMembers = flatten(meshlets);
        triangleMeshletStart = starts(meshlets);

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
        int meshletCount = meshlets.size();
        meshletMinX = new double[meshletCount]; meshletMinY = new double[meshletCount]; meshletMinZ = new double[meshletCount];
        meshletMaxX = new double[meshletCount]; meshletMaxY = new double[meshletCount]; meshletMaxZ = new double[meshletCount];
    }

    void refit(double dtPredict) {
        if (builtNodeCount != vehicle.nodes.count || builtTriangleCount != vehicle.triangles.count) rebuild();

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

        for (int chunk = 0; chunk < nodeChunkCount(); chunk++) {
            resetBounds(nodeChunkMinX, nodeChunkMinY, nodeChunkMinZ,
                    nodeChunkMaxX, nodeChunkMaxY, nodeChunkMaxZ, chunk);
            for (int member = nodeChunkStart[chunk]; member < nodeChunkStart[chunk + 1]; member++) {
                int node = nodeMembers[member];
                includeNodeBounds(nodeChunkMinX, nodeChunkMinY, nodeChunkMinZ,
                        nodeChunkMaxX, nodeChunkMaxY, nodeChunkMaxZ, chunk, node);
            }
        }
        rebuildNodeChunkSap();
        rebuildNodeSaps();

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

        for (int meshlet = 0; meshlet < triangleMeshletCount(); meshlet++) {
            resetBounds(meshletMinX, meshletMinY, meshletMinZ,
                    meshletMaxX, meshletMaxY, meshletMaxZ, meshlet);
            for (int member = triangleMeshletStart[meshlet]; member < triangleMeshletStart[meshlet + 1]; member++) {
                int triangle = triangleMembers[member];
                if (triangleMinX[triangle] == Double.POSITIVE_INFINITY) continue;
                includeBounds(meshletMinX, meshletMinY, meshletMinZ,
                        meshletMaxX, meshletMaxY, meshletMaxZ, meshlet,
                        triangleMinX[triangle], triangleMinY[triangle], triangleMinZ[triangle],
                        triangleMaxX[triangle], triangleMaxY[triangle], triangleMaxZ[triangle]);
            }
        }
    }

    int nodeChunkCount() { return nodeChunkStart.length - 1; }
    int triangleMeshletCount() { return triangleMeshletStart.length - 1; }
    int oversizedTriangleCount() { return oversizedTriangleCount; }
    int nodeChunkEnd(int chunk) { return nodeChunkStart[chunk + 1]; }
    boolean nodeChunkHasSelfCollision(int chunk) { return nodeChunkHasSelfCollision[chunk]; }
    int triangleMeshletStart(int meshlet) { return triangleMeshletStart[meshlet]; }
    int triangleMeshletEnd(int meshlet) { return triangleMeshletStart[meshlet + 1]; }
    int triangleAt(int member) { return triangleMembers[member]; }

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
        int axis = longestNodeAxis(indices, start, end);
        Arrays.sort(indices, start, end, Comparator.comparingDouble(node -> nodeCoordinate(node, axis)));
        int middle = (start + end) >>> 1;
        splitNodes(indices, start, middle, leaves);
        splitNodes(indices, middle, end, leaves);
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
        for (int axis = 0; axis < 3; axis++) {
            int axisOffset = axisOffset(axis);
            for (int chunk = 0; chunk < nodeChunkCount(); chunk++) {
                int start = axisOffset + nodeChunkStart[chunk];
                int end = axisOffset + nodeChunkStart[chunk + 1];
                for (int member = nodeChunkStart[chunk]; member < nodeChunkStart[chunk + 1]; member++) {
                    int node = nodeMembers[member];
                    sortedNodeKeys[axisOffset + member] = sortKey(axisNodeMin(node, axis), node);
                }
                Arrays.sort(sortedNodeKeys, start, end);
                double prefixMax = Double.NEGATIVE_INFINITY;
                for (int sorted = start; sorted < end; sorted++) {
                    int node = (int) sortedNodeKeys[sorted];
                    prefixMax = Math.max(prefixMax, axisNodeMax(node, axis));
                    sortedNodePrefixMax[sorted] = prefixMax;
                }
            }
        }
    }

    private void splitTriangles(Integer[] indices, int start, int end, List<int[]> leaves) {
        if (end <= start) return;
        if (end - start <= MAX_TRIANGLES_PER_MESHLET) {
            leaves.add(copy(indices, start, end));
            return;
        }
        int axis = longestTriangleCentroidAxis(indices, start, end);
        Arrays.sort(indices, start, end, Comparator.comparingDouble(triangle -> triangleCentroidCoordinate(triangle, axis)));
        int middle = (start + end) >>> 1;
        splitTriangles(indices, start, middle, leaves);
        splitTriangles(indices, middle, end, leaves);
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

    private int longestNodeAxis(Integer[] indices, int start, int end) {
        double[] range = ranges(indices, start, end, false);
        return longestAxis(range);
    }

    private int longestTriangleCentroidAxis(Integer[] indices, int start, int end) {
        double[] range = ranges(indices, start, end, true);
        return longestAxis(range);
    }

    private double[] ranges(Integer[] indices, int start, int end, boolean triangles) {
        double minX = Double.POSITIVE_INFINITY, minY = Double.POSITIVE_INFINITY, minZ = Double.POSITIVE_INFINITY;
        double maxX = Double.NEGATIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY, maxZ = Double.NEGATIVE_INFINITY;
        for (int i = start; i < end; i++) {
            double x = triangles ? triangleCentroidCoordinate(indices[i], 0) : nodeCoordinate(indices[i], 0);
            double y = triangles ? triangleCentroidCoordinate(indices[i], 1) : nodeCoordinate(indices[i], 1);
            double z = triangles ? triangleCentroidCoordinate(indices[i], 2) : nodeCoordinate(indices[i], 2);
            minX = Math.min(minX, x); minY = Math.min(minY, y); minZ = Math.min(minZ, z);
            maxX = Math.max(maxX, x); maxY = Math.max(maxY, y); maxZ = Math.max(maxZ, z);
        }
        return new double[]{maxX - minX, maxY - minY, maxZ - minZ};
    }

    private static int longestAxis(double[] range) {
        return longestAxis(range[0], range[1], range[2]);
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
