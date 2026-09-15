package me.mzy.beamcraft.client.physics;

import java.util.Arrays;

/**
 * Allocation-free node sweep-and-prune grouped by vehicle part.
 *
 * <p>Nodes are sorted independently inside each part. Collidable triangle-part
 * bounds are collected at the same broad-phase rebuild and used to decide which
 * node-part segments a triangle may query. This keeps detached or distant parts
 * from widening one vehicle-wide search interval while preserving the legacy
 * per-node topology filters.</p>
 */
public class DynamicAxisSweep {
    static final int MAX_NODES = SoftBodyCollisionManager.MAX_GLOBAL_NODES;
    private static final int MAX_PARTS = MAX_NODES;

    private final long[] sortKeys = new long[MAX_NODES];
    private final double[] sortedPrefixMax = new double[MAX_NODES];

    private final double[] cacheMinX = new double[MAX_NODES];
    private final double[] cacheMinY = new double[MAX_NODES];
    private final double[] cacheMinZ = new double[MAX_NODES];
    private final double[] cacheMaxX = new double[MAX_NODES];
    private final double[] cacheMaxY = new double[MAX_NODES];
    private final double[] cacheMaxZ = new double[MAX_NODES];
    private final SoftBodyVehicle[] cacheVeh = new SoftBodyVehicle[MAX_NODES];
    private final int[] cacheNodeId = new int[MAX_NODES];
    private final int[] cachePartProxy = new int[MAX_NODES];

    private final SoftBodyVehicle[] partVehicle = new SoftBodyVehicle[MAX_PARTS];
    private final int[] partLocalId = new int[MAX_PARTS];
    private final int[] partNodeCount = new int[MAX_PARTS];
    private final int[] partStart = new int[MAX_PARTS + 1];
    private final int[] partWrite = new int[MAX_PARTS];
    private final byte[] partActiveAxis = new byte[MAX_PARTS];

    private final double[] nodeMinX = new double[MAX_PARTS];
    private final double[] nodeMinY = new double[MAX_PARTS];
    private final double[] nodeMinZ = new double[MAX_PARTS];
    private final double[] nodeMaxX = new double[MAX_PARTS];
    private final double[] nodeMaxY = new double[MAX_PARTS];
    private final double[] nodeMaxZ = new double[MAX_PARTS];
    private final double[] selfMinX = new double[MAX_PARTS];
    private final double[] selfMinY = new double[MAX_PARTS];
    private final double[] selfMinZ = new double[MAX_PARTS];
    private final double[] selfMaxX = new double[MAX_PARTS];
    private final double[] selfMaxY = new double[MAX_PARTS];
    private final double[] selfMaxZ = new double[MAX_PARTS];
    private final boolean[] hasSelfNodes = new boolean[MAX_PARTS];

    private final double[] triangleMinX = new double[MAX_PARTS];
    private final double[] triangleMinY = new double[MAX_PARTS];
    private final double[] triangleMinZ = new double[MAX_PARTS];
    private final double[] triangleMaxX = new double[MAX_PARTS];
    private final double[] triangleMaxY = new double[MAX_PARTS];
    private final double[] triangleMaxZ = new double[MAX_PARTS];
    private final boolean[] hasTriangles = new boolean[MAX_PARTS];

    private int count;
    private int partCount;

    public void clear() {
        count = 0;
        partCount = 0;
    }

    public void insertNodes(SoftBodyVehicle vehicle, double dtPredict) {
        double eX = vehicle.entityX, eY = vehicle.entityY, eZ = vehicle.entityZ;
        for (int node = 0; node < vehicle.nodes.count; node++) {
            if (!vehicle.nodes.collision[node]) continue;
            if (count >= MAX_NODES) {
                throw new IllegalStateException("Collidable node count exceeds SAP capacity " + MAX_NODES);
            }

            int part = findOrCreatePart(vehicle, vehicle.nodes.partId[node]);
            double x = eX + vehicle.nodes.posX[node];
            double y = eY + vehicle.nodes.posY[node];
            double z = eZ + vehicle.nodes.posZ[node];
            double futureX = x + vehicle.nodes.velX[node] * dtPredict;
            double futureY = y + vehicle.nodes.velY[node] * dtPredict;
            double futureZ = z + vehicle.nodes.velZ[node] * dtPredict;

            double minX = Math.min(x, futureX), minY = Math.min(y, futureY), minZ = Math.min(z, futureZ);
            double maxX = Math.max(x, futureX), maxY = Math.max(y, futureY), maxZ = Math.max(z, futureZ);
            cacheMinX[count] = minX;
            cacheMinY[count] = minY;
            cacheMinZ[count] = minZ;
            cacheMaxX[count] = maxX;
            cacheMaxY[count] = maxY;
            cacheMaxZ[count] = maxZ;
            cacheVeh[count] = vehicle;
            cacheNodeId[count] = node;
            cachePartProxy[count] = part;
            partNodeCount[part]++;
            includeNodeBounds(part, minX, minY, minZ, maxX, maxY, maxZ, false);
            if (vehicle.nodes.selfCollision[node]) {
                includeNodeBounds(part, minX, minY, minZ, maxX, maxY, maxZ, true);
                hasSelfNodes[part] = true;
            }
            count++;
        }
    }

    /** Collects swept bounds for collidable triangles, grouped by their authored part. */
    public void insertTriangles(SoftBodyVehicle vehicle, double dtPredict) {
        double eX = vehicle.entityX, eY = vehicle.entityY, eZ = vehicle.entityZ;
        for (int triangle = 0; triangle < vehicle.triangles.count; triangle++) {
            if (!vehicle.triangles.collision[triangle] || vehicle.triangles.broken[triangle]) continue;
            int localPart = vehicle.triangles.partId[triangle];
            if (localPart < 0) continue;
            int part = findOrCreatePart(vehicle, localPart);
            int a = vehicle.triangles.node1[triangle];
            int b = vehicle.triangles.node2[triangle];
            int c = vehicle.triangles.node3[triangle];

            double ax = eX + vehicle.nodes.posX[a], ay = eY + vehicle.nodes.posY[a], az = eZ + vehicle.nodes.posZ[a];
            double bx = eX + vehicle.nodes.posX[b], by = eY + vehicle.nodes.posY[b], bz = eZ + vehicle.nodes.posZ[b];
            double cx = eX + vehicle.nodes.posX[c], cy = eY + vehicle.nodes.posY[c], cz = eZ + vehicle.nodes.posZ[c];
            double fax = ax + vehicle.nodes.velX[a] * dtPredict;
            double fay = ay + vehicle.nodes.velY[a] * dtPredict;
            double faz = az + vehicle.nodes.velZ[a] * dtPredict;
            double fbx = bx + vehicle.nodes.velX[b] * dtPredict;
            double fby = by + vehicle.nodes.velY[b] * dtPredict;
            double fbz = bz + vehicle.nodes.velZ[b] * dtPredict;
            double fcx = cx + vehicle.nodes.velX[c] * dtPredict;
            double fcy = cy + vehicle.nodes.velY[c] * dtPredict;
            double fcz = cz + vehicle.nodes.velZ[c] * dtPredict;

            double minX = min6(ax, bx, cx, fax, fbx, fcx) - CollisionPipeline.SOFT_BROADPHASE_MARGIN;
            double minY = min6(ay, by, cy, fay, fby, fcy) - CollisionPipeline.SOFT_BROADPHASE_MARGIN;
            double minZ = min6(az, bz, cz, faz, fbz, fcz) - CollisionPipeline.SOFT_BROADPHASE_MARGIN;
            double maxX = max6(ax, bx, cx, fax, fbx, fcx) + CollisionPipeline.SOFT_BROADPHASE_MARGIN;
            double maxY = max6(ay, by, cy, fay, fby, fcy) + CollisionPipeline.SOFT_BROADPHASE_MARGIN;
            double maxZ = max6(az, bz, cz, faz, fbz, fcz) + CollisionPipeline.SOFT_BROADPHASE_MARGIN;
            includeTriangleBounds(part, minX, minY, minZ, maxX, maxY, maxZ);
            hasTriangles[part] = true;
        }
    }

    public void updateAndSort() {
        if (count == 0) return;

        int offset = 0;
        for (int part = 0; part < partCount; part++) {
            partStart[part] = offset;
            partWrite[part] = offset;
            offset += partNodeCount[part];
            partActiveAxis[part] = chooseAxis(part);
        }
        partStart[partCount] = offset;

        for (int index = 0; index < count; index++) {
            int part = cachePartProxy[index];
            sortKeys[partWrite[part]++] = sortKey(axisMin(index, partActiveAxis[part]), index);
        }

        for (int part = 0; part < partCount; part++) {
            int start = partStart[part], end = partStart[part + 1];
            if (end <= start) continue;
            Arrays.sort(sortKeys, start, end);
            double prefixMax = Double.NEGATIVE_INFINITY;
            for (int sorted = start; sorted < end; sorted++) {
                int original = (int) sortKeys[sorted];
                double maximum = axisMax(original, partActiveAxis[part]);
                if (maximum > prefixMax) prefixMax = maximum;
                sortedPrefixMax[sorted] = prefixMax;
            }
        }
    }

    public int queryCollisionNodesInAABB(
            double minX, double minY, double minZ,
            double maxX, double maxY, double maxZ,
            SoftBodyVehicle triangleVehicle,
            int triangleNodeA, int triangleNodeB, int triangleNodeC,
            int trianglePartId,
            SweepResultBuffer result) {
        if (count == 0) return 0;

        int trianglePart = findPart(triangleVehicle, trianglePartId);
        boolean usePartPairs = trianglePart >= 0 && hasTriangles[trianglePart];
        int rawHits = 0;
        for (int nodePart = 0; nodePart < partCount; nodePart++) {
            if (partNodeCount[nodePart] == 0) continue;
            if (usePartPairs && !partPairMayOverlap(trianglePart, nodePart)) continue;
            rawHits += queryPart(nodePart,
                    minX, minY, minZ, maxX, maxY, maxZ,
                    triangleVehicle, triangleNodeA, triangleNodeB, triangleNodeC,
                    trianglePartId, result);
        }
        return rawHits;
    }

    private int queryPart(int part,
                          double minX, double minY, double minZ,
                          double maxX, double maxY, double maxZ,
                          SoftBodyVehicle triangleVehicle,
                          int triangleNodeA, int triangleNodeB, int triangleNodeC,
                          int trianglePartId,
                          SweepResultBuffer result) {
        int start = partStart[part], end = partStart[part + 1];
        int axis = partActiveAxis[part];
        double targetMin = axis == 0 ? minX : axis == 1 ? minY : minZ;
        double targetMax = axis == 0 ? maxX : axis == 1 ? maxY : maxZ;

        int left = start, right = end - 1, startIndex = end;
        while (left <= right) {
            int middle = (left + right) >>> 1;
            if (sortedPrefixMax[middle] >= targetMin) {
                startIndex = middle;
                right = middle - 1;
            } else {
                left = middle + 1;
            }
        }

        long maxKeyLimit = sortableLimit(targetMax);
        int rawHits = 0;
        for (int sorted = startIndex; sorted < end; sorted++) {
            long key = sortKeys[sorted];
            if (key > maxKeyLimit) break;
            int original = (int) key;
            if (cacheMaxX[original] < minX || cacheMinX[original] > maxX
                    || cacheMaxY[original] < minY || cacheMinY[original] > maxY
                    || cacheMaxZ[original] < minZ || cacheMinZ[original] > maxZ) continue;

            rawHits++;
            SoftBodyVehicle hitVehicle = cacheVeh[original];
            int hitNode = cacheNodeId[original];
            if (hitVehicle == triangleVehicle) {
                if (!triangleVehicle.nodes.selfCollision[hitNode]) continue;
                if (hitNode == triangleNodeA || hitNode == triangleNodeB || hitNode == triangleNodeC) continue;
                if (trianglePartId >= 0 && trianglePartId < triangleVehicle.matrixPartStride
                        && triangleVehicle.nodeInPartMatrix != null
                        && triangleVehicle.nodeInPartMatrix[
                        hitNode * triangleVehicle.matrixPartStride + trianglePartId]) continue;
            }
            result.add(hitVehicle, hitNode);
        }
        return rawHits;
    }

    private boolean partPairMayOverlap(int trianglePart, int nodePart) {
        boolean sameVehicle = partVehicle[trianglePart] == partVehicle[nodePart];
        if (sameVehicle && !hasSelfNodes[nodePart]) return false;
        if (sameVehicle && sameAuthoredPartIsExcluded(nodePart, trianglePart)) return false;
        return sameVehicle
                ? overlaps(selfMinX[nodePart], selfMinY[nodePart], selfMinZ[nodePart],
                selfMaxX[nodePart], selfMaxY[nodePart], selfMaxZ[nodePart], trianglePart)
                : overlaps(nodeMinX[nodePart], nodeMinY[nodePart], nodeMinZ[nodePart],
                nodeMaxX[nodePart], nodeMaxY[nodePart], nodeMaxZ[nodePart], trianglePart);
    }

    private boolean sameAuthoredPartIsExcluded(int nodePart, int trianglePart) {
        if (partLocalId[nodePart] != partLocalId[trianglePart]) return false;
        SoftBodyVehicle vehicle = partVehicle[nodePart];
        int localPart = partLocalId[nodePart];
        return vehicle.nodeInPartMatrix != null
                && localPart >= 0 && localPart < vehicle.matrixPartStride;
    }

    private boolean overlaps(double minX, double minY, double minZ,
                             double maxX, double maxY, double maxZ, int trianglePart) {
        return maxX >= triangleMinX[trianglePart] && minX <= triangleMaxX[trianglePart]
                && maxY >= triangleMinY[trianglePart] && minY <= triangleMaxY[trianglePart]
                && maxZ >= triangleMinZ[trianglePart] && minZ <= triangleMaxZ[trianglePart];
    }

    private int findOrCreatePart(SoftBodyVehicle vehicle, int localPart) {
        for (int part = 0; part < partCount; part++) {
            if (partVehicle[part] == vehicle && partLocalId[part] == localPart) return part;
        }
        if (partCount >= MAX_PARTS) {
            throw new IllegalStateException("Collision part count exceeds SAP capacity " + MAX_PARTS);
        }
        int part = partCount++;
        partVehicle[part] = vehicle;
        partLocalId[part] = localPart;
        partNodeCount[part] = 0;
        hasSelfNodes[part] = false;
        hasTriangles[part] = false;
        nodeMinX[part] = nodeMinY[part] = nodeMinZ[part] = Double.POSITIVE_INFINITY;
        nodeMaxX[part] = nodeMaxY[part] = nodeMaxZ[part] = Double.NEGATIVE_INFINITY;
        selfMinX[part] = selfMinY[part] = selfMinZ[part] = Double.POSITIVE_INFINITY;
        selfMaxX[part] = selfMaxY[part] = selfMaxZ[part] = Double.NEGATIVE_INFINITY;
        triangleMinX[part] = triangleMinY[part] = triangleMinZ[part] = Double.POSITIVE_INFINITY;
        triangleMaxX[part] = triangleMaxY[part] = triangleMaxZ[part] = Double.NEGATIVE_INFINITY;
        return part;
    }

    private int findPart(SoftBodyVehicle vehicle, int localPart) {
        if (localPart < 0) return -1;
        for (int part = 0; part < partCount; part++) {
            if (partVehicle[part] == vehicle && partLocalId[part] == localPart) return part;
        }
        return -1;
    }

    private void includeNodeBounds(int part,
                                   double minX, double minY, double minZ,
                                   double maxX, double maxY, double maxZ,
                                   boolean self) {
        double[] minsX = self ? selfMinX : nodeMinX;
        double[] minsY = self ? selfMinY : nodeMinY;
        double[] minsZ = self ? selfMinZ : nodeMinZ;
        double[] maxsX = self ? selfMaxX : nodeMaxX;
        double[] maxsY = self ? selfMaxY : nodeMaxY;
        double[] maxsZ = self ? selfMaxZ : nodeMaxZ;
        if (minX < minsX[part]) minsX[part] = minX;
        if (minY < minsY[part]) minsY[part] = minY;
        if (minZ < minsZ[part]) minsZ[part] = minZ;
        if (maxX > maxsX[part]) maxsX[part] = maxX;
        if (maxY > maxsY[part]) maxsY[part] = maxY;
        if (maxZ > maxsZ[part]) maxsZ[part] = maxZ;
    }

    private void includeTriangleBounds(int part,
                                       double minX, double minY, double minZ,
                                       double maxX, double maxY, double maxZ) {
        if (minX < triangleMinX[part]) triangleMinX[part] = minX;
        if (minY < triangleMinY[part]) triangleMinY[part] = minY;
        if (minZ < triangleMinZ[part]) triangleMinZ[part] = minZ;
        if (maxX > triangleMaxX[part]) triangleMaxX[part] = maxX;
        if (maxY > triangleMaxY[part]) triangleMaxY[part] = maxY;
        if (maxZ > triangleMaxZ[part]) triangleMaxZ[part] = maxZ;
    }

    private byte chooseAxis(int part) {
        double spanX = nodeMaxX[part] - nodeMinX[part];
        double spanY = nodeMaxY[part] - nodeMinY[part];
        double spanZ = nodeMaxZ[part] - nodeMinZ[part];
        if (spanX >= spanY && spanX >= spanZ) return 0;
        if (spanY >= spanX && spanY >= spanZ) return 1;
        return 2;
    }

    private double axisMin(int index, int axis) {
        return axis == 0 ? cacheMinX[index] : axis == 1 ? cacheMinY[index] : cacheMinZ[index];
    }

    private double axisMax(int index, int axis) {
        return axis == 0 ? cacheMaxX[index] : axis == 1 ? cacheMaxY[index] : cacheMaxZ[index];
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

    private static double min6(double a, double b, double c, double d, double e, double f) {
        return Math.min(Math.min(Math.min(a, b), Math.min(c, d)), Math.min(e, f));
    }

    private static double max6(double a, double b, double c, double d, double e, double f) {
        return Math.max(Math.max(Math.max(a, b), Math.max(c, d)), Math.max(e, f));
    }
}
