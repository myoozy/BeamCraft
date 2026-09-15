package me.mzy.beamcraft.client.physics;

import java.util.Arrays;

/**
 * Dynamic 1D sweep-and-prune over all collidable nodes.
 * The widest axis is selected on every rebuild to avoid a persistently crowded axis.
 */
public class DynamicAxisSweep {
    static final int MAX_NODES = SoftBodyCollisionManager.MAX_GLOBAL_NODES;

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

    private int count;
    private int activeAxis;

    public void clear() {
        count = 0;
    }

    public void insertNodes(SoftBodyVehicle vehicle, double dtPredict) {
        double eX = vehicle.entityX, eY = vehicle.entityY, eZ = vehicle.entityZ;
        for (int node = 0; node < vehicle.nodes.count; node++) {
            if (!vehicle.nodes.collision[node]) continue;
            if (count >= MAX_NODES) {
                throw new IllegalStateException("Collidable node count exceeds SAP capacity " + MAX_NODES);
            }

            double x = eX + vehicle.nodes.posX[node];
            double y = eY + vehicle.nodes.posY[node];
            double z = eZ + vehicle.nodes.posZ[node];
            double futureX = x + vehicle.nodes.velX[node] * dtPredict;
            double futureY = y + vehicle.nodes.velY[node] * dtPredict;
            double futureZ = z + vehicle.nodes.velZ[node] * dtPredict;
            cacheMinX[count] = Math.min(x, futureX);
            cacheMinY[count] = Math.min(y, futureY);
            cacheMinZ[count] = Math.min(z, futureZ);
            cacheMaxX[count] = Math.max(x, futureX);
            cacheMaxY[count] = Math.max(y, futureY);
            cacheMaxZ[count] = Math.max(z, futureZ);
            cacheVeh[count] = vehicle;
            cacheNodeId[count] = node;
            count++;
        }
    }

    public void updateAndSort() {
        if (count == 0) return;

        double minX = cacheMinX[0], maxX = cacheMaxX[0];
        double minY = cacheMinY[0], maxY = cacheMaxY[0];
        double minZ = cacheMinZ[0], maxZ = cacheMaxZ[0];
        for (int i = 1; i < count; i++) {
            if (cacheMinX[i] < minX) minX = cacheMinX[i];
            if (cacheMaxX[i] > maxX) maxX = cacheMaxX[i];
            if (cacheMinY[i] < minY) minY = cacheMinY[i];
            if (cacheMaxY[i] > maxY) maxY = cacheMaxY[i];
            if (cacheMinZ[i] < minZ) minZ = cacheMinZ[i];
            if (cacheMaxZ[i] > maxZ) maxZ = cacheMaxZ[i];
        }

        double spanX = maxX - minX;
        double spanY = maxY - minY;
        double spanZ = maxZ - minZ;
        if (spanX >= spanY && spanX >= spanZ) activeAxis = 0;
        else if (spanY >= spanX && spanY >= spanZ) activeAxis = 1;
        else activeAxis = 2;

        for (int i = 0; i < count; i++) {
            sortKeys[i] = sortKey(axisMin(i), i);
        }
        Arrays.sort(sortKeys, 0, count);

        double prefixMax = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < count; i++) {
            int original = (int) sortKeys[i];
            double maximum = axisMax(original);
            if (maximum > prefixMax) prefixMax = maximum;
            sortedPrefixMax[i] = prefixMax;
        }
    }

    /**
     * Queries one triangle's node candidates and applies topology-only filters.
     * The return value is the raw 3D AABB hit count used by diagnostics.
     */
    public int queryCollisionNodesInAABB(
            double minX, double minY, double minZ,
            double maxX, double maxY, double maxZ,
            SoftBodyVehicle triangleVehicle,
            int triangleNodeA, int triangleNodeB, int triangleNodeC,
            int trianglePartId,
            SweepResultBuffer result) {
        if (count == 0) return 0;

        double targetMin = activeAxis == 0 ? minX : activeAxis == 1 ? minY : minZ;
        double targetMax = activeAxis == 0 ? maxX : activeAxis == 1 ? maxY : maxZ;
        int left = 0, right = count - 1, startIndex = count;
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
        for (int sorted = startIndex; sorted < count; sorted++) {
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

    private double axisMin(int index) {
        return activeAxis == 0 ? cacheMinX[index] : activeAxis == 1 ? cacheMinY[index] : cacheMinZ[index];
    }

    private double axisMax(int index) {
        return activeAxis == 0 ? cacheMaxX[index] : activeAxis == 1 ? cacheMaxY[index] : cacheMaxZ[index];
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
