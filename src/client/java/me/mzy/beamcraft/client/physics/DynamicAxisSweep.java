package me.mzy.beamcraft.client.physics;

import java.util.Arrays;

/**
 * Allocation-free swept node SAP with one sorted segment per vehicle.
 *
 * <p>Part bounds gate each vehicle segment before its binary search. This keeps
 * detached parts from turning a vehicle-wide bound into false candidates, while
 * avoiding the many binary searches caused by sorting every authored part as a
 * separate segment.</p>
 */
public class DynamicAxisSweep {
    static final int MAX_NODES = SoftBodyCollisionManager.MAX_GLOBAL_NODES;
    private static final int MAX_GROUPS = MAX_NODES;

    private final long[] sortKeys = new long[MAX_NODES];
    private final double[] sortedPrefixMax = new double[MAX_NODES];
    private final long[] selfSortKeys = new long[MAX_NODES];
    private final double[] selfSortedPrefixMax = new double[MAX_NODES];
    private final double[] cacheMinX = new double[MAX_NODES];
    private final double[] cacheMinY = new double[MAX_NODES];
    private final double[] cacheMinZ = new double[MAX_NODES];
    private final double[] cacheMaxX = new double[MAX_NODES];
    private final double[] cacheMaxY = new double[MAX_NODES];
    private final double[] cacheMaxZ = new double[MAX_NODES];
    private final SoftBodyVehicle[] cacheVeh = new SoftBodyVehicle[MAX_NODES];
    private final int[] cacheNodeId = new int[MAX_NODES];
    private final int[] cacheVehicleProxy = new int[MAX_NODES];
    private final boolean[] cacheSelfCollision = new boolean[MAX_NODES];

    private final SoftBodyVehicle[] vehicleRef = new SoftBodyVehicle[MAX_GROUPS];
    private final int[] vehicleNodeCount = new int[MAX_GROUPS];
    private final int[] vehicleStart = new int[MAX_GROUPS + 1];
    private final int[] vehicleWrite = new int[MAX_GROUPS];
    private final int[] vehicleSelfNodeCount = new int[MAX_GROUPS];
    private final int[] vehicleSelfStart = new int[MAX_GROUPS + 1];
    private final int[] vehicleSelfWrite = new int[MAX_GROUPS];
    private final int[] vehiclePartStart = new int[MAX_GROUPS];
    private final int[] vehiclePartEnd = new int[MAX_GROUPS];
    private final byte[] vehicleActiveAxis = new byte[MAX_GROUPS];
    private final boolean[] vehicleHasSelfNodes = new boolean[MAX_GROUPS];
    private final double[] vehicleMinX = new double[MAX_GROUPS];
    private final double[] vehicleMinY = new double[MAX_GROUPS];
    private final double[] vehicleMinZ = new double[MAX_GROUPS];
    private final double[] vehicleMaxX = new double[MAX_GROUPS];
    private final double[] vehicleMaxY = new double[MAX_GROUPS];
    private final double[] vehicleMaxZ = new double[MAX_GROUPS];

    private final int[] partVehicleProxy = new int[MAX_GROUPS];
    private final int[] partLocalId = new int[MAX_GROUPS];
    private final double[] partMinX = new double[MAX_GROUPS];
    private final double[] partMinY = new double[MAX_GROUPS];
    private final double[] partMinZ = new double[MAX_GROUPS];
    private final double[] partMaxX = new double[MAX_GROUPS];
    private final double[] partMaxY = new double[MAX_GROUPS];
    private final double[] partMaxZ = new double[MAX_GROUPS];
    private final double[] partSelfMinX = new double[MAX_GROUPS];
    private final double[] partSelfMinY = new double[MAX_GROUPS];
    private final double[] partSelfMinZ = new double[MAX_GROUPS];
    private final double[] partSelfMaxX = new double[MAX_GROUPS];
    private final double[] partSelfMaxY = new double[MAX_GROUPS];
    private final double[] partSelfMaxZ = new double[MAX_GROUPS];
    private final boolean[] partHasSelfNodes = new boolean[MAX_GROUPS];

    private int count;
    private int vehicleCount;
    private int partCount;

    public void clear() {
        count = 0;
        vehicleCount = 0;
        partCount = 0;
    }

    public void insertNodes(SoftBodyVehicle vehicle, double dtPredict) {
        if (vehicleCount >= MAX_GROUPS) {
            throw new IllegalStateException("Vehicle count exceeds SAP capacity " + MAX_GROUPS);
        }
        int vehicleProxy = vehicleCount++;
        vehicleRef[vehicleProxy] = vehicle;
        vehicleNodeCount[vehicleProxy] = 0;
        vehicleSelfNodeCount[vehicleProxy] = 0;
        vehiclePartStart[vehicleProxy] = partCount;
        vehicleHasSelfNodes[vehicleProxy] = false;
        vehicleMinX[vehicleProxy] = vehicleMinY[vehicleProxy] = vehicleMinZ[vehicleProxy]
                = Double.POSITIVE_INFINITY;
        vehicleMaxX[vehicleProxy] = vehicleMaxY[vehicleProxy] = vehicleMaxZ[vehicleProxy]
                = Double.NEGATIVE_INFINITY;

        double eX = vehicle.entityX, eY = vehicle.entityY, eZ = vehicle.entityZ;
        for (int node = 0; node < vehicle.nodes.count; node++) {
            if (!vehicle.nodes.collision[node]) continue;
            if (count >= MAX_NODES) {
                throw new IllegalStateException("Collidable node count exceeds SAP capacity " + MAX_NODES);
            }

            int part = findOrCreatePart(vehicleProxy, vehicle.nodes.partId[node]);
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
            cacheVehicleProxy[count] = vehicleProxy;
            cacheSelfCollision[count] = vehicle.nodes.selfCollision[node];
            count++;
            vehicleNodeCount[vehicleProxy]++;

            includeBounds(vehicleMinX, vehicleMinY, vehicleMinZ,
                    vehicleMaxX, vehicleMaxY, vehicleMaxZ,
                    vehicleProxy, minX, minY, minZ, maxX, maxY, maxZ);
            includeBounds(partMinX, partMinY, partMinZ, partMaxX, partMaxY, partMaxZ,
                    part, minX, minY, minZ, maxX, maxY, maxZ);
            if (vehicle.nodes.selfCollision[node]) {
                vehicleHasSelfNodes[vehicleProxy] = true;
                vehicleSelfNodeCount[vehicleProxy]++;
                partHasSelfNodes[part] = true;
                includeBounds(partSelfMinX, partSelfMinY, partSelfMinZ,
                        partSelfMaxX, partSelfMaxY, partSelfMaxZ,
                        part, minX, minY, minZ, maxX, maxY, maxZ);
            }
        }
        vehiclePartEnd[vehicleProxy] = partCount;
    }

    public void updateAndSort() {
        int offset = 0, selfOffset = 0;
        for (int vehicle = 0; vehicle < vehicleCount; vehicle++) {
            vehicleStart[vehicle] = offset;
            vehicleWrite[vehicle] = offset;
            offset += vehicleNodeCount[vehicle];
            vehicleSelfStart[vehicle] = selfOffset;
            vehicleSelfWrite[vehicle] = selfOffset;
            selfOffset += vehicleSelfNodeCount[vehicle];
            vehicleActiveAxis[vehicle] = chooseAxis(vehicle);
        }
        vehicleStart[vehicleCount] = offset;
        vehicleSelfStart[vehicleCount] = selfOffset;

        for (int index = 0; index < count; index++) {
            int vehicle = cacheVehicleProxy[index];
            sortKeys[vehicleWrite[vehicle]++] = sortKey(axisMin(index, vehicleActiveAxis[vehicle]), index);
            if (cacheSelfCollision[index]) {
                selfSortKeys[vehicleSelfWrite[vehicle]++] = sortKey(
                        axisMin(index, vehicleActiveAxis[vehicle]), index);
            }
        }

        for (int vehicle = 0; vehicle < vehicleCount; vehicle++) {
            int start = vehicleStart[vehicle], end = vehicleStart[vehicle + 1];
            if (end <= start) continue;
            Arrays.sort(sortKeys, start, end);
            double prefixMax = Double.NEGATIVE_INFINITY;
            for (int sorted = start; sorted < end; sorted++) {
                int original = (int) sortKeys[sorted];
                double maximum = axisMax(original, vehicleActiveAxis[vehicle]);
                if (maximum > prefixMax) prefixMax = maximum;
                sortedPrefixMax[sorted] = prefixMax;
            }

            int selfStart = vehicleSelfStart[vehicle], selfEnd = vehicleSelfStart[vehicle + 1];
            Arrays.sort(selfSortKeys, selfStart, selfEnd);
            prefixMax = Double.NEGATIVE_INFINITY;
            for (int sorted = selfStart; sorted < selfEnd; sorted++) {
                int original = (int) selfSortKeys[sorted];
                double maximum = axisMax(original, vehicleActiveAxis[vehicle]);
                if (maximum > prefixMax) prefixMax = maximum;
                selfSortedPrefixMax[sorted] = prefixMax;
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
        int rawHits = 0;
        for (int vehicle = 0; vehicle < vehicleCount; vehicle++) {
            if (!vehicleMayOverlap(vehicle, triangleVehicle, minX, minY, minZ, maxX, maxY, maxZ)) continue;
            boolean self = vehicleRef[vehicle] == triangleVehicle;
            rawHits += queryVehicle(vehicle, self, minX, minY, minZ, maxX, maxY, maxZ,
                    triangleVehicle, triangleNodeA, triangleNodeB, triangleNodeC,
                    trianglePartId, result);
        }
        return rawHits;
    }

    private boolean vehicleMayOverlap(int vehicle, SoftBodyVehicle triangleVehicle,
                                      double minX, double minY, double minZ,
                                      double maxX, double maxY, double maxZ) {
        boolean self = vehicleRef[vehicle] == triangleVehicle;
        if (self && !vehicleHasSelfNodes[vehicle]) return false;
        for (int part = vehiclePartStart[vehicle]; part < vehiclePartEnd[vehicle]; part++) {
            if (self && !partHasSelfNodes[part]) continue;
            double pMinX = self ? partSelfMinX[part] : partMinX[part];
            double pMinY = self ? partSelfMinY[part] : partMinY[part];
            double pMinZ = self ? partSelfMinZ[part] : partMinZ[part];
            double pMaxX = self ? partSelfMaxX[part] : partMaxX[part];
            double pMaxY = self ? partSelfMaxY[part] : partMaxY[part];
            double pMaxZ = self ? partSelfMaxZ[part] : partMaxZ[part];
            if (pMaxX >= minX && pMinX <= maxX
                    && pMaxY >= minY && pMinY <= maxY
                    && pMaxZ >= minZ && pMinZ <= maxZ) return true;
        }
        return false;
    }

    private int queryVehicle(int vehicle, boolean self,
                             double minX, double minY, double minZ,
                             double maxX, double maxY, double maxZ,
                             SoftBodyVehicle triangleVehicle,
                             int triangleNodeA, int triangleNodeB, int triangleNodeC,
                             int trianglePartId,
                             SweepResultBuffer result) {
        long[] keys = self ? selfSortKeys : sortKeys;
        double[] prefixMaxima = self ? selfSortedPrefixMax : sortedPrefixMax;
        int start = self ? vehicleSelfStart[vehicle] : vehicleStart[vehicle];
        int end = self ? vehicleSelfStart[vehicle + 1] : vehicleStart[vehicle + 1];
        int axis = vehicleActiveAxis[vehicle];
        double targetMin = axis == 0 ? minX : axis == 1 ? minY : minZ;
        double targetMax = axis == 0 ? maxX : axis == 1 ? maxY : maxZ;

        int left = start, right = end - 1, startIndex = end;
        while (left <= right) {
            int middle = (left + right) >>> 1;
            if (prefixMaxima[middle] >= targetMin) {
                startIndex = middle;
                right = middle - 1;
            } else {
                left = middle + 1;
            }
        }

        long maxKeyLimit = sortableLimit(targetMax);
        int rawHits = 0;
        for (int sorted = startIndex; sorted < end; sorted++) {
            long key = keys[sorted];
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

    private int findOrCreatePart(int vehicle, int localPart) {
        for (int part = vehiclePartStart[vehicle]; part < partCount; part++) {
            if (partVehicleProxy[part] == vehicle && partLocalId[part] == localPart) return part;
        }
        if (partCount >= MAX_GROUPS) {
            throw new IllegalStateException("Collision part count exceeds SAP capacity " + MAX_GROUPS);
        }
        int part = partCount++;
        partVehicleProxy[part] = vehicle;
        partLocalId[part] = localPart;
        partHasSelfNodes[part] = false;
        partMinX[part] = partMinY[part] = partMinZ[part] = Double.POSITIVE_INFINITY;
        partMaxX[part] = partMaxY[part] = partMaxZ[part] = Double.NEGATIVE_INFINITY;
        partSelfMinX[part] = partSelfMinY[part] = partSelfMinZ[part] = Double.POSITIVE_INFINITY;
        partSelfMaxX[part] = partSelfMaxY[part] = partSelfMaxZ[part] = Double.NEGATIVE_INFINITY;
        return part;
    }

    private byte chooseAxis(int vehicle) {
        double spanX = vehicleMaxX[vehicle] - vehicleMinX[vehicle];
        double spanY = vehicleMaxY[vehicle] - vehicleMinY[vehicle];
        double spanZ = vehicleMaxZ[vehicle] - vehicleMinZ[vehicle];
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

    private static void includeBounds(double[] minsX, double[] minsY, double[] minsZ,
                                      double[] maxsX, double[] maxsY, double[] maxsZ, int index,
                                      double minX, double minY, double minZ,
                                      double maxX, double maxY, double maxZ) {
        if (minX < minsX[index]) minsX[index] = minX;
        if (minY < minsY[index]) minsY[index] = minY;
        if (minZ < minsZ[index]) minsZ[index] = minZ;
        if (maxX > maxsX[index]) maxsX[index] = maxX;
        if (maxY > maxsY[index]) maxsY[index] = maxY;
        if (maxZ > maxsZ[index]) maxsZ[index] = maxZ;
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
