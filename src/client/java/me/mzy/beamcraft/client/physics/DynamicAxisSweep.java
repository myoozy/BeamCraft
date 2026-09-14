package me.mzy.beamcraft.client.physics;

import java.util.Arrays;

/**
 * 动态 1D 扫掠与裁剪加速结构 (Dynamic Sweep and Prune)
 * 每次选取跨度最大的轴作为排序轴，避免单轴拥挤
 */
public class DynamicAxisSweep {
    // Keep the SAP and collision-coloring capacities aligned. PhysicsWorld
    // validates the total node count before assigning global node offsets.
    static final int MAX_NODES = SoftBodyCollisionManager.MAX_GLOBAL_NODES;

    private final long[] sortKeys = new long[MAX_NODES];

    // 缓存 X, Y, Z 三个坐标，统一坐标判断
    // Swept node bounds over one broad-phase interval. All arrays are retained
    // so rebuilding the SAP in the physics hot loop remains allocation-free.
    private final double[] cacheMinX = new double[MAX_NODES];
    private final double[] cacheMinY = new double[MAX_NODES];
    private final double[] cacheMinZ = new double[MAX_NODES];
    private final double[] cacheMaxX = new double[MAX_NODES];
    private final double[] cacheMaxY = new double[MAX_NODES];
    private final double[] cacheMaxZ = new double[MAX_NODES];
    /** Prefix maximum of the active-axis swept maxima in sorted order. */
    private final double[] sortedPrefixMax = new double[MAX_NODES];
    private final SoftBodyVehicle[] cacheVeh = new SoftBodyVehicle[MAX_NODES];
    private final int[] cacheNodeId = new int[MAX_NODES];

    private int count = 0;

    // 0 = X轴, 1 = Y轴, 2 = Z轴
    private int activeAxis = 0;

    public void clear() { count = 0; }

    public void insertNodes(SoftBodyVehicle veh, double dtPredict) {
        double eX = veh.entityX, eY = veh.entityY, eZ = veh.entityZ;
        for (int i = 0; i < veh.nodes.count; i++) {
            if (!veh.nodes.collision[i]) continue;
            if (count >= MAX_NODES) {
                throw new IllegalStateException("Collidable node count exceeds SAP capacity " + MAX_NODES);
            }
            double x = eX + veh.nodes.posX[i];
            double y = eY + veh.nodes.posY[i];
            double z = eZ + veh.nodes.posZ[i];
            double futureX = x + veh.nodes.velX[i] * dtPredict;
            double futureY = y + veh.nodes.velY[i] * dtPredict;
            double futureZ = z + veh.nodes.velZ[i] * dtPredict;
            cacheMinX[count] = Math.min(x, futureX);
            cacheMinY[count] = Math.min(y, futureY);
            cacheMinZ[count] = Math.min(z, futureZ);
            cacheMaxX[count] = Math.max(x, futureX);
            cacheMaxY[count] = Math.max(y, futureY);
            cacheMaxZ[count] = Math.max(z, futureZ);
            cacheVeh[count] = veh;
            cacheNodeId[count] = i;
            count++;
        }
    }

    public void updateAndSort() {
        if (count == 0) return;

        // 1. 动态选择最佳排序轴 (跨度最大的轴)
        double minX = cacheMinX[0], maxX = cacheMaxX[0];
        double minY = cacheMinY[0], maxY = cacheMaxY[0];
        double minZ = cacheMinZ[0], maxZ = cacheMaxZ[0];

        for (int i = 0; i < count; i++) {
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

        // 挑选跨度最大的作为主轴
        if (spanX >= spanY && spanX >= spanZ) activeAxis = 0;
        else if (spanY >= spanX && spanY >= spanZ) activeAxis = 1;
        else activeAxis = 2;

        // 2. 将主轴的值编码为排序键
        for (int i = 0; i < count; i++) {
            double val = axisMin(i);
            long intVal = Double.doubleToRawLongBits(val);
            if (intVal < 0) intVal = Long.MIN_VALUE - intVal;

            // 高 32 位是排序键，低 32 位存放节点索引 (queryNodesInAABB 取回为 origIdx)
            sortKeys[i] = (intVal & 0xFFFFFFFF00000000L) | (i & 0xFFFFFFFFL);
        }

        // 3. 排序 (主轴可能切换，每次重新排序)
        Arrays.sort(sortKeys, 0, count);

        // A lower-bound on swept minima alone is unsafe: an interval may begin
        // before the query yet extend into it. This monotonic prefix lets the
        // query skip only entries whose swept maxima are all before targetMin.
        double prefixMax = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < count; i++) {
            int origIdx = (int) (sortKeys[i] & 0xFFFFFFFFL);
            double nodeMax = axisMax(origIdx);
            if (nodeMax > prefixMax) prefixMax = nodeMax;
            sortedPrefixMax[i] = prefixMax;
        }
    }

    /**
     * Queries one triangle's node candidates and applies topology-only filters
     * before writing the result buffer. The returned count is the raw 3D AABB
     * hit count, retained for broad-phase diagnostics.
     */
    public int queryCollisionNodesInAABB(
            double minX, double minY, double minZ,
            double maxX, double maxY, double maxZ,
            SoftBodyVehicle triangleVehicle,
            int triangleNodeA, int triangleNodeB, int triangleNodeC,
            int trianglePartId,
            SweepResultBuffer result) {
        if (count == 0) return 0;

        // 当前主轴的目标范围
        double targetMin = (activeAxis == 0) ? minX : ((activeAxis == 1) ? minY : minZ);
        double targetMax = (activeAxis == 0) ? maxX : ((activeAxis == 1) ? maxY : maxZ);

        int left = 0, right = count - 1, startIdx = count;
        while (left <= right) {
            int mid = (left + right) >>> 1;
            if (sortedPrefixMax[mid] >= targetMin) {
                startIdx = mid;
                right = mid - 1;
            } else {
                left = mid + 1;
            }
        }

        long targetIntMax = Double.doubleToRawLongBits(targetMax);
        if (targetIntMax < 0) targetIntMax = Long.MIN_VALUE - targetIntMax;
        long maxKeyLimit = (targetIntMax & 0xFFFFFFFF00000000L) | 0xFFFFFFFFL;

        int rawHitCount = 0;
        for (int i = startIdx; i < count; i++) {
            long key = sortKeys[i];
            if (key > maxKeyLimit) break; // 超过当前轴最大值，提前结束

            int origIdx = (int) (key & 0xFFFFFFFFL);

            // 精细 AABB 裁剪
            if (cacheMaxX[origIdx] < minX || cacheMinX[origIdx] > maxX
                    || cacheMaxY[origIdx] < minY || cacheMinY[origIdx] > maxY
                    || cacheMaxZ[origIdx] < minZ || cacheMinZ[origIdx] > maxZ) continue;

            rawHitCount++;
            SoftBodyVehicle hitVehicle = cacheVeh[origIdx];
            int hitNodeId = cacheNodeId[origIdx];
            if (hitVehicle == triangleVehicle) {
                if (!triangleVehicle.nodes.selfCollision[hitNodeId]) continue;
                if (hitNodeId == triangleNodeA || hitNodeId == triangleNodeB || hitNodeId == triangleNodeC) continue;
                if (trianglePartId >= 0 && trianglePartId < triangleVehicle.matrixPartStride
                        && triangleVehicle.nodeInPartMatrix[
                                hitNodeId * triangleVehicle.matrixPartStride + trianglePartId]) {
                    continue;
                }
            }

            result.add(hitVehicle, hitNodeId);
        }
        return rawHitCount;
    }

    private double axisMin(int index) {
        return activeAxis == 0 ? cacheMinX[index]
                : activeAxis == 1 ? cacheMinY[index] : cacheMinZ[index];
    }

    private double axisMax(int index) {
        return activeAxis == 0 ? cacheMaxX[index]
                : activeAxis == 1 ? cacheMaxY[index] : cacheMaxZ[index];
    }
}
