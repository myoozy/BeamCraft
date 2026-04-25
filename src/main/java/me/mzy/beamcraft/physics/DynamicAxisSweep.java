package me.mzy.beamcraft.physics;

import java.util.Arrays;

/**
 * 动态 1D 扫掠与裁剪加速结构 (Dynamic Sweep and Prune)
 * 实时计算方差最大的轴进行扫掠，彻底免疫单轴拥挤
 */
public class DynamicAxisSweep {
    private static final int MAX_NODES = 16384;

    private final long[] sortKeys = new long[MAX_NODES];

    // 为了统一坐标判断，现在缓存 X, Y, Z 三个坐标
    private final float[] cacheX = new float[MAX_NODES];
    private final float[] cacheY = new float[MAX_NODES];
    private final float[] cacheZ = new float[MAX_NODES];
    private final SoftBodyVehicle[] cacheVeh = new SoftBodyVehicle[MAX_NODES];
    private final int[] cacheNodeId = new int[MAX_NODES];

    private int count = 0;

    // 0 = X轴, 1 = Y轴, 2 = Z轴
    private int activeAxis = 0;

    public void clear() { count = 0; }

    public void insertNodes(SoftBodyVehicle veh) {
        double eX = veh.entityX, eY = veh.entityY, eZ = veh.entityZ;
        for (int i = 0; i < veh.nodes.count; i++) {
            if (!veh.nodes.collision[i]) continue;
            cacheX[count] = (float) (eX + veh.nodes.posX[i]);
            cacheY[count] = (float) (eY + veh.nodes.posY[i]);
            cacheZ[count] = (float) (eZ + veh.nodes.posZ[i]);
            cacheVeh[count] = veh;
            cacheNodeId[count] = i;
            count++;
        }
    }

    public void updateAndSort() {
        if (count == 0) return;

        // 1. 动态选择最佳排序轴 (跨度最大的轴)
        float minX = cacheX[0], maxX = cacheX[0];
        float minY = cacheY[0], maxY = cacheY[0];
        float minZ = cacheZ[0], maxZ = cacheZ[0];

        for (int i = 0; i < count; i++) {
            // 更新坐标 (这里你可以加入车辆实体位移更新的逻辑)
            // ...
            float x = cacheX[i], y = cacheY[i], z = cacheZ[i];
            if (x < minX) minX = x; if (x > maxX) maxX = x;
            if (y < minY) minY = y; if (y > maxY) maxY = y;
            if (z < minZ) minZ = z; if (z > maxZ) maxZ = z;
        }

        float spanX = maxX - minX;
        float spanY = maxY - minY;
        float spanZ = maxZ - minZ;

        // 挑选跨度最大的作为主轴
        if (spanX >= spanY && spanX >= spanZ) activeAxis = 0;
        else if (spanY >= spanX && spanY >= spanZ) activeAxis = 1;
        else activeAxis = 2;

        // 2. 将主轴的值编码为排序键
        for (int i = 0; i < count; i++) {
            float val = (activeAxis == 0) ? cacheX[i] : ((activeAxis == 1) ? cacheY[i] : cacheZ[i]);
            int intVal = Float.floatToRawIntBits(val);
            if (intVal < 0) intVal = 0x80000000 - intVal;

            long origIdx = sortKeys[i] & 0xFFFFFFFFL;
            // 如果是初次插入，origIdx 可能是错的，所以这里简单处理直接取当前索引 i
            // 也可以每帧从头排序，Arrays.sort 的 Dual-Pivot 对基本类型极快
            sortKeys[i] = ((long) intVal << 32) | (i & 0xFFFFFFFFL);
        }

        // 3. 排序 (由于轴可能切换，这里用原生的 Arrays.sort 最稳妥)
        Arrays.sort(sortKeys, 0, count);
    }

    public void queryNodesInAABB(float minX, float minY, float minZ, float maxX, float maxY, float maxZ, SweepResultBuffer result) {
        if (count == 0) return;

        // 获取当前主轴的目标范围
        float targetMin = (activeAxis == 0) ? minX : ((activeAxis == 1) ? minY : minZ);
        float targetMax = (activeAxis == 0) ? maxX : ((activeAxis == 1) ? maxY : maxZ);

        int targetIntMin = Float.floatToRawIntBits(targetMin);
        if (targetIntMin < 0) targetIntMin = 0x80000000 - targetIntMin;
        long targetKeyMin = ((long) targetIntMin << 32);

        int left = 0, right = count - 1, startIdx = count;
        while (left <= right) {
            int mid = (left + right) >>> 1;
            if (sortKeys[mid] >= targetKeyMin) {
                startIdx = mid;
                right = mid - 1;
            } else {
                left = mid + 1;
            }
        }

        int targetIntMax = Float.floatToRawIntBits(targetMax);
        if (targetIntMax < 0) targetIntMax = 0x80000000 - targetIntMax;
        long maxKeyLimit = ((long) targetIntMax << 32) | 0xFFFFFFFFL;

        for (int i = startIdx; i < count; i++) {
            long key = sortKeys[i];
            if (key > maxKeyLimit) break; // 超过当前轴最大值，直接 Prune 裁剪

            int origIdx = (int) (key & 0xFFFFFFFFL);

            // 统一的精细 AABB 裁剪
            float x = cacheX[origIdx], y = cacheY[origIdx], z = cacheZ[origIdx];
            if (x < minX || x > maxX || y < minY || y > maxY || z < minZ || z > maxZ) continue;

            result.add(cacheVeh[origIdx], cacheNodeId[origIdx]);
        }
    }
}