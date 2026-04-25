package me.mzy.beamcraft.physics;

import java.util.Arrays;

/**
 * 极致的一维扫掠加速结构 (1D Sweep and Prune)
 * 完全替代 3D Hash Grid，彻底消灭空空间遍历
 */
public class AxisSweep {
    private static final int MAX_NODES = 16384;

    // 我们将节点的世界坐标 X 强转为 Float 的位表示，拼上所属车辆和节点ID，塞进一个 long 里
    // 这样在 Java 中只需要对一个一维 long[] 排序，速度快到不可思议
    private final long[] sortKeys = new long[MAX_NODES];

    // 缓存 Y 和 Z 坐标，用于后续极速 AABB 剔除，减少对 SoftBodyVehicle 的指针跳转
    private final float[] cacheY = new float[MAX_NODES];
    private final float[] cacheZ = new float[MAX_NODES];
    private final SoftBodyVehicle[] cacheVeh = new SoftBodyVehicle[MAX_NODES];
    private final int[] cacheNodeId = new int[MAX_NODES];

    private int count = 0;

    public void clear() {
        count = 0;
    }

    public void insertNodes(SoftBodyVehicle veh) {
        double eX = veh.entityX, eY = veh.entityY, eZ = veh.entityZ;
        for (int i = 0; i < veh.nodes.count; i++) {
            if (!veh.nodes.collision[i]) continue;

            float wX = (float) (eX + veh.nodes.posX[i]);
            float wY = (float) (eY + veh.nodes.posY[i]);
            float wZ = (float) (eZ + veh.nodes.posZ[i]);

            // 将 float X 转换为保持排序顺序的 32 位整数
            int intX = Float.floatToRawIntBits(wX);
            if (intX < 0) intX = 0x80000000 - intX;

            // 高 32 位存 X，保证排序准确；低 32 位存一个自增的内部索引
            long key = ((long) intX << 32) | (count & 0xFFFFFFFFL);

            sortKeys[count] = key;
            cacheY[count] = wY;
            cacheZ[count] = wZ;
            cacheVeh[count] = veh;
            cacheNodeId[count] = i;

            count++;
        }
    }

    /**
     * 在查询前必须调用一次排序
     */
    public void build() {
        // Java 底层的 Dual-Pivot Quicksort，排几千个 long 只需要不到 0.1 毫秒
        Arrays.sort(sortKeys, 0, count);
    }

    // 删掉原来的 build() 和 Arrays.sort，用这个代替
    // 这个方法在 99% 有序的数据上，耗时几乎为 0.00x ms！
    public void updateAndSort() {
        // 1. 原地更新所有节点的最新坐标 (不要 clear() 重新插入)
        for (int i = 0; i < count; i++) {
            SoftBodyVehicle veh = cacheVeh[i];
            int nodeId = cacheNodeId[i];

            float wX = (float) (veh.entityX + veh.nodes.posX[nodeId]);
            cacheY[i] = (float) (veh.entityY + veh.nodes.posY[nodeId]);
            cacheZ[i] = (float) (veh.entityZ + veh.nodes.posZ[nodeId]);

            int intX = Float.floatToRawIntBits(wX);
            if (intX < 0) intX = 0x80000000 - intX;

            // 保留低 32 位的原始索引，更新高 32 位的 X 坐标
            sortKeys[i] = ((long) intX << 32) | (sortKeys[i] & 0xFFFFFFFFL);
        }

        // 2. 使用微型插入排序 (Insertion Sort) 修复由于节点微小移动产生的乱序
        for (int i = 1; i < count; i++) {
            long key = sortKeys[i];
            float cy = cacheY[i], cz = cacheZ[i];
            SoftBodyVehicle cv = cacheVeh[i];
            int cid = cacheNodeId[i];

            int j = i - 1;
            // 只要前面的元素比当前元素大，就往后挪一位 (大部分情况下，进不去这个 while 循环！)
            while (j >= 0 && sortKeys[j] > key) {
                sortKeys[j + 1] = sortKeys[j];
                cacheY[j + 1] = cacheY[j];
                cacheZ[j + 1] = cacheZ[j];
                cacheVeh[j + 1] = cacheVeh[j];
                cacheNodeId[j + 1] = cacheNodeId[j];
                j--;
            }
            sortKeys[j + 1] = key;
            cacheY[j + 1] = cy;
            cacheZ[j + 1] = cz;
            cacheVeh[j + 1] = cv;
            cacheNodeId[j + 1] = cid;
        }
    }

    /**
     * $O(\log N)$ 的二分查找 + 单向扫掠，彻底告别 3D 空网格遍历
     */
    public void queryNodesInAABB(float minX, float minY, float minZ, float maxX, float maxY, float maxZ, QueryResultBuffer result) {
        if (count == 0) return;

        // 1. 编码我们的目标 minX
        int targetIntX = Float.floatToRawIntBits(minX);
        if (targetIntX < 0) targetIntX = 0x80000000 - targetIntX;
        long targetKey = ((long) targetIntX << 32); // 低 32 位为 0

        // 2. 二分查找：找到 X 轴上第一个大于等于 minX 的点
        int left = 0;
        int right = count - 1;
        int startIdx = count;
        while (left <= right) {
            int mid = (left + right) >>> 1;
            if (sortKeys[mid] >= targetKey) {
                startIdx = mid;
                right = mid - 1;
            } else {
                left = mid + 1;
            }
        }

        // 3. 扫掠：从 startIdx 往右遍历，一旦节点的 X 超出 maxX，立刻终止！(Prune)
        int maxIntX = Float.floatToRawIntBits(maxX);
        if (maxIntX < 0) maxIntX = 0x80000000 - maxIntX;
        long maxKeyLimit = ((long) maxIntX << 32) | 0xFFFFFFFFL;

        for (int i = startIdx; i < count; i++) {
            long key = sortKeys[i];

            // 【核心裁剪】：如果当前点的 X 已经大于 AABB 的 maxX，后面的点绝不可能相交，直接结束！
            if (key > maxKeyLimit) break;

            int origIdx = (int) (key & 0xFFFFFFFFL);

            // 4. 判断 Y 和 Z 是否在范围内
            float y = cacheY[origIdx];
            if (y < minY || y > maxY) continue;

            float z = cacheZ[origIdx];
            if (z < minZ || z > maxZ) continue;

            // 存入结果
            result.add(cacheVeh[origIdx], cacheNodeId[origIdx]);
        }
    }
}
