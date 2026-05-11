package me.mzy.beamcraft.client.physics;

import java.util.Arrays;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * 软体碰撞调度与染色分批中心 (0 GC 设计)
 * 专职处理车与车、车自身的复杂碰撞，输出无冲突的并行批次
 */
public class SoftBodyCollisionManager {
    public static final int MAX_CONTACTS = 16384;
    public static final int MAX_BATCHES = 16;

    // 全局节点最大数量 (按需调整，比如 32768 足够容纳十几辆极其复杂的车)
    public static final int MAX_GLOBAL_NODES = 32768;

    public final AtomicInteger contactCount = new AtomicInteger(0);

    // --- 接触对缓存 (SoA 风格) ---
    public final SoftBodyVehicle[] contactNodeVeh = new SoftBodyVehicle[MAX_CONTACTS];
    public final int[] contactNodeId = new int[MAX_CONTACTS];
    public final SoftBodyVehicle[] contactTriVeh = new SoftBodyVehicle[MAX_CONTACTS];
    public final int[] contactTriA = new int[MAX_CONTACTS];
    public final int[] contactTriB = new int[MAX_CONTACTS];
    public final int[] contactTriC = new int[MAX_CONTACTS];

    // --- 批次数据结构 ---
    public final int[][] batches = new int[MAX_BATCHES][MAX_CONTACTS];
    public final int[] batchSize = new int[MAX_BATCHES];
    public int activeBatchCount = 0;

    // --- 染色标记数组 ---
    public final int[] nodeLastBatch = new int[MAX_GLOBAL_NODES];

    public void clearContacts() {
        contactCount.set(0);
    }

    public void addContact(SoftBodyVehicle nodeVeh, int nodeId, SoftBodyVehicle triVeh, int nA, int nB, int nC) {
        // 多线程并发抢占数组索引，极其高效 (没有任何锁等待，只需一条 CPU 原子指令)
        int idx = contactCount.getAndIncrement();

        if (idx >= MAX_CONTACTS) {
            contactCount.decrementAndGet(); // 满了就退回
            return;
        }

        contactNodeVeh[idx] = nodeVeh;
        contactNodeId[idx] = nodeId;
        contactTriVeh[idx] = triVeh;
        contactTriA[idx] = nA;
        contactTriB[idx] = nB;
        contactTriC[idx] = nC;
    }

    public void buildAndColorBatches() {
        activeBatchCount = 0;
        Arrays.fill(batchSize, 0);
        Arrays.fill(nodeLastBatch, -1);

        int currentCount = contactCount.get(); // 获取当前的真实数量
        if (currentCount == 0) return;

        for (int i = 0; i < currentCount; i++) {
            SoftBodyVehicle nVeh = contactNodeVeh[i];
            SoftBodyVehicle tVeh = contactTriVeh[i];

            // 使用偏移量计算全局唯一 ID，内存绝对连续且不越界！
            int globalHit = nVeh.globalNodeOffset + contactNodeId[i];
            int globalA   = tVeh.globalNodeOffset + contactTriA[i];
            int globalB   = tVeh.globalNodeOffset + contactTriB[i];
            int globalC   = tVeh.globalNodeOffset + contactTriC[i];

            int maxBatch = nodeLastBatch[globalHit];
            if (nodeLastBatch[globalA] > maxBatch) maxBatch = nodeLastBatch[globalA];
            if (nodeLastBatch[globalB] > maxBatch) maxBatch = nodeLastBatch[globalB];
            if (nodeLastBatch[globalC] > maxBatch) maxBatch = nodeLastBatch[globalC];

            int targetBatch = maxBatch + 1;

            if (targetBatch >= MAX_BATCHES) {
                targetBatch = MAX_BATCHES - 1;
            }

            batches[targetBatch][batchSize[targetBatch]] = i;
            batchSize[targetBatch]++;

            if (targetBatch + 1 > activeBatchCount) {
                activeBatchCount = targetBatch + 1;
            }

            nodeLastBatch[globalHit] = targetBatch;
            nodeLastBatch[globalA]   = targetBatch;
            nodeLastBatch[globalB]   = targetBatch;
            nodeLastBatch[globalC]   = targetBatch;
        }
    }
}