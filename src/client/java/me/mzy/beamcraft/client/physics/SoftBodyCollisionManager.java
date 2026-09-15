package me.mzy.beamcraft.client.physics;

import java.util.Arrays;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * 软体碰撞调度与着色分批
 * 处理车与车、车自身的碰撞接触，输出无冲突的并行批次
 */
public class SoftBodyCollisionManager {
    public static final int MAX_CONTACTS = 16384;
    public static final int MAX_BATCHES = 16;
    /** The last batch is a correctness fallback and must always be solved serially. */
    public static final int OVERFLOW_BATCH_INDEX = MAX_BATCHES - 1;
    private static final int NORMAL_BATCH_COUNT = OVERFLOW_BATCH_INDEX;
    private static final int NORMAL_BATCH_MASK = (1 << NORMAL_BATCH_COUNT) - 1;

    // 全局节点最大数量 (按需调整；32768 可容纳十余辆结构复杂的车)
    public static final int MAX_GLOBAL_NODES = 32768;

    public final AtomicInteger contactCount = new AtomicInteger(0);

    // --- 接触对缓存 (SoA 风格) ---
    public final SoftBodyVehicle[] contactNodeVeh = new SoftBodyVehicle[MAX_CONTACTS];
    public final int[] contactNodeId = new int[MAX_CONTACTS];
    public final SoftBodyVehicle[] contactTriVeh = new SoftBodyVehicle[MAX_CONTACTS];
    public final int[] contactTriA = new int[MAX_CONTACTS];
    public final int[] contactTriB = new int[MAX_CONTACTS];
    public final int[] contactTriC = new int[MAX_CONTACTS];

    // A separation certificate is expressed in triangle-A-relative coordinates.
    // Common translation therefore costs nothing, while deformation, acceleration,
    // and direction changes consume the recorded clearance using actual positions.
    private final float[] separationSlack = new float[MAX_CONTACTS];
    private final float[] separationRefPax = new float[MAX_CONTACTS];
    private final float[] separationRefPay = new float[MAX_CONTACTS];
    private final float[] separationRefPaz = new float[MAX_CONTACTS];
    private final float[] separationRefBax = new float[MAX_CONTACTS];
    private final float[] separationRefBay = new float[MAX_CONTACTS];
    private final float[] separationRefBaz = new float[MAX_CONTACTS];
    private final float[] separationRefCax = new float[MAX_CONTACTS];
    private final float[] separationRefCay = new float[MAX_CONTACTS];
    private final float[] separationRefCaz = new float[MAX_CONTACTS];

    // --- 批次数据结构 ---
    public final int[][] batches = new int[MAX_BATCHES][MAX_CONTACTS];
    public final int[] batchSize = new int[MAX_BATCHES];
    public int activeBatchCount = 0;

    // --- 染色标记数组 ---
    /** Bit {@code n} means that this node is already used by normal batch {@code n}. */
    public final int[] nodeUsedBatchMask = new int[MAX_GLOBAL_NODES];

    public void clearContacts() {
        contactCount.set(0);
    }

    public boolean addContact(SoftBodyVehicle nodeVeh, int nodeId, SoftBodyVehicle triVeh, int nA, int nB, int nC) {
        // 多线程通过一条原子指令抢占数组索引，无锁等待
        int idx = contactCount.getAndIncrement();

        if (idx >= MAX_CONTACTS) {
            contactCount.decrementAndGet(); // 已满则回退
            return false;
        }

        contactNodeVeh[idx] = nodeVeh;
        contactNodeId[idx] = nodeId;
        contactTriVeh[idx] = triVeh;
        contactTriA[idx] = nA;
        contactTriB[idx] = nB;
        contactTriC[idx] = nC;
        separationSlack[idx] = 0.0f;
        return true;
    }

    boolean separationCertificateStillValid(int contactId,
                                            float pax, float pay, float paz,
                                            float bax, float bay, float baz,
                                            float cax, float cay, float caz) {
        float slack = separationSlack[contactId];
        if (!(slack > 0.0f)) return false;

        // L1 distances are cheap upper bounds on Euclidean movement. Expressing
        // all three vectors relative to A makes rigid translation cancel exactly.
        float pointMovement = Math.abs(pax - separationRefPax[contactId])
                + Math.abs(pay - separationRefPay[contactId])
                + Math.abs(paz - separationRefPaz[contactId]);
        float bMovement = Math.abs(bax - separationRefBax[contactId])
                + Math.abs(bay - separationRefBay[contactId])
                + Math.abs(baz - separationRefBaz[contactId]);
        float cMovement = Math.abs(cax - separationRefCax[contactId])
                + Math.abs(cay - separationRefCay[contactId])
                + Math.abs(caz - separationRefCaz[contactId]);
        return pointMovement + Math.max(bMovement, cMovement) < slack;
    }

    void recordSeparationCertificate(int contactId, float slack,
                                     float pax, float pay, float paz,
                                     float bax, float bay, float baz,
                                     float cax, float cay, float caz) {
        if (!(slack > 0.0f) || !Float.isFinite(slack)) {
            separationSlack[contactId] = 0.0f;
            return;
        }
        separationRefPax[contactId] = pax;
        separationRefPay[contactId] = pay;
        separationRefPaz[contactId] = paz;
        separationRefBax[contactId] = bax;
        separationRefBay[contactId] = bay;
        separationRefBaz[contactId] = baz;
        separationRefCax[contactId] = cax;
        separationRefCay[contactId] = cay;
        separationRefCaz[contactId] = caz;
        separationSlack[contactId] = slack;
    }

    void invalidateSeparationCertificate(int contactId) {
        separationSlack[contactId] = 0.0f;
    }

    public void buildAndColorBatches() {
        activeBatchCount = 0;
        Arrays.fill(batchSize, 0);
        Arrays.fill(nodeUsedBatchMask, 0);

        int currentCount = contactCount.get(); // 当前实际数量
        if (currentCount == 0) return;

        for (int i = 0; i < currentCount; i++) {
            SoftBodyVehicle nVeh = contactNodeVeh[i];
            SoftBodyVehicle tVeh = contactTriVeh[i];

            // 用偏移量计算全局唯一 ID
            int globalHit = nVeh.globalNodeOffset + contactNodeId[i];
            int globalA   = tVeh.globalNodeOffset + contactTriA[i];
            int globalB   = tVeh.globalNodeOffset + contactTriB[i];
            int globalC   = tVeh.globalNodeOffset + contactTriC[i];

            int usedBatchMask = nodeUsedBatchMask[globalHit]
                    | nodeUsedBatchMask[globalA]
                    | nodeUsedBatchMask[globalB]
                    | nodeUsedBatchMask[globalC];
            int availableBatchMask = (~usedBatchMask) & NORMAL_BATCH_MASK;
            int targetBatch = availableBatchMask != 0
                    ? Integer.numberOfTrailingZeros(availableBatchMask)
                    : OVERFLOW_BATCH_INDEX;

            batches[targetBatch][batchSize[targetBatch]] = i;
            batchSize[targetBatch]++;

            if (targetBatch + 1 > activeBatchCount) {
                activeBatchCount = targetBatch + 1;
            }

            // Overflow contacts execute serially, so they neither need a color nor
            // reserve one. A later contact can still reuse any safe normal batch.
            if (targetBatch != OVERFLOW_BATCH_INDEX) {
                int targetBatchBit = 1 << targetBatch;
                nodeUsedBatchMask[globalHit] |= targetBatchBit;
                nodeUsedBatchMask[globalA]   |= targetBatchBit;
                nodeUsedBatchMask[globalB]   |= targetBatchBit;
                nodeUsedBatchMask[globalC]   |= targetBatchBit;
            }
        }
    }
}
