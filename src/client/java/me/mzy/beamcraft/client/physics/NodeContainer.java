package me.mzy.beamcraft.client.physics;

import me.mzy.beamcraft.utility.Utility;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

/**
 * Manages node data and physics state using a Structure of Arrays (SoA) approach.
 */
public class NodeContainer {
    public static final int INIT_NODE_CAP = 128;

    // Mapping from JBeam node ID (e.g., "f1r") to internal array index
    public final Map<String, Integer> nameToIndex = new HashMap<>();

    public int count = 0;
    public String[] names = new String[INIT_NODE_CAP];
    public int[] partId = new int[INIT_NODE_CAP];
    public int[] wheelId = new int[INIT_NODE_CAP];
    public java.util.List<String>[] assignedGroups = new java.util.List[INIT_NODE_CAP];

    // Initial local coordinates from JBeam
    public float[] baseX = new float[INIT_NODE_CAP];
    public float[] baseY = new float[INIT_NODE_CAP];
    public float[] baseZ = new float[INIT_NODE_CAP];

    // Current local offsets relative to the vehicle entity origin
    public float[] posX = new float[INIT_NODE_CAP];
    public float[] posY = new float[INIT_NODE_CAP];
    public float[] posZ = new float[INIT_NODE_CAP];

    public float[] prevPosX = new float[INIT_NODE_CAP];
    public float[] prevPosY = new float[INIT_NODE_CAP];
    public float[] prevPosZ = new float[INIT_NODE_CAP];

    // 渲染只读缓冲：在每次物理世界 tick 结束时，把 posX/Y/Z 拷贝进来
    public float[] renderSnapPrevX = new float[INIT_NODE_CAP];
    public float[] renderSnapPrevY = new float[INIT_NODE_CAP];
    public float[] renderSnapPrevZ = new float[INIT_NODE_CAP];
    public float[] renderSnapCurrX = new float[INIT_NODE_CAP];
    public float[] renderSnapCurrY = new float[INIT_NODE_CAP];
    public float[] renderSnapCurrZ = new float[INIT_NODE_CAP];

    public float[] velX = new float[INIT_NODE_CAP];
    public float[] velY = new float[INIT_NODE_CAP];
    public float[] velZ = new float[INIT_NODE_CAP];

    public float[] forceX = new float[INIT_NODE_CAP];
    public float[] forceY = new float[INIT_NODE_CAP];
    public float[] forceZ = new float[INIT_NODE_CAP];

    public float[] mass = new float[INIT_NODE_CAP];
    public float[] friction = new float[INIT_NODE_CAP];
    public float[] slidingFriction = new float[INIT_NODE_CAP];
    public boolean[] collision = new boolean[INIT_NODE_CAP];
    public boolean[] selfCollision = new boolean[INIT_NODE_CAP];

    public int[] collisionRate = new int[INIT_NODE_CAP];
    public int[] sleepRate = new int[INIT_NODE_CAP];

    public int[] degree = new int[INIT_NODE_CAP];

    // Physics-thread scratch used only by getMedianPosition(). Keeping it per
    // vehicle avoids allocations and avoids sharing mutable sort storage across
    // vehicles processed in parallel.
    private float[] medianScratch = new float[INIT_NODE_CAP];

    private void ensureNodeCapacity() {
        if (count >= posX.length) {
            int newSize = posX.length * 2;
            names = Utility.expand(names, newSize);
            partId = Utility.expand(partId, newSize); wheelId = Utility.expand(wheelId, newSize);
            baseX = Utility.expand(baseX, newSize); baseY = Utility.expand(baseY, newSize); baseZ = Utility.expand(baseZ, newSize);
            posX = Utility.expand(posX, newSize);   posY = Utility.expand(posY, newSize);   posZ = Utility.expand(posZ, newSize);
            prevPosX = Utility.expand(prevPosX, newSize); prevPosY = Utility.expand(prevPosY, newSize); prevPosZ = Utility.expand(prevPosZ, newSize);
            velX = Utility.expand(velX, newSize);   velY = Utility.expand(velY, newSize);   velZ = Utility.expand(velZ, newSize);
            forceX = Utility.expand(forceX, newSize); forceY = Utility.expand(forceY, newSize); forceZ = Utility.expand(forceZ, newSize);
            mass = Utility.expand(mass, newSize);   friction = Utility.expand(friction, newSize); slidingFriction = Utility.expand(slidingFriction, newSize);
            collision = Utility.expand(collision, newSize); selfCollision = Utility.expand(selfCollision, newSize);
            collisionRate = Utility.expand(collisionRate, newSize); sleepRate = Utility.expand(sleepRate, newSize);
            degree = Utility.expand(degree, newSize);

            renderSnapPrevX = Utility.expand(renderSnapPrevX, newSize);
            renderSnapPrevY = Utility.expand(renderSnapPrevY, newSize);
            renderSnapPrevZ = Utility.expand(renderSnapPrevZ, newSize);
            renderSnapCurrX = Utility.expand(renderSnapCurrX, newSize);
            renderSnapCurrY = Utility.expand(renderSnapCurrY, newSize);
            renderSnapCurrZ = Utility.expand(renderSnapCurrZ, newSize);

            assignedGroups = java.util.Arrays.copyOf(assignedGroups, newSize);

            System.out.println("⚠️ [NodeContainer] Resized to: " + newSize);
        }
    }

    /**
     * Adds a node to the container or accumulates mass if the node already exists.
     */
    public int addNode(PhysicsSpecs.NodeSpec spec) {
        ensureNodeCapacity();

        int idx;

        if (nameToIndex.containsKey(spec.name())) {
            int existingIdx = nameToIndex.get(spec.name());
            mass[existingIdx] += spec.mass();

            if (spec.groups() != null && !spec.groups().isEmpty()) {
                java.util.List<String> existingGroups = assignedGroups[existingIdx];
                if (existingGroups == null) {
                    assignedGroups[existingIdx] = new java.util.ArrayList<>(spec.groups());
                } else {
                    for (String group : spec.groups()) {
                        if (!existingGroups.contains(group)) {
                            existingGroups.add(group);
                        }
                    }
                }
            }

            idx = existingIdx;
        } else {
            mass[count] = spec.mass();
            names[count] = spec.name();
            nameToIndex.put(spec.name(), count);

            if (spec.groups() != null && !spec.groups().isEmpty()) {
                assignedGroups[count] = new java.util.ArrayList<>(spec.groups());
            } else {
                assignedGroups[count] = null;
            }

            partId[count] = spec.partId();
            wheelId[count] = -1;
            collisionRate[count] = 0;
            sleepRate[count] = 0;
            degree[count] = 0;

            velX[count] = 0;  velY[count] = 0;  velZ[count] = 0;
            forceX[count] = 0; forceY[count] = 0; forceZ[count] = 0;

            idx = count;
            count++;
        }

        baseX[idx] = spec.x(); baseY[idx] = spec.y(); baseZ[idx] = spec.z();
        posX[idx] = spec.x();  posY[idx] = spec.y();  posZ[idx] = spec.z();

        friction[idx] = spec.friction();
        slidingFriction[idx] = spec.slidingFriction() > PhysicsWorld.KINDA_SMALL_NUMBER ? spec.slidingFriction() : spec.friction();

        collision[idx] = spec.collision();
        selfCollision[idx] = spec.selfCollision();

        return idx;
    }
    public void bindToTire(String nodeName, int wheelIdx) {
        if (nameToIndex.containsKey(nodeName)) {
            int idx = nameToIndex.get(nodeName);
            bindToTire(idx, wheelIdx);
        }
    }

    public void bindToTire(int nodeIdx, int wheelIdx) {
        if (0 <= nodeIdx && nodeIdx <= count - 1) {
            wheelId[nodeIdx] = wheelIdx;
        }
    }

    public void clear() {
        // 清空速度、受力
        for (int i = 0; i < count; i++) {
            velX[i] = 0.0f;
            velY[i] = 0.0f;
            velZ[i] = 0.0f;
            forceX[i] = 0.0f;
            forceY[i] = 0.0f;
            forceZ[i] = 0.0f;
            sleepRate[i] = 0;
        }
        count = 0;
        nameToIndex.clear();
    }

    public void writeRenderBuffer() {
        for (int i = 0; i < count; i++) {
            renderSnapCurrX[i] = posX[i];
            renderSnapCurrY[i] = posY[i];
            renderSnapCurrZ[i] = posZ[i];
            renderSnapPrevX[i] = prevPosX[i];
            renderSnapPrevY[i] = prevPosY[i];
            renderSnapPrevZ[i] = prevPosZ[i];
        }
    }

    /**
     * 一次性旋转载具的所有节点（支持偏航、俯仰、滚转）
     * 可以在生成后调用，也可以在运行时作为独立工具调用
     */
    public void rotateNodes(float deltaYawDeg, float deltaPitchDeg, float deltaRollDeg) {

        // 转换为弧度
        float radYaw = (float) Math.toRadians(-deltaYawDeg); // MC 的 Yaw 顺逆时针是反的
        float radPitch = (float) Math.toRadians(-deltaPitchDeg);
        float radRoll = (float) Math.toRadians(deltaRollDeg);

        net.minecraft.util.math.Vec3d pos;
        net.minecraft.util.math.Vec3d basePos;

        for (int i = 0; i < count; i++) {
            // 1. 旋转当前坐标
            pos = new net.minecraft.util.math.Vec3d(posX[i], posY[i], posZ[i]);
            pos = pos.rotateZ(radRoll).rotateX(radPitch).rotateY(radYaw);

            posX[i] = (float) pos.x;
            posY[i] = (float) pos.y;
            posZ[i] = (float) pos.z;
        }
    }
    
    /**
     * 一次性移动载具的所有节点
     * 可以在生成后调用，也可以在运行时作为独立工具调用
     */
    public void moveNodes(float deltaX, float deltaY, float deltaZ) {
        for(int i = 0; i < count; i++) {
            posX[i] = posX[i] + deltaX;
            posY[i] = posY[i] + deltaY;
            posZ[i] = posZ[i] + deltaZ;
        }
    }

    public void stopNodes() {
        for(int i = 0; i < count; i++) {
            velX[i] = 0.0f;
            velY[i] = 0.0f;
            velZ[i] = 0.0f;
            forceX[i] = 0.0f;
            forceY[i] = 0.0f;
            forceZ[i] = 0.0f;
        }
    }

    public void reset() {
        stopNodes();
        for(int i = 0; i < count; i++) {
            posX[i] = baseX[i];
            posY[i] = baseY[i];
            posZ[i] = baseZ[i];
        }
    }

    /**
     * 计算所有节点的质心（总质量加权平均位置）
     * @param out An array of length 3 to store the x, y, z coordinates of the com; 长度为3的数组，用于接收质心的 x, y, z
     */
    public void getCenterOfMass(float[] out) {
        float totalMass = 0.0f;
        float cx = 0.0f, cy = 0.0f, cz = 0.0f;

        for (int i = 0; i < count; i++) {
            float m = mass[i];
            totalMass += m;
            cx += posX[i] * m;
            cy += posY[i] * m;
            cz += posZ[i] * m;
        }

        if (totalMass > 1e-8) {
            float invMass = 1.0f / totalMass;
            out[0] = cx * invMass;
            out[1] = cy * invMass;
            out[2] = cz * invMass;
        } else {
            out[0] = 0.0f;
            out[1] = 0.0f;
            out[2] = 0.0f;
        }
    }

    /**
     * Returns the coordinate-wise median of the current local node positions.
     * This is a robust origin for floating-point recentering: a minority of
     * detached nodes can travel arbitrarily far without dragging the vehicle
     * entity origin away from the main body. It is intentionally not a physical
     * center of mass and must not be used by force or inertia calculations.
     */
    public void getMedianPosition(float[] out) {
        if (medianScratch.length < count) {
            medianScratch = new float[Math.max(count, medianScratch.length * 2)];
        }
        out[0] = medianOfFinite(posX, count, medianScratch);
        out[1] = medianOfFinite(posY, count, medianScratch);
        out[2] = medianOfFinite(posZ, count, medianScratch);
    }

    /** Computes a median while ignoring NaN/infinite values, using caller-owned scratch storage. */
    public static float medianOfFinite(float[] values, int count, float[] scratch) {
        if (count < 0 || count > values.length || scratch.length < count) {
            throw new IllegalArgumentException("invalid median input or undersized scratch buffer");
        }
        int finiteCount = 0;
        for (int i = 0; i < count; i++) {
            float value = values[i];
            if (Float.isFinite(value)) {
                scratch[finiteCount++] = value;
            }
        }
        if (finiteCount == 0) {
            return 0.0f;
        }
        Arrays.sort(scratch, 0, finiteCount);
        int middle = finiteCount >>> 1;
        if ((finiteCount & 1) != 0) {
            return scratch[middle];
        }
        return (scratch[middle - 1] + scratch[middle]) * 0.5f;
    }
}
