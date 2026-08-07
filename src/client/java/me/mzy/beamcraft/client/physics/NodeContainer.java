package me.mzy.beamcraft.client.physics;

import me.mzy.beamcraft.utility.Utility;

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
        return addNode(
                spec.name(),
                spec.x(),
                spec.y(),
                spec.z(),
                spec.mass(),
                spec.friction(),
                spec.slidingFriction(),
                spec.partId(),
                spec.collision(),
                spec.selfCollision(),
                spec.groups()
        );
    }

    public int addNode(String name, double x, double y, double z, double nodeMass, double nodeFriction, double nodeSlidingFriction,
                        int nodePartId, boolean nodeCollision, boolean nodeSelfCollision, java.util.List<String> groups) {
        ensureNodeCapacity();

        int idx;

        if (nameToIndex.containsKey(name)) {
            // if exists, add weight to it, then return
            int existingIdx = nameToIndex.get(name);
            mass[existingIdx] += (float) nodeMass;

            // 合并 groups
            if (groups != null && !groups.isEmpty()) {
                java.util.List<String> existingGroups = assignedGroups[existingIdx];
                if (existingGroups == null) {
                    // 原有组列表为空，直接新建
                    assignedGroups[existingIdx] = new java.util.ArrayList<>(groups);
                } else {
                    // 去重合并（可根据需要改用 addAll 允许重复）
                    for (String g : groups) {
                        if (!existingGroups.contains(g)) {
                            existingGroups.add(g);
                        }
                    }
                }
            }

            idx = existingIdx;
        }
        else {
            mass[count] = (float) nodeMass;
            names[count] = name;
            nameToIndex.put(name, count);

            // 存入组列表快照
            if (groups != null && !groups.isEmpty()) {
                assignedGroups[count] = new java.util.ArrayList<>(groups);
            } else {
                assignedGroups[count] = null;
            }

            partId[count] = nodePartId;
            wheelId[count] = -1;
            collisionRate[count] = 0;
            sleepRate[count] = 0;
            degree[count] = 0;

            // clear velocity and force
            velX[count] = 0;  velY[count] = 0;  velZ[count] = 0;
            forceX[count] = 0; forceY[count] = 0; forceZ[count] = 0;

            idx = count;
            count++;
        }

        baseX[idx] = (float) x; baseY[idx] = (float) y; baseZ[idx] = (float) z;
        posX[idx] = (float) x;  posY[idx] = (float) y;  posZ[idx] = (float) z;

        friction[idx] = (float) nodeFriction;
        slidingFriction[idx] = (float) (nodeSlidingFriction > PhysicsWorld.KINDA_SMALL_NUMBER ?  nodeSlidingFriction : nodeFriction);

        collision[idx] = nodeCollision;
        selfCollision[idx] = nodeSelfCollision;

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
}
