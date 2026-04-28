package me.mzy.beamcraft.physics;

import jdk.jshell.execution.Util;
import me.mzy.beamcraft.utility.Utility;
import net.minecraft.util.math.Vec3d;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicLongArray;

/**
 * Manages node data and physics state using a Structure of Arrays (SoA) approach.
 */
public class NodeContainer {
    public static final int INIT_NODE_CAP = 128;

    private double pendingDx = 0, pendingDy = 0, pendingDz = 0;
    private float pendingYaw = 0, pendingPitch = 0, pendingRoll = 0;
    private boolean hasPending = false;

    // Mapping from JBeam node ID (e.g., "f1r") to internal array index
    public final Map<String, Integer> nameToIndex = new HashMap<>();

    public int count = 0;
    public String[] names = new String[INIT_NODE_CAP];
    public int[] partId = new int[INIT_NODE_CAP];

    // Initial local coordinates from JBeam
    public double[] baseX = new double[INIT_NODE_CAP];
    public double[] baseY = new double[INIT_NODE_CAP];
    public double[] baseZ = new double[INIT_NODE_CAP];

    // Current local offsets relative to the vehicle entity origin
    public double[] posX = new double[INIT_NODE_CAP];
    public double[] posY = new double[INIT_NODE_CAP];
    public double[] posZ = new double[INIT_NODE_CAP];

    public double[] prevPosX = new double[INIT_NODE_CAP];
    public double[] prevPosY = new double[INIT_NODE_CAP];
    public double[] prevPosZ = new double[INIT_NODE_CAP];

    public double[] velX = new double[INIT_NODE_CAP];
    public double[] velY = new double[INIT_NODE_CAP];
    public double[] velZ = new double[INIT_NODE_CAP];

    public double[] forceX = new double[INIT_NODE_CAP];
    public double[] forceY = new double[INIT_NODE_CAP];
    public double[] forceZ = new double[INIT_NODE_CAP];

    public double[] mass = new double[INIT_NODE_CAP];
    public double[] friction = new double[INIT_NODE_CAP];
    public boolean[] collision = new boolean[INIT_NODE_CAP];
    public boolean[] selfCollision = new boolean[INIT_NODE_CAP];

    public int[] collisionRate = new int[INIT_NODE_CAP];
    public int[] sleepRate = new int[INIT_NODE_CAP];

    private void ensureNodeCapacity() {
        if (count >= posX.length) {
            int newSize = posX.length * 2;
            names = Utility.expand(names, newSize);
            partId = Utility.expand(partId, newSize);
            baseX = Utility.expand(baseX, newSize); baseY = Utility.expand(baseY, newSize); baseZ = Utility.expand(baseZ, newSize);
            posX = Utility.expand(posX, newSize);   posY = Utility.expand(posY, newSize);   posZ = Utility.expand(posZ, newSize);
            prevPosX = Utility.expand(prevPosX, newSize); prevPosY = Utility.expand(prevPosY, newSize); prevPosZ = Utility.expand(prevPosZ, newSize);
            velX = Utility.expand(velX, newSize);   velY = Utility.expand(velY, newSize);   velZ = Utility.expand(velZ, newSize);
            forceX = Utility.expand(forceX, newSize); forceY = Utility.expand(forceY, newSize); forceZ = Utility.expand(forceZ, newSize);
            mass = Utility.expand(mass, newSize);   friction = Utility.expand(friction, newSize);
            collision = Utility.expand(collision, newSize); selfCollision = Utility.expand(selfCollision, newSize);
            collisionRate = Utility.expand(collisionRate, newSize); sleepRate = Utility.expand(sleepRate, newSize);

            System.out.println("⚠️ [NodeContainer] Resized to: " + newSize);
        }
    }

    /**
     * Adds a node to the container or accumulates mass if the node already exists.
     */
    public void addNode(String name, double x, double y, double z, double nodeMass, double nodeFriction,
                        int nodePartId, boolean nodeCollision, boolean nodeSelfCollision) {
        ensureNodeCapacity();

        if (nameToIndex.containsKey(name)) {
            // if exists, add weight to it, then return
            int existingIdx = nameToIndex.get(name);
            mass[existingIdx] += nodeMass;
            return;
        }

        names[count] = name;
        baseX[count] = x; baseY[count] = y; baseZ[count] = z;
        posX[count] = x;  posY[count] = y;  posZ[count] = z;

        // clear velocity and force
        velX[count] = 0;  velY[count] = 0;  velZ[count] = 0;
        forceX[count] = 0; forceY[count] = 0; forceZ[count] = 0;

        mass[count] = nodeMass;
        friction[count] = nodeFriction;
        nameToIndex.put(name, count);
        partId[count] = nodePartId;
        collision[count] = nodeCollision;
        selfCollision[count] = nodeSelfCollision;

        collisionRate[count] = 0;
        sleepRate[count] = 0;

        count++;
    }

    public void clear() {
        count = 0;
        nameToIndex.clear();

        // 清空速度、受力
        for (int i = 0; i < velX.length; i++) {
            velX[i] = 0.0;
            velY[i] = 0.0;
            velZ[i] = 0.0;
            forceX[i] = 0.0;
            forceY[i] = 0.0;
            forceZ[i] = 0.0;
            sleepRate[i] = 0;
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

            posX[i] = pos.x;
            posY[i] = pos.y;
            posZ[i] = pos.z;
        }
    }
    
    /**
     * 一次性移动载具的所有节点
     * 可以在生成后调用，也可以在运行时作为独立工具调用
     */
    public void moveNodes(double deltaX, double deltaY, double deltaZ) {
        for(int i = 0; i < count; i++) {
            posX[i] = posX[i] + deltaX;
            posY[i] = posY[i] + deltaY;
            posZ[i] = posZ[i] + deltaZ;
        }
    }

    public void stopNodes() {
        for(int i = 0; i < count; i++) {
            velX[i] = 0.0;
            velY[i] = 0.0;
            velZ[i] = 0.0;
            forceX[i] = 0.0;
            forceY[i] = 0.0;
            forceZ[i] = 0.0;
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
    public void getCenterOfMass(double[] out) {
        double totalMass = 0.0;
        double cx = 0.0, cy = 0.0, cz = 0.0;

        for (int i = 0; i < count; i++) {
            double m = mass[i];
            totalMass += m;
            cx += posX[i] * m;
            cy += posY[i] * m;
            cz += posZ[i] * m;
        }

        if (totalMass > 1e-8) {
            double invMass = 1.0 / totalMass;
            out[0] = cx * invMass;
            out[1] = cy * invMass;
            out[2] = cz * invMass;
        } else {
            out[0] = 0.0;
            out[1] = 0.0;
            out[2] = 0.0;
        }
    }
}
