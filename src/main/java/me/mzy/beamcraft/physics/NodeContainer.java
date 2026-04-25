package me.mzy.beamcraft.physics;

import jdk.jshell.execution.Util;
import me.mzy.beamcraft.utility.Utility;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicLongArray;

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

    // Initial local coordinates from JBeam
    public double[] baseX = new double[INIT_NODE_CAP];
    public double[] baseY = new double[INIT_NODE_CAP];
    public double[] baseZ = new double[INIT_NODE_CAP];

    // Current local offsets relative to the vehicle entity origin
    public double[] posX = new double[INIT_NODE_CAP];
    public double[] posY = new double[INIT_NODE_CAP];
    public double[] posZ = new double[INIT_NODE_CAP];

    public double[] velX = new double[INIT_NODE_CAP];
    public double[] velY = new double[INIT_NODE_CAP];
    public double[] velZ = new double[INIT_NODE_CAP];

    public double[] forceX = new double[INIT_NODE_CAP];
    public double[] forceY = new double[INIT_NODE_CAP];
    public double[] forceZ = new double[INIT_NODE_CAP];

    // 【并发外力缓冲池】专门用来接收别的车撞过来产生的惩罚力
    public AtomicLongArray deltaPx = new AtomicLongArray(INIT_NODE_CAP);
    public AtomicLongArray deltaPy = new AtomicLongArray(INIT_NODE_CAP);
    public AtomicLongArray deltaPz = new AtomicLongArray(INIT_NODE_CAP);
    public AtomicLongArray deltaVx = new AtomicLongArray(INIT_NODE_CAP);
    public AtomicLongArray deltaVy = new AtomicLongArray(INIT_NODE_CAP);
    public AtomicLongArray deltaVz = new AtomicLongArray(INIT_NODE_CAP);

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
            velX = Utility.expand(velX, newSize);   velY = Utility.expand(velY, newSize);   velZ = Utility.expand(velZ, newSize);
            forceX = Utility.expand(forceX, newSize); forceY = Utility.expand(forceY, newSize); forceZ = Utility.expand(forceZ, newSize);
            mass = Utility.expand(mass, newSize);   friction = Utility.expand(friction, newSize);
            collision = Utility.expand(collision, newSize); selfCollision = Utility.expand(selfCollision, newSize);
            collisionRate = Utility.expand(collisionRate, newSize); sleepRate = Utility.expand(sleepRate, newSize);

            // 手动扩容 AtomicLongArray
            AtomicLongArray newDeltaPx = new AtomicLongArray(newSize);
            AtomicLongArray newDeltaPy = new AtomicLongArray(newSize);
            AtomicLongArray newDeltaPz = new AtomicLongArray(newSize);
            AtomicLongArray newDeltaVx = new AtomicLongArray(newSize);
            AtomicLongArray newDeltaVy = new AtomicLongArray(newSize);
            AtomicLongArray newDeltaVz = new AtomicLongArray(newSize);
            for (int i = 0; i < count; i++) {
                newDeltaPx.set(i, deltaPx.get(i));
                newDeltaPy.set(i, deltaPy.get(i));
                newDeltaPz.set(i, deltaPz.get(i));
                newDeltaVx.set(i, deltaVx.get(i));
                newDeltaVy.set(i, deltaVy.get(i));
                newDeltaVz.set(i, deltaVz.get(i));
            }
            deltaPx = newDeltaPx;
            deltaPy = newDeltaPy;
            deltaPz = newDeltaPz;
            deltaVx = newDeltaVx;
            deltaVy = newDeltaVy;
            deltaVz = newDeltaVz;

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
     * 线程安全的受力叠加方法：任何车都可以并发调用这个方法来揍这辆车！
     */
    public void applyPositionAndVelocityDeltaSafe(int nodeId, double dPx, double dPy, double dPz,
                                                  double dVx, double dVy, double dVz) {
        // 使用与之前相同的 CAS (Compare-And-Swap) 自旋锁进行累加
        addDoubleToAtomic(deltaPx, nodeId, dPx);
        addDoubleToAtomic(deltaPy, nodeId, dPy);
        addDoubleToAtomic(deltaPz, nodeId, dPz);
        addDoubleToAtomic(deltaVx, nodeId, dVx);
        addDoubleToAtomic(deltaVy, nodeId, dVy);
        addDoubleToAtomic(deltaVz, nodeId, dVz);
    }

    // 利用底层的 CAS 自旋锁实现无阻塞的高效 Double 累加
    private void addDoubleToAtomic(AtomicLongArray array, int index, double delta) {
        long currentLong, nextLong;
        do {
            currentLong = array.get(index);
            double currentVal = Double.longBitsToDouble(currentLong);
            nextLong = Double.doubleToRawLongBits(currentVal + delta);
            // 如果在这极短的一瞬间没有其他线程修改它，就写入成功；否则重试(自旋)
        } while (!array.compareAndSet(index, currentLong, nextLong));
    }

    public void flushCollisionDeltas() {
        for (int i = 0; i < count; i++) {
            // 取出并应用位移和速度的变化
            posX[i] += Double.longBitsToDouble(deltaPx.getAndSet(i, 0L));
            posY[i] += Double.longBitsToDouble(deltaPy.getAndSet(i, 0L));
            posZ[i] += Double.longBitsToDouble(deltaPz.getAndSet(i, 0L));

            velX[i] += Double.longBitsToDouble(deltaVx.getAndSet(i, 0L));
            velY[i] += Double.longBitsToDouble(deltaVy.getAndSet(i, 0L));
            velZ[i] += Double.longBitsToDouble(deltaVz.getAndSet(i, 0L));
        }
    }
}
