package me.mzy.beamcraft.physics;

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

    public double[] mass = new double[INIT_NODE_CAP];
    public double[] friction = new double[INIT_NODE_CAP];
    public boolean[] collision = new boolean[INIT_NODE_CAP];

    private void ensureNodeCapacity() {
        if (count >= posX.length) {
            int newSize = posX.length * 2;
            names = Utility.expand(names, newSize);
            partId = Utility.expand(partId, newSize);
            baseX = Utility.expand(baseX, newSize); baseY = Utility.expand(baseY, newSize); baseZ = Utility.expand(baseZ, newSize);
            posX = Utility.expand(posX, newSize);   posY = Utility.expand(posY, newSize);   posZ = Utility.expand(posZ, newSize);
            velX = Utility.expand(velX, newSize);   velY = Utility.expand(velY, newSize);   velZ = Utility.expand(velZ, newSize);
            forceX = Utility.expand(forceX, newSize); forceY = Utility.expand(forceY, newSize); forceZ = Utility.expand(forceZ, newSize);
            mass = Utility.expand(mass, newSize);   friction = Utility.expand(friction, newSize); collision =  Utility.expand(collision, newSize);
            System.out.println("⚠️ [NodeContainer] Resized to: " + newSize);
        }
    }

    /**
     * Adds a node to the container or accumulates mass if the node already exists.
     */
    public void addNode(String name, double x, double y, double z, double nodeMass, double nodeFriction, int nodePartId, boolean nodeCollision) {
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
        }
    }
}
