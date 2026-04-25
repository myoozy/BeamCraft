package me.mzy.beamcraft.physics;

import jdk.jshell.execution.Util;
import me.mzy.beamcraft.utility.Utility;

public class TriangleContainer {
    public static final int INIT_TRI_CAP = 128;

    public int count = 0;
    public int[] node1 = new int[INIT_TRI_CAP];
    public int[] node2 = new int[INIT_TRI_CAP];
    public int[] node3 = new int[INIT_TRI_CAP];

    // 三角面的零件归属
    public int[] partId = new int[INIT_TRI_CAP];

    public boolean[] collision = new boolean[INIT_TRI_CAP];

    private void ensureTriangleCapacity() {
        if (count >= node1.length) {
            int newSize = node1.length * 2;
            node1 = Utility.expand(node1, newSize);
            node2 = Utility.expand(node2, newSize);
            node3 = Utility.expand(node3, newSize);
            partId = Utility.expand(partId, newSize);
            collision = Utility.expand(collision, newSize);
            System.out.println("⚠️ [TriangleContainer] Resized to: " + newSize);
        }
    }

    public void addTriangle(int index1, int index2, int index3, int triPartId, boolean triCollision) {
        ensureTriangleCapacity();

        node1[count] = index1;
        node2[count] = index2;
        node3[count] = index3;
        partId[count] = triPartId;
        collision[count] = triCollision;
        count++;
    }

    public void clear() {
        count = 0;
    }
}
