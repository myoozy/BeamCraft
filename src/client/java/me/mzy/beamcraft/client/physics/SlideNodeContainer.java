package me.mzy.beamcraft.client.physics;

import me.mzy.beamcraft.utility.Utility;

public class SlideNodeContainer {
    public static final int INIT_SLIDENODE_CAP = 64;

    public int count = 0;
    public int[] nodeId = new int[INIT_SLIDENODE_CAP];
    public int[] railA = new int[INIT_SLIDENODE_CAP];
    public int[] railB = new int[INIT_SLIDENODE_CAP];
    public float[] spring = new float[INIT_SLIDENODE_CAP];
    public float[] damp = new float[INIT_SLIDENODE_CAP];
    public float[] restDist = new float[INIT_SLIDENODE_CAP];

    private void ensureCapacity() {
        if (count >= nodeId.length) {
            int newSize = nodeId.length * 2;
            nodeId = Utility.expand(nodeId, newSize);
            railA = Utility.expand(railA, newSize);
            railB = Utility.expand(railB, newSize);
            spring = Utility.expand(spring, newSize);
            damp = Utility.expand(damp, newSize);
            restDist = Utility.expand(restDist, newSize);
            System.out.println("⚠️ [SlideNodeContainer] Resized to: " + newSize);
        }
    }

    public void addSlideNode(PhysicsSpecs.SlideNodeSpec spec, int nId, int railAId, int railBId, float slideRestDist) {
        ensureCapacity();
        nodeId[count] = nId;
        railA[count] = railAId;
        railB[count] = railBId;
        spring[count] = spec.spring();
        damp[count] = spec.damp();
        restDist[count] = slideRestDist;
        count++;
    }

    public void clear() {
        count = 0;
    }
}
