package me.mzy.beamcraft.physics;

import me.mzy.beamcraft.utility.Utility;

public class SlideNodeContainer {
    public static final int INIT_SLIDENODE_CAP = 64;

    public int count = 0;
    public int[] nodeId = new int[INIT_SLIDENODE_CAP];
    public int[] railA = new int[INIT_SLIDENODE_CAP];
    public int[] railB = new int[INIT_SLIDENODE_CAP];
    public double[] spring = new double[INIT_SLIDENODE_CAP];
    public double[] damp = new double[INIT_SLIDENODE_CAP];
    public double[] restDist = new double[INIT_SLIDENODE_CAP];

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

    public void addSlideNode(int nId, int railAId, int railBId, double slideSpring, double slideDamp, double slideRestDist) {
        ensureCapacity();
        nodeId[count] = nId;
        railA[count] = railAId;
        railB[count] = railBId;
        spring[count] = slideSpring;
        damp[count] = slideDamp;
        restDist[count] = slideRestDist;
        count++;
    }

    public void clear() {
        count = 0;
    }
}