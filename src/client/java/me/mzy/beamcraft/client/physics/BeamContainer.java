package me.mzy.beamcraft.client.physics;

import me.mzy.beamcraft.utility.Utility;

import java.util.HashMap;

public class BeamContainer {
    public static final int INIT_BEAM_CAP = 256;

    // --- 梁类型枚举 ---
    public static final int BEAM_NORMAL = 0;
    public static final int BEAM_SUPPORT = 1;
    public static final int BEAM_BOUNDED = 2;
    public static final int BEAM_LBEAM = 3;
    public static final int BEAM_HYDRO = 4;
    public static final int BEAM_ANISOTROPIC = 5;

    public java.util.List<String>[] assignedBreakGroups;

    public int count = 0;
    public int[] node1;
    public int[] node2;
    public float[] restLength;
    public float[] baseRestLength;
    public float[] targetRestLength;
    public float[] precompTimeTotal;
    public float[] precompTimer;
    public float[] spring;
    public float[] damp;
    public float[] deform;
    public float[] strength;
    public boolean[] broken;
    public int[] breakGroupType;
    public int[] wheelId;

    public BeamContainer() {
        node1 = new int[INIT_BEAM_CAP];
        node2 = new int[INIT_BEAM_CAP];
        restLength = new float[INIT_BEAM_CAP];
        baseRestLength = new float[INIT_BEAM_CAP];
        targetRestLength = new float[INIT_BEAM_CAP];
        precompTimeTotal = new float[INIT_BEAM_CAP];
        precompTimer = new float[INIT_BEAM_CAP];
        spring = new float[INIT_BEAM_CAP];
        damp = new float[INIT_BEAM_CAP];
        deform = new float[INIT_BEAM_CAP];
        strength = new float[INIT_BEAM_CAP];
        broken = new boolean[INIT_BEAM_CAP];
        breakGroupType = new int[INIT_BEAM_CAP];
        assignedBreakGroups = new java.util.List[INIT_BEAM_CAP];
        wheelId = new int[INIT_BEAM_CAP];
    }

    private void ensureCapacity() {
        if (count >= node1.length) {
            int newSize = node1.length * 2;
            resize(newSize);
            System.out.println("⚠️ [BeamContainer] Resized to: " + newSize);
        }
    }

    protected void resize(int newSize) {
        assignedBreakGroups = java.util.Arrays.copyOf(assignedBreakGroups, newSize);
        node1 = Utility.expand(node1, newSize);
        node2 = Utility.expand(node2, newSize);
        restLength = Utility.expand(restLength, newSize);
        baseRestLength = Utility.expand(baseRestLength, newSize);
        targetRestLength = Utility.expand(targetRestLength, newSize);
        precompTimeTotal = Utility.expand(precompTimeTotal, newSize);
        precompTimer = Utility.expand(precompTimer, newSize);
        spring = Utility.expand(spring, newSize);
        damp = Utility.expand(damp, newSize);
        deform = Utility.expand(deform, newSize);
        strength = Utility.expand(strength, newSize);
        broken = Utility.expand(broken, newSize);
        breakGroupType = Utility.expand(breakGroupType, newSize);
        wheelId = Utility.expand(wheelId, newSize);
    }

    protected int addBeamInternal(java.util.List<String> breakGroups, int breakGroupType,
                                  int node1Idx, int node2Idx, double nodeDist,
                                  double spring, double damp,
                                  double deform, double strength,
                                  double precomp, double precompRange, double precompTime) {
        ensureCapacity();
        int idx = this.count;

        if (breakGroups != null && !breakGroups.isEmpty()) {
            this.assignedBreakGroups[count] = new java.util.ArrayList<>(breakGroups);
        } else {
            this.assignedBreakGroups[count] = null;
        }

        this.node1[idx] = node1Idx;
        this.node2[idx] = node2Idx;

        float targetLen = (float) ((nodeDist * precomp) + precompRange);
        this.targetRestLength[idx] = targetLen;

        if (precompTime > 0.0) {
            this.restLength[idx] = (float) nodeDist;
            this.precompTimer[idx] = (float) precompTime;
            this.precompTimeTotal[idx] = (float) precompTime;
        } else {
            this.restLength[idx] = targetLen;
            this.precompTimer[idx] = 0.0f;
            this.precompTimeTotal[idx] = 0.0f;
        }

        this.baseRestLength[idx] = this.restLength[idx];
        this.spring[idx] = (float) spring;
        this.damp[idx] = (float) damp;
        this.deform[idx] = (float) deform;
        this.strength[idx] = (float) strength;
        this.broken[idx] = false;
        this.breakGroupType[idx] = breakGroupType;
        this.wheelId[idx] = -1;

        count++;
        return idx;
    }

    public int addBeam(java.util.List<String> breakGroups, int breakGroupType,
                        int node1Idx, int node2Idx, double nodeDist,
                        double spring, double damp,
                        double deform, double strength,
                        double precomp, double precompRange, double precompTime) {
        return addBeamInternal(breakGroups, breakGroupType, node1Idx, node2Idx, nodeDist, spring, damp,
                deform, strength, precomp, precompRange, precompTime);
    }

    public void bindToTire(int beamIdx, int wheelIdx) {
        if (0 <= beamIdx && beamIdx <= count - 1) {
            wheelId[beamIdx] = wheelIdx;
        }
    }

    public void clear() {
        reset();
        count = 0;
    }

    public void reset() {
        for (int i = 0; i < count; i++) {
            broken[i] = false;
            restLength[i] = baseRestLength[i];
            precompTimer[i] = precompTimeTotal[i];
        }
    }

    public void updatePrecompression(float mcDt) {
        for (int i = 0; i < count; i++) {
            if (precompTimer[i] > 0) {
                precompTimer[i] -= mcDt;
                if (precompTimer[i] <= 0) {
                    precompTimer[i] = 0;
                    restLength[i] = targetRestLength[i];
                } else {
                    float progress = 1.0f - (precompTimer[i] / precompTimeTotal[i]);
                    restLength[i] = baseRestLength[i] + (targetRestLength[i] - baseRestLength[i]) * progress;
                }
            }
        }
    }
}
