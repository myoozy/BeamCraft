package me.mzy.beamcraft.physics;

import me.mzy.beamcraft.BeamCraft;
import me.mzy.beamcraft.utility.Utility;

public class BeamContainer {
    public static final int INIT_BEAM_CAP = 256;

    // --- 梁类型枚举 ---
    public static final int BEAM_NORMAL = 0;
    public static final int BEAM_SUPPORT = 1;
    public static final int BEAM_BOUNDED = 2;
    public static final int BEAM_LBEAM = 3;
    public static final int BEAM_HYDRO = 4;
    public static final int BEAM_ANISOTROPIC = 5;

    public int count = 0;
    public int[] node1;
    public int[] node2;
    public double[] restLength;
    public double[] baseRestLength;
    public double[] targetRestLength;
    public double[] precompTimeTotal;
    public double[] precompTimer;
    public double[] spring;
    public double[] damp;
    public double[] deform;
    public double[] strength;
    public boolean[] broken;

    public BeamContainer() {
        node1 = new int[INIT_BEAM_CAP];
        node2 = new int[INIT_BEAM_CAP];
        restLength = new double[INIT_BEAM_CAP];
        baseRestLength = new double[INIT_BEAM_CAP];
        targetRestLength = new double[INIT_BEAM_CAP];
        precompTimeTotal = new double[INIT_BEAM_CAP];
        precompTimer = new double[INIT_BEAM_CAP];
        spring = new double[INIT_BEAM_CAP];
        damp = new double[INIT_BEAM_CAP];
        deform = new double[INIT_BEAM_CAP];
        strength = new double[INIT_BEAM_CAP];
        broken = new boolean[INIT_BEAM_CAP];
    }

    private void ensureCapacity() {
        if (count >= node1.length) {
            int newSize = node1.length * 2;
            resize(newSize);
            System.out.println("⚠️ [BeamContainer] Resized to: " + newSize);
        }
    }

    protected void resize(int newSize) {
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
    }

    protected int addBeamInternal(int node1Idx, int node2Idx, double nodeDist,
                                  double beamSpring, double beamDamp,
                                  double beamDeform, double beamStrength,
                                  double precomp, double precompRange, double precompTime) {
        ensureCapacity();
        int idx = count;

        node1[idx] = node1Idx;
        node2[idx] = node2Idx;

        double targetLen = (nodeDist * precomp) + precompRange;
        targetRestLength[idx] = targetLen;

        if (precompTime > 0.0) {
            restLength[idx] = nodeDist;
            precompTimer[idx] = precompTime;
            precompTimeTotal[idx] = precompTime;
        } else {
            restLength[idx] = targetLen;
            precompTimer[idx] = 0.0;
            precompTimeTotal[idx] = 0.0;
        }

        baseRestLength[idx] = restLength[idx];
        spring[idx] = beamSpring;
        damp[idx] = beamDamp;
        deform[idx] = beamDeform;
        strength[idx] = beamStrength;
        broken[idx] = false;

        count++;
        return idx;
    }

    public void addBeam(int node1Idx, int node2Idx, double nodeDist,
                        double beamSpring, double beamDamp,
                        double beamDeform, double beamStrength,
                        double precomp, double precompRange, double precompTime) {
        addBeamInternal(node1Idx, node2Idx, nodeDist, beamSpring, beamDamp,
                beamDeform, beamStrength, precomp, precompRange, precompTime);
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

    public void updatePrecompression(double mcDt) {
        for (int i = 0; i < count; i++) {
            if (precompTimer[i] > 0) {
                precompTimer[i] -= mcDt;
                if (precompTimer[i] <= 0) {
                    precompTimer[i] = 0;
                    restLength[i] = targetRestLength[i];
                } else {
                    double progress = 1.0 - (precompTimer[i] / precompTimeTotal[i]);
                    restLength[i] = baseRestLength[i] + (targetRestLength[i] - baseRestLength[i]) * progress;
                }
            }
        }
    }
}