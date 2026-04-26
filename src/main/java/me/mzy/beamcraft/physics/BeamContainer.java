package me.mzy.beamcraft.physics;

import me.mzy.beamcraft.utility.Utility;

public class BeamContainer {
    public static final int INIT_BEAM_CAP = 256;

    // --- 梁类型枚举 ---
    public static final int BEAM_NORMAL = 0;
    public static final int BEAM_SUPPORT = 1;
    public static final int BEAM_BOUNDED = 2;

    public int count = 0;
    public int[] node1 = new int[INIT_BEAM_CAP];
    public int[] node2 = new int[INIT_BEAM_CAP];

    // 梁属性
    public int[] type = new int[INIT_BEAM_CAP];           // 存储枚举值
    public double[] restLength = new double[INIT_BEAM_CAP];
    public double[] baseRestLength = new double[INIT_BEAM_CAP]; // 满血复活记录
    public double[] spring = new double[INIT_BEAM_CAP];
    public double[] damp = new double[INIT_BEAM_CAP];
    public double[] deform = new double[INIT_BEAM_CAP];
    public double[] strength = new double[INIT_BEAM_CAP];
    public double[] shortBound = new double[INIT_BEAM_CAP]; // 最短极限比例
    public double[] longBound = new double[INIT_BEAM_CAP];  // 最长极限比例
    public double[] limitSpring = new double[INIT_BEAM_CAP]; // 极限反弹力
    public double[] limitDamp = new double[INIT_BEAM_CAP];   // 极限阻尼

    public boolean[] broken = new boolean[INIT_BEAM_CAP];

    private void ensureBeamCapacity() {
        if (count >= node1.length) {
            int newSize = node1.length * 2;
            node1 = Utility.expand(node1, newSize);
            node2 = Utility.expand(node2, newSize);
            restLength = Utility.expand(restLength, newSize);
            baseRestLength = Utility.expand(baseRestLength, newSize);
            spring = Utility.expand(spring, newSize);
            damp = Utility.expand(damp, newSize);
            deform = Utility.expand(deform, newSize);
            strength = Utility.expand(strength, newSize);
            broken = Utility.expand(broken, newSize);
            type = Utility.expand(type, newSize);
            shortBound = Utility.expand(shortBound, newSize);
            longBound = Utility.expand(longBound, newSize);
            limitSpring = Utility.expand(limitSpring, newSize);
            limitDamp = Utility.expand(limitDamp, newSize);

            System.out.println("⚠️ [BeamContainer] Resized to: " + newSize);
        }
    }

    public void addBeam(int beamIndex1, int beamIndex2, double nodeDist, double beamSpring, double beamDamp, double beamDeform, double beamStrength,
                        int beamType, double precomp, double beamShortBound, double beamLongBound,
                        double shortBoundRange, double longBoundRange,
                        double beamLimitSpring, double beamLimitDamp) {
        ensureBeamCapacity();

        node1[count] = beamIndex1;
        node2[count] = beamIndex2;

        // Resting length = initial distance * pre-compression factor
        // If precomp is 0.8, this spring will contract as much as possible the moment it is created!
        restLength[count] = nodeDist * precomp;
        baseRestLength[count] = restLength[count];

        spring[count] = beamSpring;
        damp[count] = beamDamp;
        deform[count] = beamDeform;
        strength[count] = beamStrength;

        // If a Range (in absolute meters) is provided, convert it directly to a percentage!
        if (shortBoundRange >= 0) beamShortBound = shortBoundRange / nodeDist;
        if (longBoundRange >= 0) beamLongBound = longBoundRange / nodeDist;

        type[count] = beamType;
        shortBound[count] = beamShortBound;
        longBound[count] = beamLongBound;
        limitSpring[count] = beamLimitSpring;
        limitDamp[count] = beamLimitDamp;

        broken[count] = false;

        count++;
    }

    public void clear() {
        count = 0;
        reset();
    }

    public void reset(){
        for (int i = 0; i < count; i++) {
            broken[i] = false;
            restLength[i] = baseRestLength[i];
        }
    }
}