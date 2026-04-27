package me.mzy.beamcraft.physics;

import me.mzy.beamcraft.utility.Utility;

public class BeamContainer {
    public static final int INIT_BEAM_CAP = 256;

    // --- 梁类型枚举 & 位掩码 ---
    public static final int BEAM_NORMAL = 0;
    public static final int BEAM_SUPPORT = 1;
    public static final int BEAM_BOUNDED = 2;
    public static final int MASK_TYPE = 0xFF;
    public static final int FLAG_HAS_COMPLEX_DAMP = 1 << 8;
    public static final int FLAG_HAS_BOUND = 1 << 9;

    // ==========================================
    // 内存布局定义 (AoS 步长对齐到 2的幂)
    // ==========================================

    // int 型数据：每根梁占 4 个格子 (步长 4 -> 移位 2)
    public static final int INT_STRIDE = 4;
    public static final int I_NODE1 = 0;
    public static final int I_NODE2 = 1;
    public static final int I_TYPE  = 2;
    // 第 4 个格子留空占位，确保对齐

    // double 型数据：每根梁占 16 个格子 (步长 16 -> 移位 4)
    public static final int DBL_STRIDE = 16;
    public static final int D_REST_LEN     = 0;
    public static final int D_SPRING       = 1;
    public static final int D_DAMP         = 2;
    public static final int D_DEFORM       = 3;
    public static final int D_STRENGTH     = 4;
    public static final int D_SHORT_BOUND  = 5;
    public static final int D_LONG_BOUND   = 6;
    public static final int D_LIMIT_SPRING = 7;
    public static final int D_LIMIT_DAMP   = 8;
    public static final int D_DAMP_SPLIT   = 9;
    public static final int D_DAMP_FAST    = 10;
    public static final int D_DAMP_REB     = 11;
    public static final int D_DAMP_REB_FAST= 12;
    // 13, 14, 15 留空占位，确保对齐

    public int count = 0;

    // 只有这三个数组！预取器彻底解放！
    public int[] intData = new int[INIT_BEAM_CAP * INT_STRIDE];
    public double[] dblData = new double[INIT_BEAM_CAP * DBL_STRIDE];
    public boolean[] broken = new boolean[INIT_BEAM_CAP]; // broken 频繁读写，保留单独数组防止弄脏双精度缓存行

    // 满血复活记录 (这个只在 reset 时用，不需要放进热循环)
    public double[] baseRestLength = new double[INIT_BEAM_CAP];

    private void ensureBeamCapacity() {
        if (count * INT_STRIDE >= intData.length) {
            int newCap = (count * 2);
            intData = Utility.expand(intData, newCap * INT_STRIDE);
            dblData = Utility.expand(dblData, newCap * DBL_STRIDE);
            broken = Utility.expand(broken, newCap);
            baseRestLength = Utility.expand(baseRestLength, newCap);
            System.out.println("⚠️ [BeamContainer] AoS Resized to capacity: " + newCap);
        }
    }

    public void addBeam(int beamIndex1, int beamIndex2, double nodeDist, double beamSpring, double beamDamp, double beamDeform, double beamStrength,
                        int beamType, double precomp, double beamShortBound, double beamLongBound,
                        double shortBoundRange, double longBoundRange,
                        double beamLimitSpring, double beamLimitDamp,
                        double inDampVelSplit, double inDampFast, double inDampRebound, double inDampReboundFast) {
        ensureBeamCapacity();

        int iBase = count << 2; // count * 4
        int dBase = count << 4; // count * 16

        // 1. 写入 int 数据
        intData[iBase + I_NODE1] = beamIndex1;
        intData[iBase + I_NODE2] = beamIndex2;

        boolean bnd = (beamLimitSpring > 0) || (beamShortBound != 1.0) || (beamLongBound != 1.0) || (beamType == BEAM_BOUNDED);
        boolean cpx = (inDampVelSplit >= 0 && inDampVelSplit < Double.MAX_VALUE);
        int finalType = beamType & MASK_TYPE;
        if (bnd) finalType |= FLAG_HAS_BOUND;
        if (cpx) finalType |= FLAG_HAS_COMPLEX_DAMP;
        intData[iBase + I_TYPE] = finalType;

        // 2. 写入 double 数据
        double rLen = nodeDist * precomp;
        dblData[dBase + D_REST_LEN] = rLen;
        baseRestLength[count] = rLen;

        dblData[dBase + D_SPRING]   = beamSpring;
        dblData[dBase + D_DAMP]     = beamDamp;
        dblData[dBase + D_DEFORM]   = beamDeform;
        dblData[dBase + D_STRENGTH] = beamStrength;

        if (shortBoundRange >= 0) beamShortBound = shortBoundRange / nodeDist;
        if (longBoundRange >= 0) beamLongBound = longBoundRange / nodeDist;
        dblData[dBase + D_SHORT_BOUND]  = beamShortBound;
        dblData[dBase + D_LONG_BOUND]   = beamLongBound;
        dblData[dBase + D_LIMIT_SPRING] = beamLimitSpring;
        dblData[dBase + D_LIMIT_DAMP]   = beamLimitDamp;

        dblData[dBase + D_DAMP_SPLIT]   = (inDampVelSplit < 0) ? Double.MAX_VALUE : inDampVelSplit;
        dblData[dBase + D_DAMP_FAST]    = (inDampFast < 0) ? beamDamp : inDampFast;
        dblData[dBase + D_DAMP_REB]     = (inDampRebound < 0) ? beamDamp : inDampRebound;
        dblData[dBase + D_DAMP_REB_FAST]= (inDampReboundFast < 0) ? ((inDampRebound < 0) ? beamDamp : inDampRebound) : inDampReboundFast;

        broken[count] = false;
        count++;
    }

    public void clear() {
        count = 0;
    }

    public void reset(){
        for (int i = 0; i < count; i++) {
            broken[i] = false;
            dblData[(i << 4) + D_REST_LEN] = baseRestLength[i];
        }
    }
}