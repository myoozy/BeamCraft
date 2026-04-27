package me.mzy.beamcraft.physics;

import me.mzy.beamcraft.utility.Utility;

public class BeamContainer {
    public static final int INIT_BEAM_CAP = 256;

    // --- 梁类型枚举 ---
    public static final int BEAM_NORMAL = 0;
    public static final int BEAM_SUPPORT = 1;
    public static final int BEAM_BOUNDED = 2;

    // 位掩码定义
    public static final int MASK_TYPE = 0xFF; // 取低8位作为真实类型
    public static final int FLAG_HAS_COMPLEX_DAMP = 1 << 8; // 第9位作为复杂阻尼开关
    public static final int FLAG_HAS_BOUND = 1 << 9;        // 第10位作为限界开关

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
    public double[] dampVelocitySplit = new double[INIT_BEAM_CAP];
    public double[] dampFast = new double[INIT_BEAM_CAP];
    public double[] dampRebound = new double[INIT_BEAM_CAP];
    public double[] dampReboundFast = new double[INIT_BEAM_CAP];

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
            dampVelocitySplit = Utility.expand(dampVelocitySplit, newSize);
            dampFast = Utility.expand(dampFast, newSize);
            dampRebound = Utility.expand(dampRebound, newSize);
            dampReboundFast = Utility.expand(dampReboundFast, newSize);

            System.out.println("⚠️ [BeamContainer] Resized to: " + newSize);
        }
    }

    public void addBeam(int beamIndex1, int beamIndex2, double nodeDist, double beamSpring, double beamDamp,
                        double beamDeform, double beamStrength, int beamType, double precomp,
                        double beamShortBound, double beamLongBound,
                        double shortBoundRange, double longBoundRange,
                        double beamLimitSpring, double beamLimitDamp,
                        double inDampVelSplit, double inDampFast, double inDampRebound, double inDampReboundFast) {
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

        // 💥 脏数据清洗逻辑：如果没有提供特定阻尼，就向下回退到基础阻尼
        dampVelocitySplit[count] = (inDampVelSplit < 0) ? PhysicsWorld.KINDA_BIG_NUMBER : inDampVelSplit;
        dampFast[count] = (inDampFast < 0) ? beamDamp : inDampFast;
        dampRebound[count] = (inDampRebound < 0) ? beamDamp : inDampRebound;
        // FastRebound 如果没填，优先回退到普通的 Rebound
        dampReboundFast[count] = (inDampReboundFast < 0) ? dampRebound[count] : inDampReboundFast;

        // 确定真实类型和两个开关状态
        int finalType = beamType & MASK_TYPE;
        boolean bnd = (beamLimitSpring > 0) || (beamShortBound != 1.0) || (beamLongBound != 1.0) || (beamType == BEAM_BOUNDED);
        boolean cpx = (inDampVelSplit >= 0 && inDampVelSplit < Double.MAX_VALUE);

        // 位运算合并（白嫖内存！）
        if (bnd) finalType |= FLAG_HAS_BOUND;
        if (cpx) finalType |= FLAG_HAS_COMPLEX_DAMP;

        type[count] = finalType;

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