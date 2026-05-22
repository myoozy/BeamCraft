package me.mzy.beamcraft.client.physics;

import me.mzy.beamcraft.utility.Utility;

/**
 * 限界梁容器：在普通梁基础上增加长度限界、极限反弹力以及复杂阻尼模型。
 */
public class BoundedBeamContainer extends BeamContainer {
    // 限界与阻尼特有属性
    public double[] shortBoundRange;      // 最短极限长度
    public double[] longBoundRange;       // 最长极限长度
    public double[] shortBound;           // 最短极限比例
    public double[] longBound;            // 最长极限比例
    public double[] limitSpring;     // 极限反弹力
    public double[] limitDamp;       // 极限阻尼
    public double[] dampVelocitySplit; // 速度分界点
    public double[] dampFast;        // 高速阻尼
    public double[] dampRebound;     // 回弹阻尼
    public double[] dampReboundFast; // 高速回弹阻尼

    private static final double KINDA_BIG_NUMBER = 1e9;

    public BoundedBeamContainer() {
        super();
        shortBoundRange = new double[INIT_BEAM_CAP];
        longBoundRange = new double[INIT_BEAM_CAP];
        shortBound =  new double[INIT_BEAM_CAP];
        longBound =  new double[INIT_BEAM_CAP];
        limitSpring = new double[INIT_BEAM_CAP];
        limitDamp = new double[INIT_BEAM_CAP];
        dampVelocitySplit = new double[INIT_BEAM_CAP];
        dampFast = new double[INIT_BEAM_CAP];
        dampRebound = new double[INIT_BEAM_CAP];
        dampReboundFast = new double[INIT_BEAM_CAP];
    }

    @Override
    protected void resize(int newSize) {
        super.resize(newSize);
        shortBoundRange = Utility.expand(shortBoundRange, newSize);
        longBoundRange = Utility.expand(longBoundRange, newSize);
        shortBound = Utility.expand(shortBound, newSize);
        longBound = Utility.expand(longBound, newSize);
        limitSpring = Utility.expand(limitSpring, newSize);
        limitDamp = Utility.expand(limitDamp, newSize);
        dampVelocitySplit = Utility.expand(dampVelocitySplit, newSize);
        dampFast = Utility.expand(dampFast, newSize);
        dampRebound = Utility.expand(dampRebound, newSize);
        dampReboundFast = Utility.expand(dampReboundFast, newSize);
    }

    /**
     * 添加限界梁（包含所有特有参数）。
     * @param inDampVelSplit   速度分界点，<0 时使用极大值（表示禁用高速阻尼）
     * @param inDampFast       高速阻尼，<0 时回退至普通阻尼
     * @param inDampRebound     回弹阻尼，<0 时回退至普通阻尼
     * @param inDampReboundFast 高速回弹阻尼，<0 时回退至 inDampRebound
     */
    public int addBeam(java.util.List<String> breakGroups,
                        int node1Idx, int node2Idx, double nodeDist,
                        double beamSpring, double beamDamp,
                        double beamDeform, double beamStrength,
                        double precomp, double precompRange, double precompTime,
                        double beamShortBound, double beamLongBound,
                        double beamShortBoundRange, double beamLongBoundRange,
                        double beamLimitSpring, double beamLimitDamp,
                        double inDampVelSplit, double inDampFast,
                        double inDampRebound, double inDampReboundFast) {
        // 复杂阻尼默认值处理
        double finalVelSplit = (inDampVelSplit < 0) ? KINDA_BIG_NUMBER : inDampVelSplit;
        double finalFast = (inDampFast < 0) ? beamDamp : inDampFast;
        double finalRebound = (inDampRebound < 0) ? beamDamp : inDampRebound;
        double finalReboundFast = (inDampReboundFast < 0) ? finalRebound : inDampReboundFast;

        // 复用父类公共属性的添加
        int idx = addBeamInternal(breakGroups, node1Idx, node2Idx, nodeDist,
                beamSpring, beamDamp, beamDeform, beamStrength,
                precomp, precompRange, precompTime);

        shortBoundRange[idx] = beamShortBoundRange;
        longBoundRange[idx] = beamLongBoundRange;
        shortBound[idx] = beamShortBound;
        longBound[idx] = beamLongBound;
        
        limitSpring[idx] = beamLimitSpring;
        limitDamp[idx] = beamLimitDamp;
        dampVelocitySplit[idx] = finalVelSplit;
        dampFast[idx] = finalFast;
        dampRebound[idx] = finalRebound;
        dampReboundFast[idx] = finalReboundFast;

        return idx;
    }
}