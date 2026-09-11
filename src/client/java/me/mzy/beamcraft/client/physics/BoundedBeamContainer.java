package me.mzy.beamcraft.client.physics;

import me.mzy.beamcraft.utility.Utility;

/**
 * 限界梁容器：在普通梁基础上增加长度限界、极限反弹力以及复杂阻尼模型。
 */
public class BoundedBeamContainer extends BeamContainer {
    // 限界与阻尼特有属性
    public float[] shortBoundRange;      // 最短极限长度
    public float[] longBoundRange;       // 最长极限长度
    public float[] shortBound;           // 最短极限比例
    public float[] longBound;            // 最长极限比例
    public float[] boundZone;       // 从普通属性过渡到极限属性的穿透距离（米）
    public float[] limitSpring;     // 极限反弹力
    public float[] limitDamp;       // 极限阻尼（长度收缩方向）
    public float[] limitDampRebound; // 极限回弹阻尼（长度增长方向）
    public float[] dampVelocitySplit; // 速度分界点
    public float[] dampFast;        // 高速阻尼
    public float[] dampRebound;     // 回弹阻尼
    public float[] dampReboundFast; // 高速回弹阻尼

    public BoundedBeamContainer() {
        super();
        shortBoundRange = new float[INIT_BEAM_CAP];
        longBoundRange = new float[INIT_BEAM_CAP];
        shortBound =  new float[INIT_BEAM_CAP];
        longBound =  new float[INIT_BEAM_CAP];
        boundZone = new float[INIT_BEAM_CAP];
        limitSpring = new float[INIT_BEAM_CAP];
        limitDamp = new float[INIT_BEAM_CAP];
        limitDampRebound = new float[INIT_BEAM_CAP];
        dampVelocitySplit = new float[INIT_BEAM_CAP];
        dampFast = new float[INIT_BEAM_CAP];
        dampRebound = new float[INIT_BEAM_CAP];
        dampReboundFast = new float[INIT_BEAM_CAP];
    }

    @Override
    protected void resize(int newSize) {
        super.resize(newSize);
        shortBoundRange = Utility.expand(shortBoundRange, newSize);
        longBoundRange = Utility.expand(longBoundRange, newSize);
        shortBound = Utility.expand(shortBound, newSize);
        longBound = Utility.expand(longBound, newSize);
        boundZone = Utility.expand(boundZone, newSize);
        limitSpring = Utility.expand(limitSpring, newSize);
        limitDamp = Utility.expand(limitDamp, newSize);
        limitDampRebound = Utility.expand(limitDampRebound, newSize);
        dampVelocitySplit = Utility.expand(dampVelocitySplit, newSize);
        dampFast = Utility.expand(dampFast, newSize);
        dampRebound = Utility.expand(dampRebound, newSize);
        dampReboundFast = Utility.expand(dampReboundFast, newSize);
    }

    /**
     * 添加限界梁（包含所有特有参数）。
     * @param inDampVelSplit   速度分界点，<0 时使用 Float.MAX_VALUE（表示不启用高速阻尼）
     * @param inDampFast       高速阻尼，<0 时回退至普通阻尼
     * @param inDampRebound     回弹阻尼，<0 时回退至普通阻尼
     * @param inDampReboundFast 高速回弹阻尼，<0 时回退至 inDampRebound
     */
    public int addBeam(PhysicsSpecs.BeamSpec spec, int node1Idx, int node2Idx, float nodeDist) {
        float finalVelSplit = spec.dampVelSplit() < 0.0f ? Float.MAX_VALUE : spec.dampVelSplit();
        // beamLimitDampRebound is unspecified by default and falls back to beamLimitDamp.
        float finalLimitRebound = spec.limitDampRebound() < 0.0f
                ? spec.limitDamp() : spec.limitDampRebound();
        float finalFast = spec.dampFast() < 0.0f ? spec.damp() : spec.dampFast();
        float finalRebound = spec.dampRebound() < 0.0f ? spec.damp() : spec.dampRebound();
        float finalReboundFast = spec.dampReboundFast() < 0.0f ? finalRebound : spec.dampReboundFast();

        int idx = addBeamInternal(spec, node1Idx, node2Idx, nodeDist);

        shortBoundRange[idx] = spec.shortBoundRange();
        longBoundRange[idx] = spec.longBoundRange();
        shortBound[idx] = spec.shortBound();
        longBound[idx] = spec.longBound();
        boundZone[idx] = spec.boundZone();
        limitSpring[idx] = spec.limitSpring();
        limitDamp[idx] = spec.limitDamp();
        limitDampRebound[idx] = finalLimitRebound;
        dampVelocitySplit[idx] = finalVelSplit;
        dampFast[idx] = finalFast;
        dampRebound[idx] = finalRebound;
        dampReboundFast[idx] = finalReboundFast;

        return idx;
    }
}
