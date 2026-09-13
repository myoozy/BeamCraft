package me.mzy.beamcraft.client.physics;

import me.mzy.beamcraft.utility.Utility;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * 限界梁容器：在普通梁基础上增加长度限界、极限反弹力以及复杂阻尼模型。
 *
 * <p>Besides the runtime coefficients the container keeps the <em>authored</em>
 * damping values and the stability ceiling the assembled configuration admits.
 * An actuator (see {@code AdaptiveDamperActuators}) derives the runtime value as
 * {@code authored * coefficient} and clamps it back to that ceiling, so a mode
 * change can never compound and a hard-to-soft switch cannot lose the authored
 * value.
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
    public float[] dampVelocitySplit; // 速度分界点（压缩方向，也是回弹方向的回退值）
    public float[] dampVelocitySplitRebound; // 回弹（伸长）方向的速度分界点覆盖值
    public float[] dampFast;        // 高速阻尼
    public float[] dampRebound;     // 回弹阻尼
    public float[] dampReboundFast; // 高速回弹阻尼

    // --- 致动器（自适应阻尼）支撑数据 ---
    /** Immutable authored counterparts of the runtime damping channels above. */
    public float[] authoredDamp;
    public float[] authoredDampRebound;
    public float[] authoredDampFast;
    public float[] authoredDampReboundFast;
    public float[] authoredDampVelocitySplit;
    /**
     * Largest value a damping channel may take after the cutoff-aware
     * directional stability limiter has run; {@link Float#MAX_VALUE} until the
     * assembled-part lifecycle computes it.
     */
    public float[] dampStabilityCeiling;

    /** Name -> bounded-beam indices, rebuilt once per assembly. Never mutated afterwards. */
    private Map<String, int[]> nameIndex = Map.of();

    private static final int[] EMPTY_INDICES = new int[0];

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
        dampVelocitySplitRebound = new float[INIT_BEAM_CAP];
        dampFast = new float[INIT_BEAM_CAP];
        dampRebound = new float[INIT_BEAM_CAP];
        dampReboundFast = new float[INIT_BEAM_CAP];
        authoredDamp = new float[INIT_BEAM_CAP];
        authoredDampRebound = new float[INIT_BEAM_CAP];
        authoredDampFast = new float[INIT_BEAM_CAP];
        authoredDampReboundFast = new float[INIT_BEAM_CAP];
        authoredDampVelocitySplit = new float[INIT_BEAM_CAP];
        dampStabilityCeiling = new float[INIT_BEAM_CAP];
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
        dampVelocitySplitRebound = Utility.expand(dampVelocitySplitRebound, newSize);
        dampFast = Utility.expand(dampFast, newSize);
        dampRebound = Utility.expand(dampRebound, newSize);
        dampReboundFast = Utility.expand(dampReboundFast, newSize);
        authoredDamp = Utility.expand(authoredDamp, newSize);
        authoredDampRebound = Utility.expand(authoredDampRebound, newSize);
        authoredDampFast = Utility.expand(authoredDampFast, newSize);
        authoredDampReboundFast = Utility.expand(authoredDampReboundFast, newSize);
        authoredDampVelocitySplit = Utility.expand(authoredDampVelocitySplit, newSize);
        dampStabilityCeiling = Utility.expand(dampStabilityCeiling, newSize);
    }

    /**
     * Returns every bounded-beam index carrying {@code beamName}, in build order;
     * empty when the name is unknown. A duplicate authored name therefore targets
     * all of its beams. The returned array belongs to the index and must not be
     * mutated by callers.
     */
    public int[] indicesForName(String beamName) {
        int[] indices = beamName == null ? null : nameIndex.get(beamName);
        return indices == null ? EMPTY_INDICES : indices;
    }

    /**
     * Rebuilds the name lookup from the currently registered beams. Called once at
     * the end of assembly, after which the map is immutable and safe to read from
     * any thread.
     */
    public void rebuildNameIndex() {
        Map<String, List<Integer>> grouped = new HashMap<>();
        for (int i = 0; i < count; i++) {
            String beamName = name[i];
            if (beamName == null) continue;
            grouped.computeIfAbsent(beamName, key -> new ArrayList<>(2)).add(i);
        }
        Map<String, int[]> rebuilt = new HashMap<>(Math.max(4, grouped.size() * 2));
        for (Map.Entry<String, List<Integer>> entry : grouped.entrySet()) {
            List<Integer> values = entry.getValue();
            int[] indices = new int[values.size()];
            for (int i = 0; i < indices.length; i++) indices[i] = values.get(i);
            rebuilt.put(entry.getKey(), indices);
        }
        nameIndex = Map.copyOf(rebuilt);
    }

    /**
     * 添加限界梁（包含所有特有参数）。
     * @param inDampVelSplit   速度分界点，<0 时使用 Float.MAX_VALUE（表示不启用高速阻尼）
     * @param inDampVelSplitRebound 回弹方向的速度分界点，<0 时回退至 inDampVelSplit
     * @param inDampFast       高速阻尼，<0 时回退至普通阻尼
     * @param inDampRebound     回弹阻尼，<0 时回退至普通阻尼
     * @param inDampReboundFast 高速回弹阻尼，<0 时回退至 inDampRebound
     */
    public int addBeam(PhysicsSpecs.BeamSpec spec, int node1Idx, int node2Idx, float nodeDist) {
        float finalVelSplit = spec.dampVelSplit() < 0.0f ? Float.MAX_VALUE : spec.dampVelSplit();
        // beamDampVelocitySplitRebound only replaces the split while the beam lengthens;
        // compression always uses the common beamDampVelocitySplit.
        float finalVelSplitRebound = spec.dampVelSplitRebound() < 0.0f
                ? finalVelSplit : spec.dampVelSplitRebound();
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
        dampVelocitySplitRebound[idx] = finalVelSplitRebound;
        dampFast[idx] = finalFast;
        dampRebound[idx] = finalRebound;
        dampReboundFast[idx] = finalReboundFast;

        // The authored values are the immutable base every actuator re-derives from.
        authoredDamp[idx] = spec.damp();
        authoredDampRebound[idx] = finalRebound;
        authoredDampFast[idx] = finalFast;
        authoredDampReboundFast[idx] = finalReboundFast;
        authoredDampVelocitySplit[idx] = finalVelSplit;
        // Until the assembled-part lifecycle computes the cutoff-aware safety
        // ceiling, a mode change must not clip the authored coefficients.
        dampStabilityCeiling[idx] = Float.MAX_VALUE;

        return idx;
    }

    @Override
    public void clear() {
        super.clear();
        nameIndex = Map.of();
    }
}
