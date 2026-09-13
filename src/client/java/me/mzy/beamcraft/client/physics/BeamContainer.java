package me.mzy.beamcraft.client.physics;

import me.mzy.beamcraft.utility.Utility;

import java.util.HashMap;

public class BeamContainer {
    public static final int INIT_BEAM_CAP = 256;

    /**
     * Floor on the filter's high-frequency gain when it scales the stability
     * ceiling, i.e. an authored {@code dampCutoffHz} may raise the damping ceiling
     * by at most {@code 1 / 0.02} (50x). A very low cutoff legitimately removes
     * nearly all high-frequency damping so the mathematical ceiling is unbounded
     * there; this is a numerical guard against pathological authoring, not a
     * physical limit.
     */
    static final float MIN_CUTOFF_HIGH_FREQUENCY_GAIN = 0.02f;

    // --- 梁类型枚举 ---
    public static final int BEAM_NORMAL = 0;
    public static final int BEAM_SUPPORT = 1;
    public static final int BEAM_BOUNDED = 2;
    public static final int BEAM_LBEAM = 3;
    public static final int BEAM_HYDRO = 4;
    public static final int BEAM_ANISOTROPIC = 5;

    public java.util.List<String>[] assignedBreakGroups;
    public java.util.List<String>[] assignedDeformGroups;

    public int count = 0;
    public int[] node1;
    public int[] node2;
    public float[] restLength;
    public float[] baseRestLength;
    public float[] targetRestLength;
    /** Runtime multiplier supplied by an attached actuator; 1 for ordinary beams. */
    public float[] actuationRatio;
    public float[] precompTimeTotal;
    public float[] precompTimer;
    public float[] spring;
    public float[] damp;
    public float[] deform;
    public float[] baseDeform;
    public float[] deformLimitStress;
    public float[] maxDeform;
    public float[] strength;
    public boolean[] broken;
    /**
     * Direct failures detected during the current internal-force phase.
     * Topology side effects are committed only after every beam family has
     * evaluated the same start-of-substep topology.
     */
    private boolean[] pendingBreak;
    private int[] pendingBreakIndices;
    private int pendingBreakCount;
    public int[] breakGroupType;
    public float[] deformationTriggerRatio;
    public boolean[] deformGroupTriggered;
    private int[] deformTriggerIndices;
    private int deformTriggerCount;
    public boolean[] disableTriangleBreaking;
    public int[] wheelId;
    /**
     * Optional authored BeamNG beam {@code name}; {@code null} for unnamed beams.
     * Actuators address beams through this name rather than through a raw index.
     */
    public String[] name;
    /** BeamNG {@code dampCutoffHz}; {@code <= 0} disables the filter (raw relative velocity). */
    public float[] dampCutoffHz;
    /** Persistent one-pole filter state: the low-passed axial relative velocity. */
    public float[] dampFilterState;
    /** Smoothing coefficient cached for the current sub-step dt. */
    public float[] dampFilterAlpha;
    private float dampFilterAlphaDt = -1.0f;
    /**
     * Whether this family honours {@code dampCutoffHz}. BeamNG documents the
     * property for normal, bounded and L-beams only, so support and anisotropic
     * beams keep their raw relative-velocity damping exactly as before.
     */
    private final boolean dampingCutoffSupported;

    public BeamContainer() {
        this(true);
    }

    BeamContainer(boolean dampingCutoffSupported) {
        this.dampingCutoffSupported = dampingCutoffSupported;
        node1 = new int[INIT_BEAM_CAP];
        node2 = new int[INIT_BEAM_CAP];
        restLength = new float[INIT_BEAM_CAP];
        baseRestLength = new float[INIT_BEAM_CAP];
        targetRestLength = new float[INIT_BEAM_CAP];
        actuationRatio = new float[INIT_BEAM_CAP];
        precompTimeTotal = new float[INIT_BEAM_CAP];
        precompTimer = new float[INIT_BEAM_CAP];
        spring = new float[INIT_BEAM_CAP];
        damp = new float[INIT_BEAM_CAP];
        deform = new float[INIT_BEAM_CAP];
        baseDeform = new float[INIT_BEAM_CAP];
        deformLimitStress = new float[INIT_BEAM_CAP];
        maxDeform = new float[INIT_BEAM_CAP];
        strength = new float[INIT_BEAM_CAP];
        broken = new boolean[INIT_BEAM_CAP];
        pendingBreak = new boolean[INIT_BEAM_CAP];
        pendingBreakIndices = new int[INIT_BEAM_CAP];
        breakGroupType = new int[INIT_BEAM_CAP];
        deformationTriggerRatio = new float[INIT_BEAM_CAP];
        deformGroupTriggered = new boolean[INIT_BEAM_CAP];
        deformTriggerIndices = new int[INIT_BEAM_CAP];
        disableTriangleBreaking = new boolean[INIT_BEAM_CAP];
        assignedBreakGroups = new java.util.List[INIT_BEAM_CAP];
        assignedDeformGroups = new java.util.List[INIT_BEAM_CAP];
        wheelId = new int[INIT_BEAM_CAP];
        name = new String[INIT_BEAM_CAP];
        dampCutoffHz = new float[INIT_BEAM_CAP];
        dampFilterState = new float[INIT_BEAM_CAP];
        dampFilterAlpha = new float[INIT_BEAM_CAP];
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
        assignedDeformGroups = java.util.Arrays.copyOf(assignedDeformGroups, newSize);
        node1 = Utility.expand(node1, newSize);
        node2 = Utility.expand(node2, newSize);
        restLength = Utility.expand(restLength, newSize);
        baseRestLength = Utility.expand(baseRestLength, newSize);
        targetRestLength = Utility.expand(targetRestLength, newSize);
        actuationRatio = Utility.expand(actuationRatio, newSize);
        precompTimeTotal = Utility.expand(precompTimeTotal, newSize);
        precompTimer = Utility.expand(precompTimer, newSize);
        spring = Utility.expand(spring, newSize);
        damp = Utility.expand(damp, newSize);
        deform = Utility.expand(deform, newSize);
        baseDeform = Utility.expand(baseDeform, newSize);
        deformLimitStress = Utility.expand(deformLimitStress, newSize);
        maxDeform = Utility.expand(maxDeform, newSize);
        strength = Utility.expand(strength, newSize);
        broken = Utility.expand(broken, newSize);
        pendingBreak = Utility.expand(pendingBreak, newSize);
        pendingBreakIndices = Utility.expand(pendingBreakIndices, newSize);
        breakGroupType = Utility.expand(breakGroupType, newSize);
        deformationTriggerRatio = Utility.expand(deformationTriggerRatio, newSize);
        deformGroupTriggered = Utility.expand(deformGroupTriggered, newSize);
        deformTriggerIndices = Utility.expand(deformTriggerIndices, newSize);
        disableTriangleBreaking = Utility.expand(disableTriangleBreaking, newSize);
        wheelId = Utility.expand(wheelId, newSize);
        name = java.util.Arrays.copyOf(name, newSize);
        dampCutoffHz = Utility.expand(dampCutoffHz, newSize);
        dampFilterState = Utility.expand(dampFilterState, newSize);
        dampFilterAlpha = Utility.expand(dampFilterAlpha, newSize);
    }

    protected int addBeamInternal(PhysicsSpecs.BeamSpec spec, int node1Idx, int node2Idx, float nodeDist) {
        ensureCapacity();
        int idx = this.count;

        if (spec.breakGroups() != null && !spec.breakGroups().isEmpty()) {
            this.assignedBreakGroups[count] = new java.util.ArrayList<>(spec.breakGroups());
        } else {
            this.assignedBreakGroups[count] = null;
        }
        if (spec.deformGroups() != null && !spec.deformGroups().isEmpty()) {
            this.assignedDeformGroups[count] = new java.util.ArrayList<>(spec.deformGroups());
        } else {
            this.assignedDeformGroups[count] = null;
        }

        this.node1[idx] = node1Idx;
        this.node2[idx] = node2Idx;

        // An authored precompressionRange (metric delta, including an explicit 0 or a
        // negative value) replaces the beamPrecompression multiplier entirely; an
        // unauthored one leaves the multiplier in effect.
        float targetLen = spec.precompRangeDefined()
                ? nodeDist + spec.precompRange()
                : nodeDist * spec.precomp();
        this.targetRestLength[idx] = targetLen;

        if (spec.precompTime() > 0.0f) {
            this.restLength[idx] = nodeDist;
            this.precompTimer[idx] = spec.precompTime();
            this.precompTimeTotal[idx] = spec.precompTime();
        } else {
            this.restLength[idx] = targetLen;
            this.precompTimer[idx] = 0.0f;
            this.precompTimeTotal[idx] = 0.0f;
        }

        this.baseRestLength[idx] = this.restLength[idx];
        this.actuationRatio[idx] = 1.0f;
        this.spring[idx] = spec.spring();
        this.damp[idx] = spec.damp();
        this.deform[idx] = spec.deform();
        this.baseDeform[idx] = spec.deform();
        this.deformLimitStress[idx] = spec.deformLimitStress();
        this.strength[idx] = spec.strength();
        float hardeningLimit = spec.deform() + Math.max(0.0f, spec.deformLimitStress());
        this.maxDeform[idx] = Math.min(spec.strength(), hardeningLimit);
        this.broken[idx] = false;
        this.pendingBreak[idx] = false;
        this.breakGroupType[idx] = spec.breakGroupType();
        this.deformationTriggerRatio[idx] = spec.deformationTriggerRatio();
        this.deformGroupTriggered[idx] = false;
        if (this.assignedDeformGroups[idx] != null
                && Float.isFinite(spec.deformationTriggerRatio())
                && spec.deformationTriggerRatio() >= 0.0f) {
            this.deformTriggerIndices[deformTriggerCount++] = idx;
        }
        this.disableTriangleBreaking[idx] = spec.disableTriangleBreaking();
        this.wheelId[idx] = -1;
        this.name[idx] = spec.name() == null || spec.name().isEmpty() ? null : spec.name();

        this.dampCutoffHz[idx] = spec.dampCutoffHz();
        this.dampFilterState[idx] = 0.0f;
        this.dampFilterAlpha[idx] = dampingCutoffSupported && dampFilterAlphaDt > 0.0f
                ? dampingCutoffAlpha(spec.dampCutoffHz(), dampFilterAlphaDt)
                : 0.0f;

        count++;
        return idx;
    }

    public int addBeam(PhysicsSpecs.BeamSpec spec, int node1Idx, int node2Idx, float nodeDist) {
        return addBeamInternal(spec, node1Idx, node2Idx, nodeDist);
    }

    public void bindToTire(int beamIdx, int wheelIdx) {
        if (0 <= beamIdx && beamIdx <= count - 1) {
            wheelId[beamIdx] = wheelIdx;
        }
    }

    public void clear() {
        reset();
        count = 0;
        deformTriggerCount = 0;
    }

    /**
     * Causal one-pole smoothing coefficient for {@code dampCutoffHz}.
     *
     * <p>The filter solves {@code dy/dt = (x - y) / tau} with
     * {@code tau = 1 / (2 pi fc)}; its exact per-step solution is
     * {@code y += alpha * (x - y)} with {@code alpha = 1 - exp(-2 pi fc dt)}.
     * That makes it unconditionally stable for any dt and cutoff, unit DC gain
     * (steady state is never attenuated), free of step overshoot, and timestep
     * consistent: two steps of {@code dt} equal one step of {@code 2 dt}.</p>
     */
    static float dampingCutoffAlpha(float cutoffHz, float dt) {
        if (!(cutoffHz > 0.0f) || !(dt > 0.0f)) return 0.0f;
        return (float) Math.clamp(-Math.expm1(-2.0 * Math.PI * cutoffHz * dt), 0.0, 1.0);
    }

    /** True when beam {@code i} authored a positive {@code dampCutoffHz} this family honours. */
    public boolean dampingCutoffEnabled(int i) {
        return dampingCutoffSupported && dampCutoffHz[i] > 0.0f;
    }

    /** Recomputes the cached filter coefficients when the sub-step dt changes. */
    void prepareDampingFilters(float dt) {
        if (dt == dampFilterAlphaDt) return;
        dampFilterAlphaDt = dt;
        for (int i = 0; i < count; i++) {
            dampFilterAlpha[i] = dampingCutoffAlpha(dampCutoffHz[i], dt);
        }
    }

    /**
     * Advances beam {@code i}'s damping filter by one sub-step and returns the
     * velocity the damping force must use. Only call when
     * {@link #dampingCutoffEnabled(int)}; disabled beams keep the raw velocity.
     */
    float filteredDampingVelocity(int i, float relVel) {
        float alpha = dampFilterAlpha[i];
        float filtered = dampFilterState[i] + alpha * (relVel - dampFilterState[i]);
        dampFilterState[i] = filtered;
        return filtered;
    }

    /**
     * Steady-state gain the filter applies to the alternating (Nyquist) velocity
     * the linear stability budget is built around, {@code alpha / (2 - alpha)}.
     * Returns 1 when the filter is disabled, so cutoff-free beams keep today's
     * stability accounting exactly.
     */
    public float cutoffHighFrequencyGain(int i, float dt) {
        if (!dampingCutoffEnabled(i)) return 1.0f;
        float alpha = dampingCutoffAlpha(dampCutoffHz[i], dt);
        return Math.max(MIN_CUTOFF_HIGH_FREQUENCY_GAIN, alpha / (2.0f - alpha));
    }

    /** Clears the filter state so a reset cannot inject a startup transient. */
    void resetDampingFilters() {
        java.util.Arrays.fill(dampFilterState, 0, count, 0.0f);
    }

    public void reset() {
        clearPendingBreaks();
        resetDampingFilters();
        for (int i = 0; i < count; i++) {
            broken[i] = false;
            deformGroupTriggered[i] = false;
            restLength[i] = baseRestLength[i];
            actuationRatio[i] = 1.0f;
            deform[i] = baseDeform[i];
            precompTimer[i] = precompTimeTotal[i];
        }
    }

    int deformTriggerCount() {
        return deformTriggerCount;
    }

    int deformTriggerIndex(int triggerIndex) {
        return deformTriggerIndices[triggerIndex];
    }

    void queueBreak(int beamIndex) {
        if (broken[beamIndex] || pendingBreak[beamIndex]) return;
        pendingBreak[beamIndex] = true;
        pendingBreakIndices[pendingBreakCount++] = beamIndex;
    }

    int pendingBreakCount() {
        return pendingBreakCount;
    }

    int pendingBreakIndex(int pendingIndex) {
        return pendingBreakIndices[pendingIndex];
    }

    void clearPendingBreaks() {
        for (int i = 0; i < pendingBreakCount; i++) {
            pendingBreak[pendingBreakIndices[i]] = false;
        }
        pendingBreakCount = 0;
    }

    public float effectiveRestLength(int beamIndex) {
        return restLength[beamIndex] * actuationRatio[beamIndex];
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
