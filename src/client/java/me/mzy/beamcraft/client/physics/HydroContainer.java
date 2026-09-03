package me.mzy.beamcraft.client.physics;

import me.mzy.beamcraft.client.physics.electrics.ElectricBus;
import me.mzy.beamcraft.client.physics.electrics.ElectricSignals;
import me.mzy.beamcraft.client.physics.electrics.ElectricSnapshot;
import me.mzy.beamcraft.utility.Utility;

/**
 * SoA runtime state for BeamNG-style hydros. The linked normal beam retains all
 * structural properties; this container only controls its reference-length ratio.
 */
public final class HydroContainer {
    private static final int INITIAL_CAPACITY = 16;
    private static final float EPSILON = 1.0e-8f;

    public int count;
    public int[] beamIndex = new int[INITIAL_CAPACITY];
    public int[] inputSignalId = new int[INITIAL_CAPACITY];
    public String[] inputSource = new String[INITIAL_CAPACITY];
    public float[] inLimit = new float[INITIAL_CAPACITY];
    public float[] outLimit = new float[INITIAL_CAPACITY];
    public float[] inputFactor = new float[INITIAL_CAPACITY];
    public float[] inputCenter = new float[INITIAL_CAPACITY];
    public float[] inputInLimit = new float[INITIAL_CAPACITY];
    public float[] inputOutLimit = new float[INITIAL_CAPACITY];
    public float[] inRate = new float[INITIAL_CAPACITY];
    public float[] outRate = new float[INITIAL_CAPACITY];
    public float[] autoCenterRate = new float[INITIAL_CAPACITY];
    /** NaN when the optional steering-wheel metadata is absent. */
    public float[] steeringWheelLock = new float[INITIAL_CAPACITY];
    public float[] centerRatio = new float[INITIAL_CAPACITY];
    public float[] state = new float[INITIAL_CAPACITY];
    public float[] command = new float[INITIAL_CAPACITY];

    private float[] multIn = new float[INITIAL_CAPACITY];
    private float[] multOut = new float[INITIAL_CAPACITY];
    private float[] offsetIn = new float[INITIAL_CAPACITY];
    private float[] offsetOut = new float[INITIAL_CAPACITY];
    public int addHydro(PhysicsSpecs.HydroSpec spec, int linkedBeamIndex,
                        BeamContainer beams, ElectricBus electrics) {
        ensureCapacity();
        int index = count++;

        beamIndex[index] = linkedBeamIndex;
        inputSource[index] = normalizeInputSource(spec.inputSource());
        inputSignalId[index] = electrics.register(inputSource[index]);
        inLimit[index] = spec.inLimit();
        outLimit[index] = spec.outLimit();
        inputFactor[index] = spec.inputFactor();
        inputCenter[index] = spec.inputCenter() * spec.inputFactor();
        inputInLimit[index] = spec.inputInLimit() * spec.inputFactor();
        inputOutLimit[index] = spec.inputOutLimit() * spec.inputFactor();
        if (spec.inputFactor() < 0.0f) {
            float swap = inputInLimit[index];
            inputInLimit[index] = inputOutLimit[index];
            inputOutLimit[index] = swap;
        }

        inRate[index] = Math.max(0.0f, spec.inRate());
        outRate[index] = Math.max(0.0f, spec.outRate());
        autoCenterRate[index] = Math.max(0.0f, spec.autoCenterRate());
        steeringWheelLock[index] = spec.steeringWheelLock() == null ? Float.NaN : spec.steeringWheelLock();
        initializeMapping(index);
        state[index] = centerRatio[index];
        command[index] = centerRatio[index];
        applyRatio(beams, linkedBeamIndex, state[index]);
        return index;
    }

    /** Advances all actuator states by one physics substep. */
    public void update(float dt, BeamContainer beams, ElectricSnapshot electrics) {
        float safeDt = Math.max(0.0f, dt);
        for (int i = 0; i < count; i++) {
            int linkedBeam = beamIndex[i];
            if (linkedBeam < 0 || linkedBeam >= beams.count || beams.broken[linkedBeam]) {
                continue;
            }

            float raw = clamp((float) electrics.get(inputSignalId[i]), inputInLimit[i], inputOutLimit[i]);
            float scaledInput = raw * inputFactor[i];
            scaledInput = clamp(scaledInput, inputInLimit[i], inputOutLimit[i]);
            float target = mapInputToRatio(i, scaledInput);
            command[i] = target;

            boolean returningToCenter = Math.abs(scaledInput - inputCenter[i]) <= EPSILON;
            float rate;
            if (returningToCenter) {
                rate = autoCenterRate[i];
            } else {
                rate = target < state[i] ? inRate[i] : outRate[i];
            }
            state[i] = moveTowards(state[i], target, rate * safeDt);
            applyRatio(beams, linkedBeam, state[i]);
        }
    }

    public void reset(BeamContainer beams) {
        for (int i = 0; i < count; i++) {
            state[i] = centerRatio[i];
            command[i] = centerRatio[i];
            int linkedBeam = beamIndex[i];
            if (linkedBeam >= 0 && linkedBeam < beams.count) {
                applyRatio(beams, linkedBeam, state[i]);
            }
        }
    }

    public void clear() {
        count = 0;
    }

    private void initializeMapping(int i) {
        float midpoint = (inputOutLimit[i] + inputInLimit[i]) * 0.5f;
        if (inputCenter[i] >= midpoint) {
            centerRatio[i] = 1.0f + (outLimit[i] - 1.0f)
                    * safeDivide(inputCenter[i] - midpoint, inputOutLimit[i] - midpoint);
        } else {
            centerRatio[i] = 1.0f - (1.0f - inLimit[i])
                    * safeDivide(midpoint - inputCenter[i], midpoint - inputInLimit[i]);
        }

        multOut[i] = safeDivide(outLimit[i] - centerRatio[i], inputOutLimit[i] - inputCenter[i]);
        offsetOut[i] = centerRatio[i] - inputCenter[i] * multOut[i];
        multIn[i] = safeDivide(centerRatio[i] - inLimit[i], inputCenter[i] - inputInLimit[i]);
        offsetIn[i] = centerRatio[i] - inputCenter[i] * multIn[i];
    }

    private float mapInputToRatio(int i, float input) {
        return input >= inputCenter[i]
                ? offsetOut[i] + input * multOut[i]
                : offsetIn[i] + input * multIn[i];
    }

    private void ensureCapacity() {
        if (count < beamIndex.length) {
            return;
        }
        int size = beamIndex.length * 2;
        beamIndex = Utility.expand(beamIndex, size);
        inputSignalId = Utility.expand(inputSignalId, size);
        inputSource = java.util.Arrays.copyOf(inputSource, size);
        inLimit = Utility.expand(inLimit, size);
        outLimit = Utility.expand(outLimit, size);
        inputFactor = Utility.expand(inputFactor, size);
        inputCenter = Utility.expand(inputCenter, size);
        inputInLimit = Utility.expand(inputInLimit, size);
        inputOutLimit = Utility.expand(inputOutLimit, size);
        inRate = Utility.expand(inRate, size);
        outRate = Utility.expand(outRate, size);
        autoCenterRate = Utility.expand(autoCenterRate, size);
        steeringWheelLock = Utility.expand(steeringWheelLock, size);
        centerRatio = Utility.expand(centerRatio, size);
        state = Utility.expand(state, size);
        command = Utility.expand(command, size);
        multIn = Utility.expand(multIn, size);
        multOut = Utility.expand(multOut, size);
        offsetIn = Utility.expand(offsetIn, size);
        offsetOut = Utility.expand(offsetOut, size);
    }

    private static String normalizeInputSource(String source) {
        if (source == null || source.isBlank() || source.equals("steering")) {
            return ElectricSignals.STEERING_INPUT;
        }
        return source;
    }

    private static float moveTowards(float current, float target, float maxDelta) {
        if (current < target) {
            return Math.min(current + maxDelta, target);
        }
        return Math.max(current - maxDelta, target);
    }

    private static float clamp(float value, float min, float max) {
        return Math.max(min, Math.min(max, value));
    }

    private static float safeDivide(float numerator, float denominator) {
        return Math.abs(denominator) <= EPSILON ? 0.0f : numerator / denominator;
    }

    private static void applyRatio(BeamContainer beams, int beamIndex, float ratio) {
        beams.actuationRatio[beamIndex] = Math.max(EPSILON, ratio);
    }
}
