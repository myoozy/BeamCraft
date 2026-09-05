package me.mzy.beamcraft.client.physics;

import me.mzy.beamcraft.client.physics.electrics.ElectricBus;
import me.mzy.beamcraft.client.physics.electrics.ElectricSignals;
import me.mzy.beamcraft.client.physics.electrics.ElectricSnapshot;
import me.mzy.beamcraft.utility.Utility;

/**
 * Shared SoA command mapping and rate limiting for linear and torsional hydros.
 * The owning container applies {@link #state} as either a length ratio or angle.
 */
public final class HydroActuatorController {
    private static final int INITIAL_CAPACITY = 16;
    private static final float EPSILON = 1.0e-8f;

    public int count;
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
    public float[] center = new float[INITIAL_CAPACITY];
    public float[] state = new float[INITIAL_CAPACITY];
    public float[] command = new float[INITIAL_CAPACITY];

    private float[] multIn = new float[INITIAL_CAPACITY];
    private float[] multOut = new float[INITIAL_CAPACITY];
    private float[] offsetIn = new float[INITIAL_CAPACITY];
    private float[] offsetOut = new float[INITIAL_CAPACITY];

    public int add(PhysicsSpecs.HydroActuatorSpec spec, float neutralOutput, ElectricBus electrics) {
        ensureCapacity();
        int index = count++;

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
        steeringWheelLock[index] = spec.steeringWheelLock() == null
                ? Float.NaN : spec.steeringWheelLock();
        initializeMapping(index, neutralOutput);
        state[index] = center[index];
        command[index] = center[index];
        return index;
    }

    public float update(int index, float dt, ElectricSnapshot electrics) {
        float raw = clamp((float) electrics.get(inputSignalId[index]),
                inputInLimit[index], inputOutLimit[index]);
        float scaledInput = clamp(raw * inputFactor[index],
                inputInLimit[index], inputOutLimit[index]);
        float target = mapInput(index, scaledInput);
        command[index] = target;

        boolean returningToCenter = Math.abs(scaledInput - inputCenter[index]) <= EPSILON;
        float rate = returningToCenter
                ? autoCenterRate[index]
                : target < state[index] ? inRate[index] : outRate[index];
        state[index] = moveTowards(state[index], target, rate * Math.max(0.0f, dt));
        return state[index];
    }

    public void reset() {
        for (int i = 0; i < count; i++) {
            state[i] = center[i];
            command[i] = center[i];
        }
    }

    public void clear() {
        count = 0;
    }

    private void initializeMapping(int i, float neutralOutput) {
        float midpoint = (inputOutLimit[i] + inputInLimit[i]) * 0.5f;
        if (inputCenter[i] >= midpoint) {
            center[i] = neutralOutput + (outLimit[i] - neutralOutput)
                    * safeDivide(inputCenter[i] - midpoint, inputOutLimit[i] - midpoint);
        } else {
            center[i] = neutralOutput - (neutralOutput - inLimit[i])
                    * safeDivide(midpoint - inputCenter[i], midpoint - inputInLimit[i]);
        }

        multOut[i] = safeDivide(outLimit[i] - center[i], inputOutLimit[i] - inputCenter[i]);
        offsetOut[i] = center[i] - inputCenter[i] * multOut[i];
        multIn[i] = safeDivide(center[i] - inLimit[i], inputCenter[i] - inputInLimit[i]);
        offsetIn[i] = center[i] - inputCenter[i] * multIn[i];
    }

    private float mapInput(int i, float input) {
        return input >= inputCenter[i]
                ? offsetOut[i] + input * multOut[i]
                : offsetIn[i] + input * multIn[i];
    }

    private void ensureCapacity() {
        if (count < inputSignalId.length) {
            return;
        }
        int size = inputSignalId.length * 2;
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
        center = Utility.expand(center, size);
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
}
