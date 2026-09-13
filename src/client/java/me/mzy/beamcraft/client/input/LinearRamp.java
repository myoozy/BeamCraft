package me.mzy.beamcraft.client.input;

/** Moves an input value toward its target using independent linear rise and fall times. */
public final class LinearRamp {
    private final float riseTime;
    private final float fallTime;
    private float value;

    public LinearRamp(double riseTime, double fallTime) {
        this.riseTime = sanitizeTime(riseTime);
        this.fallTime = sanitizeTime(fallTime);
    }

    public float update(boolean active, float deltaTime) {
        return update(active ? 1.0f : 0.0f, deltaTime);
    }

    public float update(float target, float deltaTime) {
        target = Math.max(-1.0f, Math.min(1.0f, target));
        float remainingTime = Math.max(0.0f, deltaTime);

        if (value * target < 0.0f) {
            if (fallTime <= 0.0f) {
                value = 0.0f;
            } else {
                float timeToZero = Math.abs(value) * fallTime;
                if (remainingTime < timeToZero) {
                    value = Math.copySign(Math.abs(value) - remainingTime / fallTime, value);
                    return value;
                }
                value = 0.0f;
                remainingTime -= timeToZero;
            }
        }

        boolean rising = Math.abs(target) > Math.abs(value);
        float transitionTime = rising ? riseTime : fallTime;
        if (transitionTime <= 0.0f) {
            value = target;
            return value;
        }

        float step = remainingTime / transitionTime;
        if (value < target) {
            value = Math.min(target, value + step);
        } else {
            value = Math.max(target, value - step);
        }
        return value;
    }

    public void reset() {
        value = 0.0f;
    }

    public float value() {
        return value;
    }

    private static float sanitizeTime(double time) {
        return Double.isFinite(time) && time > 0.0 ? (float) time : 0.0f;
    }
}
