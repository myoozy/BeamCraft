package me.mzy.beamcraft.client.input;

import me.mzy.beamcraft.client.physics.electrics.ElectricBus;
import me.mzy.beamcraft.client.physics.electrics.ElectricSignals;
import me.mzy.beamcraft.client.physics.electrics.ElectricSnapshot;
import me.mzy.beamcraft.client.physics.electrics.ElectricValues;

/**
 * Worker-owned filtered view of the player's continuous controls.
 *
 * <p>Raw targets are latched once at the start of each 50 ms prepared physics step.
 * Keyboard ramps then advance with physics substep time while unrelated electric signals
 * continue to come from the normal snapshot stream.
 */
public final class DriverInputFilter implements ElectricValues {
    private final int steeringSignalId;
    private final int throttleSignalId;
    private final int brakeSignalId;
    private final int clutchSignalId;

    private LinearRamp steering = new LinearRamp(0.0, 0.0);
    private LinearRamp throttle = new LinearRamp(0.0, 0.0);
    private LinearRamp brake = new LinearRamp(0.0, 0.0);
    private LinearRamp clutch = new LinearRamp(0.0, 0.0);

    private ElectricValues delegate = ElectricSnapshot.EMPTY;
    private float steeringTarget;
    private float throttleTarget;
    private float brakeTarget;
    private float clutchTarget;
    private float steeringValue;
    private float throttleValue;
    private float brakeValue;
    private float clutchValue;
    private boolean targetsLatched;

    public DriverInputFilter(ElectricBus electrics) {
        steeringSignalId = electrics.register(ElectricSignals.STEERING_INPUT);
        throttleSignalId = electrics.register(ElectricSignals.THROTTLE_INPUT);
        brakeSignalId = electrics.register(ElectricSignals.BRAKE_INPUT);
        clutchSignalId = electrics.register(ElectricSignals.CLUTCH_INPUT);
    }

    /** Replaces filter rates at a safe main-thread physics boundary. */
    public void configure(double steeringRiseTime, double steeringFallTime,
                          double throttleRiseTime, double throttleFallTime,
                          double brakeRiseTime, double brakeFallTime,
                          double clutchRiseTime, double clutchFallTime) {
        steering = new LinearRamp(steeringRiseTime, steeringFallTime);
        throttle = new LinearRamp(throttleRiseTime, throttleFallTime);
        brake = new LinearRamp(brakeRiseTime, brakeFallTime);
        clutch = new LinearRamp(clutchRiseTime, clutchFallTime);
        reset();
    }

    /** Captures the latest raw controls for the whole upcoming prepared step. */
    public void latchTargets(ElectricSnapshot snapshot) {
        ElectricSnapshot source = snapshot == null ? ElectricSnapshot.EMPTY : snapshot;
        steeringTarget = clampSigned(source.get(steeringSignalId));
        throttleTarget = clampUnsigned(source.get(throttleSignalId));
        brakeTarget = clampUnsigned(source.get(brakeSignalId));
        clutchTarget = clampUnsigned(source.get(clutchSignalId));
        targetsLatched = true;
    }

    /** Advances filtered controls once and returns this allocation-free electric view. */
    public ElectricValues update(float deltaTime, ElectricValues currentSnapshot) {
        delegate = currentSnapshot == null ? ElectricSnapshot.EMPTY : currentSnapshot;
        if (!targetsLatched) {
            steeringTarget = clampSigned(delegate.get(steeringSignalId));
            throttleTarget = clampUnsigned(delegate.get(throttleSignalId));
            brakeTarget = clampUnsigned(delegate.get(brakeSignalId));
            clutchTarget = clampUnsigned(delegate.get(clutchSignalId));
        }
        steeringValue = steering.update(steeringTarget, deltaTime);
        throttleValue = throttle.update(throttleTarget, deltaTime);
        brakeValue = brake.update(brakeTarget, deltaTime);
        clutchValue = clutch.update(clutchTarget, deltaTime);
        return this;
    }

    @Override
    public double get(int signalId) {
        if (signalId == steeringSignalId) return steeringValue;
        if (signalId == throttleSignalId) return throttleValue;
        if (signalId == brakeSignalId) return brakeValue;
        if (signalId == clutchSignalId) return clutchValue;
        return delegate.get(signalId);
    }

    public void reset() {
        steering.reset();
        throttle.reset();
        brake.reset();
        clutch.reset();
        steeringTarget = 0.0f;
        throttleTarget = 0.0f;
        brakeTarget = 0.0f;
        clutchTarget = 0.0f;
        steeringValue = 0.0f;
        throttleValue = 0.0f;
        brakeValue = 0.0f;
        clutchValue = 0.0f;
        targetsLatched = false;
        delegate = ElectricSnapshot.EMPTY;
    }

    private static float clampSigned(double value) {
        return (float) Math.max(-1.0, Math.min(1.0, value));
    }

    private static float clampUnsigned(double value) {
        return (float) Math.max(0.0, Math.min(1.0, value));
    }
}
