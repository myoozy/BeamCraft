package me.mzy.beamcraft.client.physics.powertrain;

import me.mzy.beamcraft.client.physics.NodeContainer;
import me.mzy.beamcraft.client.physics.SoftBodyVehicle;
import me.mzy.beamcraft.client.physics.electrics.ElectricSignals;
import me.mzy.beamcraft.client.physics.electrics.ElectricSnapshot;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.DeviceSpec;

import java.util.ArrayList;
import java.util.List;

/**
 * BeamNG-compatible rigid powertrain forest with a single elastic boundary at each
 * friction clutch.
 *
 * <p>This class is orchestration, control and debug only. All hot data lives in the flat
 * SoA containers of a {@link PowertrainData} (exposed here for the substep and for HUD
 * diagnostics); all build-time normalization and graph compilation lives in
 * {@link PowertrainCompiler}. The substep path ({@link #solve}) reads the containers
 * directly with no graph allocation or object traversal.
 *
 * <p>Each engine is a real combustion actuation model, not an always-running curve:
 * <ul>
 *   <li>combustion torque exists only while {@code sparkEnabled && fuelEnabled} (both cut
 *       by the rev limiter) and crank speed is above {@code crankingAV};</li>
 *   <li>a starter motor (external torque, boolean input) cranks a stalled engine back up
 *       past the combustion threshold;</li>
 *   <li>a BeamNG-style proportional idle controller separates the player throttle from
 *       the actual throttle; a calculated top-screw floor covers idle losses plus 5%;</li>
 *   <li>a time/soft rev limiter cuts spark+fuel for {@code revLimiterCutTime} and only
 *       retriggers while the crank is still above the hysteresis threshold — RPM is never
 *       clamped directly.</li>
 * </ul>
 *
 * <p>Gearboxes are runtime SoA: the active ratio scales the compile-time first-gear wheel
 * paths, a shift request disconnects the torque path (active ratio 0) for the shift
 * duration, and all shift/limiter timers are decremented only by {@code solve(dt)}.
 * The clutch torque is governed solely by {@link ImplicitClutchSolver} and its friction
 * capacity — it may stall the engine; it is never clamped to the engine's sustainable
 * torque.
 */
public final class PowertrainSystem {
    static final float RPM_TO_AV = (float) (Math.PI / 30.0);
    static final float AV_TO_RPM = 1.0f / RPM_TO_AV;

    private final SoftBodyVehicle vehicle;
    private final List<DeviceSpec> pendingSpecs = new ArrayList<>();

    // Runtime containers adopted on each finalizeSetup. Read by solve() and by the HUD.
    private PowertrainData data = new PowertrainData();
    public final PowertrainTopologyContainer topology = data.topology;
    public final CombustionEngineContainer engines = data.engines;
    public final FrictionClutchContainer clutches = data.clutches;
    public final DrivenWheelPathContainer wheelPaths = data.wheelPaths;
    public final TorqueReactionContainer reactions = data.reactions;
    public final GearboxContainer gearboxes = data.gearboxes;
    public final ShaftContainer shafts = data.shafts;
    public final DifferentialContainer differentials = data.differentials;
    public final TorsionReactorContainer torsionReactors = data.torsionReactors;

    private final int throttleSignalId;
    private final int clutchSignalId;
    private final int starterSignalId;
    private final int shiftUpSignalId;
    private final int shiftDownSignalId;
    private long lastShiftUpEvent;
    private long lastShiftDownEvent;
    private volatile float debugEngineRPM;
    private volatile float debugThrottle;
    private volatile float debugActualThrottle;
    private volatile float debugClutchEngagement;
    private volatile float debugClutchTorque;
    private volatile float debugCombustionTorque;
    private volatile int debugTorqueCurveCount;
    private volatile boolean debugStarterActive;
    private volatile boolean debugSparkEnabled;
    private volatile boolean debugFuelEnabled;
    private volatile boolean debugLimiterActive;
    private volatile float debugLimiterCutRemaining;
    private volatile int debugCurrentGearIndex;
    private volatile String debugCurrentGearName;
    private volatile float debugActiveRatio;
    private volatile float debugShiftRemaining;

    public PowertrainSystem(SoftBodyVehicle vehicle) {
        this.vehicle = vehicle;
        throttleSignalId = vehicle.electrics.register(ElectricSignals.THROTTLE_INPUT);
        clutchSignalId = vehicle.electrics.register(ElectricSignals.CLUTCH_INPUT);
        starterSignalId = vehicle.electrics.register(ElectricSignals.STARTER_INPUT);
        shiftUpSignalId = vehicle.electrics.register(ElectricSignals.SHIFT_UP_EVENT);
        shiftDownSignalId = vehicle.electrics.register(ElectricSignals.SHIFT_DOWN_EVENT);
    }

    public void addSpecs(List<DeviceSpec> specs) {
        if (specs != null) pendingSpecs.addAll(specs);
    }

    /** Called on the client thread after every selected JBeam part is parsed. */
    public void finalizeSetup() {
        data.clear();
        debugTorqueCurveCount = 0;
        if (pendingSpecs.isEmpty()) {
            data.diagnostic = "no powertrain data";
            return;
        }
        List<DeviceSpec> specs = new ArrayList<>(pendingSpecs);
        pendingSpecs.clear();
        PowertrainCompiler.compile(vehicle, specs, data);
        debugTorqueCurveCount = engines.unitCount > 0 ? engines.curveCount[0] : 0;
        debugActualThrottle = engines.unitCount > 0 ? engines.actualThrottle[0] : 0.0f;
        debugStarterActive = false;
        debugSparkEnabled = engines.unitCount > 0 && engines.sparkEnabled[0];
        debugFuelEnabled = engines.unitCount > 0 && engines.fuelEnabled[0];
        debugLimiterActive = false;
        debugLimiterCutRemaining = 0.0f;
        debugCurrentGearIndex = engines.unitCount > 0 ? gearboxes.currentGearIndex[0] : 0;
        debugCurrentGearName = engines.unitCount > 0 ? gearName(0, gearboxes.currentGearIndex[0]) : "?";
        debugActiveRatio = engines.unitCount > 0 ? gearboxes.activeRatio[0] : 0.0f;
        debugShiftRemaining = 0.0f;
    }

    /** Compatibility helper for tests and tools; production input writes the same bus directly. */
    public void setControls(float throttle, float clutchPedal) {
        setControls(throttle, clutchPedal, false);
    }

    /** Compatibility helper including the starter-motor request. */
    public void setControls(float throttle, float clutchPedal, boolean starter) {
        vehicle.electrics.set(throttleSignalId, Math.clamp(throttle, 0.0f, 1.0f));
        vehicle.electrics.set(clutchSignalId, Math.clamp(clutchPedal, 0.0f, 1.0f));
        vehicle.electrics.set(starterSignalId, starter ? 1.0 : 0.0);
    }

    /** Requests an upshift on every compiled gearbox (next forward gear, else no-op). */
    public void requestShiftUp() {
        vehicle.electrics.set(shiftUpSignalId, vehicle.electrics.get(shiftUpSignalId) + 1.0);
    }

    /** Requests a downshift on every compiled gearbox (toward reverse, else no-op). */
    public void requestShiftDown() {
        vehicle.electrics.set(shiftDownSignalId, vehicle.electrics.get(shiftDownSignalId) + 1.0);
    }

    /** Adds wheel and reaction forces for the current substep. */
    public void solve(float dt) {
        solve(dt, vehicle.electrics.snapshot());
    }

    /** Adds wheel and reaction forces using the electric snapshot for this substep block. */
    public void solve(float dt, ElectricSnapshot electrics) {
        if (dt <= 0.0f) return;
        ElectricSnapshot input = electrics == null ? ElectricSnapshot.EMPTY : electrics;
        consumeShiftEvents(input);
        if (engines.unitCount == 0 || dt <= 0.0f) return;
        float throttle = Math.clamp((float) input.get(throttleSignalId), 0.0f, 1.0f);
        float clutchPedal = Math.clamp((float) input.get(clutchSignalId), 0.0f, 1.0f);
        boolean starter = input.get(starterSignalId) >= 0.5;
        debugThrottle = throttle;
        debugClutchEngagement = 1.0f - clutchPedal;
        float engagement = 1.0f - clutchPedal;
        for (int unit = 0; unit < engines.unitCount; unit++) {
            engines.starterActive[unit] = starter;

            float rpm = Math.max(0.0f, engines.engineAV[unit]) * AV_TO_RPM;
            updateRevLimiter(unit, rpm, dt);

            boolean crankRunning = engines.engineAV[unit] >= engines.crankingAV[unit];
            boolean combustionEnabled = crankRunning
                    && engines.sparkEnabled[unit] && engines.fuelEnabled[unit];
            float idleOutput = idleControllerOutput(unit, crankRunning);
            float actualThrottle = Math.max(throttle, idleOutput);
            engines.playerThrottle[unit] = throttle;
            engines.actualThrottle[unit] = actualThrottle;
            float availableCombustionTorque = Math.max(0.0f, interpolateTorque(unit, rpm));
            float combustionTorque = combustionEnabled
                    ? actualThrottle * availableCombustionTorque : 0.0f;
            float normalizedCombustionOutput = availableCombustionTorque > 1.0e-6f
                    ? Math.clamp(combustionTorque / availableCombustionTorque, 0.0f, 1.0f)
                    : 0.0f;
            engines.availableCombustionTorque[unit] = availableCombustionTorque;
            engines.combustionTorque[unit] = combustionTorque;
            engines.normalizedCombustionOutput[unit] = normalizedCombustionOutput;

            float starterTorque = engines.starterActive[unit] && engines.engineAV[unit] < engines.starterMaxAV[unit]
                    ? engines.starterTorque[unit] : 0.0f;

            float engineAVBeforeExternal = Math.max(0.0f, engines.engineAV[unit]);
            float loss = engines.engineFriction[unit]
                    + engines.engineDynamicFriction[unit] * engineAVBeforeExternal;
            // BeamNG's additional engineBrakeTorque depends on instantEngineLoad. Leave
            // it inactive until BeamCraft has that intake/load model instead of inventing
            // a throttle or clutch-load proxy.
            float driveTorque = Math.max(0.0f, combustionTorque + starterTorque);
            // Integrate driving torque first and preserve that no-loss result. Resistance
            // is dissipative: if applying it crosses zero, the crank stopped during this
            // substep and must not be accelerated in the opposite direction.
            float drivenAV = engineAVBeforeExternal + dt * driveTorque / engines.engineInertia[unit];
            float lossDeltaAV = dt * Math.max(0.0f, loss) / engines.engineInertia[unit];
            float resistedAV = drivenAV - Math.copySign(lossDeltaAV, drivenAV);
            engines.engineAV[unit] = Math.signum(drivenAV) != Math.signum(resistedAV)
                    ? 0.0f : resistedAV;
            float externalTorque = (engines.engineAV[unit] - engineAVBeforeExternal)
                    * engines.engineInertia[unit] / dt;

            // Gearbox shift timer + dynamic ratio. activeRatio is 0 in neutral and during a
            // shift, which disconnects the torque path without skipping engine integration.
            updateShift(unit, dt);
            float activeRatio = gearboxes.activeRatio[unit];

            float clutchTorque = 0.0f;
            if (Math.abs(activeRatio) > 1e-6f) {
                float pathBaseRatio = gearboxes.pathBaseRatio[unit];
                float ratioFactor = pathBaseRatio > 1e-6f ? activeRatio / pathBaseRatio : 1.0f;
                float drivelineAV = 0.0f;
                float compliance = 0.0f;
                int pStart = wheelPaths.pathStart[unit];
                int pEnd = pStart + wheelPaths.pathCount[unit];
                for (int p = pStart; p < pEnd; p++) {
                    int wheel = wheelPaths.pathWheel[p];
                    float gain = wheelPaths.pathGain[p] * ratioFactor;
                    // The wheel API's forward-positive convention is opposite to the
                    // engine shaft convention. Use the same signed transform here and
                    // below for torque so T*w power is conserved through the rigid map.
                    drivelineAV -= gain * vehicle.wheels.getAngularVelocity(wheel);
                    float inertia = vehicle.wheels.getRotationalInertia(wheel);
                    if (inertia > 1e-7f) compliance += gain * gain / inertia;
                }
                if (compliance > 1e-9f) {
                    float drivelineInertia = 1.0f / compliance;
                    ImplicitClutchSolver.solveInto(
                            dt, engines.engineAV[unit] - drivelineAV, engines.engineInertia[unit], drivelineInertia,
                            clutches.clutchSpring[unit], clutches.clutchDampingRatio[unit], clutches.clutchCapacity[unit],
                            engagement, clutches.clutchTorque, clutches.clutchAngle, unit);
                    clutchTorque = clutches.clutchTorque[unit];
                    engines.engineAV[unit] -= dt * clutchTorque / engines.engineInertia[unit];
                    for (int p = pStart; p < pEnd; p++) {
                        float gain = wheelPaths.pathGain[p] * ratioFactor;
                        // Engine -> wheel drive torque has the corrected sign (opposite to the
                        // clutch torque handed to the driveline), so positive engine spin drives
                        // the vehicle forward in the game's wheel-AV convention.
                        vehicle.wheels.applyDriveTorque(wheelPaths.pathWheel[p], -clutchTorque * gain);
                    }
                } else {
                    clutches.clutchTorque[unit] = 0.0f;
                }
            } else {
                // Neutral / mid-shift: no torque path, clutch released.
                clutches.clutchTorque[unit] = 0.0f;
                clutches.clutchAngle[unit] = 0.0f;
            }
            if (engines.engineAV[unit] < 0.0f) engines.engineAV[unit] = 0.0f;

            // BeamNG applies crank inertial torque at the engine reaction nodes,
            // while each torsionReactor closes the downstream driveline torque
            // on its own axis. Keeping those axes separate matters on longitudinal
            // layouts where the crank and wheel axes are perpendicular.
            applyReactionTorque(reactions.reactionStart[unit], reactions.reactionCount[unit],
                    -(externalTorque - clutchTorque));
            int rEnd = reactions.reactorStart[unit] + reactions.reactorCount[unit];
            for (int reactor = reactions.reactorStart[unit]; reactor < rEnd; reactor++) {
                applyReactionTorque(reactions.reactorNodeStart[reactor], reactions.reactorNodeCount[reactor],
                        -clutchTorque * reactions.reactorGain[reactor]);
            }

            if (unit == 0) {
                debugEngineRPM = engines.engineAV[0] * AV_TO_RPM;
                debugClutchTorque = clutches.clutchTorque[0];
                debugCombustionTorque = combustionTorque;
                debugTorqueCurveCount = engines.curveCount[0];
                debugActualThrottle = actualThrottle;
                debugStarterActive = starter;
                debugSparkEnabled = engines.sparkEnabled[0];
                debugFuelEnabled = engines.fuelEnabled[0];
                debugLimiterActive = engines.limiterCutRemaining[0] > 0.0f;
                debugLimiterCutRemaining = engines.limiterCutRemaining[0];
                debugCurrentGearIndex = gearboxes.currentGearIndex[0];
                debugCurrentGearName = gearName(0, gearboxes.currentGearIndex[0]);
                debugActiveRatio = gearboxes.activeRatio[0];
                debugShiftRemaining = gearboxes.shiftRemaining[0];
            }
        }
    }

    /**
     * Rev limiter, driven purely by accumulated substep {@code dt}. While a cut is active,
     * spark and fuel are disabled for {@code revLimiterCutTime}; when the cut expires the
     * limiter only retriggers if the crank is still above the hysteresis threshold
     * ({@code revLimiterRPM - revLimiterMaxRPMDrop}), otherwise combustion resumes and the
     * engine is allowed to recover. RPM is never clamped directly.
     */
    private void updateRevLimiter(int unit, float rpm, float dt) {
        float limit = engines.revLimiterRPM[unit];
        if (limit <= 0.0f) {
            engines.limiterCutRemaining[unit] = 0.0f;
            engines.sparkEnabled[unit] = true;
            engines.fuelEnabled[unit] = true;
            return;
        }
        float hysteresis = limit - Math.max(0.0f, engines.revLimiterMaxRPMDrop[unit]);
        if (engines.limiterCutRemaining[unit] > 0.0f) {
            engines.limiterCutRemaining[unit] -= dt;
            engines.sparkEnabled[unit] = false;
            engines.fuelEnabled[unit] = false;
            if (engines.limiterCutRemaining[unit] <= 0.0f) {
                engines.limiterCutRemaining[unit] = 0.0f;
                if (rpm >= hysteresis) {
                    // Still above the hysteresis threshold: retrigger the cut.
                    engines.limiterCutRemaining[unit] = Math.max(0.0f, engines.revLimiterCutTime[unit]);
                    engines.sparkEnabled[unit] = false;
                    engines.fuelEnabled[unit] = false;
                } else {
                    engines.sparkEnabled[unit] = true;
                    engines.fuelEnabled[unit] = true;
                }
            }
        } else if (rpm >= limit) {
            engines.limiterCutRemaining[unit] = Math.max(0.0f, engines.revLimiterCutTime[unit]);
            engines.sparkEnabled[unit] = false;
            engines.fuelEnabled[unit] = false;
        } else {
            engines.sparkEnabled[unit] = true;
            engines.fuelEnabled[unit] = true;
        }
    }

    /** BeamNG-style positive-error idle controller with a current-speed loss feedforward. */
    private float idleControllerOutput(int unit, boolean running) {
        float idle = engines.idleAV[unit];
        float engineAV = Math.max(0.0f, engines.engineAV[unit]);
        float torque = interpolateTorque(unit, engineAV * AV_TO_RPM);
        float loss = engines.engineFriction[unit]
                + engines.engineDynamicFriction[unit] * engineAV;
        float feedforward = torque > 1e-3f
                ? Math.clamp(loss / torque + 0.05f, 0.0f, 1.0f)
                : (loss > 1e-3f ? 1.0f : 0.0f);
        engines.idleLossThrottle[unit] = feedforward;
        if (idle <= 1e-6f || !running) return feedforward;
        float positiveError = Math.max(idle - engines.engineAV[unit], 0.0f);
        if (positiveError <= 0.0f) return 0.0f;
        float proportional = Math.min(positiveError * engines.idleControllerP[unit],
                engines.maxIdleThrottle[unit]);
        return Math.max(feedforward, proportional);
    }

    /**
     * Advances the gearbox shift timer and keeps the active ratio in sync. The timer only
     * changes here, inside {@link #solve(float)}, never on the wall/game clock.
     */
    private void updateShift(int unit, float dt) {
        int pending = gearboxes.pendingGearIndex[unit];
        if (pending >= 0 && pending != gearboxes.currentGearIndex[unit]) {
            gearboxes.shiftRemaining[unit] -= dt;
            if (gearboxes.shiftRemaining[unit] <= 0.0f) {
                gearboxes.shiftRemaining[unit] = 0.0f;
                gearboxes.currentGearIndex[unit] = pending;
                gearboxes.activeRatio[unit] = gearboxRatio(unit, pending);
                gearboxes.pendingGearIndex[unit] = -1;
            } else {
                gearboxes.activeRatio[unit] = 0.0f; // torque path disconnected during the shift
            }
        } else {
            gearboxes.activeRatio[unit] = gearboxRatio(unit, gearboxes.currentGearIndex[unit]);
        }
    }

    private void requestShift(int unit, int target) {
        if (gearboxes.fixedFirstGear[unit]) return;
        int count = gearboxes.gearCount[unit];
        if (count <= 1 || target < 0 || target >= count) return;
        if (target == gearboxes.currentGearIndex[unit] && gearboxes.pendingGearIndex[unit] < 0) return;
        gearboxes.pendingGearIndex[unit] = target;
        if (gearboxes.shiftRemaining[unit] <= 0.0f) {
            gearboxes.shiftRemaining[unit] = Math.max(0.0f, gearboxes.shiftDuration[unit]);
            gearboxes.activeRatio[unit] = 0.0f; // torque path disconnected from shift start
        }
    }

    private void consumeShiftEvents(ElectricSnapshot input) {
        long up = eventSequence(input.get(shiftUpSignalId));
        long down = eventSequence(input.get(shiftDownSignalId));
        int upCount = eventCount(lastShiftUpEvent, up);
        int downCount = eventCount(lastShiftDownEvent, down);
        lastShiftUpEvent = up;
        lastShiftDownEvent = down;

        for (int event = 0; event < upCount; event++) {
            for (int unit = 0; unit < gearboxes.unitCount; unit++) {
                int base = gearboxes.pendingGearIndex[unit] >= 0
                        ? gearboxes.pendingGearIndex[unit] : gearboxes.currentGearIndex[unit];
                requestShift(unit, base + 1);
            }
        }
        for (int event = 0; event < downCount; event++) {
            for (int unit = 0; unit < gearboxes.unitCount; unit++) {
                int base = gearboxes.pendingGearIndex[unit] >= 0
                        ? gearboxes.pendingGearIndex[unit] : gearboxes.currentGearIndex[unit];
                requestShift(unit, base - 1);
            }
        }
    }

    private static long eventSequence(double value) {
        return Double.isFinite(value) && value > 0.0 ? (long) Math.floor(value) : 0L;
    }

    private static int eventCount(long previous, long current) {
        if (current <= previous) return 0;
        return (int) Math.min(current - previous, 32L);
    }

    private float gearboxRatio(int unit, int index) {
        int start = gearboxes.gearStart[unit];
        int count = gearboxes.gearCount[unit];
        if (index >= 0 && index < count) return gearboxes.gearRatios[start + index];
        return 0.0f;
    }

    /** JBeam gear label: R for a negative ratio, N for zero, then 1, 2, … by forward order. */
    private String gearName(int unit, int index) {
        float ratio = gearboxRatio(unit, index);
        if (ratio < 0.0f) return "R";
        if (Math.abs(ratio) < 1e-6f) return "N";
        int forward = 1;
        int start = gearboxes.gearStart[unit];
        int count = gearboxes.gearCount[unit];
        for (int i = 0; i < index && i < count; i++) {
            if (gearboxes.gearRatios[start + i] > 1e-6f) forward++;
        }
        return Integer.toString(forward);
    }

    public void reset() {
        for (int i = 0; i < engines.unitCount; i++) {
            engines.engineAV[i] = engines.idleAV[i];
            clutches.clutchAngle[i] = 0.0f;
            clutches.clutchTorque[i] = 0.0f;
            engines.sparkEnabled[i] = true;
            engines.fuelEnabled[i] = true;
            engines.starterActive[i] = false;
            engines.playerThrottle[i] = 0.0f;
            engines.actualThrottle[i] = engines.idleLossThrottle[i];
            engines.availableCombustionTorque[i] = 0.0f;
            engines.combustionTorque[i] = 0.0f;
            engines.normalizedCombustionOutput[i] = 0.0f;
            engines.limiterCutRemaining[i] = 0.0f;
            gearboxes.currentGearIndex[i] = gearboxes.initialGearIndex[i];
            gearboxes.pendingGearIndex[i] = -1;
            gearboxes.activeRatio[i] = gearboxRatio(i, gearboxes.initialGearIndex[i]);
            gearboxes.shiftRemaining[i] = 0.0f;
        }
        lastShiftUpEvent = 0L;
        lastShiftDownEvent = 0L;
        vehicle.electrics.set(throttleSignalId, 0.0);
        vehicle.electrics.set(clutchSignalId, 0.0);
        vehicle.electrics.set(starterSignalId, 0.0);
        vehicle.electrics.set(shiftUpSignalId, 0.0);
        vehicle.electrics.set(shiftDownSignalId, 0.0);
        debugEngineRPM = engines.unitCount > 0 ? engines.engineAV[0] * AV_TO_RPM : 0.0f;
        debugThrottle = 0.0f;
        debugActualThrottle = engines.unitCount > 0 ? engines.actualThrottle[0] : 0.0f;
        debugClutchEngagement = 1.0f;
        debugClutchTorque = 0.0f;
        debugCombustionTorque = 0.0f;
        debugTorqueCurveCount = engines.unitCount > 0 ? engines.curveCount[0] : 0;
        debugStarterActive = false;
        debugSparkEnabled = engines.unitCount > 0 && engines.sparkEnabled[0];
        debugFuelEnabled = engines.unitCount > 0 && engines.fuelEnabled[0];
        debugLimiterActive = false;
        debugLimiterCutRemaining = 0.0f;
        debugCurrentGearIndex = engines.unitCount > 0 ? gearboxes.currentGearIndex[0] : 0;
        debugCurrentGearName = engines.unitCount > 0 ? gearName(0, gearboxes.currentGearIndex[0]) : "?";
        debugActiveRatio = engines.unitCount > 0 ? gearboxes.activeRatio[0] : 0.0f;
        debugShiftRemaining = 0.0f;
    }

    public void clear() {
        pendingSpecs.clear();
        data.clear();
        lastShiftUpEvent = 0L;
        lastShiftDownEvent = 0L;
        debugEngineRPM = 0.0f;
        debugThrottle = 0.0f;
        debugActualThrottle = 0.0f;
        debugClutchEngagement = 0.0f;
        debugClutchTorque = 0.0f;
        debugCombustionTorque = 0.0f;
        debugTorqueCurveCount = 0;
        debugStarterActive = false;
        debugSparkEnabled = false;
        debugFuelEnabled = false;
        debugLimiterActive = false;
        debugLimiterCutRemaining = 0.0f;
        debugCurrentGearIndex = 0;
        debugCurrentGearName = "?";
        debugActiveRatio = 0.0f;
        debugShiftRemaining = 0.0f;
    }

    public float debugEngineRPM() { return debugEngineRPM; }
    public float debugThrottle() { return debugThrottle; }
    public float debugActualThrottle() { return debugActualThrottle; }
    public float debugClutchEngagement() { return debugClutchEngagement; }
    public float debugClutchTorque() { return debugClutchTorque; }
    public String diagnostic() { return data.diagnostic; }

    /** Combustion torque (actual throttle × interpolated torque curve) of unit 0. */
    public float debugCombustionTorque() { return debugCombustionTorque; }

    /** Number of torque-curve points compiled for unit 0 (0 when no unit is compiled). */
    public int debugTorqueCurveCount() { return debugTorqueCurveCount; }

    public boolean debugStarterActive() { return debugStarterActive; }
    public boolean debugSparkEnabled() { return debugSparkEnabled; }
    public boolean debugFuelEnabled() { return debugFuelEnabled; }
    public boolean debugLimiterActive() { return debugLimiterActive; }
    public float debugLimiterCutRemaining() { return debugLimiterCutRemaining; }
    public int debugCurrentGearIndex() { return debugCurrentGearIndex; }
    public String debugCurrentGearName() { return debugCurrentGearName; }
    public float debugActiveRatio() { return debugActiveRatio; }
    public float debugShiftRemaining() { return debugShiftRemaining; }

    private float interpolateTorque(int unit, float rpm) {
        int start = engines.curveStart[unit];
        int count = engines.curveCount[unit];
        if (count <= 0) return 0.0f;
        if (count == 1 || rpm <= engines.curveRPM[start]) return engines.curveTorque[start];
        int end = start + count;
        for (int i = start + 1; i < end; i++) {
            if (rpm <= engines.curveRPM[i]) {
                float span = engines.curveRPM[i] - engines.curveRPM[i - 1];
                float t = span > 1e-6f ? (rpm - engines.curveRPM[i - 1]) / span : 0.0f;
                return engines.curveTorque[i - 1] + (engines.curveTorque[i] - engines.curveTorque[i - 1]) * t;
            }
        }
        return engines.curveTorque[end - 1];
    }

    private void applyReactionTorque(int start, int count, float torque) {
        if (count < 3 || Math.abs(torque) < 1e-8f) return;
        NodeContainer nodes = vehicle.nodes;
        int n1 = reactions.reactionNodes[start];
        int n2 = reactions.reactionNodes[start + 1];
        float ax = nodes.posX[n2] - nodes.posX[n1];
        float ay = nodes.posY[n2] - nodes.posY[n1];
        float az = nodes.posZ[n2] - nodes.posZ[n1];
        float length = (float) Math.sqrt(ax * ax + ay * ay + az * az);
        if (length < 1e-8f) return;
        TorqueReactionSolver.apply(nodes, reactions.reactionNodes, start, count,
                torque * ax / length, torque * ay / length, torque * az / length);
    }

}
