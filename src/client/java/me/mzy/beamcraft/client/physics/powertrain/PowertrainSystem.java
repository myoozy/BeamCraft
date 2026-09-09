package me.mzy.beamcraft.client.physics.powertrain;

import me.mzy.beamcraft.client.physics.NodeContainer;
import me.mzy.beamcraft.client.physics.SoftBodyVehicle;
import me.mzy.beamcraft.client.physics.TorqueReactionSolver;
import me.mzy.beamcraft.client.physics.electrics.ElectricSignals;
import me.mzy.beamcraft.client.physics.electrics.ElectricSnapshot;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.DeviceSpec;

import java.util.ArrayList;
import java.util.List;

/**
 * BeamNG-compatible rigid powertrain forest with one compliant {@code clutchlike}
 * boundary (friction clutch or torque converter) downstream of each combustion engine.
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
 * The clutch torque is governed solely by {@link ImplicitCouplingSolver} and its friction
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
    public final ClutchlikeContainer clutchlikes = data.clutchlikes;
    public final TorqueConverterContainer torqueConverters = data.torqueConverters;
    public final DrivenWheelPathContainer wheelPaths = data.wheelPaths;
    public final TorqueReactionContainer reactions = data.reactions;
    public final GearboxContainer gearboxes = data.gearboxes;
    public final RangeBoxContainer rangeBoxes = data.rangeBoxes;
    public final SplitShaftContainer splitShafts = data.splitShafts;
    public final ShaftContainer shafts = data.shafts;
    public final DifferentialContainer differentials = data.differentials;
    public final TorsionReactorContainer torsionReactors = data.torsionReactors;

    private final int throttleSignalId;
    private final int clutchSignalId;
    private final int starterSignalId;
    private final int shiftUpSignalId;
    private final int shiftDownSignalId;
    private final int rangeBoxToggleSignalId;
    private final int defaultLockupSignalId;
    private long lastShiftUpEvent;
    private long lastShiftDownEvent;
    private long lastRangeBoxToggleEvent;
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
        rangeBoxToggleSignalId = vehicle.electrics.register(ElectricSignals.RANGE_BOX_TOGGLE_EVENT);
        defaultLockupSignalId = vehicle.electrics.register(ElectricSignals.LOCKUP_CLUTCH_RATIO);
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

    /** Toggles every compiled range box between BeamNG-compatible high and low modes. */
    public void requestRangeBoxToggle() {
        vehicle.electrics.set(rangeBoxToggleSignalId,
                vehicle.electrics.get(rangeBoxToggleSignalId) + 1.0);
    }

    /** Selects high or low range immediately without changing the primary gearbox gear. */
    public void setRangeBoxLow(boolean low) {
        for (int unit = 0; unit < rangeBoxes.unitCount; unit++) {
            setRangeBoxMode(unit, low);
        }
    }

    /** Selects the basic operating mode of every split shaft with the given device name. */
    public void setSplitShaftMode(String deviceName, String mode) {
        byte resolved = switch (mode == null ? "" : mode.toLowerCase(java.util.Locale.ROOT)) {
            case "locked", "lock" -> SplitShaftContainer.MODE_LOCKED;
            case "viscous" -> SplitShaftContainer.MODE_VISCOUS;
            case "disconnected", "disconnect", "open" -> SplitShaftContainer.MODE_DISCONNECTED;
            default -> -1;
        };
        if (resolved < 0) return;
        for (int split = 0; split < splitShafts.count; split++) {
            if (!splitShafts.deviceName[split].equals(deviceName)) continue;
            if (resolved == SplitShaftContainer.MODE_DISCONNECTED && !splitShafts.canDisconnect[split]) continue;
            splitShafts.activeMode[split] = resolved;
            SplitShaftSolver.clear(splitShafts, split);
        }
    }

    /** Updates the lock-clutch command for a named split shaft (0 = open, 1 = engaged). */
    public void setSplitShaftClutchRatio(String deviceName, float ratio) {
        float clamped = Math.clamp(ratio, 0.0f, 1.0f);
        for (int split = 0; split < splitShafts.count; split++) {
            if (splitShafts.deviceName[split].equals(deviceName)) {
                splitShafts.clutchRatio[split] = clamped;
            }
        }
    }

    /** Writes the default BeamNG-compatible torque-converter lock-up command (0 = open, 1 = locked). */
    public void setTorqueConverterLockup(float ratio) {
        vehicle.electrics.set(defaultLockupSignalId, Math.clamp(ratio, 0.0f, 1.0f));
    }

    /** Writes a converter-specific lock-up signal selected by {@code lockupClutchRatioName}. */
    public void setTorqueConverterLockup(String signalName, float ratio) {
        String resolved = signalName == null || signalName.isBlank()
                ? ElectricSignals.LOCKUP_CLUTCH_RATIO : signalName;
        int signalId = vehicle.electrics.register(resolved);
        vehicle.electrics.set(signalId, Math.clamp(ratio, 0.0f, 1.0f));
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
        consumeRangeBoxEvents(input);
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
            boolean belowIdle = crankRunning && engines.engineAV[unit] < engines.idleAV[unit];
            float idleOutput = idleControllerOutput(unit, crankRunning, belowIdle, dt);
            // KinetiForge-style pedal mapping: the physical throttle plate spans from
            // the steady idle opening to wide open throttle. Fuel is nevertheless cut
            // at zero pedal once the crank has recovered to the idle target.
            float pedalThrottle = Math.fma(1.0f - engines.idleLossThrottle[unit], throttle,
                    engines.idleLossThrottle[unit]);
            float actualThrottle = belowIdle ? Math.max(pedalThrottle, idleOutput) : pedalThrottle;
            boolean combustionRequested = throttle > 1.0e-6f || belowIdle;
            boolean combustionEnabled = crankRunning && combustionRequested
                    && engines.sparkEnabled[unit] && engines.fuelEnabled[unit];
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
            float pathBaseRatio = gearboxes.pathBaseRatio[unit];
            float gearboxFactor = pathBaseRatio > 1e-6f ? activeRatio / pathBaseRatio : 1.0f;
            float rangeBaseRatio = rangeBoxes.pathBaseRatio[unit];
            float rangeFactor = rangeBaseRatio > 1.0e-6f
                    ? rangeBoxes.activeRatio[unit] / rangeBaseRatio : 1.0f;
            float ratioFactor = gearboxFactor * rangeFactor;

            float couplerInputTorque = 0.0f;
            float couplerOutputTorque = 0.0f;
            float drivelineAV = 0.0f;
            float drivelineCompliance = 0.0f;
            int pStart = wheelPaths.pathStart[unit];
            int pEnd = pStart + wheelPaths.pathCount[unit];
            for (int p = pStart; p < pEnd; p++) {
                int wheel = wheelPaths.pathWheel[p];
                float gain = adjustedPathGain(wheelPaths.pathGain[p], wheelPaths.pathFlags[p],
                        gearboxFactor, rangeFactor);
                drivelineAV += gain * vehicle.wheels.getAngularVelocity(wheel);
                float inertia = vehicle.wheels.getRotationalInertia(wheel);
                if (inertia > 1e-7f) drivelineCompliance += gain * gain / inertia;
            }
            if (drivelineCompliance > 1e-9f) {
                float drivelineInertia = 1.0f / drivelineCompliance;
                if (clutchlikes.type[unit] == ClutchlikeContainer.TYPE_TORQUE_CONVERTER) {
                    float lockupRatio = Math.clamp(
                            (float) input.get(torqueConverters.lockupSignalId[unit]), 0.0f, 1.0f);
                    TorqueConverterSolver.solveInto(
                            dt, engines.engineAV[unit], drivelineAV,
                            engines.engineInertia[unit], drivelineInertia,
                            torqueConverters.couplingAVRatio[unit], torqueConverters.stallTorqueRatio[unit],
                            torqueConverters.converterStiffness[unit], torqueConverters.converterDiameter[unit],
                            torqueConverters.converterTorqueLimit[unit], lockupRatio,
                            torqueConverters, unit);
                    couplerInputTorque = torqueConverters.inputTorque[unit];
                    couplerOutputTorque = torqueConverters.outputTorque[unit];
                } else {
                    ImplicitCouplingSolver.solveInto(
                            dt, engines.engineAV[unit] - drivelineAV,
                            engines.engineInertia[unit], drivelineInertia,
                            clutches.clutchSpring[unit], clutches.clutchDampingRatio[unit],
                            clutches.clutchCapacity[unit], engagement,
                            clutches.clutchTorque, clutches.clutchAngle, unit);
                    couplerInputTorque = clutches.clutchTorque[unit];
                    couplerOutputTorque = couplerInputTorque;
                }
                engines.engineAV[unit] -= dt * couplerInputTorque / engines.engineInertia[unit];
                for (int p = pStart; p < pEnd; p++) {
                    float gain = adjustedPathGain(wheelPaths.pathGain[p], wheelPaths.pathFlags[p],
                            gearboxFactor, rangeFactor);
                    vehicle.wheels.applyDriveTorqueAndReaction(
                            wheelPaths.pathWheel[p], couplerOutputTorque * gain);
                }
            } else {
                clutches.clutchTorque[unit] = 0.0f;
                clutches.clutchAngle[unit] = 0.0f;
                torqueConverters.inputTorque[unit] = 0.0f;
                torqueConverters.outputTorque[unit] = 0.0f;
                torqueConverters.lockupTorque[unit] = 0.0f;
                torqueConverters.lockupAngle[unit] = 0.0f;
            }

            // Split shafts are independent coupling edges. They remain active in an
            // upstream gearbox neutral and can therefore still couple their axle groups.
            int splitEnd = splitShafts.unitStart[unit] + splitShafts.unitCount[unit];
            for (int split = splitShafts.unitStart[unit]; split < splitEnd; split++) {
                float primaryAV = 0.0f;
                float primaryCompliance = 0.0f;
                int primaryEnd = splitShafts.primaryPathStart[split]
                        + splitShafts.primaryPathCount[split];
                for (int p = splitShafts.primaryPathStart[split]; p < primaryEnd; p++) {
                    int wheel = splitShafts.pathWheel[p];
                    float gain = adjustedPathGain(splitShafts.pathGain[p], splitShafts.pathFlags[p],
                            gearboxFactor, rangeFactor);
                    primaryAV += gain * vehicle.wheels.getAngularVelocity(wheel);
                    float inertia = vehicle.wheels.getRotationalInertia(wheel);
                    if (inertia > 1e-7f) primaryCompliance += gain * gain / inertia;
                }
                float secondaryAV = 0.0f;
                float secondaryCompliance = 0.0f;
                int secondaryEnd = splitShafts.secondaryPathStart[split]
                        + splitShafts.secondaryPathCount[split];
                for (int p = splitShafts.secondaryPathStart[split]; p < secondaryEnd; p++) {
                    int wheel = splitShafts.pathWheel[p];
                    float gain = adjustedPathGain(splitShafts.pathGain[p], splitShafts.pathFlags[p],
                            gearboxFactor, rangeFactor);
                    secondaryAV += gain * vehicle.wheels.getAngularVelocity(wheel);
                    float inertia = vehicle.wheels.getRotationalInertia(wheel);
                    if (inertia > 1e-7f) secondaryCompliance += gain * gain / inertia;
                }
                if (primaryCompliance <= 1e-9f || secondaryCompliance <= 1e-9f) {
                    SplitShaftSolver.clear(splitShafts, split);
                    continue;
                }
                float splitTorque = SplitShaftSolver.solve(
                        dt, primaryAV, secondaryAV, 1.0f / primaryCompliance,
                        1.0f / secondaryCompliance, splitShafts, split);
                for (int p = splitShafts.primaryPathStart[split]; p < primaryEnd; p++) {
                    float gain = adjustedPathGain(splitShafts.pathGain[p], splitShafts.pathFlags[p],
                            gearboxFactor, rangeFactor);
                    vehicle.wheels.applyDriveTorqueAndReaction(splitShafts.pathWheel[p], -splitTorque * gain);
                }
                for (int p = splitShafts.secondaryPathStart[split]; p < secondaryEnd; p++) {
                    float gain = adjustedPathGain(splitShafts.pathGain[p], splitShafts.pathFlags[p],
                            gearboxFactor, rangeFactor);
                    vehicle.wheels.applyDriveTorqueAndReaction(splitShafts.pathWheel[p], splitTorque * gain);
                }
            }
            if (engines.engineAV[unit] < 0.0f) engines.engineAV[unit] = 0.0f;

            // BeamNG applies crank inertial torque at the engine reaction nodes,
            // while each torsionReactor closes the downstream driveline torque
            // on its own axis. Keeping those axes separate matters on longitudinal
            // layouts where the crank and wheel axes are perpendicular.
            applyReactionTorque(reactions.reactionStart[unit], reactions.reactionCount[unit],
                    externalTorque - couplerInputTorque);
            int rEnd = reactions.reactorStart[unit] + reactions.reactorCount[unit];
            for (int reactor = reactions.reactorStart[unit]; reactor < rEnd; reactor++) {
                applyReactionTorque(reactions.reactorNodeStart[reactor], reactions.reactorNodeCount[reactor],
                        couplerOutputTorque * reactions.reactorGain[reactor] * ratioFactor);
            }

            if (unit == 0) {
                debugEngineRPM = engines.engineAV[0] * AV_TO_RPM;
                debugClutchTorque = clutchlikes.type[0] == ClutchlikeContainer.TYPE_TORQUE_CONVERTER
                        ? torqueConverters.outputTorque[0] : clutches.clutchTorque[0];
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

    /** Smooth below-idle recovery with a model-based converter-load floor. */
    private float idleControllerOutput(int unit, boolean running, boolean belowIdle, float dt) {
        float baseThrottle = engines.idleLossThrottle[unit];
        if (!running || !belowIdle) {
            engines.idleControlThrottle[unit] = baseThrottle;
            return baseThrottle;
        }

        float engineAV = Math.max(0.0f, engines.engineAV[unit]);
        float torque = interpolateTorque(unit, engineAV * AV_TO_RPM);
        float loss = engines.engineFriction[unit]
                + engines.engineDynamicFriction[unit] * engineAV;
        // The current converter reaction is available on the following physics substep.
        // Solving the simplified linear combustion model backwards provides the opening
        // that balances that load without turning it into an unconditional torque source.
        float converterLoad = clutchlikes.type[unit] == ClutchlikeContainer.TYPE_TORQUE_CONVERTER
                ? Math.max(0.0f, torqueConverters.inputTorque[unit])
                : 0.0f;
        float requiredByLoad = torque > 1.0e-3f
                ? Math.clamp((loss + converterLoad) / torque, 0.0f, 1.0f)
                : 1.0f;

        // Equivalent to FInterpTo(current, 1, dt, 1): a deliberately slow recovery
        // avoids the substep-to-substep chatter of a raw proportional controller.
        float alpha = Math.clamp(dt, 0.0f, 1.0f);
        float smoothedRecovery = Math.fma(1.0f - engines.idleControlThrottle[unit], alpha,
                engines.idleControlThrottle[unit]);
        float result = Math.max(requiredByLoad, smoothedRecovery);
        engines.idleControlThrottle[unit] = result;
        return result;
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

    private void consumeRangeBoxEvents(ElectricSnapshot input) {
        long toggle = eventSequence(input.get(rangeBoxToggleSignalId));
        int toggleCount = eventCount(lastRangeBoxToggleEvent, toggle);
        lastRangeBoxToggleEvent = toggle;
        if ((toggleCount & 1) == 0) return;
        for (int unit = 0; unit < rangeBoxes.unitCount; unit++) {
            if (rangeBoxes.device[unit] >= 0) {
                setRangeBoxMode(unit, !rangeBoxes.lowMode[unit]);
            }
        }
    }

    private void setRangeBoxMode(int unit, boolean low) {
        if (unit < 0 || unit >= rangeBoxes.unitCount || rangeBoxes.device[unit] < 0) return;
        rangeBoxes.lowMode[unit] = low;
        rangeBoxes.activeRatio[unit] = low
                ? rangeBoxes.lowRatio[unit] : rangeBoxes.highRatio[unit];
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

    private static float adjustedPathGain(float baseGain, byte flags,
                                          float gearboxFactor, float rangeFactor) {
        float gain = baseGain;
        if ((flags & DrivenWheelPathContainer.FLAG_GEARBOX) != 0) gain *= gearboxFactor;
        if ((flags & DrivenWheelPathContainer.FLAG_RANGE_BOX) != 0) gain *= rangeFactor;
        return gain;
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
            engines.idleControlThrottle[i] = engines.idleLossThrottle[i];
            clutches.clutchAngle[i] = 0.0f;
            clutches.clutchTorque[i] = 0.0f;
            torqueConverters.lockupAngle[i] = 0.0f;
            torqueConverters.lockupTorque[i] = 0.0f;
            torqueConverters.inputTorque[i] = 0.0f;
            torqueConverters.outputTorque[i] = 0.0f;
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
            setRangeBoxMode(i, false);
            gearboxes.shiftRemaining[i] = 0.0f;
        }
        for (int split = 0; split < splitShafts.count; split++) {
            SplitShaftSolver.clear(splitShafts, split);
            splitShafts.activeMode[split] = splitShafts.initialMode[split];
            splitShafts.clutchRatio[split] = splitShafts.defaultClutchRatio[split];
        }
        lastShiftUpEvent = 0L;
        lastShiftDownEvent = 0L;
        lastRangeBoxToggleEvent = 0L;
        vehicle.electrics.set(throttleSignalId, 0.0);
        vehicle.electrics.set(clutchSignalId, 0.0);
        vehicle.electrics.set(starterSignalId, 0.0);
        vehicle.electrics.set(shiftUpSignalId, 0.0);
        vehicle.electrics.set(shiftDownSignalId, 0.0);
        vehicle.electrics.set(rangeBoxToggleSignalId, 0.0);
        vehicle.electrics.set(defaultLockupSignalId, 0.0);
        for (int signalId : torqueConverters.lockupSignalId) vehicle.electrics.set(signalId, 0.0);
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
        lastRangeBoxToggleEvent = 0L;
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
    public String debugRangeBoxMode() {
        if (rangeBoxes.unitCount == 0 || rangeBoxes.device[0] < 0) return "-";
        return rangeBoxes.lowMode[0] ? "low" : "high";
    }
    public float debugRangeBoxRatio() {
        return rangeBoxes.unitCount > 0 ? rangeBoxes.activeRatio[0] : 1.0f;
    }

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
