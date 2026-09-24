/*
 * This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
 * If a copy of the bCDDL was not distributed with this file, see LICENSES/bCDDL-1.1.txt.
 * Adapted from BeamNG.drive Lua powertrain sources. Java adaptation and modifications
 * contributed by M1AO and BeamCraft contributors. See SOURCE_PROVENANCE.md.
 */
package me.mzy.beamcraft.client.physics.powertrain;

import me.mzy.beamcraft.client.physics.NodeContainer;
import me.mzy.beamcraft.client.physics.SoftBodyVehicle;
import me.mzy.beamcraft.client.physics.TorqueReactionSolver;
import me.mzy.beamcraft.client.physics.electrics.ElectricSignals;
import me.mzy.beamcraft.client.physics.electrics.ElectricSnapshot;
import me.mzy.beamcraft.client.physics.electrics.ElectricValues;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.DeviceSpec;

import java.util.ArrayList;
import java.util.List;

/**
 * BeamNG-compatible rigid powertrain forest with one compliant {@code clutchlike}
 * boundary (friction clutch, torque converter or DCT) downstream of each combustion engine.
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
 *   <li>below the idle target, the controller directly solves the throttle needed to
 *       balance current losses/load plus a 5% recovery margin; above idle it yields;</li>
 *   <li>a time/soft rev limiter cuts spark+fuel for {@code revLimiterCutTime} and only
 *       retriggers while the crank is still above the hysteresis threshold — RPM is never
 *       clamped directly.</li>
 * </ul>
 *
 * <p>Gearboxes are runtime SoA: the active ratio scales the compile-time first-gear wheel
 * paths. Conventional boxes disconnect for the shift duration; DCTs linearly hand torque
 * between their two selected ratios. All shift/limiter timers use {@code solve(dt)} only.
 * The clutch torque is governed solely by {@link ImplicitCouplingSolver} and its friction
 * capacity — it may stall the engine; it is never clamped to the engine's sustainable
 * torque.
 */
public final class PowertrainSystem {
    static final float RPM_TO_AV = (float) (Math.PI / 30.0);
    static final float AV_TO_RPM = 1.0f / RPM_TO_AV;
    private static final float PSI_TO_PA = 6894.7573f;
    private static final float TURBO_REFERENCE_DT = 0.01f;

    private final SoftBodyVehicle vehicle;
    private final List<DeviceSpec> pendingSpecs = new ArrayList<>();

    // Runtime containers adopted on each finalizeSetup. Read by solve() and by the HUD.
    private PowertrainData data = new PowertrainData();
    public final PowertrainTopologyContainer topology = data.topology;
    public final CombustionEngineContainer engines = data.engines;
    public final ElectricMotorContainer electricMotors = data.electricMotors;
    public final TurbochargerContainer turbochargers = data.turbochargers;
    public final SuperchargerContainer superchargers = data.superchargers;
    public final FrictionClutchContainer clutches = data.clutches;
    public final ClutchlikeContainer clutchlikes = data.clutchlikes;
    public final TorqueConverterContainer torqueConverters = data.torqueConverters;
    public final DctGearboxContainer dctGearboxes = data.dctGearboxes;
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
    private volatile float debugTurboRPM;
    private volatile float debugTurboBoostPSI;
    private volatile float debugSuperchargerRPM;
    private volatile float debugSuperchargerBoostPSI;
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
        debugTurboRPM = 0.0f;
        debugTurboBoostPSI = 0.0f;
        debugSuperchargerRPM = 0.0f;
        debugSuperchargerBoostPSI = 0.0f;
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

    /** Selects one of the modes declared by a named differential's {@code diffType}. */
    public void setDifferentialMode(String deviceName, String mode) {
        byte resolved = DifferentialSolver.mode(mode);
        if (resolved < 0) return;
        int modeBit = 1 << resolved;
        for (int differential = 0; differential < differentials.count; differential++) {
            if (!differentials.deviceName[differential].equals(deviceName)) continue;
            if ((differentials.availableModes[differential] & modeBit) == 0) continue;
            differentials.activeMode[differential] = resolved;
            DifferentialSolver.clear(differentials, differential);
        }
    }

    /** Commands an active differential clutch; controllers may call this with a 0..1 ratio. */
    public void setDifferentialActiveLock(String deviceName, float ratio) {
        float clamped = Math.clamp(ratio, 0.0f, 1.0f);
        for (int differential = 0; differential < differentials.count; differential++) {
            if (differentials.deviceName[differential].equals(deviceName)) {
                differentials.activeLockCoef[differential] = clamped;
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
    public void solve(float dt, ElectricValues electrics) {
        if (dt <= 0.0f) return;
        ElectricValues input = electrics == null ? ElectricSnapshot.EMPTY : electrics;
        consumeShiftEvents(input);
        consumeRangeBoxEvents(input);
        if (engines.unitCount == 0 && electricMotors.motorCount == 0) return;
        float throttle = Math.clamp((float) input.get(throttleSignalId), 0.0f, 1.0f);
        solveElectricMotors(throttle);
        if (engines.unitCount == 0) {
            debugThrottle = throttle;
            return;
        }
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
            float forcedInductionCoef = updateTurbocharger(unit, throttle, rpm, crankRunning, dt)
                    * updateSupercharger(unit, throttle, rpm, dt);
            float idleOutput = idleControllerOutput(unit, crankRunning, belowIdle, forcedInductionCoef);
            // Above idle, the player command is not held above a synthetic idle opening.
            // The idle controller only takes authority after the crank falls below target.
            float actualThrottle = belowIdle ? Math.max(throttle, idleOutput) : throttle;
            boolean combustionRequested = throttle > 1.0e-6f || belowIdle;
            boolean combustionEnabled = crankRunning && combustionRequested
                    && engines.sparkEnabled[unit] && engines.fuelEnabled[unit];
            engines.playerThrottle[unit] = throttle;
            engines.actualThrottle[unit] = actualThrottle;
            float availableCombustionTorque = Math.max(0.0f,
                    interpolateTorque(unit, rpm) * forcedInductionCoef);
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
            float engineBrakeTorque = engines.engineBrakeTorque[unit]
                    * (1.0f - normalizedCombustionOutput);
            float loss = engines.engineFriction[unit]
                    + engines.engineDynamicFriction[unit] * engineAVBeforeExternal
                    + engineBrakeTorque;
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
            boolean isDct = clutchlikes.type[unit] == ClutchlikeContainer.TYPE_DCT_GEARBOX;
            for (int p = pStart; p < pEnd; p++) {
                int wheel = wheelPaths.pathWheel[p];
                float gain = adjustedPathGain(wheelPaths.pathGain[p], wheelPaths.pathFlags[p],
                        isDct ? 1.0f : gearboxFactor, rangeFactor);
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
                } else if (isDct) {
                    float ratio1 = gearboxRatio(unit, dctGearboxes.gearIndex1[unit]);
                    float ratio2 = gearboxRatio(unit, dctGearboxes.gearIndex2[unit]);
                    float factor1 = pathBaseRatio > 1.0e-6f ? ratio1 / pathBaseRatio : 0.0f;
                    float factor2 = pathBaseRatio > 1.0e-6f ? ratio2 / pathBaseRatio : 0.0f;
                    float launch1 = dctLaunchEngagement(unit, dctGearboxes.gearIndex1[unit],
                            factor1 * drivelineAV, throttle);
                    float launch2 = dctLaunchEngagement(unit, dctGearboxes.gearIndex2[unit],
                            factor2 * drivelineAV, throttle);
                    DctCouplingSolver.solveInto(
                            dt, engines.engineAV[unit], drivelineAV,
                            engines.engineInertia[unit], drivelineCompliance,
                            factor1, factor2, dctGearboxes.clutchSpring[unit],
                            dctGearboxes.clutchDampingRatio1[unit], dctGearboxes.clutchDampingRatio2[unit],
                            dctGearboxes.clutchCapacity[unit],
                            dctGearboxes.engagement1[unit] * engagement * launch1,
                            dctGearboxes.engagement2[unit] * engagement * launch2,
                            dctGearboxes, unit);
                    couplerInputTorque = dctGearboxes.clutchTorque1[unit]
                            + dctGearboxes.clutchTorque2[unit];
                    couplerOutputTorque = dctGearboxes.clutchTorque1[unit] * factor1
                            + dctGearboxes.clutchTorque2[unit] * factor2;
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
                            isDct ? 1.0f : gearboxFactor, rangeFactor);
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
                dctGearboxes.clutchTorque1[unit] = 0.0f;
                dctGearboxes.clutchTorque2[unit] = 0.0f;
                dctGearboxes.clutchAngle1[unit] = 0.0f;
                dctGearboxes.clutchAngle2[unit] = 0.0f;
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

            // A differential's ordinary drive torque is already distributed by the rigid
            // wheel paths. This pass adds only the equal-and-opposite limited-slip torque.
            int differentialEnd = differentials.unitStart[unit] + differentials.unitCount[unit];
            for (int differential = differentials.unitStart[unit];
                 differential < differentialEnd; differential++) {
                float outputAV1 = 0.0f;
                float outputCompliance1 = 0.0f;
                int output1End = differentials.output1PathStart[differential]
                        + differentials.output1PathCount[differential];
                for (int p = differentials.output1PathStart[differential]; p < output1End; p++) {
                    int wheel = differentials.pathWheel[p];
                    float gain = adjustedPathGain(differentials.pathGain[p], differentials.pathFlags[p],
                            gearboxFactor, rangeFactor);
                    outputAV1 += gain * vehicle.wheels.getAngularVelocity(wheel);
                    float inertia = vehicle.wheels.getRotationalInertia(wheel);
                    if (inertia > 1.0e-7f) outputCompliance1 += gain * gain / inertia;
                }
                float outputAV2 = 0.0f;
                float outputCompliance2 = 0.0f;
                int output2End = differentials.output2PathStart[differential]
                        + differentials.output2PathCount[differential];
                for (int p = differentials.output2PathStart[differential]; p < output2End; p++) {
                    int wheel = differentials.pathWheel[p];
                    float gain = adjustedPathGain(differentials.pathGain[p], differentials.pathFlags[p],
                            gearboxFactor, rangeFactor);
                    outputAV2 += gain * vehicle.wheels.getAngularVelocity(wheel);
                    float inertia = vehicle.wheels.getRotationalInertia(wheel);
                    if (inertia > 1.0e-7f) outputCompliance2 += gain * gain / inertia;
                }
                if (outputCompliance1 <= 1.0e-9f || outputCompliance2 <= 1.0e-9f) {
                    DifferentialSolver.clear(differentials, differential);
                    continue;
                }

                float differentialInputTorque = 0.0f;
                int termEnd = differentials.inputTermStart[differential]
                        + differentials.inputTermCount[differential];
                for (int term = differentials.inputTermStart[differential]; term < termEnd; term++) {
                    int sourceSplit = differentials.termSplit[term];
                    float sourceTorque = sourceSplit < 0
                            ? couplerOutputTorque : splitOutputTorque(sourceSplit);
                    float gain = adjustedPathGain(differentials.termGain[term],
                            differentials.termFlags[term], isDct ? 1.0f : gearboxFactor, rangeFactor);
                    differentialInputTorque += sourceTorque * gain;
                }
                float lockTorque = DifferentialSolver.solve(
                        dt, outputAV1, outputAV2,
                        1.0f / outputCompliance1, 1.0f / outputCompliance2,
                        differentialInputTorque, differentials, differential);
                for (int p = differentials.output1PathStart[differential]; p < output1End; p++) {
                    float gain = adjustedPathGain(differentials.pathGain[p], differentials.pathFlags[p],
                            gearboxFactor, rangeFactor);
                    vehicle.wheels.applyDriveTorqueAndReaction(differentials.pathWheel[p], -lockTorque * gain);
                }
                for (int p = differentials.output2PathStart[differential]; p < output2End; p++) {
                    float gain = adjustedPathGain(differentials.pathGain[p], differentials.pathFlags[p],
                            gearboxFactor, rangeFactor);
                    vehicle.wheels.applyDriveTorqueAndReaction(differentials.pathWheel[p], lockTorque * gain);
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
                float reactorTorque = 0.0f;
                int termEnd = reactions.reactorTermStart[reactor]
                        + reactions.reactorTermCount[reactor];
                for (int term = reactions.reactorTermStart[reactor]; term < termEnd; term++) {
                    int sourceSplit = reactions.termSplit[term];
                    float sourceTorque = sourceSplit < 0
                            ? couplerOutputTorque : splitOutputTorque(sourceSplit);
                    float gain = adjustedPathGain(reactions.termGain[term], reactions.termFlags[term],
                            isDct ? 1.0f : gearboxFactor, rangeFactor);
                    reactorTorque += sourceTorque * gain;
                }
                applyReactionTorque(reactions.reactorNodeStart[reactor],
                        reactions.reactorNodeCount[reactor], reactorTorque);
            }

            if (unit == 0) {
                debugEngineRPM = engines.engineAV[0] * AV_TO_RPM;
                debugClutchTorque = clutchlikes.type[0] == ClutchlikeContainer.TYPE_TORQUE_CONVERTER
                        ? torqueConverters.outputTorque[0]
                        : clutchlikes.type[0] == ClutchlikeContainer.TYPE_DCT_GEARBOX
                        ? dctGearboxes.clutchTorque1[0] + dctGearboxes.clutchTorque2[0]
                        : clutches.clutchTorque[0];
                debugCombustionTorque = combustionTorque;
                debugTurboRPM = turbochargers.turboAV[0] * AV_TO_RPM;
                debugTurboBoostPSI = turbochargers.pressurePa[0] / PSI_TO_PA;
                debugSuperchargerRPM = superchargers.blowerRPM[0];
                debugSuperchargerBoostPSI = superchargers.pressurePa[0] / PSI_TO_PA;
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

    /** Below-idle rescue obtained by solving the current loss/load torque balance. */
    private float idleControllerOutput(int unit, boolean running, boolean belowIdle,
                                       float forcedInductionCoef) {
        if (!running || !belowIdle) {
            engines.idleControlThrottle[unit] = 0.0f;
            return 0.0f;
        }

        float engineAV = Math.max(0.0f, engines.engineAV[unit]);
        float torque = interpolateTorque(unit, engineAV * AV_TO_RPM) * forcedInductionCoef;
        float loss = engines.engineFriction[unit]
                + engines.engineDynamicFriction[unit] * engineAV;
        // The current converter reaction is available on the following physics substep.
        // Solving the simplified linear combustion model backwards provides the opening
        // that balances that load without turning it into an unconditional torque source.
        float converterLoad = clutchlikes.type[unit] == ClutchlikeContainer.TYPE_TORQUE_CONVERTER
                ? Math.max(0.0f, torqueConverters.inputTorque[unit])
                : 0.0f;
        // At idle the normalized combustion output equals the commanded throttle in the
        // current linear model. Solve
        //   throttle*T = loss + brake*(1-throttle) + converterLoad
        // for the feedforward opening needed to stop the RPM fall.
        float brake = engines.engineBrakeTorque[unit];
        float requiredByLoad = torque + brake > 1.0e-3f
                ? Math.clamp((loss + brake + converterLoad) / (torque + brake) + 0.05f, 0.0f, 1.0f)
                : 1.0f;
        engines.idleControlThrottle[unit] = requiredByLoad;
        return requiredByLoad;
    }

    /** Advances one BeamNG-compatible empirical turbo model at the physics substep rate. */
    private float updateTurbocharger(int unit, float requestedThrottle, float engineRPM,
                                     boolean running, float dt) {
        if (!turbochargers.existing[unit]) return 1.0f;

        float currentLoad = engines.normalizedCombustionOutput[unit];
        boolean lowLoad = currentLoad < turbochargers.bovOpenThreshold[unit]
                || requestedThrottle <= 1.0e-6f;
        boolean suddenDrop = turbochargers.lastCombustionOutput[unit] - currentLoad
                > turbochargers.bovOpenChangeThreshold[unit];
        boolean bov = turbochargers.bovEnabled[unit] && (lowLoad || suddenDrop);
        turbochargers.bovEngaged[unit] = bov;
        turbochargers.lastCombustionOutput[unit] = currentLoad;

        float targetBoost = 0.5f * (turbochargers.wastegateStartPa[unit]
                + turbochargers.wastegateLimitPa[unit]);
        float boostError = turbochargers.rawPressurePa[unit] - targetBoost;
        float integral = Math.clamp(turbochargers.wastegateIntegral[unit] + boostError * dt,
                -50.0f, 500.0f);
        float derivative = dt > 1.0e-9f
                ? (boostError - turbochargers.lastBoostError[unit]) / dt : 0.0f;
        float wastegate = bov ? 0.0f : Math.clamp(1.0f
                - boostError * turbochargers.wastegateP[unit]
                - integral * turbochargers.wastegateI[unit]
                - derivative * turbochargers.wastegateD[unit], 0.0f, 1.0f);
        turbochargers.wastegateIntegral[unit] = integral;
        turbochargers.lastBoostError[unit] = boostError;
        turbochargers.wastegateFactor[unit] = wastegate;

        float engineSpeedRatio = Math.clamp(engineRPM
                / Math.max(1.0f, engines.revLimiterRPM[unit]), 0.0f, 1.0f);
        float exhaustCurve = interpolateTurboEngineCurve(unit, engineRPM, false);
        float exhaustDrive = (0.1f + currentLoad * 0.8f)
                * requestedThrottle * requestedThrottle * engineSpeedRatio * exhaustCurve
                * turbochargers.maxExhaustPower[unit];
        float turboAV = turbochargers.turboAV[unit];
        float bovBackPressure = bov ? 0.4f : 1.0f;
        float compressorLoad = turboAV * turboAV * turbochargers.backPressureCoef[unit]
                * bovBackPressure;
        float axisTorque = ((exhaustDrive * wastegate) - compressorLoad
                - turbochargers.frictionCoef[unit]) * TURBO_REFERENCE_DT;
        turboAV = Math.clamp(turboAV + dt * axisTorque / turbochargers.inertia[unit],
                0.0f, turbochargers.maxAV[unit]);
        turbochargers.turboAV[unit] = turboAV;

        float turboRPM = turboAV * AV_TO_RPM;
        float rawPressure = running ? interpolateTurboPressure(unit, turboRPM) * PSI_TO_PA : 0.0f;
        turbochargers.rawPressurePa[unit] = rawPressure;
        float pressureTarget = bov ? 0.0f : rawPressure;
        float pressure = turbochargers.pressurePa[unit];
        float pressureRate = pressureTarget > pressure ? 200.0f * PSI_TO_PA
                : turbochargers.pressureFallRatePa[unit];
        float maxPressureStep = Math.max(0.0f, pressureRate) * dt;
        pressure += Math.clamp(pressureTarget - pressure, -maxPressureStep, maxPressureStep);
        turbochargers.pressurePa[unit] = pressure;

        float efficiency = interpolateTurboEngineCurve(unit, engineRPM, true);
        return Math.max(0.0f, 1.0f + 0.0000087f * pressure * efficiency);
    }

    /** Advances the belt-driven blower and pressure response at the physics substep rate. */
    private float updateSupercharger(int unit, float requestedThrottle, float engineRPM, float dt) {
        if (!superchargers.existing[unit]) return 1.0f;

        float engage = Math.clamp((engineRPM - superchargers.clutchEngageRPM[unit])
                / superchargers.clutchEngageRange[unit], 0.0f, 1.0f);
        float disengage = Math.clamp((superchargers.clutchDisengageRPM[unit]
                + superchargers.clutchDisengageRange[unit] - engineRPM)
                / superchargers.clutchDisengageRange[unit], 0.0f, 1.0f);
        float blowerRPM = engineRPM * superchargers.gearRatio[unit] * Math.min(engage, disengage);
        superchargers.blowerRPM[unit] = blowerRPM;

        // BeamNG's generated pressure curve returns its last sample above configured maxRPM.
        float pressureCurveRPM = Math.min(blowerRPM, superchargers.maxBlowerRPM[unit]);
        float relativeRPM = Math.clamp(pressureCurveRPM
                / superchargers.maxBlowerRPM[unit], 0.0f, 1.0f);
        float efficiency = Math.max(0.0f,
                superchargers.efficiencyB1[unit] * relativeRPM * relativeRPM
                        + superchargers.efficiencyB2[unit] * relativeRPM
                        + superchargers.efficiencyB3[unit]);
        float phase = superchargers.pulsePhase[unit]
                + superchargers.pulseLobes[unit] * blowerRPM * (1.0f / 60.0f) * dt;
        if (phase >= 2.0f * Math.PI) phase %= (float) (2.0 * Math.PI);
        superchargers.pulsePhase[unit] = phase;
        float pulse = 0.5f * (1.0f + (float) Math.sin(phase));
        float pulseCoef = Math.fma(1.0f - superchargers.pulseFloor[unit], pulse,
                superchargers.pulseFloor[unit]);
        if (pulseCoef > 0.9f) pulseCoef = 1.0f;

        float boostControl = interpolateSuperchargerController(unit, requestedThrottle * 100.0f);
        float rawPressure = requestedThrottle < 0.01f ? 0.0f
                : efficiency * superchargers.pressurePSIPerRPM[unit] * pressureCurveRPM
                * pulseCoef * boostControl * PSI_TO_PA;
        superchargers.rawPressurePa[unit] = rawPressure;
        float pressure = superchargers.pressurePa[unit];
        float maxPressureChange = superchargers.pressureRatePa[unit] * dt;
        pressure += Math.clamp(rawPressure - pressure, -maxPressureChange, maxPressureChange);
        superchargers.pressurePa[unit] = Math.max(0.0f, pressure);

        float lostTorqueCoef = superchargers.crankLossPerRPM[unit] * blowerRPM;
        superchargers.lostTorqueCoef[unit] = lostTorqueCoef;
        return Math.max(0.0f, 1.0f + 0.0000087f * pressure - lostTorqueCoef);
    }

    private float interpolateSuperchargerController(int unit, float throttlePercent) {
        int start = superchargers.controllerStart[unit];
        int count = superchargers.controllerCount[unit];
        if (count <= 0) return 1.0f;
        if (throttlePercent <= superchargers.controllerThrottle[start]) {
            return superchargers.controllerFactor[start];
        }
        int end = start + count - 1;
        for (int i = start + 1; i <= end; i++) {
            if (throttlePercent > superchargers.controllerThrottle[i]) continue;
            float x0 = superchargers.controllerThrottle[i - 1];
            float x1 = superchargers.controllerThrottle[i];
            float t = x1 > x0 ? (throttlePercent - x0) / (x1 - x0) : 0.0f;
            return Math.fma(t, superchargers.controllerFactor[i]
                    - superchargers.controllerFactor[i - 1], superchargers.controllerFactor[i - 1]);
        }
        return superchargers.controllerFactor[end];
    }

    private float interpolateTurboPressure(int unit, float rpm) {
        int start = turbochargers.pressureStart[unit];
        int count = turbochargers.pressureCount[unit];
        if (count <= 0) return 0.0f;
        if (rpm <= turbochargers.pressureRPM[start]) return turbochargers.pressurePSI[start];
        int end = start + count - 1;
        for (int i = start + 1; i <= end; i++) {
            if (rpm > turbochargers.pressureRPM[i]) continue;
            float x0 = turbochargers.pressureRPM[i - 1], x1 = turbochargers.pressureRPM[i];
            float t = x1 > x0 ? (rpm - x0) / (x1 - x0) : 0.0f;
            return Math.fma(t, turbochargers.pressurePSI[i] - turbochargers.pressurePSI[i - 1],
                    turbochargers.pressurePSI[i - 1]);
        }
        return turbochargers.pressurePSI[end];
    }

    private float interpolateTurboEngineCurve(int unit, float rpm, boolean efficiencyCurve) {
        int start = turbochargers.engineStart[unit];
        int count = turbochargers.engineCount[unit];
        if (count <= 0) return 0.0f;
        float[] values = efficiencyCurve ? turbochargers.efficiency : turbochargers.exhaustFactor;
        if (rpm <= turbochargers.engineRPM[start]) return values[start];
        int end = start + count - 1;
        for (int i = start + 1; i <= end; i++) {
            if (rpm > turbochargers.engineRPM[i]) continue;
            float x0 = turbochargers.engineRPM[i - 1], x1 = turbochargers.engineRPM[i];
            float t = x1 > x0 ? (rpm - x0) / (x1 - x0) : 0.0f;
            return Math.fma(t, values[i] - values[i - 1], values[i - 1]);
        }
        return values[end];
    }

    /**
     * Advances the gearbox shift timer and keeps the active ratio in sync. The timer only
     * changes here, inside {@link #solve(float)}, never on the wall/game clock.
     */
    private void updateShift(int unit, float dt) {
        if (clutchlikes.type[unit] == ClutchlikeContainer.TYPE_DCT_GEARBOX) {
            updateDctShift(unit, dt);
            return;
        }
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
        if (clutchlikes.type[unit] == ClutchlikeContainer.TYPE_DCT_GEARBOX) return;
        if (gearboxes.shiftRemaining[unit] <= 0.0f) {
            gearboxes.shiftRemaining[unit] = Math.max(0.0f, gearboxes.shiftDuration[unit]);
            gearboxes.activeRatio[unit] = 0.0f; // torque path disconnected from shift start
        }
    }

    private void consumeShiftEvents(ElectricValues input) {
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

    private void consumeRangeBoxEvents(ElectricValues input) {
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

    /** Basic manual DCT controller: adjacent preselection followed by a linear clutch handoff. */
    private void updateDctShift(int unit, float dt) {
        if (dctGearboxes.shiftTarget[unit] < 0) {
            int requested = gearboxes.pendingGearIndex[unit];
            int current = gearboxes.currentGearIndex[unit];
            if (requested >= 0 && requested != current) {
                int next = current + Integer.signum(requested - current);
                startDctHandoff(unit, next);
            } else {
                if (requested == current) gearboxes.pendingGearIndex[unit] = -1;
                gearboxes.activeRatio[unit] = gearboxRatio(unit, current);
                preselectDctGear(unit);
                return;
            }
        }

        float duration = Math.max(0.0f, gearboxes.shiftDuration[unit]);
        gearboxes.shiftRemaining[unit] = Math.max(0.0f, gearboxes.shiftRemaining[unit] - dt);
        float progress = duration <= 1.0e-6f
                ? 1.0f : Math.clamp(1.0f - gearboxes.shiftRemaining[unit] / duration, 0.0f, 1.0f);
        int from = dctGearboxes.primaryClutch[unit];
        int to = dctGearboxes.targetClutch[unit];
        boolean fromNeutral = Math.abs(gearboxRatio(unit, gearboxes.currentGearIndex[unit])) <= 1.0e-6f;
        if (to < 0) {
            setDctEngagements(unit, from == 0 ? 1.0f - progress : 0.0f,
                    from == 1 ? 1.0f - progress : 0.0f);
        } else if (fromNeutral) {
            setDctEngagements(unit, to == 0 ? progress : 0.0f, to == 1 ? progress : 0.0f);
        } else {
            setDctEngagements(unit,
                    from == 0 ? 1.0f - progress : progress,
                    from == 1 ? 1.0f - progress : progress);
        }
        gearboxes.activeRatio[unit] = dctEngagementWeightedRatio(unit);

        if (progress >= 1.0f) {
            int completed = dctGearboxes.shiftTarget[unit];
            gearboxes.currentGearIndex[unit] = completed;
            if (to >= 0) dctGearboxes.primaryClutch[unit] = (byte) to;
            setDctEngagements(unit, to == 0 ? 1.0f : 0.0f, to == 1 ? 1.0f : 0.0f);
            dctGearboxes.shiftTarget[unit] = -1;
            dctGearboxes.targetClutch[unit] = -1;
            gearboxes.shiftRemaining[unit] = 0.0f;
            gearboxes.activeRatio[unit] = gearboxRatio(unit, completed);
            if (gearboxes.pendingGearIndex[unit] == completed) gearboxes.pendingGearIndex[unit] = -1;
        }
    }

    private void startDctHandoff(int unit, int targetGear) {
        float targetRatio = gearboxRatio(unit, targetGear);
        int targetClutch = -1;
        if (Math.abs(targetRatio) > 1.0e-6f) {
            targetClutch = dctClutchForGear(unit, targetGear);
            if (targetClutch == 0) dctGearboxes.gearIndex1[unit] = targetGear;
            else dctGearboxes.gearIndex2[unit] = targetGear;
        }
        dctGearboxes.shiftTarget[unit] = targetGear;
        dctGearboxes.targetClutch[unit] = (byte) targetClutch;
        gearboxes.shiftRemaining[unit] = Math.max(0.0f, gearboxes.shiftDuration[unit]);
    }

    private int dctClutchForGear(int unit, int gearIndex) {
        if (dctGearboxes.gearIndex1[unit] == gearIndex) return 0;
        if (dctGearboxes.gearIndex2[unit] == gearIndex) return 1;
        float ratio = gearboxRatio(unit, gearIndex);
        if (ratio < 0.0f) return 0;
        int forwardOrdinal = 0;
        int start = gearboxes.gearStart[unit];
        for (int i = 0; i <= gearIndex && i < gearboxes.gearCount[unit]; i++) {
            if (gearboxes.gearRatios[start + i] > 1.0e-6f) forwardOrdinal++;
        }
        return (forwardOrdinal & 1) == 1 ? 0 : 1;
    }

    /** Keeps the open shaft on the adjacent manual gear without ever requesting a shift. */
    private void preselectDctGear(int unit) {
        int current = gearboxes.currentGearIndex[unit];
        if (gearboxRatio(unit, current) <= 1.0e-6f) return;
        int candidate = engines.playerThrottle[unit] > 1.0e-3f ? current + 1 : current - 1;
        if (candidate < 0 || candidate >= gearboxes.gearCount[unit]
                || gearboxRatio(unit, candidate) <= 1.0e-6f) return;
        int clutch = dctClutchForGear(unit, candidate);
        if (clutch == dctGearboxes.primaryClutch[unit]) return;
        if (clutch == 0) dctGearboxes.gearIndex1[unit] = candidate;
        else dctGearboxes.gearIndex2[unit] = candidate;
    }

    private void setDctEngagements(int unit, float first, float second) {
        dctGearboxes.engagement1[unit] = Math.clamp(first, 0.0f, 1.0f);
        dctGearboxes.engagement2[unit] = Math.clamp(second, 0.0f, 1.0f);
    }

    private float dctEngagementWeightedRatio(int unit) {
        return gearboxRatio(unit, dctGearboxes.gearIndex1[unit]) * dctGearboxes.engagement1[unit]
                + gearboxRatio(unit, dctGearboxes.gearIndex2[unit]) * dctGearboxes.engagement2[unit];
    }

    /** Gentle first/reverse launch and stall prevention; it never chooses a gear. */
    private float dctLaunchEngagement(int unit, int gearIndex, float gearedDrivelineAV, float throttle) {
        float ratio = gearboxRatio(unit, gearIndex);
        if (Math.abs(ratio) <= 1.0e-6f) return 0.0f;
        int first = firstPositiveGearIndex(unit);
        if (ratio > 0.0f && gearIndex != first) return 1.0f;
        float idle = Math.max(engines.idleAV[unit], 1.0f);
        float referenceAV = Math.max(engines.engineAV[unit], Math.abs(gearedDrivelineAV));
        float launchStart = idle * (1.0f + 0.25f * Math.clamp(throttle, 0.0f, 1.0f));
        float linear = Math.clamp((referenceAV - launchStart) / idle, 0.0f, 1.0f);
        return linear * linear;
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

    private int firstPositiveGearIndex(int unit) {
        return nextPositiveGearIndex(unit, -1);
    }

    private int nextPositiveGearIndex(int unit, int after) {
        int count = gearboxes.gearCount[unit];
        for (int index = Math.max(0, after + 1); index < count; index++) {
            if (gearboxRatio(unit, index) > 1.0e-6f) return index;
        }
        return -1;
    }

    private static float adjustedPathGain(float baseGain, byte flags,
                                          float gearboxFactor, float rangeFactor) {
        float gain = baseGain;
        if ((flags & DrivenWheelPathContainer.FLAG_GEARBOX) != 0) gain *= gearboxFactor;
        if ((flags & DrivenWheelPathContainer.FLAG_RANGE_BOX) != 0) gain *= rangeFactor;
        return gain;
    }

    private float splitOutputTorque(int split) {
        return splitShafts.activeMode[split] == SplitShaftContainer.MODE_VISCOUS
                ? splitShafts.viscousTorque[split] : splitShafts.lockTorque[split];
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
            engines.idleControlThrottle[i] = 0.0f;
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
            engines.actualThrottle[i] = 0.0f;
            engines.availableCombustionTorque[i] = 0.0f;
            engines.combustionTorque[i] = 0.0f;
            engines.normalizedCombustionOutput[i] = 0.0f;
            turbochargers.turboAV[i] = 0.0f;
            turbochargers.pressurePa[i] = 0.0f;
            turbochargers.rawPressurePa[i] = 0.0f;
            turbochargers.wastegateIntegral[i] = 0.0f;
            turbochargers.lastBoostError[i] = 0.0f;
            turbochargers.wastegateFactor[i] = 1.0f;
            turbochargers.lastCombustionOutput[i] = 0.0f;
            turbochargers.bovEngaged[i] = false;
            superchargers.blowerRPM[i] = 0.0f;
            superchargers.pressurePa[i] = 0.0f;
            superchargers.rawPressurePa[i] = 0.0f;
            superchargers.lostTorqueCoef[i] = 0.0f;
            superchargers.pulsePhase[i] = 0.0f;
            engines.limiterCutRemaining[i] = 0.0f;
            gearboxes.currentGearIndex[i] = gearboxes.initialGearIndex[i];
            gearboxes.pendingGearIndex[i] = -1;
            gearboxes.activeRatio[i] = gearboxRatio(i, gearboxes.initialGearIndex[i]);
            setRangeBoxMode(i, false);
            gearboxes.shiftRemaining[i] = 0.0f;
            if (clutchlikes.type[i] == ClutchlikeContainer.TYPE_DCT_GEARBOX) {
                int first = firstPositiveGearIndex(i);
                int second = nextPositiveGearIndex(i, first);
                dctGearboxes.gearIndex1[i] = first >= 0 ? first : gearboxes.initialGearIndex[i];
                dctGearboxes.gearIndex2[i] = second >= 0 ? second : dctGearboxes.gearIndex1[i];
                dctGearboxes.primaryClutch[i] = 0;
                dctGearboxes.targetClutch[i] = -1;
                dctGearboxes.shiftTarget[i] = -1;
                boolean firstEngaged = gearboxes.initialGearIndex[i] == first;
                setDctEngagements(i, firstEngaged ? 1.0f : 0.0f, 0.0f);
                dctGearboxes.clutchAngle1[i] = 0.0f;
                dctGearboxes.clutchAngle2[i] = 0.0f;
                dctGearboxes.clutchTorque1[i] = 0.0f;
                dctGearboxes.clutchTorque2[i] = 0.0f;
            }
        }
        for (int split = 0; split < splitShafts.count; split++) {
            SplitShaftSolver.clear(splitShafts, split);
            splitShafts.activeMode[split] = splitShafts.initialMode[split];
            splitShafts.clutchRatio[split] = splitShafts.defaultClutchRatio[split];
        }
        for (int differential = 0; differential < differentials.count; differential++) {
            DifferentialSolver.clear(differentials, differential);
            differentials.activeMode[differential] = differentials.initialMode[differential];
            differentials.activeLockCoef[differential] = 0.0f;
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
        debugTurboRPM = 0.0f;
        debugTurboBoostPSI = 0.0f;
        debugSuperchargerRPM = 0.0f;
        debugSuperchargerBoostPSI = 0.0f;
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
        debugTurboRPM = 0.0f;
        debugTurboBoostPSI = 0.0f;
        debugSuperchargerRPM = 0.0f;
        debugSuperchargerBoostPSI = 0.0f;
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
    public float debugTurboRPM() { return debugTurboRPM; }
    public float debugTurboBoostPSI() { return debugTurboBoostPSI; }
    public boolean debugTurboExisting() { return turbochargers.existing.length > 0 && turbochargers.existing[0]; }
    public float debugSuperchargerRPM() { return debugSuperchargerRPM; }
    public float debugSuperchargerBoostPSI() { return debugSuperchargerBoostPSI; }
    public boolean debugSuperchargerExisting() {
        return superchargers.existing.length > 0 && superchargers.existing[0];
    }

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

    private void solveElectricMotors(float throttle) {
        for (int motor = 0; motor < electricMotors.motorCount; motor++) {
            float motorAV = 0.0f;
            int end = electricMotors.pathStart[motor] + electricMotors.pathCount[motor];
            for (int path = electricMotors.pathStart[motor]; path < end; path++) {
                motorAV += electricMotors.pathGain[path]
                        * vehicle.wheels.getAngularVelocity(electricMotors.pathWheel[path]);
            }
            float torque = throttle * interpolateMotorTorque(motor, Math.abs(motorAV) * AV_TO_RPM);
            electricMotors.motorAV[motor] = motorAV;
            electricMotors.outputTorque[motor] = torque;
            for (int path = electricMotors.pathStart[motor]; path < end; path++) {
                vehicle.wheels.applyDriveTorqueAndReaction(
                        electricMotors.pathWheel[path], torque * electricMotors.pathGain[path]);
            }
        }
    }

    private float interpolateMotorTorque(int motor, float rpm) {
        int start = electricMotors.curveStart[motor];
        int count = electricMotors.curveCount[motor];
        if (count <= 0) return 0.0f;
        if (count == 1 || rpm <= electricMotors.curveRPM[start]) {
            return electricMotors.curveTorque[start];
        }
        int end = start + count;
        for (int i = start + 1; i < end; i++) {
            if (rpm <= electricMotors.curveRPM[i]) {
                float span = electricMotors.curveRPM[i] - electricMotors.curveRPM[i - 1];
                float t = span > 1.0e-6f ? (rpm - electricMotors.curveRPM[i - 1]) / span : 0.0f;
                return electricMotors.curveTorque[i - 1]
                        + (electricMotors.curveTorque[i] - electricMotors.curveTorque[i - 1]) * t;
            }
        }
        return electricMotors.curveTorque[end - 1];
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
