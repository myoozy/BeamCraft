package me.mzy.beamcraft.client.physics.powertrain;

import me.mzy.beamcraft.client.physics.SoftBodyVehicle;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.CombustionEngineSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.DeviceSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.DevicePatchSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.DifferentialSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.FrictionClutchSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.GearboxSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.ShaftSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.TorquePoint;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.TorsionReactorSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.UnsupportedConfig;

import static me.mzy.beamcraft.client.physics.powertrain.PowertrainTopologyContainer.TYPE_CLUTCH;
import static me.mzy.beamcraft.client.physics.powertrain.PowertrainTopologyContainer.TYPE_DIFFERENTIAL;
import static me.mzy.beamcraft.client.physics.powertrain.PowertrainTopologyContainer.TYPE_ENGINE;
import static me.mzy.beamcraft.client.physics.powertrain.PowertrainTopologyContainer.TYPE_GEARBOX;
import static me.mzy.beamcraft.client.physics.powertrain.PowertrainTopologyContainer.TYPE_UNSUPPORTED;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Build-time powertrain compiler. Turns the accumulated, still-mutable {@link List}<{@link DeviceSpec}>
 * rows into the flat SoA containers of a {@link PowertrainData}: it normalizes value modifiers
 * across active parts, validates the device graph (duplicate names, input resolution, occupied
 * ports, cycles), compiles engine→clutch units, driven wheel paths and torque reactions, and
 * resolves every node reference to a {@link me.mzy.beamcraft.client.physics.NodeContainer} index.
 *
 * <p>Nothing in this class runs on the physics substep path; {@link PowertrainSystem} only reads
 * the resulting containers.
 */
final class PowertrainCompiler {
    private static final Logger LOGGER = LoggerFactory.getLogger("BeamCraft/Powertrain");
    private PowertrainCompiler() {
    }

    /**
     * Compiles {@code rawSpecs} into {@code data}, which must have been cleared first
     * (callers are {@link PowertrainSystem#finalizeSetup}). Sets {@code data.diagnostic}.
     */
    static void compile(SoftBodyVehicle vehicle, List<DeviceSpec> rawSpecs, PowertrainData data) {
        if (rawSpecs == null || rawSpecs.isEmpty()) {
            data.diagnostic = "no powertrain data";
            return;
        }

        List<DeviceSpec> specs = PowertrainSpecNormalizer.normalize(rawSpecs);
        PowertrainTopologyContainer topology = data.topology;
        int deviceCount = specs.size();
        topology.allocateDevices(deviceCount);
        Arrays.fill(topology.parentDevice, -1);

        Map<String, Integer> names = new HashMap<>();
        for (int i = 0; i < deviceCount; i++) {
            DeviceSpec spec = specs.get(i);
            topology.deviceName[i] = spec.name();
            topology.deviceType[i] = typeOf(spec);
            topology.deviceRatio[i] = ratioOf(spec);
            Integer previous = names.put(spec.name(), i);
            if (previous != null) {
                LOGGER.warn("Duplicate powertrain device '{}'; disabling this vehicle's powertrain", spec.name());
                data.clear();
                data.diagnostic = "duplicate device: " + spec.name();
                return;
            }
        }

        int[] childSizes = new int[deviceCount];
        Map<Long, Integer> occupiedPorts = new HashMap<>();
        int detachedDevices = 0;
        for (int i = 0; i < deviceCount; i++) {
            DeviceSpec spec = specs.get(i);
            String inputName = spec.inputName();
            if (inputName == null || inputName.isBlank() || "dummy".equalsIgnoreCase(inputName)) continue;
            Integer parent = names.get(inputName);
            if (parent == null || parent == i) {
                LOGGER.warn("Powertrain device '{}' has invalid input '{}'; isolating this branch",
                        spec.name(), inputName);
                topology.deviceType[i] = TYPE_UNSUPPORTED;
                detachedDevices++;
                continue;
            }
            topology.parentDevice[i] = parent;
            topology.parentPort[i] = Math.max(1, spec.inputIndex());
            long portKey = ((long) parent << 32) | (topology.parentPort[i] & 0xffffffffL);
            if (occupiedPorts.put(portKey, i) != null) {
                LOGGER.warn("Powertrain output {}:{} is connected more than once; isolating device '{}'",
                        inputName, topology.parentPort[i], spec.name());
                topology.parentDevice[i] = -1;
                topology.deviceType[i] = TYPE_UNSUPPORTED;
                detachedDevices++;
                continue;
            }
            childSizes[parent]++;
        }
        if (hasCycle(topology)) {
            LOGGER.warn("Powertrain graph contains a cycle; powertrain disabled");
            data.clear();
            data.diagnostic = "cycle in powertrain tree";
            return;
        }

        int totalChildren = 0;
        for (int i = 0; i < deviceCount; i++) {
            topology.childStart[i] = totalChildren;
            topology.childCount[i] = (short) childSizes[i];
            totalChildren += childSizes[i];
        }
        topology.allocateChildren(totalChildren);
        int[] cursors = topology.childStart.clone();
        for (int i = 0; i < deviceCount; i++) {
            int parent = topology.parentDevice[i];
            if (parent >= 0) topology.children[cursors[parent]++] = i;
        }

        collectDeviceLayouts(data, specs);

        List<UnitBuild> units = new ArrayList<>();
        for (int engine = 0; engine < deviceCount; engine++) {
            if (topology.deviceType[engine] != TYPE_ENGINE) continue;
            if (topology.childCount[engine] != 1) {
                LOGGER.warn("Combustion engine '{}' needs exactly one frictionClutch child in this MVP",
                        topology.deviceName[engine]);
                continue;
            }
            int clutch = topology.children[topology.childStart[engine]];
            if (topology.deviceType[clutch] != TYPE_CLUTCH) {
                LOGGER.warn("Combustion engine '{}' is not followed by a frictionClutch",
                        topology.deviceName[engine]);
                continue;
            }
            UnitBuild unit = buildUnit(vehicle, specs, topology, engine, clutch);
            if (!unit.paths.isEmpty()) units.add(unit);
        }
        compileUnits(data, units);
        if (data.engines.unitCount > 0) {
            data.diagnostic = detachedDevices == 0 ? "ready" : "ready; " + detachedDevices + " detached device(s)";
            LOGGER.info("Compiled BeamCraft powertrain: {} devices, {} engine/clutch unit(s), {} driven wheel path(s)",
                    deviceCount, data.engines.unitCount, data.wheelPaths.pathWheel.length);
        } else {
            data.diagnostic = "no supported engine-to-wheel path";
        }
    }

    // ---------------------------------------------------------------- device layouts

    private static void collectDeviceLayouts(PowertrainData data, List<DeviceSpec> specs) {
        for (DeviceSpec spec : specs) {
            switch (spec) {
                case ShaftSpec ignored -> data.shafts.count++;
                case DifferentialSpec ignored -> data.differentials.count++;
                case TorsionReactorSpec ignored -> data.torsionReactors.count++;
                default -> { }
            }
        }
        // Gearboxes are compiled per-unit as runtime SoA in compileUnits, not here.
        data.shafts.allocate(data.shafts.count);
        data.differentials.allocate(data.differentials.count);
        data.torsionReactors.allocate(data.torsionReactors.count);

        int shaft = 0, differential = 0, reactor = 0;
        for (int i = 0; i < specs.size(); i++) {
            switch (specs.get(i)) {
                case ShaftSpec sh -> {
                    data.shafts.device[shaft] = i;
                    data.shafts.gearRatio[shaft] = (float) sh.gearRatio();
                    data.shafts.connectedWheel[shaft] = sh.connectedWheel();
                    data.shafts.friction[shaft] = (float) sh.friction();
                    data.shafts.dynamicFriction[shaft] = (float) sh.dynamicFriction();
                    data.shafts.torqueLossCoef[shaft] = (float) sh.torqueLossCoef();
                    shaft++;
                }
                case DifferentialSpec df -> {
                    data.differentials.device[differential] = i;
                    data.differentials.gearRatio[differential] = (float) df.gearRatio();
                    data.differentials.diffTorqueSplit[differential] = (float) df.diffTorqueSplit();
                    data.differentials.diffType[differential] = df.diffType();
                    data.differentials.friction[differential] = (float) df.friction();
                    data.differentials.dynamicFriction[differential] = (float) df.dynamicFriction();
                    data.differentials.torqueLossCoef[differential] = (float) df.torqueLossCoef();
                    differential++;
                }
                case TorsionReactorSpec tr -> {
                    data.torsionReactors.device[reactor] = i;
                    data.torsionReactors.gearRatio[reactor] = (float) tr.gearRatio();
                    data.torsionReactors.connectedWheel[reactor] = tr.connectedWheel();
                    data.torsionReactors.friction[reactor] = (float) tr.friction();
                    data.torsionReactors.dynamicFriction[reactor] = (float) tr.dynamicFriction();
                    data.torsionReactors.torqueLossCoef[reactor] = (float) tr.torqueLossCoef();
                    reactor++;
                }
                default -> { }
            }
        }
    }

    // ---------------------------------------------------------------- unit compilation

    private static UnitBuild buildUnit(SoftBodyVehicle vehicle, List<DeviceSpec> specs,
                                       PowertrainTopologyContainer topology, int engine, int clutch) {
        CombustionEngineSpec engineSpec = (CombustionEngineSpec) specs.get(engine);
        FrictionClutchSpec clutchSpec = (FrictionClutchSpec) specs.get(clutch);
        List<PathBuild> paths = new ArrayList<>();
        List<ReactorBuild> reactors = new ArrayList<>();
        boolean[] visiting = new boolean[topology.deviceCount];
        for (int i = 0; i < topology.childCount[clutch]; i++) {
            collectPaths(vehicle, specs, topology,
                    topology.children[topology.childStart[clutch] + i], 1.0f, paths, reactors, visiting);
        }

        float maxTorque = 0.0f;
        for (TorquePoint point : engineSpec.torqueCurve()) maxTorque = Math.max(maxTorque, (float) point.torque());
        float capacity = (float) clutchSpec.lockTorque();
        if (capacity <= 0.0f) capacity = Math.max(1.0f, maxTorque * 1.25f);
        float spring = (float) clutchSpec.lockSpring();
        float springScale = (float) (clutchSpec.lockSpringCoef() * clutchSpec.clutchStiffness());
        if (spring <= 0.0f) spring = capacity / Math.max(1e-3f, (float) clutchSpec.clutchFreePlay());
        spring *= Math.max(0.0f, springScale);

        int gearboxDevice = findGearboxDevice(topology, clutch);
        GearboxSpec gearbox = gearboxDevice >= 0 ? (GearboxSpec) specs.get(gearboxDevice) : null;
        List<Integer> reactions = resolveNodes(vehicle, engineSpec.torqueReactionNodes());
        return new UnitBuild(engine, clutch, engineSpec, clutchSpec, capacity, spring, paths, reactions, reactors,
                gearboxDevice, gearbox);
    }

    /**
     * Depth-first search from the clutch output for the first gearbox device on the unit's
     * driveline. MVP supports one gearbox per engine→clutch unit.
     */
    private static int findGearboxDevice(PowertrainTopologyContainer topology, int start) {
        int count = topology.childCount[start];
        for (int i = 0; i < count; i++) {
            int child = topology.children[topology.childStart[start] + i];
            if (topology.deviceType[child] == TYPE_GEARBOX) return child;
            if (topology.deviceType[child] != TYPE_UNSUPPORTED) {
                int found = findGearboxDevice(topology, child);
                if (found >= 0) return found;
            }
        }
        return -1;
    }

    private static void collectPaths(SoftBodyVehicle vehicle, List<DeviceSpec> specs,
                                     PowertrainTopologyContainer topology, int device, float incomingGain,
                                     List<PathBuild> paths, List<ReactorBuild> reactors, boolean[] visiting) {
        if (visiting[device] || topology.deviceType[device] == TYPE_UNSUPPORTED) return;
        visiting[device] = true;
        float gain = incomingGain * topology.deviceRatio[device];
        DeviceSpec spec = specs.get(device);
        if (spec instanceof TorsionReactorSpec reactorSpec) {
            List<Integer> nodes = resolveNodes(vehicle, reactorSpec.torqueReactionNodes());
            if (nodes.size() >= 3) reactors.add(new ReactorBuild(gain, nodes));
        }
        String wheelName = connectedWheel(spec);
        if (wheelName != null) {
            Integer wheel = vehicle.wheels.nameToIndex.get(wheelName);
            if (wheel != null) paths.add(new PathBuild(wheel, gain));
            else LOGGER.warn("Powertrain device '{}' references missing wheel '{}'", topology.deviceName[device], wheelName);
        }

        int count = topology.childCount[device];
        for (int i = 0; i < count; i++) {
            int child = topology.children[topology.childStart[device] + i];
            float split = 1.0f;
            if (topology.deviceType[device] == TYPE_DIFFERENTIAL) {
                float configured = Math.clamp((float) ((DifferentialSpec) spec).diffTorqueSplit(), 0.0f, 1.0f);
                split = topology.parentPort[child] <= 1 ? configured : 1.0f - configured;
            } else if (count > 1) {
                split = 1.0f / count;
            }
            collectPaths(vehicle, specs, topology, child, gain * split, paths, reactors, visiting);
        }
        visiting[device] = false;
    }

    private static void compileUnits(PowertrainData data, List<UnitBuild> units) {
        CombustionEngineContainer engines = data.engines;
        FrictionClutchContainer clutches = data.clutches;
        DrivenWheelPathContainer wheelPaths = data.wheelPaths;
        TorqueReactionContainer reactions = data.reactions;
        GearboxContainer gearboxes = data.gearboxes;
        PowertrainTopologyContainer topology = data.topology;

        int n = units.size();
        int curves = 0, paths = 0, reactionTotal = 0, reactors = 0, gearSlots = 0;
        for (UnitBuild unit : units) {
            curves += unit.engine.torqueCurve().size();
            paths += unit.paths.size();
            reactionTotal += unit.reactions.size();
            reactors += unit.reactors.size();
            for (ReactorBuild reactor : unit.reactors) reactionTotal += reactor.nodes.size();
            gearSlots += gearRatiosOf(unit).size();
        }
        engines.allocate(n, curves);
        clutches.allocate(n);
        wheelPaths.allocate(n, paths);
        reactions.allocate(n, reactionTotal, reactors);
        gearboxes.allocate(n, Math.max(1, gearSlots));

        int curveCursor = 0, pathCursor = 0, reactionCursor = 0, reactorCursor = 0, gearCursor = 0;
        for (int i = 0; i < n; i++) {
            UnitBuild unit = units.get(i);
            CombustionEngineSpec engine = unit.engine;
            engines.engineDevice[i] = unit.engineDevice; engines.clutchDevice[i] = unit.clutchDevice;
            engines.engineInertia[i] = Math.max(1e-5f, (float) engine.inertia());
            engines.idleAV[i] = Math.max(0.0f, (float) engine.idleRPM()) * PowertrainSystem.RPM_TO_AV;
            engines.engineAV[i] = engines.idleAV[i];
            engines.engineFriction[i] = Math.max(0.0f, (float) engine.friction());
            engines.engineDynamicFriction[i] = Math.max(0.0f, (float) engine.dynamicFriction());
            engines.engineBrakeTorque[i] = Math.max(0.0f, (float) engine.engineBrakeTorque());
            engines.starterMaxAV[i] = Math.max(0.0f, (float) engine.starterMaxRPM()) * PowertrainSystem.RPM_TO_AV;
            engines.crankingAV[i] = Math.max(0.0f, (float) engine.crankingRPM()) * PowertrainSystem.RPM_TO_AV;
            float peakTorque = peakTorqueOf(engine);
            engines.starterTorque[i] = Math.max(0.0f, starterTorqueOf(engine, peakTorque));
            engines.idleLossThrottle[i] = idleLossThrottleOf(engine, engines.idleAV[i]);
            engines.idleControllerP[i] = Math.max(0.0f, (float) engine.idleControllerP());
            engines.maxIdleThrottle[i] = Math.clamp((float) engine.maxIdleThrottle(), 0.0f, 1.0f);
            engines.playerThrottle[i] = 0.0f;
            engines.actualThrottle[i] = engines.idleLossThrottle[i];
            engines.revLimiterRPM[i] = Math.max(0.0f, (float) engine.revLimiterRPM());
            engines.revLimiterType[i] = "soft".equalsIgnoreCase(engine.revLimiterType())
                    ? CombustionEngineContainer.LIMITER_TYPE_SOFT
                    : CombustionEngineContainer.LIMITER_TYPE_TIME;
            engines.revLimiterCutTime[i] = Math.max(0.0f, (float) engine.revLimiterCutTime());
            engines.revLimiterMaxRPMDrop[i] = Math.max(0.0f, (float) engine.revLimiterMaxRPMDrop());
            engines.sparkEnabled[i] = true; engines.fuelEnabled[i] = true;
            engines.starterActive[i] = false;
            engines.limiterCutRemaining[i] = 0.0f;
            clutches.clutchCapacity[i] = unit.capacity; clutches.clutchSpring[i] = unit.spring;
            clutches.clutchDampingRatio[i] = Math.max(0.0f, (float) unit.clutch.lockDampRatio());

            // Gearbox runtime state (implicit single 1.0 ratio when the unit has no gearbox).
            boolean realGearbox = unit.gearbox != null && !unit.gearbox.gearRatios().isEmpty();
            List<Double> ratios = gearRatiosOf(unit);
            gearboxes.device[i] = realGearbox ? unit.gearboxDevice : -1;
            gearboxes.deviceName[i] = realGearbox ? topology.deviceName[unit.gearboxDevice] : "implicit";
            gearboxes.gearboxType[i] = realGearbox ? unit.gearbox.type() : "none";
            gearboxes.gearStart[i] = gearCursor;
            gearboxes.gearCount[i] = (short) ratios.size();
            for (double ratio : ratios) gearboxes.gearRatios[gearCursor++] = (float) ratio;
            int firstForward = firstPositiveGearIndex(ratios);
            int neutral = neutralGearIndex(ratios);
            boolean fixedFirst = realGearbox && unit.gearbox.fixedFirstGear();
            int initial = realGearbox && !fixedFirst && neutral >= 0 ? neutral : firstForward;
            if (initial < 0) initial = neutral >= 0 ? neutral : 0;
            gearboxes.initialGearIndex[i] = initial;
            gearboxes.currentGearIndex[i] = initial;
            gearboxes.pendingGearIndex[i] = -1;
            gearboxes.pathBaseRatio[i] = firstForward >= 0
                    ? gearboxes.gearRatios[gearboxes.gearStart[i] + firstForward]
                    : 1.0f;
            gearboxes.activeRatio[i] = gearboxes.gearRatios[gearboxes.gearStart[i] + initial];
            gearboxes.shiftRemaining[i] = 0.0f;
            gearboxes.shiftDuration[i] = realGearbox ? Math.max(0.0f, (float) unit.gearbox.shiftTime()) : 0.0f;
            gearboxes.fixedFirstGear[i] = fixedFirst;

            engines.curveStart[i] = curveCursor; engines.curveCount[i] = (short) engine.torqueCurve().size();
            for (TorquePoint point : engine.torqueCurve()) {
                engines.curveRPM[curveCursor] = (float) point.rpm();
                engines.curveTorque[curveCursor] = (float) point.torque();
                curveCursor++;
            }
            wheelPaths.pathStart[i] = pathCursor; wheelPaths.pathCount[i] = (short) unit.paths.size();
            for (PathBuild path : unit.paths) {
                wheelPaths.pathWheel[pathCursor] = path.wheel;
                wheelPaths.pathGain[pathCursor] = path.gain;
                pathCursor++;
            }
            reactions.reactionStart[i] = reactionCursor; reactions.reactionCount[i] = (byte) unit.reactions.size();
            for (int node : unit.reactions) reactions.reactionNodes[reactionCursor++] = node;
            reactions.reactorStart[i] = reactorCursor; reactions.reactorCount[i] = (short) unit.reactors.size();
            for (ReactorBuild reactor : unit.reactors) {
                reactions.reactorGain[reactorCursor] = reactor.gain;
                reactions.reactorNodeStart[reactorCursor] = reactionCursor;
                reactions.reactorNodeCount[reactorCursor] = (byte) reactor.nodes.size();
                for (int node : reactor.nodes) reactions.reactionNodes[reactionCursor++] = node;
                reactorCursor++;
            }
        }
    }

    // ---------------------------------------------------------------- engine/gearbox derivation

    private static float peakTorqueOf(CombustionEngineSpec engine) {
        float peak = 0.0f;
        for (TorquePoint point : engine.torqueCurve()) peak = Math.max(peak, (float) point.torque());
        return peak;
    }

    /**
     * Starter torque in N·m. When the JBeam part does not specify one, derive it from the
     * torque-curve peak so it is strong enough to overcome the parsed friction/braking:
     * {@code max(40 N·m, 0.4 * peakTorque)}.
     */
    private static float starterTorqueOf(CombustionEngineSpec engine, float peakTorque) {
        if (engine.starterTorque() > 0.0) return (float) engine.starterTorque();
        return Math.max(40.0f, 0.4f * peakTorque);
    }

    /**
     * Initial idle feedforward throttle. The solver recalculates the same loss balance at
     * the current crank speed whenever RPM falls below the idle target.
     */
    private static float idleLossThrottleOf(CombustionEngineSpec engine, float idleAV) {
        float friction = Math.max(0.0f, (float) engine.friction());
        float dynamic = Math.max(0.0f, (float) engine.dynamicFriction());
        float idleRPM = Math.max(0.0f, (float) engine.idleRPM());
        float torqueAtIdle = interpolateCurve(engine.torqueCurve(), idleRPM);
        if (torqueAtIdle <= 1e-3f) {
            return (friction + dynamic * idleAV) > 1e-3f ? 1.0f : 0.0f;
        }
        // Solve ff*T_idle = friction + dynamic*idleAV. Additional engine braking is
        // load-dependent in BeamNG and stays inactive until that model exists here.
        float feedforward = (friction + dynamic * idleAV) / torqueAtIdle;
        return Math.clamp(feedforward + 0.05f, 0.0f, 1.0f);
    }

    private static float interpolateCurve(List<TorquePoint> curve, float rpm) {
        int n = curve.size();
        if (n == 0) return 0.0f;
        if (n == 1 || rpm <= (float) curve.get(0).rpm()) return (float) curve.get(0).torque();
        for (int i = 1; i < n; i++) {
            float r = (float) curve.get(i).rpm();
            if (rpm <= r) {
                float prev = (float) curve.get(i - 1).rpm();
                float span = r - prev;
                float t = span > 1e-6f ? (rpm - prev) / span : 0.0f;
                return (float) curve.get(i - 1).torque()
                        + ((float) curve.get(i).torque() - (float) curve.get(i - 1).torque()) * t;
            }
        }
        return (float) curve.get(n - 1).torque();
    }

    private static List<Double> gearRatiosOf(UnitBuild unit) {
        if (unit.gearbox != null && !unit.gearbox.gearRatios().isEmpty()) return unit.gearbox.gearRatios();
        return List.of(1.0);
    }

    private static int firstPositiveGearIndex(List<Double> ratios) {
        for (int i = 0; i < ratios.size(); i++) {
            if (ratios.get(i) > 0.0) return i;
        }
        return -1;
    }

    private static int neutralGearIndex(List<Double> ratios) {
        for (int i = 0; i < ratios.size(); i++) {
            if (Math.abs(ratios.get(i)) < 1e-9) return i;
        }
        return -1;
    }

    // ---------------------------------------------------------------- topology helpers

    private static boolean hasCycle(PowertrainTopologyContainer topology) {
        byte[] state = new byte[topology.deviceCount];
        for (int i = 0; i < topology.deviceCount; i++) {
            int current = i;
            while (current >= 0 && state[current] == 0) {
                state[current] = 1;
                current = topology.parentDevice[current];
            }
            if (current >= 0 && state[current] == 1) return true;
            current = i;
            while (current >= 0 && state[current] == 1) {
                state[current] = 2;
                current = topology.parentDevice[current];
            }
        }
        return false;
    }

    private static List<Integer> resolveNodes(SoftBodyVehicle vehicle, List<String> names) {
        List<Integer> result = new ArrayList<>(names.size());
        for (String name : names) {
            Integer node = vehicle.nodes.nameToIndex.get(name);
            if (node != null) result.add(node);
        }
        return result;
    }

    private static byte typeOf(DeviceSpec spec) {
        return switch (spec) {
            case CombustionEngineSpec ignored -> PowertrainTopologyContainer.TYPE_ENGINE;
            case FrictionClutchSpec ignored -> PowertrainTopologyContainer.TYPE_CLUTCH;
            case GearboxSpec ignored -> PowertrainTopologyContainer.TYPE_GEARBOX;
            case ShaftSpec ignored -> PowertrainTopologyContainer.TYPE_SHAFT;
            case DifferentialSpec ignored -> PowertrainTopologyContainer.TYPE_DIFFERENTIAL;
            case TorsionReactorSpec ignored -> PowertrainTopologyContainer.TYPE_TORSION_REACTOR;
            case DevicePatchSpec ignored -> PowertrainTopologyContainer.TYPE_UNSUPPORTED;
            case UnsupportedConfig ignored -> PowertrainTopologyContainer.TYPE_UNSUPPORTED;
        };
    }

    private static float ratioOf(DeviceSpec spec) {
        return switch (spec) {
            case GearboxSpec gearbox -> (float) gearbox.firstPositiveGearRatio();
            case ShaftSpec shaft -> (float) shaft.gearRatio();
            case TorsionReactorSpec reactor -> (float) reactor.gearRatio();
            case DifferentialSpec differential -> (float) differential.gearRatio();
            default -> 1.0f;
        };
    }

    private static String connectedWheel(DeviceSpec spec) {
        return switch (spec) {
            case ShaftSpec shaft -> shaft.connectedWheel();
            case TorsionReactorSpec reactor -> reactor.connectedWheel();
            default -> null;
        };
    }

    // ---------------------------------------------------------------- build records

    private record PathBuild(int wheel, float gain) {
    }

    private record ReactorBuild(float gain, List<Integer> nodes) {
    }

    private record UnitBuild(int engineDevice, int clutchDevice, CombustionEngineSpec engine,
                             FrictionClutchSpec clutch, float capacity, float spring,
                             List<PathBuild> paths, List<Integer> reactions, List<ReactorBuild> reactors,
                             int gearboxDevice, GearboxSpec gearbox) {
    }
}
