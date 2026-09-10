package me.mzy.beamcraft.client.physics.powertrain;

import me.mzy.beamcraft.client.physics.SoftBodyVehicle;
import me.mzy.beamcraft.client.physics.electrics.ElectricSignals;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.CombustionEngineSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.ClutchlikeSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.DeviceSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.DifferentialSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.DctGearboxSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.FrictionClutchSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.GearSelectableSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.GearboxSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.ShaftSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.SplitShaftSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.SuperchargerSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.TorquePoint;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.TorsionReactorSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.TorqueConverterSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.TurbochargerSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.UnsupportedConfig;

import static me.mzy.beamcraft.client.physics.powertrain.PowertrainTopologyContainer.TYPE_CLUTCH;
import static me.mzy.beamcraft.client.physics.powertrain.PowertrainTopologyContainer.TYPE_DIFFERENTIAL;
import static me.mzy.beamcraft.client.physics.powertrain.PowertrainTopologyContainer.TYPE_DCT_GEARBOX;
import static me.mzy.beamcraft.client.physics.powertrain.PowertrainTopologyContainer.TYPE_ENGINE;
import static me.mzy.beamcraft.client.physics.powertrain.PowertrainTopologyContainer.TYPE_GEARBOX;
import static me.mzy.beamcraft.client.physics.powertrain.PowertrainTopologyContainer.TYPE_RANGE_BOX;
import static me.mzy.beamcraft.client.physics.powertrain.PowertrainTopologyContainer.TYPE_SPLIT_SHAFT;
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
 * rows into the flat SoA containers of a {@link PowertrainData}: it resolves any remaining device
 * value modifiers, validates the device graph (duplicate names, input resolution, occupied
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

        Map<String, TurbochargerSpec> turboByEngine = new HashMap<>();
        Map<String, SuperchargerSpec> superchargerByEngine = new HashMap<>();
        List<DeviceSpec> deviceSpecs = new ArrayList<>();
        for (DeviceSpec spec : rawSpecs) {
            if (spec instanceof TurbochargerSpec turbo) {
                turboByEngine.put(turbo.engineName(), turbo); // later active parts win
            } else if (spec instanceof SuperchargerSpec supercharger) {
                superchargerByEngine.put(supercharger.engineName(), supercharger);
            } else {
                deviceSpecs.add(spec);
            }
        }
        List<DeviceSpec> specs = PowertrainSpecNormalizer.normalize(deviceSpecs);
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
                LOGGER.warn("Combustion engine '{}' needs exactly one clutchlike child",
                        topology.deviceName[engine]);
                continue;
            }
            int clutchlike = topology.children[topology.childStart[engine]];
            if (!(specs.get(clutchlike) instanceof ClutchlikeSpec)) {
                LOGGER.warn("Combustion engine '{}' is not followed by a clutchlike device",
                        topology.deviceName[engine]);
                continue;
            }
            UnitBuild unit = buildUnit(vehicle, specs, topology, engine, clutchlike,
                    turboByEngine.get(topology.deviceName[engine]),
                    superchargerByEngine.get(topology.deviceName[engine]));
            if (!unit.paths.isEmpty()) units.add(unit);
        }
        compileUnits(vehicle, data, units);
        if (data.engines.unitCount > 0) {
            int turboCount = 0;
            for (boolean existing : data.turbochargers.existing) if (existing) turboCount++;
            int superchargerCount = 0;
            for (boolean existing : data.superchargers.existing) if (existing) superchargerCount++;
            data.diagnostic = detachedDevices == 0 ? "ready" : "ready; " + detachedDevices + " detached device(s)";
            LOGGER.info("Compiled BeamCraft powertrain: {} devices, {} engine/clutchlike unit(s), "
                            + "{} turbocharger(s), {} supercharger(s), {} driven wheel path(s)",
                    deviceCount, data.engines.unitCount, turboCount, superchargerCount,
                    data.wheelPaths.pathWheel.length);
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
                                       PowertrainTopologyContainer topology, int engine, int clutchlike,
                                       TurbochargerSpec turbocharger,
                                       SuperchargerSpec supercharger) {
        CombustionEngineSpec engineSpec = (CombustionEngineSpec) specs.get(engine);
        ClutchlikeSpec clutchlikeSpec = (ClutchlikeSpec) specs.get(clutchlike);
        FrictionClutchSpec clutchSpec = clutchlikeSpec instanceof FrictionClutchSpec c ? c : null;
        TorqueConverterSpec converterSpec = clutchlikeSpec instanceof TorqueConverterSpec c ? c : null;
        DctGearboxSpec dctSpec = clutchlikeSpec instanceof DctGearboxSpec d ? d : null;
        int gearboxDevice = dctSpec != null ? clutchlike : findGearboxDevice(topology, clutchlike);
        GearSelectableSpec gearbox = gearboxDevice >= 0
                ? (GearSelectableSpec) specs.get(gearboxDevice) : null;
        int rangeBoxDevice = findDeviceOfType(topology, clutchlike, TYPE_RANGE_BOX);
        GearboxSpec rangeBox = rangeBoxDevice >= 0 ? (GearboxSpec) specs.get(rangeBoxDevice) : null;
        List<PathBuild> paths = new ArrayList<>();
        List<ReactorBuild> reactors = new ArrayList<>();
        List<SplitBuild> splitShafts = new ArrayList<>();
        boolean[] visiting = new boolean[topology.deviceCount];
        byte initialFlags = dctSpec != null ? DrivenWheelPathContainer.FLAG_GEARBOX : 0;
        float initialGain = dctSpec != null ? (float) dctSpec.firstPositiveGearRatio() : 1.0f;
        if (Math.abs(initialGain) <= 1.0e-6f) initialGain = 1.0f;
        for (int i = 0; i < topology.childCount[clutchlike]; i++) {
            collectDomainPaths(vehicle, specs, topology,
                    topology.children[topology.childStart[clutchlike] + i], initialGain,
                    initialFlags, gearboxDevice, rangeBoxDevice, paths, visiting);
        }
        Arrays.fill(visiting, false);
        collectSplitBuilds(vehicle, specs, topology, clutchlike, gearboxDevice,
                rangeBoxDevice, splitShafts, visiting);
        Arrays.fill(visiting, false);
        for (int i = 0; i < topology.childCount[clutchlike]; i++) {
            collectReactors(vehicle, specs, topology,
                    topology.children[topology.childStart[clutchlike] + i],
                    1.0f, reactors, visiting);
        }

        float maxTorque = 0.0f;
        for (TorquePoint point : engineSpec.torqueCurve()) maxTorque = Math.max(maxTorque, (float) point.torque());
        float capacity = 0.0f;
        float spring = 0.0f;
        if (clutchSpec != null) {
            capacity = (float) clutchSpec.lockTorque();
            if (capacity <= 0.0f) capacity = Math.max(1.0f, maxTorque * 1.25f);
            spring = (float) clutchSpec.lockSpring();
            float springScale = (float) (clutchSpec.lockSpringCoef() * clutchSpec.clutchStiffness());
            if (spring <= 0.0f) spring = capacity / Math.max(1e-3f, (float) clutchSpec.clutchFreePlay());
            spring *= Math.max(0.0f, springScale);
        } else if (dctSpec != null) {
            capacity = (float) dctSpec.lockTorque();
            if (capacity <= 0.0f) {
                float effectiveEngineInertia = Math.max(0.0f,
                        (float) (engineSpec.inertia() + dctSpec.additionalEngineInertia()));
                capacity = Math.max(1.0f, maxTorque * 1.25f
                        + (float) engineSpec.maxRPM() * effectiveEngineInertia * (float) Math.PI / 30.0f);
            }
            spring = (float) dctSpec.lockSpring();
            if (spring <= 0.0f) spring = capacity / 0.125f;
            spring *= Math.max(0.0f, (float) dctSpec.clutchStiffness());
        }

        List<Integer> reactions = resolveNodes(vehicle, engineSpec.torqueReactionNodes());
        return new UnitBuild(engine, clutchlike, engineSpec, clutchSpec, converterSpec, dctSpec,
                capacity, spring, maxTorque, paths, reactions, reactors,
                gearboxDevice, gearbox, rangeBoxDevice, rangeBox,
                splitShafts, turbocharger, supercharger);
    }

    /**
     * Depth-first search from the clutch output for the first gearbox device on the unit's
     * driveline. MVP supports one gearbox per engine→clutch unit.
     */
    private static int findGearboxDevice(PowertrainTopologyContainer topology, int start) {
        return findDeviceOfType(topology, start, TYPE_GEARBOX);
    }

    private static int findDeviceOfType(PowertrainTopologyContainer topology, int start, byte type) {
        int count = topology.childCount[start];
        for (int i = 0; i < count; i++) {
            int child = topology.children[topology.childStart[start] + i];
            if (topology.deviceType[child] == type) return child;
            if (topology.deviceType[child] != TYPE_UNSUPPORTED) {
                int found = findDeviceOfType(topology, child, type);
                if (found >= 0) return found;
            }
        }
        return -1;
    }

    /** Collects one rigid kinematic domain, following only the primary port at split shafts. */
    private static void collectDomainPaths(SoftBodyVehicle vehicle, List<DeviceSpec> specs,
                                           PowertrainTopologyContainer topology, int device,
                                           float incomingGain, byte incomingFlags,
                                           int gearboxDevice, int rangeBoxDevice,
                                           List<PathBuild> paths, boolean[] visiting) {
        if (visiting[device] || topology.deviceType[device] == TYPE_UNSUPPORTED) return;
        visiting[device] = true;
        float gain = incomingGain * topology.deviceRatio[device];
        byte flags = incomingFlags;
        if (device == gearboxDevice) flags |= DrivenWheelPathContainer.FLAG_GEARBOX;
        if (device == rangeBoxDevice) flags |= DrivenWheelPathContainer.FLAG_RANGE_BOX;
        DeviceSpec spec = specs.get(device);
        String wheelName = connectedWheel(spec);
        if (wheelName != null) {
            Integer wheel = vehicle.wheels.nameToIndex.get(wheelName);
            if (wheel != null) paths.add(new PathBuild(wheel, gain, flags));
            else LOGGER.warn("Powertrain device '{}' references missing wheel '{}'", topology.deviceName[device], wheelName);
        }

        int count = topology.childCount[device];
        for (int i = 0; i < count; i++) {
            int child = topology.children[topology.childStart[device] + i];
            float split = 1.0f;
            if (topology.deviceType[device] == TYPE_SPLIT_SHAFT) {
                if (topology.parentPort[child] != ((SplitShaftSpec) spec).primaryOutputID()) continue;
            } else if (topology.deviceType[device] == TYPE_DIFFERENTIAL) {
                float configured = Math.clamp((float) ((DifferentialSpec) spec).diffTorqueSplit(), 0.0f, 1.0f);
                split = topology.parentPort[child] <= 1 ? configured : 1.0f - configured;
            } else if (count > 1) {
                split = 1.0f / count;
            }
            collectDomainPaths(vehicle, specs, topology, child, gain * split, flags,
                    gearboxDevice, rangeBoxDevice, paths, visiting);
        }
        visiting[device] = false;
    }

    /** Discovers every split shaft in the unit, including nested devices on secondary branches. */
    private static void collectSplitBuilds(SoftBodyVehicle vehicle, List<DeviceSpec> specs,
                                           PowertrainTopologyContainer topology, int device,
                                           int gearboxDevice, int rangeBoxDevice,
                                           List<SplitBuild> result, boolean[] visiting) {
        if (visiting[device] || topology.deviceType[device] == TYPE_UNSUPPORTED) return;
        visiting[device] = true;
        if (specs.get(device) instanceof SplitShaftSpec splitSpec) {
            List<PathBuild> primary = new ArrayList<>();
            List<PathBuild> secondary = new ArrayList<>();
            boolean[] pathVisiting = new boolean[topology.deviceCount];
            int end = topology.childStart[device] + topology.childCount[device];
            for (int cursor = topology.childStart[device]; cursor < end; cursor++) {
                int child = topology.children[cursor];
                List<PathBuild> target = topology.parentPort[child] == splitSpec.primaryOutputID()
                        ? primary : secondary;
                collectDomainPaths(vehicle, specs, topology, child, 1.0f, (byte) 0,
                        gearboxDevice, rangeBoxDevice, target, pathVisiting);
            }
            if (primary.isEmpty() || secondary.isEmpty()) {
                LOGGER.warn("Split shaft '{}' has no resolved wheel inertia on its {} output; "
                                + "the coupling stays inactive until a virtual-inertia domain is available",
                        splitSpec.name(), primary.isEmpty() ? "primary" : "secondary");
            }
            result.add(new SplitBuild(device, splitSpec, primary, secondary));
        }
        int end = topology.childStart[device] + topology.childCount[device];
        for (int cursor = topology.childStart[device]; cursor < end; cursor++) {
            collectSplitBuilds(vehicle, specs, topology, topology.children[cursor],
                    gearboxDevice, rangeBoxDevice, result, visiting);
        }
        visiting[device] = false;
    }

    /** Retains the existing body torque-reaction metadata independently of domain paths. */
    private static void collectReactors(SoftBodyVehicle vehicle, List<DeviceSpec> specs,
                                        PowertrainTopologyContainer topology, int device,
                                        float incomingGain, List<ReactorBuild> reactors,
                                        boolean[] visiting) {
        if (visiting[device] || topology.deviceType[device] == TYPE_UNSUPPORTED) return;
        visiting[device] = true;
        float gain = incomingGain * topology.deviceRatio[device];
        DeviceSpec spec = specs.get(device);
        if (spec instanceof TorsionReactorSpec reactorSpec) {
            List<Integer> nodes = resolveNodes(vehicle, reactorSpec.torqueReactionNodes());
            if (nodes.size() >= 3) reactors.add(new ReactorBuild(gain, nodes));
        }
        int count = topology.childCount[device];
        for (int i = 0; i < count; i++) {
            int child = topology.children[topology.childStart[device] + i];
            float split = 1.0f;
            if (topology.deviceType[device] == TYPE_DIFFERENTIAL) {
                float configured = Math.clamp((float) ((DifferentialSpec) spec).diffTorqueSplit(), 0.0f, 1.0f);
                split = topology.parentPort[child] <= 1 ? configured : 1.0f - configured;
            }
            collectReactors(vehicle, specs, topology, child, gain * split, reactors, visiting);
        }
        visiting[device] = false;
    }

    private static void compileUnits(SoftBodyVehicle vehicle, PowertrainData data, List<UnitBuild> units) {
        CombustionEngineContainer engines = data.engines;
        FrictionClutchContainer clutches = data.clutches;
        DrivenWheelPathContainer wheelPaths = data.wheelPaths;
        TorqueReactionContainer reactions = data.reactions;
        GearboxContainer gearboxes = data.gearboxes;
        RangeBoxContainer rangeBoxes = data.rangeBoxes;
        ClutchlikeContainer clutchlikes = data.clutchlikes;
        TorqueConverterContainer torqueConverters = data.torqueConverters;
        DctGearboxContainer dctGearboxes = data.dctGearboxes;
        SplitShaftContainer splitShafts = data.splitShafts;
        TurbochargerContainer turbochargers = data.turbochargers;
        SuperchargerContainer superchargers = data.superchargers;
        PowertrainTopologyContainer topology = data.topology;

        int n = units.size();
        int curves = 0, paths = 0, reactionTotal = 0, reactors = 0, gearSlots = 0;
        int splitCount = 0, splitPathCount = 0, turboPressurePoints = 0, turboEnginePoints = 0;
        int superchargerControllerPoints = 0;
        for (UnitBuild unit : units) {
            curves += unit.engine.torqueCurve().size();
            paths += unit.paths.size();
            reactionTotal += unit.reactions.size();
            reactors += unit.reactors.size();
            for (ReactorBuild reactor : unit.reactors) reactionTotal += reactor.nodes.size();
            gearSlots += gearRatiosOf(unit).size();
            splitCount += unit.splitShafts.size();
            for (SplitBuild split : unit.splitShafts) {
                splitPathCount += split.primaryPaths.size() + split.secondaryPaths.size();
            }
            if (unit.turbocharger != null) {
                turboPressurePoints += unit.turbocharger.pressureCurve().size();
                turboEnginePoints += unit.turbocharger.engineCurve().size();
            }
            if (unit.supercharger != null) {
                superchargerControllerPoints += unit.supercharger.boostController().size();
            }
        }
        engines.allocate(n, curves);
        turbochargers.allocate(n, turboPressurePoints, turboEnginePoints);
        superchargers.allocate(n, superchargerControllerPoints);
        clutches.allocate(n);
        clutchlikes.allocate(n);
        torqueConverters.allocate(n);
        dctGearboxes.allocate(n);
        wheelPaths.allocate(n, paths);
        reactions.allocate(n, reactionTotal, reactors);
        gearboxes.allocate(n, Math.max(1, gearSlots));
        rangeBoxes.allocate(n);
        splitShafts.allocate(n, splitCount, splitPathCount);

        int curveCursor = 0, pathCursor = 0, reactionCursor = 0, reactorCursor = 0, gearCursor = 0;
        int splitCursor = 0, splitPathCursor = 0, turboPressureCursor = 0, turboEngineCursor = 0;
        int superchargerControllerCursor = 0;
        for (int i = 0; i < n; i++) {
            UnitBuild unit = units.get(i);
            CombustionEngineSpec engine = unit.engine;
            engines.engineDevice[i] = unit.engineDevice; engines.clutchDevice[i] = unit.clutchlikeDevice;
            float additionalInertia = unit.converter != null
                    ? Math.max(0.0f, (float) unit.converter.additionalEngineInertia())
                    : unit.dct != null ? Math.max(0.0f, (float) unit.dct.additionalEngineInertia()) : 0.0f;
            engines.engineInertia[i] = Math.max(1e-5f, (float) engine.inertia() + additionalInertia);
            engines.idleAV[i] = Math.max(0.0f, (float) engine.idleRPM()) * PowertrainSystem.RPM_TO_AV;
            engines.engineAV[i] = engines.idleAV[i];
            engines.engineFriction[i] = Math.max(0.0f, (float) engine.friction());
            engines.engineDynamicFriction[i] = Math.max(0.0f, (float) engine.dynamicFriction());
            engines.engineBrakeTorque[i] = Math.max(0.0f, (float) engine.engineBrakeTorque());
            engines.starterMaxAV[i] = Math.max(0.0f, (float) engine.starterMaxRPM()) * PowertrainSystem.RPM_TO_AV;
            engines.crankingAV[i] = Math.max(0.0f, (float) engine.crankingRPM()) * PowertrainSystem.RPM_TO_AV;
            float peakTorque = peakTorqueOf(engine);
            engines.starterTorque[i] = Math.max(0.0f, starterTorqueOf(engine, peakTorque));
            engines.idleControlThrottle[i] = 0.0f;
            engines.playerThrottle[i] = 0.0f;
            engines.actualThrottle[i] = 0.0f;
            engines.availableCombustionTorque[i] = 0.0f;
            engines.combustionTorque[i] = 0.0f;
            engines.normalizedCombustionOutput[i] = 0.0f;
            engines.revLimiterRPM[i] = Math.max(0.0f, (float) engine.revLimiterRPM());
            engines.revLimiterType[i] = "soft".equalsIgnoreCase(engine.revLimiterType())
                    ? CombustionEngineContainer.LIMITER_TYPE_SOFT
                    : CombustionEngineContainer.LIMITER_TYPE_TIME;
            engines.revLimiterCutTime[i] = Math.max(0.0f, (float) engine.revLimiterCutTime());
            engines.revLimiterMaxRPMDrop[i] = Math.max(0.0f, (float) engine.revLimiterMaxRPMDrop());
            engines.sparkEnabled[i] = true; engines.fuelEnabled[i] = true;
            engines.starterActive[i] = false;
            engines.limiterCutRemaining[i] = 0.0f;
            TurbochargerSpec turbo = unit.turbocharger;
            turbochargers.wastegateFactor[i] = 1.0f;
            if (turbo != null) {
                turbochargers.existing[i] = true;
                // BeamNG calibrates its axis inertia as 0.000003 * (inertia * 100) * 2.5.
                turbochargers.inertia[i] = Math.max(1.0e-9f, (float) turbo.inertia() * 0.00075f);
                turbochargers.wastegateStartPa[i] = (float) turbo.wastegateStartPSI() * 6894.7573f;
                double wastegateLimit = Double.isFinite(turbo.wastegateLimitPSI())
                        ? turbo.wastegateLimitPSI() : turbo.wastegateStartPSI() + 0.01;
                turbochargers.wastegateLimitPa[i] = (float) wastegateLimit * 6894.7573f;
                turbochargers.maxExhaustPower[i] = Math.max(0.0f, (float) turbo.maxExhaustPower());
                turbochargers.backPressureCoef[i] = Math.max(0.0f, (float) turbo.backPressureCoef());
                turbochargers.frictionCoef[i] = Math.max(0.0f, (float) turbo.frictionCoef());
                turbochargers.pressureFallRatePa[i] = Math.max(0.0f,
                        (float) turbo.pressureRatePSI() * 6894.7573f);
                turbochargers.wastegateP[i] = Math.max(0.0f, (float) turbo.wastegatePCoef());
                turbochargers.wastegateI[i] = Math.max(0.0f, (float) turbo.wastegateICoef());
                turbochargers.wastegateD[i] = Math.max(0.0f, (float) turbo.wastegateDCoef());
                turbochargers.bovEnabled[i] = turbo.bovEnabled();
                turbochargers.bovOpenThreshold[i] = Math.max(0.0f, (float) turbo.bovOpenThreshold());
                turbochargers.bovOpenChangeThreshold[i] = Math.max(0.0f,
                        (float) turbo.bovOpenChangeThreshold());
                turbochargers.pressureStart[i] = turboPressureCursor;
                turbochargers.pressureCount[i] = (short) turbo.pressureCurve().size();
                float maxTurboRPM = 0.0f;
                for (var point : turbo.pressureCurve()) {
                    turbochargers.pressureRPM[turboPressureCursor] = (float) point.turboRPM();
                    turbochargers.pressurePSI[turboPressureCursor] = (float) point.pressurePSI();
                    maxTurboRPM = Math.max(maxTurboRPM, (float) point.turboRPM());
                    turboPressureCursor++;
                }
                turbochargers.maxAV[i] = Math.max(1.0f, maxTurboRPM * PowertrainSystem.RPM_TO_AV);
                turbochargers.engineStart[i] = turboEngineCursor;
                turbochargers.engineCount[i] = (short) turbo.engineCurve().size();
                for (var point : turbo.engineCurve()) {
                    turbochargers.engineRPM[turboEngineCursor] = (float) point.engineRPM();
                    turbochargers.efficiency[turboEngineCursor] = Math.clamp((float) point.efficiency(), 0.0f, 1.0f);
                    turbochargers.exhaustFactor[turboEngineCursor] = Math.max(0.0f,
                            (float) point.exhaustFactor());
                    turboEngineCursor++;
                }
            }
            SuperchargerSpec supercharger = unit.supercharger;
            if (supercharger != null) {
                superchargers.existing[i] = true;
                float ratio = Math.max(0.0f, (float) supercharger.gearRatio());
                superchargers.gearRatio[i] = ratio;
                float maxBlowerRPM = Double.isFinite(supercharger.maxRPM())
                        ? Math.max(0.0f, (float) supercharger.maxRPM())
                        : Math.max(0.0f, (float) engine.maxRPM()) * ratio;
                superchargers.maxBlowerRPM[i] = Math.max(1.0f, maxBlowerRPM);
                superchargers.pressurePSIPerRPM[i] = Math.max(0.0f,
                        (float) supercharger.pressurePSIPer1kRPM() * 0.001f);
                superchargers.crankLossPerRPM[i] = Math.max(0.0f,
                        (float) supercharger.crankLossPer1kRPM() * 0.001f);
                superchargers.pressureRatePa[i] = Math.max(0.0f,
                        (float) supercharger.pressureRatePSI() * 6894.7573f);
                float engageRPM = Math.max(0.0f, (float) supercharger.clutchEngageRPM());
                superchargers.clutchEngageRPM[i] = engageRPM;
                superchargers.clutchEngageRange[i] = Double.isFinite(supercharger.clutchEngageRange())
                        ? Math.max(1.0e-6f, (float) supercharger.clutchEngageRange())
                        : Math.max(1.0e-6f, engageRPM * 0.2f);
                float disengageRPM = Double.isFinite(supercharger.clutchDisengageRPM())
                        ? Math.max(0.0f, (float) supercharger.clutchDisengageRPM())
                        : Math.max(0.0f, (float) engine.maxRPM()) * 2.0f;
                superchargers.clutchDisengageRPM[i] = disengageRPM;
                superchargers.clutchDisengageRange[i] = Double.isFinite(supercharger.clutchDisengageRange())
                        ? Math.max(1.0e-6f, (float) supercharger.clutchDisengageRange())
                        : Math.max(1.0e-6f, disengageRPM * 0.05f);
                configureSuperchargerType(superchargers, i, supercharger);
                superchargers.controllerStart[i] = superchargerControllerCursor;
                superchargers.controllerCount[i] = (short) supercharger.boostController().size();
                for (var point : supercharger.boostController()) {
                    superchargers.controllerThrottle[superchargerControllerCursor] =
                            (float) point.throttlePercent();
                    superchargers.controllerFactor[superchargerControllerCursor] =
                            Math.max(0.0f, (float) point.factor());
                    superchargerControllerCursor++;
                }
            }
            clutchlikes.device[i] = unit.clutchlikeDevice;
            if (unit.clutch != null) {
                clutchlikes.type[i] = ClutchlikeContainer.TYPE_FRICTION_CLUTCH;
                clutches.clutchCapacity[i] = unit.capacity; clutches.clutchSpring[i] = unit.spring;
                clutches.clutchDampingRatio[i] = Math.max(0.0f, (float) unit.clutch.lockDampRatio());
            } else if (unit.converter != null) {
                clutchlikes.type[i] = ClutchlikeContainer.TYPE_TORQUE_CONVERTER;
                TorqueConverterSpec converter = unit.converter;
                torqueConverters.couplingAVRatio[i] = Math.max(0.05f, (float) converter.couplingAVRatio());
                torqueConverters.stallTorqueRatio[i] = Math.max(1.0f, (float) converter.stallTorqueRatio());
                torqueConverters.converterStiffness[i] = Math.max(0.0f, (float) converter.converterStiffness());
                torqueConverters.converterDiameter[i] = Math.max(0.0f, (float) converter.converterDiameter());
                float configuredLimit = Math.max(0.0f, (float) converter.converterTorque());
                torqueConverters.converterTorqueLimit[i] = configuredLimit > 0.0f ? configuredLimit
                        : Math.max(1.0f, unit.maxTorque * 1.25f
                        + (float) engine.maxRPM() * engines.engineInertia[i] * (float) Math.PI / 30.0f);
                torqueConverters.additionalEngineInertia[i] = additionalInertia;
                torqueConverters.lockupCapacity[i] = Math.max(0.0f, (float) converter.lockupClutchTorque());
                float lockupSpring = (float) converter.lockupClutchSpring();
                if (lockupSpring <= 0.0f) {
                    lockupSpring = torqueConverters.lockupCapacity[i] / 0.125f;
                }
                torqueConverters.lockupSpring[i] = Math.max(0.0f, lockupSpring);
                torqueConverters.lockupDampingRatio[i] = Math.max(0.0f, (float) converter.lockupClutchDampRatio());
                String signalName = converter.lockupClutchRatioName();
                if (signalName == null || signalName.isBlank()) signalName = ElectricSignals.LOCKUP_CLUTCH_RATIO;
                torqueConverters.lockupSignalId[i] = vehicle.electrics.register(signalName);
            } else {
                clutchlikes.type[i] = ClutchlikeContainer.TYPE_DCT_GEARBOX;
                dctGearboxes.device[i] = unit.clutchlikeDevice;
                dctGearboxes.clutchCapacity[i] = unit.capacity;
                dctGearboxes.clutchSpring[i] = unit.spring;
                dctGearboxes.clutchDampingRatio1[i] = Math.max(0.0f, (float) unit.dct.lockDampRatio1());
                dctGearboxes.clutchDampingRatio2[i] = Math.max(0.0f, (float) unit.dct.lockDampRatio2());
            }

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

            int secondForward = nextPositiveGearIndex(ratios, firstForward);
            dctGearboxes.gearIndex1[i] = firstForward >= 0 ? firstForward : initial;
            dctGearboxes.gearIndex2[i] = secondForward >= 0 ? secondForward : dctGearboxes.gearIndex1[i];
            dctGearboxes.shiftTarget[i] = -1;
            dctGearboxes.primaryClutch[i] = 0;
            dctGearboxes.targetClutch[i] = -1;
            dctGearboxes.engagement1[i] = unit.dct != null && initial == firstForward ? 1.0f : 0.0f;
            dctGearboxes.engagement2[i] = 0.0f;

            rangeBoxes.device[i] = unit.rangeBoxDevice;
            rangeBoxes.deviceName[i] = unit.rangeBox != null ? unit.rangeBox.name() : "none";
            float rangeBase = unit.rangeBox != null
                    ? (float) unit.rangeBox.firstPositiveGearRatio() : 1.0f;
            float rangeHigh = unit.rangeBox != null
                    ? smallestPositiveRatio(unit.rangeBox.gearRatios()) : 1.0f;
            float rangeLow = unit.rangeBox != null
                    ? largestPositiveRatio(unit.rangeBox.gearRatios()) : 1.0f;
            if (rangeBase <= 1.0e-6f) rangeBase = 1.0f;
            if (rangeHigh <= 1.0e-6f) rangeHigh = rangeBase;
            if (rangeLow <= 1.0e-6f) rangeLow = rangeHigh;
            rangeBoxes.pathBaseRatio[i] = rangeBase;
            rangeBoxes.highRatio[i] = rangeHigh;
            rangeBoxes.lowRatio[i] = rangeLow;
            rangeBoxes.activeRatio[i] = rangeHigh;
            rangeBoxes.lowMode[i] = false;

            splitShafts.unitStart[i] = splitCursor;
            splitShafts.unitCount[i] = (short) unit.splitShafts.size();
            for (SplitBuild splitBuild : unit.splitShafts) {
                int splitIndex = splitCursor++;
                SplitShaftSpec splitShaft = splitBuild.spec;
                splitShafts.unit[splitIndex] = i;
                splitShafts.device[splitIndex] = splitBuild.device;
                splitShafts.deviceName[splitIndex] = splitShaft.name();
                byte configuredMode = "viscous".equalsIgnoreCase(splitShaft.splitType())
                        ? SplitShaftContainer.MODE_VISCOUS : SplitShaftContainer.MODE_LOCKED;
                splitShafts.configuredMode[splitIndex] = configuredMode;
                splitShafts.initialMode[splitIndex] = splitShaft.isDisconnected()
                        ? SplitShaftContainer.MODE_DISCONNECTED : configuredMode;
                splitShafts.activeMode[splitIndex] = splitShafts.initialMode[splitIndex];
                splitShafts.primaryOutputID[splitIndex] = splitShaft.primaryOutputID();
                splitShafts.canDisconnect[splitIndex] = splitShaft.canDisconnect();
                // BeamNG normally supplies this through an electronic controller. Until
                // BeamCraft has that layer, a connected split shaft must remain useful:
                // defaultClutchRatio=0 therefore falls back to fully engaged instead of
                // silently turning an AWD configuration into primary-axle-only drive.
                float configuredClutchRatio = Math.clamp((float) splitShaft.defaultClutchRatio(), 0.0f, 1.0f);
                splitShafts.defaultClutchRatio[splitIndex] = configuredClutchRatio > 1.0e-6f
                        ? configuredClutchRatio : 1.0f;
                splitShafts.clutchRatio[splitIndex] = splitShafts.defaultClutchRatio[splitIndex];
                splitShafts.lockCapacity[splitIndex] = Math.max(0.0f, (float) splitShaft.lockTorque());
                float lockSpring = (float) splitShaft.lockSpring();
                if (lockSpring <= 0.0f) lockSpring = splitShafts.lockCapacity[splitIndex] / 0.125f;
                float springScale = Math.max(0.0f,
                        (float) (splitShaft.lockSpringCoef() * splitShaft.clutchStiffness()));
                splitShafts.lockSpring[splitIndex] = Math.max(0.0f, lockSpring * springScale);
                splitShafts.lockDampingRatio[splitIndex] = Math.max(0.0f, (float) splitShaft.lockDampRatio());
                splitShafts.viscousCoef[splitIndex] = Math.max(0.0f, (float) splitShaft.viscousCoef());
                splitShafts.viscousCapacity[splitIndex] = Math.max(0.0f, (float) splitShaft.viscousTorque());
                splitShafts.viscousExponent[splitIndex] = Math.max(0.0f, (float) splitShaft.viscousExponent());
                splitShafts.viscousSmoothing[splitIndex] = Math.max(0.0f, (float) splitShaft.viscousSmoothing());

                splitShafts.primaryPathStart[splitIndex] = splitPathCursor;
                splitShafts.primaryPathCount[splitIndex] = (short) splitBuild.primaryPaths.size();
                splitPathCursor = writeSplitPaths(splitShafts, splitBuild.primaryPaths, splitPathCursor);
                splitShafts.secondaryPathStart[splitIndex] = splitPathCursor;
                splitShafts.secondaryPathCount[splitIndex] = (short) splitBuild.secondaryPaths.size();
                splitPathCursor = writeSplitPaths(splitShafts, splitBuild.secondaryPaths, splitPathCursor);
            }

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
                wheelPaths.pathFlags[pathCursor] = path.flags;
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

    private static int writeSplitPaths(SplitShaftContainer target, List<PathBuild> paths, int cursor) {
        for (PathBuild path : paths) {
            target.pathWheel[cursor] = path.wheel;
            target.pathGain[cursor] = path.gain;
            target.pathFlags[cursor] = path.flags;
            cursor++;
        }
        return cursor;
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

    private static int nextPositiveGearIndex(List<Double> ratios, int after) {
        for (int i = Math.max(0, after + 1); i < ratios.size(); i++) {
            if (ratios.get(i) > 0.0) return i;
        }
        return -1;
    }

    private static float smallestPositiveRatio(List<Double> ratios) {
        float result = Float.POSITIVE_INFINITY;
        for (double ratio : ratios) {
            if (ratio > 0.0) result = Math.min(result, (float) ratio);
        }
        return Float.isFinite(result) ? result : 0.0f;
    }

    private static float largestPositiveRatio(List<Double> ratios) {
        float result = 0.0f;
        for (double ratio : ratios) {
            if (ratio > 0.0) result = Math.max(result, (float) ratio);
        }
        return result;
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
            case TorqueConverterSpec ignored -> PowertrainTopologyContainer.TYPE_TORQUE_CONVERTER;
            case DctGearboxSpec ignored -> PowertrainTopologyContainer.TYPE_DCT_GEARBOX;
            case GearboxSpec gearbox -> "rangebox".equalsIgnoreCase(gearbox.type())
                    ? PowertrainTopologyContainer.TYPE_RANGE_BOX
                    : PowertrainTopologyContainer.TYPE_GEARBOX;
            case ShaftSpec ignored -> PowertrainTopologyContainer.TYPE_SHAFT;
            case DifferentialSpec ignored -> PowertrainTopologyContainer.TYPE_DIFFERENTIAL;
            case TorsionReactorSpec ignored -> PowertrainTopologyContainer.TYPE_TORSION_REACTOR;
            case SplitShaftSpec ignored -> PowertrainTopologyContainer.TYPE_SPLIT_SHAFT;
            case TurbochargerSpec ignored -> PowertrainTopologyContainer.TYPE_UNSUPPORTED;
            case SuperchargerSpec ignored -> PowertrainTopologyContainer.TYPE_UNSUPPORTED;
            case UnsupportedConfig ignored -> PowertrainTopologyContainer.TYPE_UNSUPPORTED;
        };
    }

    private static float ratioOf(DeviceSpec spec) {
        return switch (spec) {
            case GearboxSpec gearbox -> (float) gearbox.firstPositiveGearRatio();
            case DctGearboxSpec gearbox -> (float) gearbox.firstPositiveGearRatio();
            case ShaftSpec shaft -> (float) shaft.gearRatio();
            case TorsionReactorSpec reactor -> (float) reactor.gearRatio();
            case DifferentialSpec differential -> (float) differential.gearRatio();
            case SplitShaftSpec splitShaft -> (float) splitShaft.gearRatio();
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

    private static void configureSuperchargerType(SuperchargerContainer container, int unit,
                                                   SuperchargerSpec supercharger) {
        float pulseFloor;
        int lobes = Math.clamp(supercharger.lobes(), 2, 4);
        switch (supercharger.superchargerType().toLowerCase()) {
            case "screws" -> {
                lobes = Math.max(3, lobes);
                container.efficiencyB1[unit] = 0.3f;
                container.efficiencyB2[unit] = 0.0f;
                container.efficiencyB3[unit] = 0.7f;
                pulseFloor = 0.98f;
            }
            case "centrifugal" -> {
                lobes = 0;
                container.efficiencyB1[unit] = 0.6f;
                container.efficiencyB2[unit] = 0.0f;
                container.efficiencyB3[unit] = 0.45f;
                pulseFloor = 1.0f;
            }
            default -> {
                container.efficiencyB1[unit] = supercharger.twistedLobes() ? -0.35f : -0.55f;
                container.efficiencyB2[unit] = 0.0f;
                container.efficiencyB3[unit] = 1.0f;
                pulseFloor = supercharger.twistedLobes() ? 0.95f : 0.9f;
            }
        }
        container.pulseLobes[unit] = lobes;
        container.pulseFloor[unit] = Double.isFinite(supercharger.pulseCoefModifier())
                ? Math.clamp((float) supercharger.pulseCoefModifier(), 0.0f, 1.0f)
                : pulseFloor;
    }

    // ---------------------------------------------------------------- build records

    private record PathBuild(int wheel, float gain, byte flags) {
    }

    private record ReactorBuild(float gain, List<Integer> nodes) {
    }

    private record SplitBuild(int device, SplitShaftSpec spec,
                              List<PathBuild> primaryPaths, List<PathBuild> secondaryPaths) {
    }

    private record UnitBuild(int engineDevice, int clutchlikeDevice, CombustionEngineSpec engine,
                             FrictionClutchSpec clutch, TorqueConverterSpec converter, DctGearboxSpec dct,
                             float capacity, float spring, float maxTorque,
                             List<PathBuild> paths, List<Integer> reactions, List<ReactorBuild> reactors,
                             int gearboxDevice, GearSelectableSpec gearbox,
                             int rangeBoxDevice, GearboxSpec rangeBox,
                             List<SplitBuild> splitShafts, TurbochargerSpec turbocharger,
                             SuperchargerSpec supercharger) {
    }
}
