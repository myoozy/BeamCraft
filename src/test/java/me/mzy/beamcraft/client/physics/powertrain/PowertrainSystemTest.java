package me.mzy.beamcraft.client.physics.powertrain;

import me.mzy.beamcraft.client.physics.SoftBodyVehicle;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.*;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

class PowertrainSystemTest {
    @Test
    void separateFinalDrivePartOverridesDifferentialRatioEvenWhenLoadedFirst() {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        vehicle.wheels.count = 2;
        vehicle.wheels.nameToIndex.put("FL", 0);
        vehicle.wheels.nameToIndex.put("FR", 1);
        PowertrainSystem system = vehicle.powertrain;
        system.addSpecs(List.of(new DevicePatchSpec(
                "differential_F", List.of(new ValueModifier("gearRatio", '=', 3.9)))));
        system.addSpecs(List.of(
                new CombustionEngineSpec("combustionEngine", "engine", "dummy", 1,
                        0.1, 800, 6000, 0, 0, 0,
                        List.of(new TorquePoint(1000, 100)), List.of(), List.of()),
                new FrictionClutchSpec("frictionClutch", "clutch", "engine", 1,
                        100, 1000, 1, 0.15, 0.1, 1, List.of()),
                new GearboxSpec("manualGearbox", "gearbox", "clutch", 1,
                        List.of(-3.0, 0.0, 3.45), false, 0, 0, 0, List.of()),
                new DifferentialSpec("differential", "differential_F", "gearbox", 1,
                        1.0, 0.5, 0, 0, 0, "open", List.of()),
                new ShaftSpec("shaft", "left", "differential_F", 1,
                        1, "FL", 0, 0, 0, List.of(), List.of(), List.of()),
                new ShaftSpec("shaft", "right", "differential_F", 2,
                        1, "FR", 0, 0, 0, List.of(), List.of(), List.of())
        ));

        system.finalizeSetup();

        assertEquals(3.9f, system.differentials.gearRatio[0], 1e-6f);
        assertArrayEquals(new float[]{3.45f * 3.9f * 0.5f, 3.45f * 3.9f * 0.5f},
                system.wheelPaths.pathGain, 1e-5f);
    }

    @Test
    void compilesFixedFirstGearAndOpenDifferentialToFlatWheelPaths() {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        vehicle.wheels.count = 2;
        vehicle.wheels.nameToIndex.put("FL", 0);
        vehicle.wheels.nameToIndex.put("FR", 1);
        PowertrainSystem system = vehicle.powertrain;
        system.addSpecs(List.of(
                new CombustionEngineSpec("combustionEngine", "engine", "dummy", 1,
                        0.2, 800, 7000, 1, 0.01, 2,
                        List.of(new TorquePoint(1000, 100), new TorquePoint(5000, 200)), List.of(), List.of()),
                new CombustionEngineSpec("combustionEngine", "engine", "dummy", 1,
                        0.1, 800, 6000, 0, 0, 0, List.of(), List.of(),
                        List.of(new ValueModifier("friction", '*', 2.0))),
                new FrictionClutchSpec("frictionClutch", "clutch", "engine", 1,
                        300, 1000, 1, 0.2, 0.125, 1, List.of()),
                new GearboxSpec("manualGearbox", "gearbox", "clutch", 1,
                        List.of(-3.0, 0.0, 3.0, 2.0), true, 0, 0, 0, List.of()),
                new DifferentialSpec("differential", "diff", "gearbox", 1,
                        4.0, 0.5, 0, 0, 0, "open", List.of()),
                new ShaftSpec("shaft", "left", "diff", 1, 1, "FL", 0, 0, 0,
                        List.of(), List.of(), List.of()),
                new ShaftSpec("shaft", "right", "diff", 2, 1, "FR", 0, 0, 0,
                        List.of(), List.of(), List.of()),
                new ShaftSpec("shaft", "unusedSpindle", "missingAxle", 1, 1, null, 0, 0, 0,
                        List.of(), List.of(), List.of())
        ));

        system.finalizeSetup();

        assertEquals(7, system.topology.deviceCount);
        assertEquals(1, system.engines.unitCount);
        assertArrayEquals(new int[]{0, 1}, system.wheelPaths.pathWheel);
        assertArrayEquals(new float[]{6.0f, 6.0f}, system.wheelPaths.pathGain, 1e-6f);
        assertEquals(3.0f, system.topology.deviceRatio[2], 1e-6f);
        assertEquals(2, system.gearboxes.currentGearIndex[0]);
        assertEquals(3.0f, system.gearboxes.activeRatio[0], 1e-6f);
        assertEquals(2.0f, system.engines.engineFriction[0], 1e-6f);
        assertEquals("ready; 1 detached device(s)", system.diagnostic());
        // The dangling branch (unusedSpindle references a missing axle) is isolated
        // to TYPE_UNSUPPORTED and never reaches a wheel path.
        assertEquals(PowertrainTopologyContainer.TYPE_UNSUPPORTED, system.topology.deviceType[6]);
    }

    @Test
    void unsupportedDeviceInChainIsIsolatedWithoutKillingSupportedPath() {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        vehicle.wheels.count = 2;
        vehicle.wheels.nameToIndex.put("FL", 0);
        vehicle.wheels.nameToIndex.put("FR", 1);
        PowertrainSystem system = vehicle.powertrain;
        system.addSpecs(List.of(
                new CombustionEngineSpec("combustionEngine", "engine", "dummy", 1,
                        0.2, 800, 7000, 1, 0.01, 2,
                        List.of(new TorquePoint(1000, 100), new TorquePoint(5000, 200)), List.of(), List.of()),
                new FrictionClutchSpec("frictionClutch", "clutch", "engine", 1,
                        300, 1000, 1, 0.2, 0.125, 1, List.of()),
                new GearboxSpec("manualGearbox", "gearbox", "clutch", 1,
                        List.of(-3.0, 0.0, 3.0, 2.0), false, 0, 0, 0, List.of()),
                new UnsupportedConfig("torqueConverter", "tc", "clutch", 2,
                        Map.of(), "unsupported", List.of()),
                new DifferentialSpec("differential", "diff", "gearbox", 1,
                        4.0, 0.5, 0, 0, 0, "open", List.of()),
                new ShaftSpec("shaft", "left", "diff", 1, 1, "FL", 0, 0, 0,
                        List.of(), List.of(), List.of()),
                new ShaftSpec("shaft", "right", "diff", 2, 1, "FR", 0, 0, 0,
                        List.of(), List.of(), List.of())
        ));

        system.finalizeSetup();

        // The torqueConverter branch is isolated; the gearbox→diff→wheels path still compiles.
        assertEquals(PowertrainTopologyContainer.TYPE_UNSUPPORTED, system.topology.deviceType[3]);
        assertEquals(1, system.engines.unitCount);
        assertEquals(2, system.wheelPaths.pathWheel.length);
        assertEquals("ready", system.diagnostic());
    }

    @Test
    void gearboxCompilesToRuntimeSoaWithFirstForwardGearActive() {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        vehicle.wheels.count = 2;
        vehicle.wheels.nameToIndex.put("FL", 0);
        vehicle.wheels.nameToIndex.put("FR", 1);
        PowertrainSystem system = vehicle.powertrain;
        system.addSpecs(List.of(
                new CombustionEngineSpec("combustionEngine", "engine", "dummy", 1,
                        0.2, 800, 7000, 1, 0.01, 2,
                        List.of(new TorquePoint(1000, 100), new TorquePoint(5000, 200)), List.of(), List.of()),
                new FrictionClutchSpec("frictionClutch", "clutch", "engine", 1,
                        300, 1000, 1, 0.2, 0.125, 1, List.of()),
                new GearboxSpec("manualGearbox", "gearbox", "clutch", 1,
                        List.of(-3.0, 0.0, 3.0, 2.0), false, 0, 0, 0, List.of()),
                new DifferentialSpec("differential", "diff", "gearbox", 1,
                        4.0, 0.5, 0, 0, 0, "open", List.of()),
                new ShaftSpec("shaft", "left", "diff", 1, 1, "FL", 0, 0, 0,
                        List.of(), List.of(), List.of()),
                new ShaftSpec("shaft", "right", "diff", 2, 1, "FR", 0, 0, 0,
                        List.of(), List.of(), List.of())
        ));

        system.finalizeSetup();

        assertEquals(1, system.gearboxes.unitCount);
        assertArrayEquals(new float[]{-3.0f, 0.0f, 3.0f, 2.0f}, system.gearboxes.gearRatios, 1e-6f);
        assertEquals(1, system.gearboxes.initialGearIndex[0]); // neutral
        assertEquals(1, system.gearboxes.currentGearIndex[0]);
        assertEquals(0.0f, system.gearboxes.activeRatio[0], 1e-6f);
        assertEquals(3.0f, system.gearboxes.pathBaseRatio[0], 1e-6f);
        assertEquals(-1, system.gearboxes.pendingGearIndex[0]);
        assertEquals(2, system.gearboxes.device[0]); // gearbox device index in the topology
        assertEquals("gearbox", system.gearboxes.deviceName[0]);
        assertEquals(0.25, system.gearboxes.shiftDuration[0], 1e-6f); // manual default shift time

        system.gearboxes.currentGearIndex[0] = 2;
        system.gearboxes.activeRatio[0] = 3.0f;
        system.reset();
        assertEquals(1, system.gearboxes.currentGearIndex[0]);
        assertEquals(0.0f, system.gearboxes.activeRatio[0], 1e-6f);
    }
}
