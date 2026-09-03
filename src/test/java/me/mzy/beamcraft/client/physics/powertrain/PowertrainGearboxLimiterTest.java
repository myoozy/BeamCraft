package me.mzy.beamcraft.client.physics.powertrain;

import me.mzy.beamcraft.client.physics.NodeContainer;
import me.mzy.beamcraft.client.physics.SoftBodyVehicle;
import me.mzy.beamcraft.client.physics.WheelContainer;
import me.mzy.beamcraft.client.physics.electrics.ElectricSignals;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.CombustionEngineSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.FrictionClutchSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.GearboxSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.ShaftSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.TorquePoint;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Rev-limiter and gearbox-shift mechanics. All timers ({@code shiftRemaining},
 * {@code limiterCutRemaining}) are only ever decremented by {@code solve(dt)} — never by
 * wall/game time — and the torque path is disconnected (active ratio 0) for the whole
 * shift while the engine still integrates.
 */
class PowertrainGearboxLimiterTest {
    private static final float DT = 0.0005f; // 2000 Hz substep

    private static SoftBodyVehicle rig(List<Double> gearRatios, double shiftTime,
                                       double revLimiterRPM, double revLimiterCutTime,
                                       double revLimiterMaxRPMDrop) {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        NodeContainer nodes = vehicle.nodes;
        nodes.posX[0] = 0f; nodes.posY[0] = 0f; nodes.posZ[0] = 0f;
        nodes.posX[1] = 1f; nodes.posY[1] = 0f; nodes.posZ[1] = 0f;
        nodes.mass[0] = 1f; nodes.mass[1] = 1f;
        nodes.count = 2;
        vehicle.wheels.count = 1;
        vehicle.wheels.nameToIndex.put("W", 0);
        addWheel(vehicle, 0, 4, new float[][]{
                {0.5f, 0f, 0.35f}, {0.5f, 0f, -0.35f}, {0.5f, 0.35f, 0f}, {0.5f, -0.35f, 0f}});
        vehicle.powertrain.addSpecs(List.of(
                new CombustionEngineSpec("combustionEngine", "engine", "dummy", 1,
                        0.072, 850, 7500, 11.5, 0.024, 38, sunburstTorque(), List.of(), List.of(),
                        88.8, 400, 100, revLimiterRPM, "time", revLimiterCutTime, revLimiterMaxRPMDrop,
                        0.01, 0.15),
                new FrictionClutchSpec("frictionClutch", "clutch", "engine", 1,
                        0, 0, 0.25, 0.15, 0.75, 1, List.of()),
                new GearboxSpec("sequentialGearbox", "gearbox", "clutch", 1,
                        gearRatios, false, 1.24, 0.00124, 0.013, List.of(), shiftTime),
                new ShaftSpec("shaft", "s", "gearbox", 1, 1, "W", 0, 0, 0,
                        List.of(), List.of(), List.of())
        ));
        vehicle.powertrain.finalizeSetup();
        return vehicle;
    }

    private static void addWheel(SoftBodyVehicle vehicle, int wIdx, int nodeBase, float[][] positions) {
        NodeContainer nodes = vehicle.nodes;
        vehicle.wheels.node1[wIdx] = 0;
        vehicle.wheels.node2[wIdx] = 1;
        vehicle.wheels.wheelDir[wIdx] = 1;
        vehicle.wheels.numRays[wIdx] = 2;
        vehicle.wheels.tireRadius[wIdx] = 0f;
        for (int ray = 0; ray < 2; ray++) {
            vehicle.wheels.hubInnerNodes[wIdx * WheelContainer.MAX_RAYS + ray] = nodeBase + ray * 2;
            vehicle.wheels.hubOuterNodes[wIdx * WheelContainer.MAX_RAYS + ray] = nodeBase + ray * 2 + 1;
        }
        for (int i = 0; i < positions.length; i++) {
            int idx = nodeBase + i;
            nodes.posX[idx] = positions[i][0];
            nodes.posY[idx] = positions[i][1];
            nodes.posZ[idx] = positions[i][2];
            nodes.mass[idx] = 2.5f;
            nodes.count = Math.max(nodes.count, idx + 1);
        }
    }

    /** Semi-implicit integration of the node forces, clearing them each substep. */
    private static void integrate(SoftBodyVehicle vehicle, float dt) {
        NodeContainer nodes = vehicle.nodes;
        for (int i = 0; i < nodes.count; i++) {
            if (nodes.mass[i] <= 1e-9f) continue;
            nodes.velX[i] += nodes.forceX[i] / nodes.mass[i] * dt;
            nodes.velY[i] += nodes.forceY[i] / nodes.mass[i] * dt;
            nodes.velZ[i] += nodes.forceZ[i] / nodes.mass[i] * dt;
            nodes.posX[i] += nodes.velX[i] * dt;
            nodes.posY[i] += nodes.velY[i] * dt;
            nodes.posZ[i] += nodes.velZ[i] * dt;
            nodes.forceX[i] = 0f;
            nodes.forceY[i] = 0f;
            nodes.forceZ[i] = 0f;
        }
    }

    private static List<TorquePoint> sunburstTorque() {
        return List.of(
                new TorquePoint(0, 0), new TorquePoint(500, 62), new TorquePoint(1000, 105),
                new TorquePoint(1500, 142), new TorquePoint(2000, 176), new TorquePoint(2500, 198),
                new TorquePoint(3000, 210), new TorquePoint(3500, 216), new TorquePoint(4000, 221),
                new TorquePoint(4500, 222), new TorquePoint(5000, 221), new TorquePoint(5500, 218),
                new TorquePoint(6000, 210), new TorquePoint(6500, 199), new TorquePoint(7000, 186),
                new TorquePoint(7500, 168), new TorquePoint(8000, 149), new TorquePoint(8500, 135),
                new TorquePoint(9000, 122), new TorquePoint(9500, 108), new TorquePoint(10000, 90));
    }

    // ---------------------------------------------------------------- gearbox shifts

    @Test
    void shiftEventWaitsForPhysicsAndThenDisconnectsTorque() {
        SoftBodyVehicle vehicle = rig(List.of(-3.21, 0.0, 3.31, 2.38), 0.5, 7500, 0.15, 300);
        assertEquals(1, vehicle.powertrain.gearboxes.initialGearIndex[0]);
        assertEquals(0.0f, vehicle.powertrain.gearboxes.activeRatio[0], 1e-6f);

        vehicle.powertrain.requestShiftUp();
        assertEquals(-1, vehicle.powertrain.gearboxes.pendingGearIndex[0],
                "client input must not mutate physics state before solve");

        vehicle.powertrain.solve(0.0f);
        assertEquals(-1, vehicle.powertrain.gearboxes.pendingGearIndex[0],
                "a zero-duration solve must not consume the event");
        vehicle.powertrain.solve(DT);
        assertEquals(2, vehicle.powertrain.gearboxes.pendingGearIndex[0]);
        assertEquals(0.5f - DT, vehicle.powertrain.gearboxes.shiftRemaining[0], 1e-5f);
        assertEquals(0.0f, vehicle.powertrain.gearboxes.activeRatio[0], 1e-6f,
                "consuming the shift event must disconnect the torque path");
        assertEquals(1, vehicle.powertrain.gearboxes.currentGearIndex[0], "still in neutral");
    }

    @Test
    void shiftEventsAreConsumedOnceFromTheElectricBus() {
        SoftBodyVehicle vehicle = rig(List.of(-3.21, 0.0, 3.31, 2.38), 0.5, 7500, 0.15, 300);
        vehicle.electrics.set(ElectricSignals.SHIFT_UP_EVENT, 1.0);

        vehicle.powertrain.solve(DT);
        assertEquals(2, vehicle.powertrain.gearboxes.pendingGearIndex[0]);

        vehicle.powertrain.solve(DT);
        assertEquals(2, vehicle.powertrain.gearboxes.pendingGearIndex[0],
                "one event snapshot must not be consumed once per substep");

        vehicle.electrics.set(ElectricSignals.SHIFT_UP_EVENT, 2.0);
        vehicle.powertrain.solve(DT);
        assertEquals(3, vehicle.powertrain.gearboxes.pendingGearIndex[0]);
    }

    @Test
    void shiftDisconnectsTorquePathDuringWholeShift() {
        SoftBodyVehicle vehicle = rig(List.of(-3.21, 0.0, 3.31, 2.38), 0.5, 7500, 0.15, 300);
        vehicle.powertrain.engines.engineAV[0] = 200.0f; // would drive the wheels if connected
        vehicle.powertrain.requestShiftUp();
        for (int step = 0; step < 100; step++) {
            vehicle.powertrain.solve(DT); // 0.05 s < 0.5 s shift
        }
        assertEquals(0.0f, vehicle.powertrain.debugActiveRatio(), 1e-6f);
        assertEquals(0.0f, vehicle.powertrain.debugClutchTorque(), 1e-6f,
                "no clutch torque may flow while the ratio is disconnected");
        assertTrue(vehicle.powertrain.gearboxes.shiftRemaining[0] > 0.0f, "still shifting");
        assertEquals(1, vehicle.powertrain.gearboxes.currentGearIndex[0], "gear not committed yet");
    }

    @Test
    void shiftCompletesAtCorrectPhysicsTime() {
        SoftBodyVehicle vehicle = rig(List.of(-3.21, 0.0, 3.31, 2.38), 0.5, 7500, 0.15, 300);
        vehicle.powertrain.requestShiftUp();
        for (int step = 0; step < 999; step++) {
            vehicle.powertrain.solve(DT); // 0.4995 s
        }
        assertTrue(vehicle.powertrain.gearboxes.shiftRemaining[0] > 0.0f,
                "0.4995 s must be before the 0.5 s shift completes");
        assertEquals(0.0f, vehicle.powertrain.debugActiveRatio(), 1e-6f);

        // The 0.5 s shift completes once accumulated dt reaches it (float accumulation may
        // need a step or two past exactly 1000 solves).
        int guard = 0;
        while (vehicle.powertrain.gearboxes.pendingGearIndex[0] != -1 && guard < 20) {
            vehicle.powertrain.solve(DT);
            guard++;
        }
        assertEquals(-1, vehicle.powertrain.gearboxes.pendingGearIndex[0],
                "shift must commit once the accumulated physics time reaches the duration");
        assertEquals(2, vehicle.powertrain.gearboxes.currentGearIndex[0]);
        assertEquals(3.31f, vehicle.powertrain.debugActiveRatio(), 1e-4f);
        assertEquals(0.0f, vehicle.powertrain.debugShiftRemaining(), 1e-6f);
    }

    @Test
    void neutralGearDisconnectsTorqueButEngineKeepsRunning() {
        SoftBodyVehicle vehicle = rig(List.of(-3.21, 0.0, 3.31, 2.38), 0.25, 7500, 0.15, 300);
        assertEquals(1, vehicle.powertrain.gearboxes.currentGearIndex[0]);
        assertEquals(0.0f, vehicle.powertrain.debugActiveRatio(), 1e-6f);
        assertEquals(0.0f, vehicle.powertrain.debugClutchTorque(), 1e-6f);

        // The engine keeps integrating and idles in neutral.
        vehicle.powertrain.setControls(0.0f, 1.0f);
        for (int step = 0; step < 500; step++) {
            vehicle.powertrain.solve(DT);
        }
        assertTrue(vehicle.powertrain.debugEngineRPM() > 600.0f,
                "the engine must keep running in neutral, got " + vehicle.powertrain.debugEngineRPM());
    }

    @Test
    void reverseGearFlipsDriveTorqueSign() {
        SoftBodyVehicle vehicle = rig(List.of(-3.21, 0.0, 3.31), 0.25, 7500, 0.15, 300);
        vehicle.powertrain.requestShiftDown(); // neutral -> reverse
        for (int step = 0; step < 600; step++) {
            vehicle.powertrain.solve(DT);
        }
        assertEquals(0, vehicle.powertrain.gearboxes.currentGearIndex[0]);
        assertEquals(-3.21f, vehicle.powertrain.debugActiveRatio(), 1e-4f);

        // Engine leading + engaged clutch: a negative ratio must flip the wheel drive
        // direction relative to first gear.
        vehicle.powertrain.engines.engineAV[0] = 100.0f;
        vehicle.powertrain.setControls(0.0f, 0.0f);
        float wheelAVBefore = vehicle.wheels.getAngularVelocity(0);
        for (int step = 0; step < 30; step++) {
            vehicle.powertrain.solve(DT);
            integrate(vehicle, DT);
        }
        assertTrue(vehicle.wheels.getAngularVelocity(0) > wheelAVBefore,
                "reverse ratio must flip the drive torque sign, got Δ "
                        + (vehicle.wheels.getAngularVelocity(0) - wheelAVBefore));
    }

    // ---------------------------------------------------------------- rev limiter

    @Test
    void limiterCyclesCombustionUsingOnlyAccumulatedDt() {
        SoftBodyVehicle vehicle = rig(List.of(3.31), 0.25, 1200.0, 0.05, 100.0);
        vehicle.powertrain.setControls(1.0f, 1.0f); // full throttle, released clutch -> free rev
        boolean sawCut = false;
        float maxRpm = 0f;
        for (int step = 0; step < 4000; step++) {
            vehicle.powertrain.solve(DT);
            maxRpm = Math.max(maxRpm, vehicle.powertrain.debugEngineRPM());
            if (vehicle.powertrain.debugLimiterActive()) sawCut = true;
        }
        assertTrue(sawCut, "the rev limiter must cut combustion");
        assertTrue(maxRpm < 1800f, "RPM must stay bounded by the limiter, got max " + maxRpm);

        // Timer mechanics: cut time counts down only via solve(dt).
        vehicle.powertrain.engines.limiterCutRemaining[0] = 0.03f;
        float before = vehicle.powertrain.engines.limiterCutRemaining[0];
        assertEquals(before, vehicle.powertrain.engines.limiterCutRemaining[0], 0f,
                "cut timer is frozen between solves");
        vehicle.powertrain.solve(DT);
        assertEquals(before - DT, vehicle.powertrain.engines.limiterCutRemaining[0], 1e-5f,
                "cut timer must decrement by exactly the accumulated physics dt");
    }
}
