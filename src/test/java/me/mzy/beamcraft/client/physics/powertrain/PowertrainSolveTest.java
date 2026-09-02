package me.mzy.beamcraft.client.physics.powertrain;

import me.mzy.beamcraft.client.physics.NodeContainer;
import me.mzy.beamcraft.client.physics.SoftBodyVehicle;
import me.mzy.beamcraft.client.physics.WheelContainer;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.CombustionEngineSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.DifferentialSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.FrictionClutchSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.GearboxSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.ShaftSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.TorquePoint;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Solver-level regression tests for the reported "full throttle does not raise RPM"
 * bug. They use a Sunburst-like FWD chain (real 2.0L engine torque table and clutch
 * params) with two drivable wheels, and assert on the debug combustion torque, torque
 * curve count, engine RPM and clutch torque:
 * <ul>
 *   <li>throttle produces positive indicated combustion torque and a real clutch torque
 *       flowing to the wheels, and the engine revs up above idle;</li>
 *   <li>with zero throttle the engine keeps a live idle instead of dead-stalling to 0 RPM
 *       (the original bug);</li>
 *   <li>rolling wheels still back-drive the engine above idle.</li>
 * </ul>
 */
class PowertrainSolveTest {
    private static final float DT = 0.0005f; // 2000 Hz substep
    private static final float IDLE_RPM = 850.0f;

    private static SoftBodyVehicle sunburstLikeVehicle() {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        NodeContainer nodes = vehicle.nodes;

        // Axle nodes 0,1 along +X; hub nodes orbit the axle midpoint (0.5, 0, 0).
        nodes.posX[0] = 0f; nodes.posY[0] = 0f; nodes.posZ[0] = 0f;
        nodes.posX[1] = 1f; nodes.posY[1] = 0f; nodes.posZ[1] = 0f;
        nodes.mass[0] = 1f; nodes.mass[1] = 1f;
        nodes.count = 2;

        vehicle.wheels.count = 2;
        vehicle.wheels.nameToIndex.put("FL", 0);
        vehicle.wheels.nameToIndex.put("FR", 1);
        // 4 hub nodes per wheel, radius 0.35, mass 2.5 each → ~1.2 kg·m² polar inertia.
        addWheel(vehicle, 0, 4, new float[][]{
                {0.5f, 0f, 0.35f}, {0.5f, 0f, -0.35f}, {0.5f, 0.35f, 0f}, {0.5f, -0.35f, 0f}});
        addWheel(vehicle, 1, 8, new float[][]{
                {0.5f, 0f, 0.35f}, {0.5f, 0f, -0.35f}, {0.5f, 0.35f, 0f}, {0.5f, -0.35f, 0f}});
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

    private static void addSunburstPowertrain(SoftBodyVehicle vehicle) {
        vehicle.powertrain.addSpecs(List.of(
                new CombustionEngineSpec("combustionEngine", "mainEngine", "dummy", 1,
                        0.072, IDLE_RPM, 7500, 11.5, 0.024, 38, sunburstTorque(), List.of(), List.of()),
                new FrictionClutchSpec("frictionClutch", "clutch", "mainEngine", 1,
                        0, 0, 0.25, 0.15, 0.75, 1, List.of()),
                new GearboxSpec("sequentialGearbox", "gearbox", "clutch", 1,
                        List.of(-3.21, 0.0, 3.31, 2.38, 1.76, 1.35, 1.06, 0.84), false, 1.24, 0.00124, 0.013, List.of()),
                new DifferentialSpec("differential", "diff", "gearbox", 1,
                        3.9, 0.5, 2.2, 0.0009, 0.028, "open", List.of()),
                new ShaftSpec("shaft", "left", "diff", 1, 1, "FL", 0, 0, 0, List.of(), List.of(), List.of()),
                new ShaftSpec("shaft", "right", "diff", 2, 1, "FR", 0, 0, 0, List.of(), List.of(), List.of())
        ));
        vehicle.powertrain.finalizeSetup();
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

    /** Selects first gear directly when a test specifically exercises the connected driveline. */
    private static void selectFirstForwardGear(SoftBodyVehicle vehicle) {
        GearboxContainer gearboxes = vehicle.powertrain.gearboxes;
        int start = gearboxes.gearStart[0];
        int end = start + gearboxes.gearCount[0];
        for (int i = start; i < end; i++) {
            if (gearboxes.gearRatios[i] > 0.0f) {
                gearboxes.currentGearIndex[0] = i - start;
                gearboxes.activeRatio[0] = gearboxes.gearRatios[i];
                return;
            }
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

    /**
     * Gives the wheel's hub nodes a rigid rotation with {@code getAngularVelocity} = wheelAV.
     * The system reads the axle as node1 - node2 = (-1, 0, 0), so a positive AV needs
     * v = axis × r · wheelAV with axis = (-1, 0, 0): velY = rz·w, velZ = -ry·w.
     */
    private static void spinWheel(SoftBodyVehicle vehicle, int wIdx, float wheelAV) {
        NodeContainer nodes = vehicle.nodes;
        int base = wIdx * WheelContainer.MAX_RAYS;
        for (int ray = 0; ray < 2; ray++) {
            int inner = vehicle.wheels.hubInnerNodes[base + ray];
            int outer = vehicle.wheels.hubOuterNodes[base + ray];
            for (int node : new int[]{inner, outer}) {
                float ry = nodes.posY[node];
                float rz = nodes.posZ[node];
                nodes.velX[node] = 0f;
                nodes.velY[node] = rz * wheelAV;
                nodes.velZ[node] = -ry * wheelAV;
            }
        }
    }

    // ---------------------------------------------------------------- regression tests

    @Test
    void fullThrottleWithSlippingClutchProducesCombustionTorqueAndRevvesEngine() {
        SoftBodyVehicle vehicle = sunburstLikeVehicle();
        addSunburstPowertrain(vehicle);
        selectFirstForwardGear(vehicle);
        assertEquals("ready", vehicle.powertrain.diagnostic());
        assertEquals(21, vehicle.powertrain.debugTorqueCurveCount(),
                "Sunburst engine torque table must compile 21 points");

        // Half-engaged clutch: enough to transmit drive torque to the wheels while letting
        // the engine rev. A fully dumped clutch would legitimately stall the engine — the
        // clutch is governed only by its friction capacity and may do so.
        vehicle.powertrain.setControls(1.0f, 0.5f);
        float peakRpm = 0f;
        for (int step = 0; step < 800; step++) {
            vehicle.powertrain.solve(DT);
            integrate(vehicle, DT);
            peakRpm = Math.max(peakRpm, vehicle.powertrain.debugEngineRPM());
        }

        assertTrue(vehicle.powertrain.debugCombustionTorque() > 0f,
                "full throttle must produce positive indicated combustion torque");
        assertTrue(vehicle.powertrain.debugClutchTorque() > 0f,
                "a slipping clutch must transmit drive torque to the wheels");
        assertTrue(peakRpm > IDLE_RPM + 200f,
                "full throttle must raise engine RPM well above idle, got peak " + peakRpm);
    }

    @Test
    void releasedThrottleKeepsALiveIdleInsteadOfDeadStalling() {
        SoftBodyVehicle vehicle = sunburstLikeVehicle();
        addSunburstPowertrain(vehicle);

        // With the clutch released, a running engine should regulate itself around idle.
        vehicle.powertrain.setControls(0.0f, 1.0f);
        for (int step = 0; step < 1000; step++) {
            vehicle.powertrain.solve(DT);
        }
        float rpm = vehicle.powertrain.debugEngineRPM();
        assertTrue(rpm > 300f, "engine must hold a live idle instead of stalling to 0, got " + rpm);
    }

    @Test
    void engineReturnsToLiveIdleAfterThrottleBlip() {
        SoftBodyVehicle vehicle = sunburstLikeVehicle();
        addSunburstPowertrain(vehicle);
        vehicle.powertrain.setControls(1.0f, 1.0f);
        for (int step = 0; step < 1000; step++) vehicle.powertrain.solve(DT);
        assertTrue(vehicle.powertrain.debugEngineRPM() > IDLE_RPM + 200.0f,
                "throttle blip must first raise engine speed");

        vehicle.powertrain.setControls(0.0f, 1.0f);
        float minimumRpm = Float.POSITIVE_INFINITY;
        for (int step = 0; step < 5000; step++) {
            vehicle.powertrain.solve(DT);
            minimumRpm = Math.min(minimumRpm, vehicle.powertrain.debugEngineRPM());
        }
        assertTrue(minimumRpm > 300.0f,
                "return from a throttle blip must not overshoot into a stall, min " + minimumRpm);
        assertTrue(vehicle.powertrain.debugEngineRPM() > 300.0f,
                "engine must settle at a live idle after the blip");
    }

    @Test
    void internalResistanceStopsAtZeroInsteadOfReversingCrank() {
        SoftBodyVehicle vehicle = sunburstLikeVehicle();
        addSunburstPowertrain(vehicle);
        vehicle.powertrain.engines.engineAV[0] = 0.001f;
        vehicle.powertrain.setControls(0.0f, 1.0f);

        vehicle.powertrain.solve(DT);

        assertEquals(0.0f, vehicle.powertrain.engines.engineAV[0], 0.0f,
                "a resistance impulse crossing zero must stop the crank exactly");
    }

    @Test
    void stalledEngineWithThrottleButNoStarterStaysDead() {
        SoftBodyVehicle vehicle = sunburstLikeVehicle();
        addSunburstPowertrain(vehicle);
        vehicle.powertrain.engines.engineAV[0] = 0.0f;

        // Combustion requires cranking speed; a dead engine cannot restart from
        // throttle alone — the previous always-running recovery trick is gone.
        vehicle.powertrain.setControls(1.0f, 1.0f);
        for (int step = 0; step < 400; step++) {
            vehicle.powertrain.solve(DT);
        }

        assertTrue(vehicle.powertrain.debugEngineRPM() < 50.0f,
                "a stalled engine must not restart from throttle alone, got "
                        + vehicle.powertrain.debugEngineRPM());
    }

    @Test
    void starterRestartsAnEngineThatHasBeenStalled() {
        SoftBodyVehicle vehicle = sunburstLikeVehicle();
        addSunburstPowertrain(vehicle);
        vehicle.powertrain.engines.engineAV[0] = 0.0f;

        // Release the clutch so the starter only has to spin the crank, then hold V.
        vehicle.powertrain.setControls(0.0f, 1.0f, true);
        float peakRpm = 0f;
        for (int step = 0; step < 800; step++) {
            vehicle.powertrain.solve(DT);
            peakRpm = Math.max(peakRpm, vehicle.powertrain.debugEngineRPM());
        }

        assertTrue(vehicle.powertrain.debugStarterActive(), "starter input must be reported");
        assertTrue(peakRpm > 500.0f,
                "the starter must crank a stalled engine back up, got peak " + peakRpm);
        assertTrue(vehicle.powertrain.debugEngineRPM() > 300.0f,
                "after the starter, combustion + idle controller must hold a running idle, got "
                        + vehicle.powertrain.debugEngineRPM());
    }

    @Test
    void rollingWheelsBackDriveTheEngineAboveIdle() {
        SoftBodyVehicle vehicle = sunburstLikeVehicle();
        addSunburstPowertrain(vehicle);
        selectFirstForwardGear(vehicle);

        // Push the car: wheels roll forward, so the driveline is faster than the idle
        // crank and the clutch back-drives the engine — this must keep working.
        spinWheel(vehicle, 0, -20.0f);
        spinWheel(vehicle, 1, -20.0f);
        vehicle.powertrain.setControls(0.0f, 0.0f);
        float peakRpm = 0f;
        for (int step = 0; step < 800; step++) {
            vehicle.powertrain.solve(DT);
            peakRpm = Math.max(peakRpm, vehicle.powertrain.debugEngineRPM());
        }
        assertTrue(peakRpm > IDLE_RPM + 150f,
                "rolling wheels must back-drive the engine above idle, got peak " + peakRpm);
    }

    // ---------------------------------------------------------------- helpers

    /**
     * Minimal single-wheel powertrain with a zero-torque curve, used to isolate the clutch
     * sign conventions. {@code gearRatio} is the gearbox's only forward ratio (gain = it).
     */
    private static SoftBodyVehicle signRig(float gearRatio, float engineInertia, float clutchCapacity) {
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
                        engineInertia, 850, 6000, 0, 0, 0, List.of(), List.of(), List.of()),
                new FrictionClutchSpec("frictionClutch", "clutch", "engine", 1,
                        clutchCapacity, 0, 1, 0.15, 0.125, 1, List.of()),
                new GearboxSpec("manualGearbox", "gearbox", "clutch", 1,
                        List.of((double) gearRatio), false, 0, 0, 0, List.of()),
                new ShaftSpec("shaft", "s", "gearbox", 1, 1, "W", 0, 0, 0, List.of(), List.of(), List.of())
        ));
        vehicle.powertrain.finalizeSetup();
        return vehicle;
    }

    /**
     * Engine driving through a unit gear ratio into a nearly-locked wheel (huge hub mass),
     * used to prove a real stall: the clutch drag exceeds the engine's low-rpm torque and
     * combustion legitimately stops below cranking speed.
     */
    private static SoftBodyVehicle lockedLoadRig() {
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
        for (int i = 4; i < 8; i++) nodes.mass[i] = 10000.0f; // effectively locked wheel
        vehicle.powertrain.addSpecs(List.of(
                new CombustionEngineSpec("combustionEngine", "engine", "dummy", 1,
                        0.072, IDLE_RPM, 7500, 11.5, 0.024, 38, sunburstTorque(), List.of(), List.of()),
                new FrictionClutchSpec("frictionClutch", "clutch", "engine", 1,
                        150, 0, 1, 0.15, 0.75, 1, List.of()),
                new GearboxSpec("manualGearbox", "gearbox", "clutch", 1,
                        List.of(1.0), false, 0, 0, 0, List.of()),
                new ShaftSpec("shaft", "s", "gearbox", 1, 1, "W", 0, 0, 0, List.of(), List.of(), List.of())
        ));
        vehicle.powertrain.finalizeSetup();
        return vehicle;
    }

    // ---------------------------------------------------------------- new-stage behaviour

    @Test
    void idleControllerCommandsThrottleAbovePlayerZeroAndHoldsUnloadedIdle() {
        SoftBodyVehicle vehicle = sunburstLikeVehicle();
        addSunburstPowertrain(vehicle);
        vehicle.powertrain.setControls(0.0f, 1.0f); // released clutch, no player throttle
        float minRpm = Float.MAX_VALUE;
        float maxRpm = Float.MIN_VALUE;
        for (int step = 0; step < 2000; step++) {
            vehicle.powertrain.solve(DT);
            minRpm = Math.min(minRpm, vehicle.powertrain.debugEngineRPM());
            maxRpm = Math.max(maxRpm, vehicle.powertrain.debugEngineRPM());
        }
        assertTrue(vehicle.powertrain.debugActualThrottle() > 0.0f,
                "idle controller must command a non-zero actual throttle with player at 0");
        assertTrue(minRpm > IDLE_RPM - 100f, "unloaded idle must be held near idle, min " + minRpm);
        assertTrue(maxRpm < IDLE_RPM + 150f, "unloaded idle must not run away, max " + maxRpm);
    }

    @Test
    void engineLeadingClutchAppliesCorrectedDriveTorqueToWheels() {
        SoftBodyVehicle vehicle = signRig(1.0f, 0.5f, 5000.0f);
        vehicle.powertrain.setControls(0.0f, 0.0f); // engaged clutch, no combustion input
        vehicle.powertrain.engines.engineAV[0] = 100.0f; // ~955 rpm positive crank spin
        for (int step = 0; step < 20; step++) {
            vehicle.powertrain.solve(DT);
            integrate(vehicle, DT);
        }
        assertTrue(vehicle.powertrain.debugClutchTorque() > 0.0f,
                "engine leading must produce positive clutch torque");
        assertTrue(vehicle.wheels.getAngularVelocity(0) < 0.0f,
                "engine->wheel drive torque must have the corrected (negative) sign, got "
                        + vehicle.wheels.getAngularVelocity(0));
        assertTrue(vehicle.powertrain.engines.engineAV[0] < 100.0f,
                "the clutch must load the engine");
    }

    @Test
    void wheelsLeadingBackDriveTheEngineWithNegativeClutchTorque() {
        SoftBodyVehicle vehicle = signRig(3.0f, 0.5f, 5000.0f);
        spinWheel(vehicle, 0, -50.0f); // forward wheel motion is negative in the corrected convention
        vehicle.powertrain.setControls(0.0f, 0.0f);
        float engineAVBefore = vehicle.powertrain.engines.engineAV[0]; // idle
        float minimumClutchTorque = Float.POSITIVE_INFINITY;
        for (int step = 0; step < 20; step++) {
            vehicle.powertrain.solve(DT);
            minimumClutchTorque = Math.min(minimumClutchTorque, vehicle.powertrain.debugClutchTorque());
            integrate(vehicle, DT);
        }
        assertTrue(minimumClutchTorque < 0.0f,
                "wheels leading must produce negative clutch torque (back-drive)");
        assertTrue(vehicle.powertrain.engines.engineAV[0] > engineAVBefore,
                "back-drive must raise engine AV, got " + vehicle.powertrain.engines.engineAV[0]
                        + " vs " + engineAVBefore);
    }

    @Test
    void engineStallsUnderLockedLoadAndDoesNotSelfRecover() {
        SoftBodyVehicle vehicle = lockedLoadRig();
        vehicle.powertrain.setControls(0.0f, 0.0f); // closed throttle, clutch engaged
        for (int step = 0; step < 3000; step++) {
            vehicle.powertrain.solve(DT);
        }
        assertTrue(vehicle.powertrain.debugEngineRPM() < 100.0f,
                "a locked load must drag the engine below cranking, got "
                        + vehicle.powertrain.debugEngineRPM());
        assertEquals(0.0f, vehicle.powertrain.debugCombustionTorque(), 1e-6f,
                "combustion must be off below cranking speed");
        for (int step = 0; step < 1000; step++) {
            vehicle.powertrain.solve(DT);
        }
        assertTrue(vehicle.powertrain.debugEngineRPM() < 100.0f,
                "a stalled engine must not self-recover without the starter");
        assertTrue(vehicle.powertrain.debugSparkEnabled() && vehicle.powertrain.debugFuelEnabled(),
                "spark/fuel stay enabled below cranking (no limiter involvement)");
        assertTrue(vehicle.powertrain.debugActualThrottle() > 0.0f,
                "the idle controller keeps a restart feedforward after the engine stalls");
        assertEquals(0.0f, vehicle.powertrain.engines.playerThrottle[0], 1e-6f);
        assertTrue(vehicle.powertrain.engines.actualThrottle[0]
                >= vehicle.powertrain.engines.idleLossThrottle[0],
                "physical throttle must include the calculated low-speed loss feedforward");
    }
}
