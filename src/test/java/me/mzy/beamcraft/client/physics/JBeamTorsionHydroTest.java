package me.mzy.beamcraft.client.physics;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class JBeamTorsionHydroTest {
    @Test
    void parsesEtkiStyleTorsionHydrosAsDrivenTorsionBars() {
        SoftBodyVehicle vehicle = vehicleWithTorsionNodes();
        JsonArray section = JsonParser.parseString("""
                [
                  ["id1:", "id2:", "id3:", "id4:"],
                  {"spring":340000, "damp":5, "deform":1000000000, "strength":10000},
                  ["a", "b", "c", "d", {"factor":0.83, "steeringWheelLock":550,
                                                "inRate":5, "outRate":5,
                                                "inputSource":"steering_input"}],
                  ["e", "b", "c", "f", {"factor":-0.57, "inputSource":"door"}]
                ]
                """).getAsJsonArray();
        JBeamAssembler.PartEntry part = new JBeamAssembler.PartEntry(
                new JsonObject(), 0, "test", null, Map.of());

        JBeamParser.parseTorsionHydros(section, vehicle, part);

        assertEquals(2, vehicle.torsionbars.count);
        assertEquals(2, vehicle.torsionHydros.count);
        assertEquals(340000.0f, vehicle.torsionbars.spring[0]);
        assertEquals(5.0f, vehicle.torsionbars.damp[0]);
        assertEquals(10000.0f, vehicle.torsionbars.strength[0]);

        HydroActuatorController controls = vehicle.torsionHydros.controls;
        assertEquals(-0.83f, controls.inLimit[0], 1.0e-6f);
        assertEquals(0.83f, controls.outLimit[0], 1.0e-6f);
        assertEquals(550.0f, controls.steeringWheelLock[0]);
        assertEquals("steering_input", controls.inputSource[0]);
        assertEquals(-1.0f, controls.inputFactor[1]);
        assertEquals("door", controls.inputSource[1]);
        assertEquals(vehicle.electrics.signalId("door"), controls.inputSignalId[1]);
    }

    @Test
    void electricInputRateLimitsTheTorsionBarTargetAngle() {
        SoftBodyVehicle vehicle = vehicleWithTorsionNodes();
        vehicle.addTorsionHydro(torsionHydro(0.83f, 5.0f));

        vehicle.electrics.set("steering_input", 1.0f);
        vehicle.torsionHydros.update(0.1f, vehicle.torsionbars, vehicle.electrics.snapshot());

        assertEquals(0.83f, vehicle.torsionHydros.controls.command[0], 1.0e-6f);
        assertEquals(0.5f, vehicle.torsionHydros.controls.state[0], 1.0e-6f);
        assertEquals(0.5f, vehicle.torsionbars.actuationAngle[0], 1.0e-6f);

        vehicle.electrics.set("steering_input", 0.0f);
        vehicle.torsionHydros.update(0.05f, vehicle.torsionbars, vehicle.electrics.snapshot());
        assertEquals(0.25f, vehicle.torsionbars.actuationAngle[0], 1.0e-6f,
                "autoCenterRate should drive the target angle back toward neutral");

        vehicle.reset();
        assertEquals(0.0f, vehicle.torsionbars.actuationAngle[0], 1.0e-6f);
    }

    @Test
    void torsionSolverUsesTheActuatedTargetAngle() {
        SoftBodyVehicle vehicle = vehicleWithTorsionNodes();
        vehicle.addTorsionHydro(torsionHydro(0.25f, 10.0f));
        vehicle.electrics.set("steering_input", 1.0f);

        vehicle.solveInternalForces(0.01f, 0.0f, vehicle.electrics.snapshot());

        double totalForce = 0.0;
        for (int i = 0; i < vehicle.nodes.count; i++) {
            totalForce += Math.abs(vehicle.nodes.forceX[i]);
            totalForce += Math.abs(vehicle.nodes.forceY[i]);
            totalForce += Math.abs(vehicle.nodes.forceZ[i]);
        }
        assertTrue(totalForce > 0.0, "changing the target angle must generate torsion forces");
    }

    @Test
    void parsesPrecompressedOneSidedTorsionLimiterWithoutLockingItsNeutralSide() {
        SoftBodyVehicle vehicle = vehicleWithTorsionNodes();
        JsonArray section = JsonParser.parseString("""
                [
                  ["id1:", "id2:", "id3:", "id4:"],
                  {"deform":25000, "strength":25000},
                  ["a", "b", "c", "d", {"precompressionAngle":1.03,
                                                "precompressionTime":0.05,
                                                "spring":290000, "damp":0,
                                                "spring2":0, "damp2":0}]
                ]
                """).getAsJsonArray();
        JBeamAssembler.PartEntry part = new JBeamAssembler.PartEntry(
                new JsonObject(), 0, "test", null, Map.of());

        JBeamParser.parseTorsionbars(section, vehicle, part);

        assertEquals(290000.0f, vehicle.torsionbars.spring[0]);
        assertEquals(0.0f, vehicle.torsionbars.spring2[0]);
        assertEquals(1.03f, vehicle.torsionbars.precompressionAngle[0]);
        assertEquals(0.05f, vehicle.torsionbars.precompressionTime[0]);

        vehicle.solveInternalForces(0.025f, 0.0f, vehicle.electrics.snapshot());
        assertEquals(0.515f, vehicle.torsionbars.precompressionState[0], 1.0e-6f);
        for (int i = 0; i < vehicle.nodes.count; i++) {
            assertEquals(0.0f, vehicle.nodes.forceX[i], 1.0e-5f);
            assertEquals(0.0f, vehicle.nodes.forceZ[i], 1.0e-5f);
        }
    }

    private static SoftBodyVehicle vehicleWithTorsionNodes() {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        vehicle.addNode(node("a", 0, 1, 0));
        vehicle.addNode(node("b", 0, 0, 0));
        vehicle.addNode(node("c", 1, 0, 0));
        vehicle.addNode(node("d", 1, 0, 1));
        vehicle.addNode(node("e", 0, -1, 0));
        vehicle.addNode(node("f", 1, 0, -1));
        return vehicle;
    }

    private static PhysicsSpecs.NodeSpec node(String name, float x, float y, float z) {
        return new PhysicsSpecs.NodeSpec(name, x, y, z, 1, 1, 1,
                0, true, false, List.of());
    }

    private static PhysicsSpecs.TorsionHydroSpec torsionHydro(float factor, float rate) {
        float extent = Math.abs(factor);
        PhysicsSpecs.TorsionBarSpec torsionBar = new PhysicsSpecs.TorsionBarSpec(
                "a", "b", "c", "d", 1000, 10, 1000, 10,
                Float.MAX_VALUE, Float.MAX_VALUE, 0.0f, 0.0f);
        return new PhysicsSpecs.TorsionHydroSpec(
                torsionBar, "steering_input", -extent, extent,
                factor < 0 ? -1.0f : 1.0f, 0.0f, -1.0f, 1.0f,
                rate, rate, rate, null);
    }
}
