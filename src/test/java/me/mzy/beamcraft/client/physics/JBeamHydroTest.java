package me.mzy.beamcraft.client.physics;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import me.mzy.beamcraft.client.physics.electrics.ElectricBus;
import me.mzy.beamcraft.client.physics.electrics.ElectricSnapshot;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;

class JBeamHydroTest {
    @Test
    void parsesDocumentedHydroPropertiesAndInheritedBeamProperties() {
        SoftBodyVehicle vehicle = vehicleWithThreeNodes();
        JsonArray section = JsonParser.parseString("""
                [
                  ["id1:", "id2:"],
                  {"beamSpring": 8001000, "beamDamp": 50},
                  ["a", "b", {"factor": 0.14, "inLimit": 0.1, "outLimit": 9, "inputFactor": 3,
                                "steeringWheelLock": 510, "inRate": 1.25}],
                  ["b", "c", {"factor": -0.20, "inputSource": "door", "outRate": 0.5}],
                  ["a", "c", {"factor": null, "inLimit": 0.4, "outLimit": 1.8, "inputFactor": 0.5,
                                "inputCenter": 0.1, "inputInLimit": -0.8, "inputOutLimit": 0.9,
                                "autoCenterRate": 0.3}]
                ]
                """).getAsJsonArray();
        JBeamAssembler.PartEntry part = new JBeamAssembler.PartEntry(
                new JsonObject(), 0, "test", null, Map.of());

        JBeamParser.parseHydros(section, vehicle, part);

        assertEquals(3, vehicle.normalBeams.count);
        assertEquals(3, vehicle.hydros.count);
        assertEquals(8001000.0f, vehicle.normalBeams.spring[0]);
        assertEquals(50.0f, vehicle.normalBeams.damp[1]);
        assertEquals(0.86f, vehicle.hydros.inLimit[0], 1.0e-6f);
        assertEquals(1.14f, vehicle.hydros.outLimit[0], 1.0e-6f);
        assertEquals(1.0f, vehicle.hydros.inputFactor[0]);
        assertEquals(510.0f, vehicle.hydros.steeringWheelLock[0]);
        assertEquals(1.25f, vehicle.hydros.inRate[0]);
        assertEquals(1.25f, vehicle.hydros.outRate[0]);
        assertEquals("door", vehicle.hydros.inputSource[1]);
        assertEquals(vehicle.electrics.signalId("door"), vehicle.hydros.inputSignalId[1]);
        assertEquals(-1.0f, vehicle.hydros.inputFactor[1]);
        assertEquals(0.5f, vehicle.hydros.outRate[1]);
        assertEquals(0.4f, vehicle.hydros.inLimit[2]);
        assertEquals(1.8f, vehicle.hydros.outLimit[2]);
        assertEquals(0.5f, vehicle.hydros.inputFactor[2]);
        assertEquals(0.05f, vehicle.hydros.inputCenter[2]);
        assertEquals(-0.4f, vehicle.hydros.inputInLimit[2]);
        assertEquals(0.45f, vehicle.hydros.inputOutLimit[2]);
        assertEquals(0.3f, vehicle.hydros.autoCenterRate[2]);
    }

    @Test
    void appliesDocumentedDefaultsWithoutInventingSteeringControl() {
        PhysicsSpecs.HydroSpec spec = JBeamParser.buildHydroSpec(
                beamSpec(), new JsonObject(), Map.of());

        assertEquals("steering_input", spec.inputSource());
        assertEquals(1.0f, spec.inLimit());
        assertEquals(2.0f, spec.outLimit());
        assertEquals(2.0f, spec.inRate());
        assertEquals(2.0f, spec.outRate());
        assertEquals(2.0f, spec.autoCenterRate());
        assertNull(spec.steeringWheelLock());
    }

    @Test
    void factorControlsDirectionAndRatesLimitActuatorMotion() {
        BeamContainer beams = new BeamContainer();
        ElectricBus electrics = new ElectricBus();
        int positiveBeam = beams.addBeam(beamSpec(), 0, 1, 2.0f);
        int negativeBeam = beams.addBeam(beamSpec(), 0, 1, 2.0f);
        HydroContainer hydros = new HydroContainer();
        hydros.addHydro(hydroSpec(0.5f, 0.25f, 0.5f, 0.1f), positiveBeam, beams, electrics);
        hydros.addHydro(hydroSpec(-0.5f, 0.25f, 0.5f, 0.1f), negativeBeam, beams, electrics);

        electrics.set("steering_input", 1.0f);
        hydros.update(1.0f, beams, electrics.snapshot());

        assertEquals(1.5f, hydros.command[0]);
        assertEquals(1.5f, hydros.state[0]);
        assertEquals(0.5f, hydros.command[1]);
        assertEquals(0.75f, hydros.state[1]);
        assertEquals(3.0f, beams.effectiveRestLength(positiveBeam));
        assertEquals(1.5f, beams.effectiveRestLength(negativeBeam));
        assertEquals(2.0f, beams.restLength[positiveBeam],
                "actuation must not overwrite the beam's neutral/deformed rest length");

        electrics.set("steering_input", 0.0f);
        hydros.update(1.0f, beams, electrics.snapshot());
        assertEquals(1.4f, hydros.state[0], 1.0e-6f,
                "autoCenterRate must be used while returning to the configured center");
        assertEquals(0.85f, hydros.state[1], 1.0e-6f);
    }

    @Test
    void brokenHydroStopsUpdatingAndResetRestoresItsCenter() {
        BeamContainer beams = new BeamContainer();
        ElectricBus electrics = new ElectricBus();
        int beam = beams.addBeam(beamSpec(), 0, 1, 2.0f);
        HydroContainer hydros = new HydroContainer();
        hydros.addHydro(hydroSpec(0.5f, 1.0f, 1.0f, 1.0f), beam, beams, electrics);
        beams.broken[beam] = true;

        electrics.set("steering_input", 1.0f);
        hydros.update(1.0f, beams, electrics.snapshot());
        assertEquals(1.0f, hydros.state[0]);

        beams.reset();
        hydros.reset(beams);
        assertFalse(beams.broken[beam]);
        assertEquals(1.0f, beams.actuationRatio[beam]);
    }

    @Test
    void hydroUsesTheFrozenSnapshotForOneElectricUpdateBlock() {
        BeamContainer beams = new BeamContainer();
        ElectricBus electrics = new ElectricBus();
        int beam = beams.addBeam(beamSpec(), 0, 1, 2.0f);
        HydroContainer hydros = new HydroContainer();
        hydros.addHydro(hydroSpec(0.5f, 1.0f, 1.0f, 1.0f), beam, beams, electrics);

        electrics.set("steering_input", 1.0);
        ElectricSnapshot preparedSnapshot = electrics.snapshot();
        electrics.set("steering_input", -1.0);

        hydros.update(1.0f, beams, preparedSnapshot);

        assertEquals(1.5f, hydros.state[0]);
        assertEquals(-1.0, electrics.get("steering_input"));
    }

    private static SoftBodyVehicle vehicleWithThreeNodes() {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        vehicle.addNode(node("a", 0.0f));
        vehicle.addNode(node("b", 1.0f));
        vehicle.addNode(node("c", 2.0f));
        return vehicle;
    }

    private static PhysicsSpecs.NodeSpec node(String name, float x) {
        return new PhysicsSpecs.NodeSpec(name, x, 0, 0, 1, 1, 1,
                0, true, false, List.of());
    }

    private static PhysicsSpecs.HydroSpec hydroSpec(float factor, float inRate,
                                                     float outRate, float autoCenterRate) {
        float extent = Math.abs(factor);
        return new PhysicsSpecs.HydroSpec(
                beamSpec(), "steering_input", 1.0f - extent, 1.0f + extent,
                factor < 0 ? -1.0f : 1.0f, 0.0f, -1.0f, 1.0f,
                inRate, outRate, autoCenterRate, null);
    }

    private static PhysicsSpecs.BeamSpec beamSpec() {
        return new PhysicsSpecs.BeamSpec(
                BeamContainer.BEAM_HYDRO, "a", "b", null,
                List.of(), 0, false,
                1000, 10, Float.MAX_VALUE, Float.MAX_VALUE,
                1, 0, 0,
                1, 1, -1, -1,
                1000, 10, -1, -1, -1, -1,
                1000, 10, 0, Float.MAX_VALUE
        );
    }
}
