package me.mzy.beamcraft.client.physics;

import com.google.gson.JsonObject;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.util.HashMap;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Opt-in regression test against an installed BeamNG vehicle corpus. */
class EtkiSteeringCorpusTest {
    @Test
    void stock2400ixSteeringBoxSurvivesAFullLeftCommand() {
        String corpus = System.getenv("BEAMCRAFT_JBEAM_CORPUS");
        Assumptions.assumeTrue(corpus != null && !corpus.isBlank());
        File corpusRoot = new File(corpus);
        Assumptions.assumeTrue(corpusRoot.isDirectory());

        Map<String, JsonObject> registry = new HashMap<>();
        Map<String, String> config = new HashMap<>();
        JBeamLoader.loadVehicle(corpusRoot, "etki", "2400ix_M.pc", registry, config);
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        assertTrue(new JBeamAssembler().assembleVehicle("etki", config, registry, vehicle));
        SoftBodyVehicle neutralVehicle = new SoftBodyVehicle(null);
        assertTrue(new JBeamAssembler().assembleVehicle("etki", config, registry, neutralVehicle));
        int steeringHydroCount = 0;
        for (int i = 0; i < vehicle.torsionHydros.count; i++) {
            if ("steering_input".equals(vehicle.torsionHydros.controls.inputSource[i])) {
                steeringHydroCount++;
            }
        }
        assertEquals(2, steeringHydroCount);

        vehicle.electrics.set("steering_input", -1.0f);
        for (int i = 0; i < 2000; i++) {
            vehicle.solveInternalForces(0.0005f, 1.0f, vehicle.electrics.snapshot());
            neutralVehicle.solveInternalForces(0.0005f, 1.0f, neutralVehicle.electrics.snapshot());
        }

        for (int i = 0; i < vehicle.torsionHydros.count; i++) {
            if (!"steering_input".equals(vehicle.torsionHydros.controls.inputSource[i])) {
                continue;
            }
            int torsionBar = vehicle.torsionHydros.torsionBarIndex[i];
            assertFalse(vehicle.torsionbars.broken[torsionBar],
                    "steering torsion hydro " + i + " broke under an ordinary command");
        }
        double rightSteering = angleDelta(
                axleYaw(vehicle, "fw1r", "fw1rr"),
                axleYaw(neutralVehicle, "fw1r", "fw1rr"));
        double leftSteering = angleDelta(
                axleYaw(vehicle, "fw1l", "fw1ll"),
                axleYaw(neutralVehicle, "fw1l", "fw1ll"));
        assertTrue(Math.abs(Math.toDegrees(rightSteering)) > 1.0);
        assertTrue(Math.abs(Math.toDegrees(leftSteering)) > 1.0);
        assertTrue(rightSteering * leftSteering > 0.0,
                "both front wheels must steer in the same direction");
    }

    private static double axleYaw(SoftBodyVehicle vehicle, String inner, String outer) {
        int a = vehicle.nodes.nameToIndex.get(inner);
        int b = vehicle.nodes.nameToIndex.get(outer);
        return Math.atan2(vehicle.nodes.posY[b] - vehicle.nodes.posY[a],
                vehicle.nodes.posX[b] - vehicle.nodes.posX[a]);
    }

    private static double angleDelta(double angle, double reference) {
        double delta = angle - reference;
        while (delta > Math.PI) delta -= Math.PI * 2.0;
        while (delta < -Math.PI) delta += Math.PI * 2.0;
        return delta;
    }
}
