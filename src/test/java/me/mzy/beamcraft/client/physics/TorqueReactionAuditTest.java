package me.mzy.beamcraft.client.physics;

import com.google.gson.JsonObject;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.util.HashMap;
import java.util.Locale;
import java.util.Map;

/**
 * Diagnostic (not an assertion) test: measures the body pitch moment each
 * torque-reaction path produces for the same driveline torque, so the balance
 * between them can be compared against BeamNG's model.
 *
 * <p>A reaction is a zero-net-force couple, so its moment is the same about every
 * reference point and the differential/normal split below is exact.
 *
 * <p>Run with {@code BEAMCRAFT_JBEAM_CORPUS=<...>/content/vehicles}.
 */
class TorqueReactionAuditTest {

    private static final float TORQUE = 1_000.0f;

    /** BeamNG tongue: `torqueReactionNodes` of each torsionReactor, from the jbeam. */
    private static final String[][] ETK_REACTORS = {
            {"front torsionReactorF", "fx2l", "fx2r", "fx4l"},
            {"rear torsionReactorR", "e3r", "e4r", "e2l"},
    };

    @Test
    void measureReactionPitchContributions() {
        String corpus = System.getenv("BEAMCRAFT_JBEAM_CORPUS");
        Assumptions.assumeTrue(corpus != null && !corpus.isBlank());
        File corpusRoot = new File(corpus);
        Assumptions.assumeTrue(corpusRoot.isDirectory());

        audit(corpusRoot, "etk800", "846x_ttsport_plus_DCT.pc", ETK_REACTORS);
        audit(corpusRoot, "vivace", "vivace_S_410q_DCT.pc", ETK_REACTORS);
    }

    private void audit(File corpusRoot, String model, String pcFile, String[][] reactors) {
        Map<String, JsonObject> registry = new HashMap<>();
        Map<String, String> config = new HashMap<>();
        JBeamLoader.loadVehicle(corpusRoot, model, pcFile, registry, config);
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        if (!new JBeamAssembler().assembleVehicle(model, config, registry, vehicle)) {
            System.out.println("!! assembly failed for " + model);
            return;
        }

        System.out.println();
        System.out.println("================================================================");
        System.out.println("TORQUE REACTION: " + model + "   applied driveline torque = " + TORQUE + " Nm");
        System.out.println("X = lateral (pitch), Y = up (yaw), Z = longitudinal (roll)");
        System.out.println("================================================================");
        System.out.printf(Locale.ROOT, "%-26s %14s %14s %14s%n",
                "path", "M.x pitch", "M.y yaw", "M.z roll");

        for (int w = 0; w < vehicle.wheels.count; w++) {
            final int wheel = w;
            report(vehicle, vehicle.wheels.name[wheel] + " wheel drive reaction",
                    () -> vehicle.wheels.applyDriveReaction(wheel, TORQUE));
            report(vehicle, vehicle.wheels.name[wheel] + " wheel brake reaction",
                    () -> vehicle.wheels.applyBrakeReaction(wheel, TORQUE));
        }

        for (String[] reactor : reactors) {
            int[] idx = new int[reactor.length - 1];
            boolean ok = true;
            for (int i = 1; i < reactor.length; i++) {
                Integer node = vehicle.nodes.nameToIndex.get(reactor[i]);
                if (node == null) { ok = false; break; }
                idx[i - 1] = node;
            }
            if (!ok) { System.out.println("   " + reactor[0] + ": nodes absent"); continue; }
            for (int node : idx) {
                System.out.printf(Locale.ROOT, "      node %-6s pos=(%7.3f %7.3f %7.3f)%n",
                        vehicle.nodes.names[node], vehicle.nodes.posX[node],
                        vehicle.nodes.posY[node], vehicle.nodes.posZ[node]);
            }
            int n1 = idx[0], n2 = idx[1];
            double dx = vehicle.nodes.posX[n2] - vehicle.nodes.posX[n1];
            double dy = vehicle.nodes.posY[n2] - vehicle.nodes.posY[n1];
            double dz = vehicle.nodes.posZ[n2] - vehicle.nodes.posZ[n1];
            double len = Math.sqrt(dx * dx + dy * dy + dz * dz);
            if (len < 1.0e-9) { System.out.println("   " + reactor[0] + ": degenerate axis"); continue; }
            // Mirrors PowertrainSystem.applyReactionTorque: the torque runs along n1 -> n2.
            float tx = (float) (TORQUE * dx / len);
            float ty = (float) (TORQUE * dy / len);
            float tz = (float) (TORQUE * dz / len);
            report(vehicle, reactor[0] + " reactor", () -> TorqueReactionSolver.apply(
                    vehicle.nodes, idx, 0, idx.length, tx, ty, tz));
        }
    }

    /** Runs one reaction application and prints the couple it produced. */
    private static void report(SoftBodyVehicle vehicle, String label, Runnable apply) {
        NodeContainer nodes = vehicle.nodes;
        for (int i = 0; i < nodes.count; i++) {
            nodes.forceX[i] = 0.0f;
            nodes.forceY[i] = 0.0f;
            nodes.forceZ[i] = 0.0f;
        }
        apply.run();

        double mx = 0.0, my = 0.0, mz = 0.0, sumX = 0.0, sumY = 0.0, sumZ = 0.0;
        for (int i = 0; i < nodes.count; i++) {
            double fx = nodes.forceX[i], fy = nodes.forceY[i], fz = nodes.forceZ[i];
            if (fx == 0.0 && fy == 0.0 && fz == 0.0) continue;
            double px = nodes.posX[i], py = nodes.posY[i], pz = nodes.posZ[i];
            mx += py * fz - pz * fy;
            my += pz * fx - px * fz;
            mz += px * fy - py * fx;
            sumX += fx; sumY += fy; sumZ += fz;
        }
        System.out.printf(Locale.ROOT, "%-26s %14.1f %14.1f %14.1f   net force (%6.1f %6.1f %6.1f)%n",
                label, mx, my, mz, sumX, sumY, sumZ);
    }
}
