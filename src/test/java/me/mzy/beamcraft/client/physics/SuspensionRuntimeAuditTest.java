package me.mzy.beamcraft.client.physics;

import com.google.gson.JsonObject;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.util.HashMap;
import java.util.Locale;
import java.util.Map;

/**
 * Diagnostic (not an assertion) test: prints the suspension stiffness and damping
 * both cars actually run with after assembly, next to the values the jbeam data
 * authored, so any reduction the stability limiter applied is visible.
 *
 * <p>Run with {@code BEAMCRAFT_JBEAM_CORPUS=<...>/content/vehicles}.
 */
class SuspensionRuntimeAuditTest {

    @Test
    void dumpRuntimeSuspensionCoefficients() {
        String corpus = System.getenv("BEAMCRAFT_JBEAM_CORPUS");
        Assumptions.assumeTrue(corpus != null && !corpus.isBlank());
        File corpusRoot = new File(corpus);
        Assumptions.assumeTrue(corpusRoot.isDirectory());

        audit(corpusRoot, "etk800", "846x_ttsport_plus_DCT.pc");
        audit(corpusRoot, "vivace", "vivace_S_410q_DCT.pc");
    }

    private void audit(File corpusRoot, String model, String pcFile) {
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
        System.out.println("SUSPENSION RUNTIME: " + model
                + "   nodes=" + vehicle.nodes.count
                + "  normal=" + vehicle.normalBeams.count
                + "  bounded=" + vehicle.boundedBeams.count
                + "  aniso=" + vehicle.anisotropicBeams.count);
        System.out.println("================================================================");

        System.out.println();
        System.out.println("-- springs (normal beams named spring_*) --");
        System.out.printf(Locale.ROOT, "%-12s %12s %12s %10s %10s %10s%n",
                "name", "spring", "damp", "restLen", "spawnLen", "precomp");
        BeamContainer n = vehicle.normalBeams;
        for (int i = 0; i < n.count; i++) {
            if (n.name[i] == null || !n.name[i].startsWith("spring_")) continue;
            float spawn = spawnLength(vehicle, n.node1[i], n.node2[i]);
            System.out.printf(Locale.ROOT, "%-12s %12.1f %12.1f %10.4f %10.4f %10.4f%n",
                    n.name[i], n.spring[i], n.damp[i], n.restLength[i], spawn,
                    n.restLength[i] - spawn);
        }

        System.out.println();
        System.out.println("-- shock channels: authored vs runtime --");
        System.out.printf(Locale.ROOT, "%-10s %9s %9s %9s %9s %9s %9s %11s%n",
                "name", "a.bump", "r.bump", "a.fast", "r.fast", "a.reb", "r.reb", "ceiling");
        BoundedBeamContainer b = vehicle.boundedBeams;
        for (int i = 0; i < b.count; i++) {
            if (b.name[i] == null || !b.name[i].startsWith("shock_")) continue;
            System.out.printf(Locale.ROOT, "%-10s %9.1f %9.1f %9.1f %9.1f %9.1f %9.1f %11.1f%n",
                    b.name[i], b.authoredDamp[i], b.damp[i],
                    b.authoredDampFast[i], b.dampFast[i],
                    b.authoredDampRebound[i], b.dampRebound[i],
                    b.dampStabilityCeiling[i]);
        }

        System.out.println();
        System.out.println("-- shock force-velocity curve actually produced (two-slope) --");
        System.out.printf(Locale.ROOT, "%-10s %9s %9s %9s %9s%n",
                "name", "v=0.25", "v=0.5", "v=1.0", "v=2.0");
        for (int i = 0; i < b.count; i++) {
            if (b.name[i] == null || !b.name[i].startsWith("shock_")) continue;
            // Compression and rebound both reported as magnitudes at the same speeds,
            // using whichever channel each sign selects.
            System.out.printf(Locale.ROOT, "%-10s", b.name[i]);
            for (float v : new float[]{0.25f, 0.5f, 1.0f, 2.0f}) {
                System.out.printf(Locale.ROOT, " %9.0f", force(b, i, v));
            }
            System.out.println();
        }

        System.out.println();
        System.out.println("-- normal beams touching the spring's two nodes (fh4r and fs1r) --");
        System.out.println("   etk800 authors 30000 front / 34000 rear; vivace 44000 front / 75000 rear");
        System.out.printf(Locale.ROOT, "%-8s %-8s %12s %10s %10s%n",
                "node1", "node2", "spring", "damp", "cutoffHz");
        for (String nodeName : new String[]{"fh4r", "fs1r", "rh5r"}) {
            Integer idx = vehicle.nodes.nameToIndex.get(nodeName);
            if (idx == null) continue;
            for (int i = 0; i < n.count; i++) {
                if (n.node1[i] != idx && n.node2[i] != idx) continue;
                System.out.printf(Locale.ROOT, "%-8s %-8s %12.1f %10.1f %10.1f%n",
                        vehicle.nodes.names[n.node1[i]], vehicle.nodes.names[n.node2[i]],
                        n.spring[i], n.damp[i], n.dampCutoffHz[i]);
            }
        }
    }

    /** Magnitude of the assembled two-slope damper force at a shaft speed. */
    private static float force(BoundedBeamContainer b, int i, float speed) {
        float split = b.dampVelocitySplit[i];
        float slow = b.dampRebound[i];
        float fast = b.dampReboundFast[i];
        if (speed <= split) return slow * speed;
        return slow * split + fast * (speed - split);
    }

    private static float spawnLength(SoftBodyVehicle v, int a, int b) {
        float dx = v.nodes.baseX[b] - v.nodes.baseX[a];
        float dy = v.nodes.baseY[b] - v.nodes.baseY[a];
        float dz = v.nodes.baseZ[b] - v.nodes.baseZ[a];
        return (float) Math.sqrt(dx * dx + dy * dy + dz * dz);
    }
}
