package me.mzy.beamcraft.client.physics;

import com.google.gson.JsonObject;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Diagnostic (not an assertion) test: shows how the {@link DirectionalStabilityLimiter}
 * spends each suspension node's directional budget, so it is visible which beams
 * starve the dampers.
 *
 * <p>Run with {@code BEAMCRAFT_JBEAM_CORPUS=<...>/content/vehicles}.
 */
class StabilityBudgetAuditTest {

    private static final String[] ETK_NODES = {
            "fh1r", "fs1r", "fh4r", "rh1r", "r1rr",
            "FR_hub_in_0", "FR_hub_out_0", "FR_tire_in_0", "FR_tire_out_0"};
    private static final String[] VIVACE_NODES = {
            "fh1r", "fs1r", "fh4r", "rh3r", "r3rr", "rh5r", "r1rr",
            "FR_hub_in_0", "FR_hub_out_0", "FR_tire_in_0", "FR_tire_out_0"};

    @Test
    void dumpPerNodeStabilityBudgetPressure() {
        String corpus = System.getenv("BEAMCRAFT_JBEAM_CORPUS");
        Assumptions.assumeTrue(corpus != null && !corpus.isBlank());
        File corpusRoot = new File(corpus);
        Assumptions.assumeTrue(corpusRoot.isDirectory());

        audit(corpusRoot, "etk800", "846x_ttsport_plus_DCT.pc", ETK_NODES);
        audit(corpusRoot, "vivace", "vivace_S_410q_DCT.pc", VIVACE_NODES);
    }

    private void audit(File corpusRoot, String model, String pcFile, String[] nodeNames) {
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
        System.out.println("STABILITY BUDGET PRESSURE: " + model);
        System.out.println("normalBeams=" + vehicle.normalBeams.count
                + " supportBeams=" + vehicle.supportBeams.count
                + " boundedBeams=" + vehicle.boundedBeams.count
                + " lBeams=" + vehicle.lBeams.count
                + " | bounded constraint offset=" + vehicle.boundedConstraintOffset());
        System.out.println("================================================================");

        int boundedOffset = vehicle.boundedConstraintOffset();
        int boundedCount = vehicle.boundedBeams.count;

        for (String nodeName : nodeNames) {
            Integer nodeIndex = vehicle.nodes.nameToIndex.get(nodeName);
            if (nodeIndex == null) {
                System.out.println("-- node " + nodeName + ": not present --");
                continue;
            }
            List<DirectionalStabilityLimiter.NodePressure> pressures =
                    vehicle.debugBudgetPressureAt(nodeIndex, PhysicsWorld.invPhysicsDT, 0.90f);
            if (pressures.isEmpty()) {
                System.out.println("-- node " + nodeName + ": no registered constraints --");
                continue;
            }
            DirectionalStabilityLimiter.NodePressure first = pressures.get(0);
            System.out.printf(Locale.ROOT,
                    "%n-- node %-6s mass=%6.2f  budget=%.3e  largestMode=%.3e  pressure=%.2fx --%n",
                    nodeName, vehicle.nodes.mass[nodeIndex], first.budget(), first.largestMode(),
                    first.pressureRatio());
            System.out.printf(Locale.ROOT, "%6s %-16s %12s %8s %12s %8s%n",
                    "id", "beam", "q", "scale", "contribution", "share");
            for (DirectionalStabilityLimiter.NodePressure p : pressures) {
                int id = p.constraintId();
                String label;
                if (id >= boundedOffset && id < boundedOffset + boundedCount) {
                    String name = vehicle.boundedBeams.name[id - boundedOffset];
                    label = "bounded:" + (name == null ? "#" + (id - boundedOffset) : name);
                } else {
                    label = "constraint#" + id;
                }
                if (p.contributionShare() < 0.001) break;
                System.out.printf(Locale.ROOT, "%6d %-16s %12.3e %8.4f %12.3e %7.1f%%%n",
                        id, label, p.q(), p.scale(), p.contribution(),
                        100.0 * p.contributionShare());
            }
        }
    }
}
