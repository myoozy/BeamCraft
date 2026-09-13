package me.mzy.beamcraft.client.physics;

import com.google.gson.JsonObject;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.TreeMap;
import java.util.Locale;

/**
 * Diagnostic (not an assertion) test: dumps the physics the wheel generator
 * actually produced for the stock wheels, grouped by beam family, so the
 * generated topology and stiffnesses can be compared against the authored
 * BeamNG data.
 *
 * <p>Run with {@code BEAMCRAFT_JBEAM_CORPUS=<...>/content/vehicles}.
 */
class WheelTopologyAuditTest {

    @Test
    void dumpGeneratedWheelTopology() {
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
        WheelContainer w = vehicle.wheels;

        System.out.println();
        System.out.println("================================================================");
        System.out.println("WHEEL TOPOLOGY: " + model + "   wheels=" + w.count
                + "  nodes=" + vehicle.nodes.count
                + "  normal=" + vehicle.normalBeams.count
                + "  aniso=" + vehicle.anisotropicBeams.count
                + "  lBeams=" + vehicle.lBeams.count
                + "  triangles=" + vehicle.triangles.count);
        System.out.println("================================================================");

        for (int wi = 0; wi < w.count; wi++) {
            System.out.printf(Locale.ROOT, "%n-- wheel %d  name=%s  numRays=%d  radius=%.4f  dir=%d%n",
                    wi, w.name[wi], w.numRays[wi], w.hubRadius[wi], w.wheelDir[wi]);
            printRing(vehicle, w, wi, "hub_in", w.hubInnerNodes);
            printRing(vehicle, w, wi, "hub_out", w.hubOuterNodes);
            printRing(vehicle, w, wi, "tire_in", w.tireInnerNodes);
            printRing(vehicle, w, wi, "tire_out", w.tireOuterNodes);
        }

        System.out.println();
        System.out.println("-- every beam touching one hub node of wheel 0 (expected hub topology) --");
        for (String probe : new String[]{"RR_hub_in_0", "FR_hub_in_0", "FR_tire_in_0"}) {
            Integer idx = vehicle.nodes.nameToIndex.get(probe);
            if (idx == null) { System.out.println("   " + probe + ": absent"); continue; }
            System.out.println("   " + probe + " (mass " + vehicle.nodes.mass[idx] + "):");
            probeNode(vehicle, vehicle.normalBeams, idx, "normal");
            probeNode(vehicle, vehicle.anisotropicBeams, idx, "aniso ");
            probeNode(vehicle, vehicle.lBeams, idx, "lbeam ");
        }

        System.out.println();
        System.out.println("-- generated beam families (grouped by coefficient tuple) --");
        Map<Integer, Boolean> generated = generatedNodes(vehicle, w);
        Map<String, Integer> families = new TreeMap<>();
        Map<String, String> samples = new LinkedHashMap<>();
        countFamily(vehicle, vehicle.normalBeams, generated, "normal", families, samples);
        countFamily(vehicle, vehicle.anisotropicBeams, generated, "anisotropic", families, samples);
        countFamily(vehicle, vehicle.lBeams, generated, "lbeam", families, samples);
        countFamily(vehicle, vehicle.boundedBeams, generated, "bounded", families, samples);
        for (Map.Entry<String, Integer> e : families.entrySet()) {
            System.out.printf(Locale.ROOT, "%8d  %-110s  %s%n",
                    e.getValue(), e.getKey(), samples.get(e.getKey()));
        }
    }

    private static void printRing(SoftBodyVehicle v, WheelContainer w, int wi, String label, int[] nodes) {
        int base = wi * WheelContainer.MAX_RAYS;
        int rays = w.numRays[wi];
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < rays; i++) {
            int idx = nodes[base + i];
            if (idx < 0) { sb.append(" [-1]"); continue; }
            sb.append(String.format(Locale.ROOT, " [%s m=%.3f]", v.nodes.names[idx], v.nodes.mass[idx]));
        }
        System.out.println("   " + label + ":" + sb);
    }

    private static Map<Integer, Boolean> generatedNodes(SoftBodyVehicle v, WheelContainer w) {
        Map<Integer, Boolean> set = new LinkedHashMap<>();
        int[] rings = {};
        for (int wi = 0; wi < w.count; wi++) {
            int base = wi * WheelContainer.MAX_RAYS;
            for (int i = 0; i < w.numRays[wi]; i++) {
                for (int[] ring : new int[][]{w.hubInnerNodes, w.hubOuterNodes,
                        w.tireInnerNodes, w.tireOuterNodes}) {
                    int idx = ring[base + i];
                    if (idx >= 0) set.put(idx, Boolean.TRUE);
                }
            }
        }
        return set;
    }

    private static void probeNode(SoftBodyVehicle v, BeamContainer beams, int node, String kind) {
        for (int i = 0; i < beams.count; i++) {
            if (beams.node1[i] != node && beams.node2[i] != node) continue;
            int other = beams.node1[i] == node ? beams.node2[i] : beams.node1[i];
            System.out.printf(Locale.ROOT, "      %s %-16s -> %-16s spring=%9.1f damp=%6.1f%n",
                    kind, v.nodes.names[node], v.nodes.names[other], beams.spring[i], beams.damp[i]);
        }
    }

    private static void countFamily(SoftBodyVehicle v, BeamContainer beams, Map<Integer, Boolean> generated,
                                    String kind, Map<String, Integer> out, Map<String, String> samples) {
        for (int i = 0; i < beams.count; i++) {
            if (!generated.containsKey(beams.node1[i]) && !generated.containsKey(beams.node2[i])) continue;
            String key = String.format(Locale.ROOT, "%-12s spring=%.1f damp=%.1f deform=%.0f strength=%.0f",
                    kind, beams.spring[i], beams.damp[i], beams.deform[i], beams.strength[i]);
            if (beams instanceof AnisotropicBeamContainer a) {
                key += String.format(Locale.ROOT, " expSpring=%.1f expDamp=%.1f tz=%.3f",
                        a.springExpansion[i], a.dampExpansion[i], a.transitionZone[i]);
            }
            out.merge(key, 1, Integer::sum);
            samples.putIfAbsent(key, v.nodes.names[beams.node1[i]] + " -> " + v.nodes.names[beams.node2[i]]);
        }
    }
}
