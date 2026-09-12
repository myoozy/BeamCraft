package me.mzy.beamcraft.client.model;

import com.google.gson.JsonObject;
import me.mzy.beamcraft.client.physics.FlexbodyContainer;
import me.mzy.beamcraft.client.physics.JBeamAssembler;
import me.mzy.beamcraft.client.physics.JBeamLoader;
import me.mzy.beamcraft.client.physics.NodeContainer;
import me.mzy.beamcraft.client.physics.SoftBodyVehicle;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.io.File;
import java.lang.reflect.Method;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Map;
import java.util.zip.ZipFile;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Local diagnostic against the stock ETK800 configuration named in the flexbody bug report. */
class Etk800FlexbodyAuditTest {
    @Test
    void dumpRelevantBindings(@TempDir Path tempDir) throws Exception {
        String corpus = System.getenv("BEAMCRAFT_JBEAM_CORPUS");
        Assumptions.assumeTrue(corpus != null && !corpus.isBlank());
        File root = new File(corpus);

        Map<String, JsonObject> registry = new HashMap<>();
        Map<String, String> config = new HashMap<>();
        JBeamLoader.loadVehicle(root, "etk800", "846x_ttsport_plus_DCT.pc", registry, config);
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        assertTrue(new JBeamAssembler().assembleVehicle("etk800", config, registry, vehicle));

        loadDae(root.toPath().resolve("etk800.zip"), "vehicles/etk800/etk800.dae",
                tempDir.resolve("etk800.dae"), "etk800");
        loadDae(root.toPath().resolve("common.zip"), "vehicles/common/tires/tires3.dae",
                tempDir.resolve("tires3.dae"), "common");
        loadDae(root.toPath().resolve("common.zip"), "vehicles/common/etk/etk_mechanical.dae",
                tempDir.resolve("etk_mechanical.dae"), "common");
        FlexbodyBindingUtil.performBinding(vehicle.flexbodies, vehicle);

        FlexbodyContainer flex = vehicle.flexbodies;
        NodeContainer nodes = vehicle.nodes;
        int vertexOffset = 0;
        for (int mesh = 0; mesh < flex.meshCount; mesh++) {
            String name = flex.meshName[mesh];
            DaeMeshLoader.RawGeometry geometry = name.isEmpty()
                    ? null : DaeMeshLoader.resolveMesh(flex.vehicleNamespace, name);
            if (geometry == null) continue;
            int meshVertexOffset = vertexOffset;
            vertexOffset += geometry.vertexCount;
            if (!(name.contains("tire") || name.contains("caliper") || name.contains("brake"))) continue;
            System.out.println("ETK_FLEX mesh=" + name + " groups=" + flex.targetGroups[mesh]);
            for (String group : flex.targetGroups[mesh]) {
                Integer groupId = flex.groupNameToId.get(group);
                if (groupId == null) continue;
                int start = flex.groupNodeOffsets[groupId];
                int count = flex.groupNodeCounts[groupId];
                StringBuilder names = new StringBuilder();
                for (int i = 0; i < count; i++) {
                    if (i > 0) names.append(',');
                    names.append(nodes.names[flex.flatGroupNodes[start + i]]);
                }
                System.out.println("  " + group + "=" + names);
            }
            int usesWheelHubAxis = 0;
            int usesGeneratedWheel = 0;
            int wheelHubAxisCenters = 0;
            int explicitZVertices = 0;
            for (int vertex = meshVertexOffset; vertex < meshVertexOffset + geometry.vertexCount; vertex++) {
                if (!flex.vUseCrossZ[vertex] && flex.vVzNode[vertex] >= 0) explicitZVertices++;
                if (nodes.names[flex.vCenterNode[vertex]].matches("[rf]w1.*")) wheelHubAxisCenters++;
                int[] bindingNodes = flex.vVzNode[vertex] >= 0
                        ? new int[]{flex.vCenterNode[vertex], flex.vVxNode[vertex], flex.vVyNode[vertex], flex.vVzNode[vertex]}
                        : new int[]{flex.vCenterNode[vertex], flex.vVxNode[vertex], flex.vVyNode[vertex]};
                for (int node : bindingNodes) {
                    String nodeName = nodes.names[node];
                    if (nodeName == null) continue;
                    if (nodeName.matches("[rf]w1.*")) usesWheelHubAxis++;
                    if (nodeName.contains("_hub_") || nodeName.contains("_tire_")) usesGeneratedWheel++;
                }
            }
            System.out.println("  vertices=" + geometry.vertexCount
                    + " wheelHubAxisRefs=" + usesWheelHubAxis
                    + " wheelHubAxisCenters=" + wheelHubAxisCenters
                    + " generatedWheelRefs=" + usesGeneratedWheel
                    + " explicitZ=" + explicitZVertices);
            if (name.contains("caliper")) {
                assertEquals(0, usesWheelHubAxis,
                        name + " must stay on the suspension side when its wheel detaches");
            } else if (name.contains("brakedisc")) {
                assertTrue(usesWheelHubAxis > 0,
                        name + " is a rotating part and must retain its wheel-axis binding");
            } else if (name.startsWith("tire_")) {
                assertEquals(geometry.vertexCount, explicitZVertices,
                        name + " should use BeamNG-style four-node locators");
            }
        }
    }

    private static void loadDae(Path archive, String entryName, Path output, String namespace) throws Exception {
        try (ZipFile zip = new ZipFile(archive.toFile())) {
            var entry = zip.getEntry(entryName);
            try (var input = zip.getInputStream(entry)) {
                Files.copy(input, output);
            }
        }
        Method method = DaeMeshLoader.class.getDeclaredMethod("loadMeshUsingAssimp", String.class, String.class);
        method.setAccessible(true);
        method.invoke(null, output.toString(), namespace);
    }
}
