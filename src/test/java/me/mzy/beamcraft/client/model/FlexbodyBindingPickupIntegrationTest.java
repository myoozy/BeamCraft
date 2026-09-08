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

import java.lang.reflect.Method;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.zip.ZipFile;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Local-only regression check against the developer's real BeamNG pickup assets. */
class FlexbodyBindingPickupIntegrationTest {

    @Test
    void d15FrontHubsAvoidPoorlyConditionedBindings(@TempDir Path tempDir) throws Exception {
        String configuredDir = System.getenv("BEAMCRAFT_VEHICLE_DIR");
        Assumptions.assumeTrue(configuredDir != null, "BEAMCRAFT_VEHICLE_DIR is not set");
        Path vehicles = Path.of(configuredDir);
        Assumptions.assumeTrue(Files.isRegularFile(vehicles.resolve("pickup.zip"))
                && Files.isRegularFile(vehicles.resolve("common.zip")), "BeamNG pickup assets are unavailable");

        Map<String, JsonObject> registry = new HashMap<>();
        Map<String, String> config = new HashMap<>();
        JBeamLoader.loadVehicle(List.of(vehicles.toFile()), "pickup", "d15_4wd_A.pc", registry, config);
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        assertTrue(new JBeamAssembler().assembleVehicle("pickup", config, registry, vehicle));

        loadDae(vehicles.resolve("pickup.zip"), "vehicles/pickup/pickup.dae",
                tempDir.resolve("pickup.dae"), "pickup");
        loadDae(vehicles.resolve("common.zip"), "vehicles/common/pickup/pickup_common.DAE",
                tempDir.resolve("pickup_common.DAE"), "common");
        FlexbodyBindingUtil.performBinding(vehicle.flexbodies, vehicle);

        MeshStats right = statsFor(vehicle.flexbodies, vehicle.nodes, "pickup_hub_FR");
        MeshStats left = statsFor(vehicle.flexbodies, vehicle.nodes, "pickup_hub_FL");
        System.out.println("D15_BINDING " + right);
        System.out.println("D15_BINDING " + left);
        System.out.println("D15_BINDING_ALL " + statsForAllMeshes(vehicle.flexbodies, vehicle.nodes));

        assertEquals(0, right.rigidVertices);
        assertEquals(0, left.rigidVertices);
        assertEquals(0, right.belowPreferredAngle);
        assertEquals(0, left.belowPreferredAngle);
        assertEquals(0, right.outsidePreferredCoordinates);
        assertEquals(0, left.outsidePreferredCoordinates);
    }

    private static void loadDae(Path archive, String entryName, Path output, String namespace) throws Exception {
        try (ZipFile zip = new ZipFile(archive.toFile())) {
            var entry = zip.getEntry(entryName);
            assertNotNull(entry, entryName);
            try (var input = zip.getInputStream(entry)) {
                Files.copy(input, output);
            }
        }
        Method method = DaeMeshLoader.class.getDeclaredMethod("loadMeshUsingAssimp", String.class, String.class);
        method.setAccessible(true);
        method.invoke(null, output.toString(), namespace);
    }

    private static MeshStats statsFor(FlexbodyContainer flex, NodeContainer nodes, String targetMesh) {
        int vertexOffset = 0;
        for (int mesh = 0; mesh < flex.meshCount; mesh++) {
            DaeMeshLoader.RawGeometry geometry = flex.meshName[mesh].isEmpty()
                    ? null : DaeMeshLoader.resolveMesh(flex.vehicleNamespace, flex.meshName[mesh]);
            if (geometry == null) continue;
            if (targetMesh.equals(flex.meshName[mesh])) {
                return collectStats(targetMesh, vertexOffset, geometry.vertexCount, flex, nodes);
            }
            vertexOffset += geometry.vertexCount;
        }
        throw new AssertionError("Missing mesh " + targetMesh);
    }

    private static AggregateStats statsForAllMeshes(FlexbodyContainer flex, NodeContainer nodes) {
        int offset = 0, vertices = 0, rigid = 0, belowPreferred = 0, outsidePreferred = 0;
        double minimumAngle = 90.0, maximumAbsCoordinate = 0.0;
        for (int mesh = 0; mesh < flex.meshCount; mesh++) {
            DaeMeshLoader.RawGeometry geometry = flex.meshName[mesh].isEmpty()
                    ? null : DaeMeshLoader.resolveMesh(flex.vehicleNamespace, flex.meshName[mesh]);
            if (geometry == null) continue;
            MeshStats stats = collectStats(flex.meshName[mesh], offset, geometry.vertexCount, flex, nodes);
            vertices += stats.vertices;
            rigid += stats.rigidVertices;
            belowPreferred += stats.belowPreferredAngle;
            outsidePreferred += stats.outsidePreferredCoordinates;
            minimumAngle = Math.min(minimumAngle, stats.minimumAngle);
            maximumAbsCoordinate = Math.max(maximumAbsCoordinate, stats.maximumAbsCoordinate);
            offset += geometry.vertexCount;
        }
        return new AggregateStats(vertices, rigid, belowPreferred, outsidePreferred,
                minimumAngle, maximumAbsCoordinate);
    }

    private static MeshStats collectStats(String mesh, int offset, int vertexCount,
                                          FlexbodyContainer flex, NodeContainer nodes) {
        int rigid = 0, belowPreferred = 0, outsidePreferred = 0;
        double minimumAngle = 90.0;
        double maximumAbsCoordinate = 0.0;
        for (int localVertex = 0; localVertex < vertexCount; localVertex++) {
            int vertex = offset + localVertex;
            if (!flex.vUseCrossZ[vertex]) {
                rigid++;
                continue;
            }
            double angle = acuteAngleDegrees(flex, nodes, vertex);
            minimumAngle = Math.min(minimumAngle, angle);
            if (angle < FlexbodyBindingUtil.PREFERRED_BASIS_ANGLE_DEGREES) belowPreferred++;
            float x = flex.vWeightX[vertex], y = flex.vWeightY[vertex];
            maximumAbsCoordinate = Math.max(maximumAbsCoordinate, Math.max(Math.abs(x), Math.abs(y)));
            if (x < FlexbodyBindingUtil.PREFERRED_LOCATOR_MIN || x > FlexbodyBindingUtil.PREFERRED_LOCATOR_MAX
                    || y < FlexbodyBindingUtil.PREFERRED_LOCATOR_MIN || y > FlexbodyBindingUtil.PREFERRED_LOCATOR_MAX) {
                outsidePreferred++;
            }
        }
        return new MeshStats(mesh, vertexCount, rigid, belowPreferred, outsidePreferred,
                minimumAngle, maximumAbsCoordinate);
    }

    private static double acuteAngleDegrees(FlexbodyContainer flex, NodeContainer nodes, int vertex) {
        int center = flex.vCenterNode[vertex], vx = flex.vVxNode[vertex], vy = flex.vVyNode[vertex];
        double ux = nodes.baseX[vx] - nodes.baseX[center];
        double uy = nodes.baseY[vx] - nodes.baseY[center];
        double uz = nodes.baseZ[vx] - nodes.baseZ[center];
        double vxv = nodes.baseX[vy] - nodes.baseX[center];
        double vyv = nodes.baseY[vy] - nodes.baseY[center];
        double vzv = nodes.baseZ[vy] - nodes.baseZ[center];
        double cosine = Math.abs(ux * vxv + uy * vyv + uz * vzv)
                / Math.sqrt((ux * ux + uy * uy + uz * uz) * (vxv * vxv + vyv * vyv + vzv * vzv));
        return Math.toDegrees(Math.acos(Math.min(1.0, cosine)));
    }

    private record MeshStats(String mesh, int vertices, int rigidVertices, int belowPreferredAngle,
                             int outsidePreferredCoordinates, double minimumAngle,
                             double maximumAbsCoordinate) {
    }

    private record AggregateStats(int vertices, int rigidVertices, int belowPreferredAngle,
                                  int outsidePreferredCoordinates, double minimumAngle,
                                  double maximumAbsCoordinate) {
    }
}
