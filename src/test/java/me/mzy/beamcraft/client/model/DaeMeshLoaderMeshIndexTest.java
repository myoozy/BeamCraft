package me.mzy.beamcraft.client.model;

import org.junit.jupiter.api.Test;

import java.nio.charset.StandardCharsets;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Tests the cheap name extraction that lets a {@code .dae} be attributed to a mesh
 * without importing it through Assimp.
 *
 * <p>This is the risky half of on-demand mesh loading: if a name is extracted under
 * a different form than Assimp's node name, the provider is never found and the
 * caller falls back to importing the whole namespace — the cost the on-demand path
 * exists to avoid. The names come from real stock assets
 * ({@code disc_brake} in {@code disc_brakes.dae}, {@code etk_wheel_04a} in
 * {@code etk_wheel.dae}, {@code tire_super_modern_sport} in {@code tires3.dae}),
 * plus the generic Blender node names {@code tires.dae} ships.
 */
class DaeMeshLoaderMeshIndexTest {

    private static Set<String> namesOf(String daeText) {
        return DaeMeshLoader.extractMeshNames(daeText.getBytes(StandardCharsets.US_ASCII));
    }

    @Test
    void readsColladaNodeNames() {
        String dae = """
                <?xml version="1.0" encoding="utf-8"?>
                <COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema">
                  <library_geometries>
                    <geometry id="disc_brake-mesh" name="disc_brake">
                      <mesh/>
                    </geometry>
                  </library_geometries>
                  <library_visual_scenes>
                    <visual_scene id="Scene" name="Scene">
                      <node id="disc_brake" name="disc_brake" type="NODE">
                        <instance_geometry url="#disc_brake-mesh"/>
                      </node>
                    </visual_scene>
                  </library_visual_scenes>
                </COLLADA>
                """;

        assertTrue(namesOf(dae).contains("disc_brake"),
                "the node name the flexbody table references must be extracted");
    }

    @Test
    void stripsTheMeshSuffixAndNumericSuffixes() {
        // COLLADA geometry ids carry a "-mesh" suffix and Blender exports attach
        // ".001" style suffixes; cleanIdentifier normalises both, and the cache keys
        // must match what Assimp reports afterwards.
        Set<String> names = namesOf("""
                <node name="etk_wheel_04a-mesh"/>
                <node name="Mesh.001"/>
                <node name="Mesh_002-mesh"/>
                """);

        assertTrue(names.contains("etk_wheel_04a"), "the -mesh suffix must be stripped");
        assertTrue(names.contains("Mesh"), "a numeric suffix must be stripped");
        assertTrue(names.contains("Mesh_002"), "both suffixes together must be stripped");
    }

    @Test
    void preservesCase() {
        // Cache keys are Assimp node names and lookups are case-sensitive, so a
        // lowercased index would never match.
        assertTrue(namesOf("<node name=\"Super_Modern_Sport\"/>").contains("Super_Modern_Sport"));
    }

    @Test
    void fallsBackToIdsWhenNamesAreAbsent() {
        assertTrue(namesOf("<geometry id=\"etk_wheel_04a\"/>").contains("etk_wheel_04a"));
    }

    @Test
    void ignoresEmptyNamesAndUnquotedTrailers() {
        Set<String> names = namesOf("<node name=\"\"/><node id=\"ok\"/><node name=");

        assertEquals(Set.of("ok"), names);
    }

    @Test
    void returnsNothingForTextWithoutNames() {
        assertTrue(namesOf("<COLLADA><library_geometries/>").isEmpty());
    }
}
