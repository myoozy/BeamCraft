package me.mzy.beamcraft.client.physics;

import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class TriangleContainerTest {
    @Test
    void breakingAnEdgeBreaksEveryAdjacentTriangleRegardlessOfNodeOrder() {
        TriangleContainer triangles = new TriangleContainer();
        triangles.addTriangle(spec(), 1, 2, 3);
        triangles.addTriangle(spec(), 2, 1, 4);
        triangles.addTriangle(spec(), 5, 6, 7);

        triangles.buildBreakIndices();
        triangles.breakByEdge(2, 1);

        assertTrue(triangles.broken[0]);
        assertTrue(triangles.broken[1]);
        assertFalse(triangles.broken[2]);
    }

    @Test
    void breakingAGroupBreaksOnlyTrianglesAssignedToThatGroup() {
        TriangleContainer triangles = new TriangleContainer();
        triangles.addTriangle(spec("doorLatch", "glass"), 1, 2, 3);
        triangles.addTriangle(spec("other"), 3, 4, 5);

        triangles.buildBreakIndices();
        triangles.breakByGroup("glass");

        assertTrue(triangles.broken[0]);
        assertFalse(triangles.broken[1]);
    }

    @Test
    void resetRestoresBrokenTrianglesWithoutChangingCollisionConfiguration() {
        TriangleContainer triangles = new TriangleContainer();
        triangles.addTriangle(spec(), 1, 2, 3);
        triangles.addTriangle(new PhysicsSpecs.TriangleSpec(
                null, null, null, List.of(), 0, false), 3, 4, 5);
        triangles.buildBreakIndices();
        triangles.breakByEdge(1, 2);
        triangles.breakByEdge(3, 4);

        triangles.reset();

        assertFalse(triangles.broken[0]);
        assertFalse(triangles.broken[1]);
        assertTrue(triangles.collision[0]);
        assertFalse(triangles.collision[1]);
    }

    private static PhysicsSpecs.TriangleSpec spec(String... breakGroups) {
        return new PhysicsSpecs.TriangleSpec(
                null, null, null, List.of(breakGroups), 0, true);
    }
}
