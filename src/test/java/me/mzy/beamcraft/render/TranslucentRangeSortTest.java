package me.mzy.beamcraft.render;

import me.mzy.beamcraft.client.render.ComputeSkinningPipeline;
import me.mzy.beamcraft.client.render.ComputeSkinningPipeline.SubMeshRange;
import org.joml.Matrix4f;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * Pure unit tests for {@link ComputeSkinningPipeline#sortTranslucentBackToFront},
 * the back-to-front ordering of translucent sub-mesh ranges for alpha blending.
 * The camera in view space looks down -Z, so the most-negative view-space depth
 * is the farthest range and must be drawn first. No GL context is required.
 */
class TranslucentRangeSortTest {

    private static SubMeshRange range(String material, float cx, float cy, float cz) {
        SubMeshRange range = new SubMeshRange(material, 0, 0);
        range.centerX = cx;
        range.centerY = cy;
        range.centerZ = cz;
        range.hasCentroid = true;
        return range;
    }

    @Test
    void farthestRangeDrawsFirstAlongCameraAxis() {
        // modelView translates the vehicle 10 units away along -Z: a vertex at
        // model (0,0,0) lands at view z=-10 (farther) and a vertex at model
        // (0,0,5) lands at view z=-5 (nearer).
        Matrix4f modelView = new Matrix4f().translate(0f, 0f, -10f);
        SubMeshRange far = range("glass_far", 0f, 0f, 0f);
        SubMeshRange near = range("glass_near", 0f, 0f, 5f);

        List<SubMeshRange> sorted = ComputeSkinningPipeline.sortTranslucentBackToFront(
                Arrays.asList(near, far), modelView);

        assertEquals(2, sorted.size());
        assertEquals(far, sorted.get(0), "the farthest range must be drawn first");
        assertEquals(near, sorted.get(1));
    }

    @Test
    void identityModelViewSortsByModelZAscending() {
        // With an identity model-view the model z is the view-space depth.
        // Most-negative z is farthest.
        SubMeshRange far = range("a", 0f, 0f, -20f);
        SubMeshRange mid = range("b", 0f, 0f, -10f);
        SubMeshRange near = range("c", 0f, 0f, -1f);

        List<SubMeshRange> sorted = ComputeSkinningPipeline.sortTranslucentBackToFront(
                Arrays.asList(near, mid, far), new Matrix4f());

        assertEquals(Arrays.asList(far, mid, near), sorted);
    }

    @Test
    void rangesWithoutCentroidSortToTheFrontAsFarthest() {
        SubMeshRange unknown = new SubMeshRange("unknown", 0, 0); // hasCentroid false
        SubMeshRange far = range("far", 0f, 0f, -30f);
        SubMeshRange near = range("near", 0f, 0f, -3f);

        List<SubMeshRange> sorted = ComputeSkinningPipeline.sortTranslucentBackToFront(
                Arrays.asList(far, near, unknown), new Matrix4f());

        assertEquals(unknown, sorted.get(0), "a centroid-less range must draw first (treated as farthest)");
        assertEquals(far, sorted.get(1));
        assertEquals(near, sorted.get(2));
    }

    @Test
    void equalDepthKeepsOriginalStableOrder() {
        SubMeshRange first = range("a", 0f, 0f, -5f);
        SubMeshRange second = range("b", 0f, 0f, -5f);
        SubMeshRange third = range("c", 0f, 0f, -5f);

        List<SubMeshRange> input = new ArrayList<>(Arrays.asList(first, second, third));
        List<SubMeshRange> sorted = ComputeSkinningPipeline.sortTranslucentBackToFront(input, new Matrix4f());

        assertEquals(input, sorted, "equal-depth ranges must keep their original relative order");
    }

    @Test
    void nullModelViewPreservesInputOrder() {
        SubMeshRange near = range("near", 0f, 0f, -3f);
        SubMeshRange far = range("far", 0f, 0f, -30f);
        List<SubMeshRange> input = new ArrayList<>(Arrays.asList(near, far));

        List<SubMeshRange> sorted = ComputeSkinningPipeline.sortTranslucentBackToFront(input, null);

        assertEquals(input, sorted);
    }

    @Test
    void doesNotMutateTheInputList() {
        SubMeshRange far = range("far", 0f, 0f, -30f);
        SubMeshRange near = range("near", 0f, 0f, -3f);
        List<SubMeshRange> input = new ArrayList<>(Arrays.asList(near, far));

        List<SubMeshRange> sorted = ComputeSkinningPipeline.sortTranslucentBackToFront(input, new Matrix4f());

        assertEquals(Arrays.asList(near, far), input, "the caller's list must not be reordered");
        assertEquals(Arrays.asList(far, near), sorted);
    }
}
