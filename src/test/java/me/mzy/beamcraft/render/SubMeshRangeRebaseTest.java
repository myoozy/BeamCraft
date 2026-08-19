package me.mzy.beamcraft.client.render;

import me.mzy.beamcraft.client.model.DaeMeshLoader;
import me.mzy.beamcraft.client.render.ComputeSkinningPipeline.SubMeshRange;
import org.junit.jupiter.api.Test;

import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Pure unit tests for {@link ComputeSkinningPipeline#rebaseSubMeshRanges}, the
 * backend-neutral calculation of per-material index ranges in the combined index
 * buffer. No GL context is required.
 */
class SubMeshRangeRebaseTest {

    @Test
    void rebasesRelativeRangesIntoCombinedSpace() {
        List<DaeMeshLoader.SubMesh> subMeshes = Arrays.asList(
                new DaeMeshLoader.SubMesh("body", 0, 30),
                new DaeMeshLoader.SubMesh("glass", 30, 12)
        );
        List<SubMeshRange> ranges = ComputeSkinningPipeline.rebaseSubMeshRanges(subMeshes, 100, 42);

        assertEquals(2, ranges.size());
        assertEquals("body", ranges.get(0).materialName);
        assertEquals(100, ranges.get(0).combinedStartIndex);
        assertEquals(30, ranges.get(0).indexCount);
        assertEquals("glass", ranges.get(1).materialName);
        assertEquals(130, ranges.get(1).combinedStartIndex);
        assertEquals(12, ranges.get(1).indexCount);
    }

    @Test
    void skipsEmptyInvalidAndNullRanges() {
        List<DaeMeshLoader.SubMesh> subMeshes = Arrays.asList(
                new DaeMeshLoader.SubMesh("empty", 0, 0),
                new DaeMeshLoader.SubMesh("negative", -5, 10),
                new DaeMeshLoader.SubMesh("ok", 20, 6),
                null
        );
        List<SubMeshRange> ranges = ComputeSkinningPipeline.rebaseSubMeshRanges(subMeshes, 50, 26);

        assertEquals(1, ranges.size());
        assertEquals("ok", ranges.get(0).materialName);
        assertEquals(70, ranges.get(0).combinedStartIndex);
        assertEquals(6, ranges.get(0).indexCount);
    }

    @Test
    void nullSubMeshListYieldsEmptyResult() {
        assertTrue(ComputeSkinningPipeline.rebaseSubMeshRanges(null, 0, 0).isEmpty());
    }

    @Test
    void concatenatedGeometriesProduceContiguousRanges() {
        List<DaeMeshLoader.SubMesh> first = Arrays.asList(new DaeMeshLoader.SubMesh("hood", 0, 24));
        List<DaeMeshLoader.SubMesh> second = Arrays.asList(
                new DaeMeshLoader.SubMesh("door", 0, 12),
                new DaeMeshLoader.SubMesh("bed", 12, 18)
        );
        List<SubMeshRange> combined = new java.util.ArrayList<>();
        combined.addAll(ComputeSkinningPipeline.rebaseSubMeshRanges(first, 0, 24));
        combined.addAll(ComputeSkinningPipeline.rebaseSubMeshRanges(second, 24, 30));

        assertEquals(3, combined.size());
        assertEquals("hood", combined.get(0).materialName);
        assertEquals(0, combined.get(0).combinedStartIndex);
        assertEquals(24, combined.get(0).indexCount);
        assertEquals("door", combined.get(1).materialName);
        assertEquals(24, combined.get(1).combinedStartIndex);
        assertEquals("bed", combined.get(2).materialName);
        assertEquals(36, combined.get(2).combinedStartIndex);
        assertEquals(18, combined.get(2).indexCount);
    }

    @Test
    void rejectsRangeCrossingGeometryEnd() {
        // start 80 + count 10 runs past this geometry's 85-index array.
        List<DaeMeshLoader.SubMesh> subMeshes = Arrays.asList(
                new DaeMeshLoader.SubMesh("fine", 0, 85),
                new DaeMeshLoader.SubMesh("spill", 80, 10)
        );
        List<SubMeshRange> ranges = ComputeSkinningPipeline.rebaseSubMeshRanges(subMeshes, 0, 85);

        assertEquals(1, ranges.size());
        assertEquals("fine", ranges.get(0).materialName);
        assertEquals(0, ranges.get(0).combinedStartIndex);
        assertEquals(85, ranges.get(0).indexCount);
    }

    @Test
    void rejectsRebasedRangeThatOverflowsInt() {
        // Rebasing near the int ceiling: combinedStart fits but the end crosses it.
        List<DaeMeshLoader.SubMesh> overflow = Arrays.asList(new DaeMeshLoader.SubMesh("over", 5, 10));
        assertTrue(ComputeSkinningPipeline.rebaseSubMeshRanges(overflow, Integer.MAX_VALUE - 10, 100).isEmpty());

        // Combined start itself overflows.
        List<DaeMeshLoader.SubMesh> overflowStart = Arrays.asList(new DaeMeshLoader.SubMesh("overStart", 1, 1));
        assertTrue(ComputeSkinningPipeline.rebaseSubMeshRanges(overflowStart, Integer.MAX_VALUE, 100).isEmpty());
    }

    @Test
    void keepsRangeJustBelowIntCeiling() {
        List<DaeMeshLoader.SubMesh> subMeshes = Arrays.asList(new DaeMeshLoader.SubMesh("edge", 5, 4));
        List<SubMeshRange> ranges =
                ComputeSkinningPipeline.rebaseSubMeshRanges(subMeshes, Integer.MAX_VALUE - 10, 100);

        assertEquals(1, ranges.size());
        assertEquals(Integer.MAX_VALUE - 5, ranges.get(0).combinedStartIndex);
        assertEquals(4, ranges.get(0).indexCount);
    }
}
