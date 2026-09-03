package me.mzy.beamcraft.client.physics;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class PhysicsRenderTimelineTest {

    @Test
    void samplesPublishedSnapshotsAgainstWallClockTime() {
        PhysicsRenderTimeline timeline = new PhysicsRenderTimeline();
        float[] x = {0.0f, 10.0f};
        float[] y = {1.0f, 11.0f};
        float[] z = {2.0f, 12.0f};
        long startNanos = 1_000_000_000L;

        PhysicsRenderTimeline.Writer writer = timeline.beginStep(
                startNanos,
                10_000_000L,
                3,
                x,
                y,
                z,
                2
        );

        x = new float[]{10.0f, 20.0f};
        y = new float[]{11.0f, 21.0f};
        z = new float[]{12.0f, 22.0f};
        assertTrue(writer.publish(1, 5_000_000L, x, y, z));

        x = new float[]{20.0f, 30.0f};
        y = new float[]{21.0f, 31.0f};
        z = new float[]{22.0f, 32.0f};
        assertTrue(writer.publish(2, 10_000_000L, x, y, z));

        float[] outX = new float[2];
        float[] outY = new float[2];
        float[] outZ = new float[2];
        assertTrue(timeline.sample(startNanos + 7_500_000L, outX, outY, outZ, 2));

        assertArrayEquals(new float[]{15.0f, 25.0f}, outX, 0.0001f);
        assertArrayEquals(new float[]{16.0f, 26.0f}, outY, 0.0001f);
        assertArrayEquals(new float[]{17.0f, 27.0f}, outZ, 0.0001f);

        float[] oneNode = new float[3];
        assertTrue(timeline.sampleNode(startNanos + 7_500_000L, 1, oneNode));
        assertArrayEquals(new float[]{25.0f, 26.0f, 27.0f}, oneNode, 0.0001f);
    }

    @Test
    void holdsNewestSnapshotWhenPhysicsHasNotPublishedTheFutureYet() {
        PhysicsRenderTimeline timeline = new PhysicsRenderTimeline();
        long startNanos = 5_000_000_000L;
        float[] zero = {0.0f};
        PhysicsRenderTimeline.Writer writer = timeline.beginStep(
                startNanos,
                50_000_000L,
                11,
                zero,
                zero,
                zero,
                1
        );
        float[] five = {5.0f};
        assertTrue(writer.publish(1, 5_000_000L, five, five, five));

        float[] outX = new float[1];
        float[] outY = new float[1];
        float[] outZ = new float[1];
        assertTrue(timeline.sample(startNanos + 30_000_000L, outX, outY, outZ, 1));
        assertEquals(5.0f, outX[0]);
    }

    @Test
    void rejectsWriterFromAnOlderStepGeneration() {
        PhysicsRenderTimeline timeline = new PhysicsRenderTimeline();
        float[] zero = {0.0f};
        PhysicsRenderTimeline.Writer oldWriter = timeline.beginStep(
                0L, 5_000_000L, 2, zero, zero, zero, 1);
        timeline.beginStep(5_000_000L, 5_000_000L, 2, zero, zero, zero, 1);

        float[] one = {1.0f};
        assertFalse(oldWriter.publish(1, 5_000_000L, one, one, one));
        assertEquals(1, timeline.publishedCount());
    }
}
