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

    @Test
    void rebasesLocalNodesWithoutConvertingTheWorldOriginToFloat() {
        PhysicsRenderTimeline timeline = new PhysicsRenderTimeline();
        float[] localX = {1.0f};
        float[] localY = {2.0f};
        float[] localZ = {3.0f};
        double worldX = 30_000_000.25;
        double worldY = 96.5;
        double worldZ = -30_000_000.75;

        timeline.beginStep(
                1_000L, 50_000_000L, 1,
                worldX, worldY, worldZ,
                localX, localY, localZ, 1
        );

        float[] outX = new float[1];
        float[] outY = new float[1];
        float[] outZ = new float[1];
        assertTrue(timeline.sampleRelativeTo(
                1_000L,
                worldX - 0.125,
                worldY + 0.25,
                worldZ - 0.5,
                outX, outY, outZ, 1
        ));

        assertArrayEquals(new float[]{1.125f}, outX, 0.0001f);
        assertArrayEquals(new float[]{1.75f}, outY, 0.0001f);
        assertArrayEquals(new float[]{3.5f}, outZ, 0.0001f);
    }

    @Test
    void preservesWorldPositionWhenTheMinecraftEntityOriginMovesAtATickBoundary() {
        PhysicsRenderTimeline timeline = new PhysicsRenderTimeline();
        float[] firstLocal = {5.0f};
        timeline.beginStep(
                0L, 50_000_000L, 1,
                1000.0, 64.0, 2000.0,
                firstLocal, firstLocal, firstLocal, 1
        );

        float[] beforeX = new float[1];
        float[] beforeY = new float[1];
        float[] beforeZ = new float[1];
        assertTrue(timeline.sampleRelativeTo(
                50_000_000L,
                1000.0, 64.0, 2000.0,
                beforeX, beforeY, beforeZ, 1
        ));

        // Physics recenters by +4 blocks. Minecraft is still rendering the
        // old origin at tickDelta=0, while the node becomes local coordinate 1.
        float[] recenteredLocal = {1.0f};
        timeline.beginStep(
                50_000_000L, 50_000_000L, 1,
                1004.0, 68.0, 2004.0,
                recenteredLocal, recenteredLocal, recenteredLocal, 1
        );
        float[] afterX = new float[1];
        float[] afterY = new float[1];
        float[] afterZ = new float[1];
        assertTrue(timeline.sampleRelativeTo(
                50_000_000L,
                1000.0, 64.0, 2000.0,
                afterX, afterY, afterZ, 1
        ));

        assertArrayEquals(beforeX, afterX, 0.0001f);
        assertArrayEquals(beforeY, afterY, 0.0001f);
        assertArrayEquals(beforeZ, afterZ, 0.0001f);
    }

    @Test
    void tickDeltaKeepsFiftyFpsSamplesContinuousAcrossTickGenerations() {
        PhysicsRenderTimeline timeline = new PhysicsRenderTimeline();
        float[] zero = {0.0f};
        PhysicsRenderTimeline.Writer first = timeline.beginStep(
                0L, 50_000_000L, 11,
                0.0, 0.0, 0.0,
                zero, zero, zero, 1
        );
        for (int snapshot = 1; snapshot <= 10; snapshot++) {
            float[] position = {snapshot * 5.0f};
            assertTrue(first.publish(snapshot, snapshot * 5_000_000L,
                    position, zero, zero));
        }

        float[] outX = new float[1];
        float[] outY = new float[1];
        float[] outZ = new float[1];
        assertTrue(timeline.sampleAtTickDelta(0.8f, outX, outY, outZ, 1));
        assertEquals(40.0f, outX[0], 0.0001f);

        // At 50 FPS the following frame can cross a tick boundary with a 0.2
        // remainder. The new generation starts at absolute simulation time 50
        // ms, so sampling it at 0.2 must produce 60 ms rather than resetting to
        // the generation's zero-time state.
        PhysicsRenderTimeline.Writer second = timeline.beginStep(
                50_000_000L, 50_000_000L, 11,
                50.0, 0.0, 0.0,
                zero, zero, zero, 1
        );
        for (int snapshot = 1; snapshot <= 10; snapshot++) {
            float[] position = {snapshot * 5.0f};
            assertTrue(second.publish(snapshot, snapshot * 5_000_000L,
                    position, zero, zero));
        }
        assertTrue(timeline.sampleAtTickDeltaRelativeTo(
                0.2f,
                0.0, 0.0, 0.0,
                outX, outY, outZ, 1
        ));
        assertEquals(60.0f, outX[0], 0.0001f);
    }
}
