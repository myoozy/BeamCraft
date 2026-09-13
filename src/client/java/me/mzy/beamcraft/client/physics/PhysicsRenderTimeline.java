package me.mzy.beamcraft.client.physics;

import java.util.concurrent.atomic.AtomicInteger;

/**
 * Reusable single-producer/single-consumer timeline of node positions for one
 * vehicle. The physics worker publishes snapshots in increasing time order;
 * the client render thread samples them against wall-clock time.
 *
 * <p>Snapshot arrays are retained and reused between steps so a 200 Hz publish
 * rate does not create short-lived arrays or GC pressure.</p>
 */
public final class PhysicsRenderTimeline {
    private static final int AXIS_COUNT = 3;

    private final AtomicInteger publishedCount = new AtomicInteger();

    private float[][] positions = new float[0][];
    private long[] offsetsNanos = new long[0];
    private volatile long generation;
    private long startedNanos;
    private long durationNanos;
    private int nodeCount;
    private double originX;
    private double originY;
    private double originZ;

    /**
     * Starts a new timeline and publishes its zero-time snapshot on the client
     * thread. The previous physics job must already be complete.
     */
    public Writer beginStep(
            long stepStartedNanos,
            long stepDurationNanos,
            int snapshotCount,
            double originX,
            double originY,
            double originZ,
            float[] posX,
            float[] posY,
            float[] posZ,
            int count
    ) {
        if (snapshotCount < 1) {
            throw new IllegalArgumentException("snapshotCount must include the zero-time snapshot");
        }
        if (count < 0) {
            throw new IllegalArgumentException("node count must be non-negative");
        }

        publishedCount.set(0);
        ensureCapacity(snapshotCount, count);
        startedNanos = stepStartedNanos;
        durationNanos = Math.max(0L, stepDurationNanos);
        nodeCount = count;
        this.originX = originX;
        this.originY = originY;
        this.originZ = originZ;
        long stepGeneration = ++generation;

        copyPositions(positions[0], posX, posY, posZ, count);
        offsetsNanos[0] = 0L;
        publishedCount.set(1);
        return new Writer(this, stepGeneration, snapshotCount, count);
    }

    /** Compatibility overload for timelines whose local coordinates have no world origin. */
    public Writer beginStep(
            long stepStartedNanos,
            long stepDurationNanos,
            int snapshotCount,
            float[] posX,
            float[] posY,
            float[] posZ,
            int count
    ) {
        return beginStep(stepStartedNanos, stepDurationNanos, snapshotCount,
                0.0, 0.0, 0.0, posX, posY, posZ, count);
    }

    /**
     * Samples the two published snapshots surrounding {@code nowNanos}. If the
     * worker is behind, the newest completed snapshot is held without blocking.
     */
    public boolean sample(
            long nowNanos,
            float[] outX,
            float[] outY,
            float[] outZ,
            int count
    ) {
        long elapsedNanos = Math.max(0L, nowNanos - startedNanos);
        return sampleAtOffset(elapsedNanos, outX, outY, outZ, count);
    }

    /** Samples this fixed physics step at Minecraft's fractional tick time. */
    public boolean sampleAtTickDelta(
            float tickDelta,
            float[] outX,
            float[] outY,
            float[] outZ,
            int count
    ) {
        return sampleAtOffset(tickDeltaOffsetNanos(tickDelta), outX, outY, outZ, count);
    }

    private boolean sampleAtOffset(
            long requestedOffsetNanos,
            float[] outX,
            float[] outY,
            float[] outZ,
            int count
    ) {
        int available = publishedCount.get();
        if (available == 0 || count != nodeCount) {
            return false;
        }

        long targetNanos = Math.max(0L, Math.min(requestedOffsetNanos, durationNanos));

        int upper = 0;
        while (upper < available && offsetsNanos[upper] < targetNanos) {
            upper++;
        }

        if (upper == 0) {
            copySnapshotToOutputs(positions[0], outX, outY, outZ, count);
            return true;
        }
        if (upper >= available) {
            copySnapshotToOutputs(positions[available - 1], outX, outY, outZ, count);
            return true;
        }

        int lower = upper - 1;
        long lowerTime = offsetsNanos[lower];
        long upperTime = offsetsNanos[upper];
        float alpha = upperTime > lowerTime
                ? (float) ((double) (targetNanos - lowerTime) / (double) (upperTime - lowerTime))
                : 0.0f;
        interpolateSnapshots(positions[lower], positions[upper], alpha, outX, outY, outZ, count);
        return true;
    }

    /**
     * Samples local node positions and rebases them from this physics step's
     * double-precision world origin to the origin used by Minecraft's current
     * entity render matrix. Only the small origin delta is converted to float.
     */
    public boolean sampleRelativeTo(
            long nowNanos,
            double relativeOriginX,
            double relativeOriginY,
            double relativeOriginZ,
            float[] outX,
            float[] outY,
            float[] outZ,
            int count
    ) {
        if (!sample(nowNanos, outX, outY, outZ, count)) {
            return false;
        }
        offsetOutputs(
                (float) (originX - relativeOriginX),
                (float) (originY - relativeOriginY),
                (float) (originZ - relativeOriginZ),
                outX, outY, outZ, count
        );
        return true;
    }

    /** Tick-synchronous counterpart of {@link #sampleRelativeTo}. */
    public boolean sampleAtTickDeltaRelativeTo(
            float tickDelta,
            double relativeOriginX,
            double relativeOriginY,
            double relativeOriginZ,
            float[] outX,
            float[] outY,
            float[] outZ,
            int count
    ) {
        if (!sampleAtTickDelta(tickDelta, outX, outY, outZ, count)) {
            return false;
        }
        offsetOutputs(
                (float) (originX - relativeOriginX),
                (float) (originY - relativeOriginY),
                (float) (originZ - relativeOriginZ),
                outX, outY, outZ, count
        );
        return true;
    }

    /** Samples one node without allocating full-vehicle interpolation buffers. */
    public boolean sampleNode(long nowNanos, int node, float[] out) {
        long elapsedNanos = Math.max(0L, nowNanos - startedNanos);
        return sampleNodeAtOffset(elapsedNanos, node, out);
    }

    /** Samples one node at Minecraft's fractional tick time. */
    public boolean sampleNodeAtTickDelta(float tickDelta, int node, float[] out) {
        return sampleNodeAtOffset(tickDeltaOffsetNanos(tickDelta), node, out);
    }

    private boolean sampleNodeAtOffset(long requestedOffsetNanos, int node, float[] out) {
        int available = publishedCount.get();
        if (available == 0 || node < 0 || node >= nodeCount || out.length < AXIS_COUNT) {
            return false;
        }

        long targetNanos = Math.max(0L, Math.min(requestedOffsetNanos, durationNanos));
        int upper = 0;
        while (upper < available && offsetsNanos[upper] < targetNanos) {
            upper++;
        }

        if (upper == 0) {
            copyNode(positions[0], node, out);
        } else if (upper >= available) {
            copyNode(positions[available - 1], node, out);
        } else {
            int lower = upper - 1;
            long lowerTime = offsetsNanos[lower];
            long upperTime = offsetsNanos[upper];
            float alpha = upperTime > lowerTime
                    ? (float) ((double) (targetNanos - lowerTime) / (double) (upperTime - lowerTime))
                    : 0.0f;
            out[0] = lerp(positions[lower][node], positions[upper][node], alpha);
            out[1] = lerp(positions[lower][nodeCount + node], positions[upper][nodeCount + node], alpha);
            out[2] = lerp(positions[lower][nodeCount * 2 + node], positions[upper][nodeCount * 2 + node], alpha);
        }
        return true;
    }

    /** Samples one node relative to a caller-supplied double-precision origin. */
    public boolean sampleNodeRelativeTo(
            long nowNanos,
            int node,
            double relativeOriginX,
            double relativeOriginY,
            double relativeOriginZ,
            float[] out
    ) {
        if (!sampleNode(nowNanos, node, out)) {
            return false;
        }
        out[0] += (float) (originX - relativeOriginX);
        out[1] += (float) (originY - relativeOriginY);
        out[2] += (float) (originZ - relativeOriginZ);
        return true;
    }

    /** Tick-synchronous counterpart of {@link #sampleNodeRelativeTo}. */
    public boolean sampleNodeAtTickDeltaRelativeTo(
            float tickDelta,
            int node,
            double relativeOriginX,
            double relativeOriginY,
            double relativeOriginZ,
            float[] out
    ) {
        if (!sampleNodeAtTickDelta(tickDelta, node, out)) {
            return false;
        }
        out[0] += (float) (originX - relativeOriginX);
        out[1] += (float) (originY - relativeOriginY);
        out[2] += (float) (originZ - relativeOriginZ);
        return true;
    }

    private void copyNode(float[] snapshot, int node, float[] out) {
        out[0] = snapshot[node];
        out[1] = snapshot[nodeCount + node];
        out[2] = snapshot[nodeCount * 2 + node];
    }

    public void clear() {
        publishedCount.set(0);
        generation++;
        nodeCount = 0;
    }

    int publishedCount() {
        return publishedCount.get();
    }

    private boolean publish(
            long writerGeneration,
            int snapshotIndex,
            long offsetNanos,
            float[] posX,
            float[] posY,
            float[] posZ,
            int count,
            int snapshotLimit
    ) {
        if (writerGeneration != generation) {
            return false;
        }
        if (count != nodeCount) {
            throw new IllegalStateException("node count changed during a physics step");
        }
        if (snapshotIndex <= 0 || snapshotIndex >= snapshotLimit) {
            throw new IndexOutOfBoundsException("snapshot index " + snapshotIndex + " outside prepared timeline");
        }
        int expectedIndex = publishedCount.get();
        if (snapshotIndex != expectedIndex) {
            throw new IllegalStateException(
                    "render snapshots must be published in order: expected " + expectedIndex + ", got " + snapshotIndex);
        }

        copyPositions(positions[snapshotIndex], posX, posY, posZ, count);
        offsetsNanos[snapshotIndex] = Math.max(0L, Math.min(offsetNanos, durationNanos));
        // Atomic set is the release publication for the array copies above.
        publishedCount.set(snapshotIndex + 1);
        return true;
    }

    private void ensureCapacity(int snapshotCount, int count) {
        if (positions.length < snapshotCount) {
            float[][] expandedPositions = new float[snapshotCount][];
            System.arraycopy(positions, 0, expandedPositions, 0, positions.length);
            positions = expandedPositions;
            offsetsNanos = new long[snapshotCount];
        }

        int requiredValues = Math.multiplyExact(count, AXIS_COUNT);
        for (int snapshot = 0; snapshot < snapshotCount; snapshot++) {
            if (positions[snapshot] == null || positions[snapshot].length < requiredValues) {
                positions[snapshot] = new float[requiredValues];
            }
        }
    }

    private static void copyPositions(
            float[] destination,
            float[] posX,
            float[] posY,
            float[] posZ,
            int count
    ) {
        System.arraycopy(posX, 0, destination, 0, count);
        System.arraycopy(posY, 0, destination, count, count);
        System.arraycopy(posZ, 0, destination, count * 2, count);
    }

    private static void copySnapshotToOutputs(
            float[] snapshot,
            float[] outX,
            float[] outY,
            float[] outZ,
            int count
    ) {
        System.arraycopy(snapshot, 0, outX, 0, count);
        System.arraycopy(snapshot, count, outY, 0, count);
        System.arraycopy(snapshot, count * 2, outZ, 0, count);
    }

    private static void interpolateSnapshots(
            float[] lower,
            float[] upper,
            float alpha,
            float[] outX,
            float[] outY,
            float[] outZ,
            int count
    ) {
        for (int node = 0; node < count; node++) {
            outX[node] = lerp(lower[node], upper[node], alpha);
            outY[node] = lerp(lower[count + node], upper[count + node], alpha);
            outZ[node] = lerp(lower[count * 2 + node], upper[count * 2 + node], alpha);
        }
    }

    private static void offsetOutputs(
            float offsetX,
            float offsetY,
            float offsetZ,
            float[] outX,
            float[] outY,
            float[] outZ,
            int count
    ) {
        for (int node = 0; node < count; node++) {
            outX[node] += offsetX;
            outY[node] += offsetY;
            outZ[node] += offsetZ;
        }
    }

    private static float lerp(float start, float end, float alpha) {
        return start + (end - start) * alpha;
    }

    private long tickDeltaOffsetNanos(float tickDelta) {
        double clamped = Math.max(0.0, Math.min(1.0, tickDelta));
        return Math.round(clamped * durationNanos);
    }

    /** Worker-side handle tied to exactly one prepared step generation. */
    public static final class Writer {
        private final PhysicsRenderTimeline owner;
        private final long generation;
        private final int snapshotLimit;
        private final int nodeCount;

        private Writer(PhysicsRenderTimeline owner, long generation, int snapshotLimit, int nodeCount) {
            this.owner = owner;
            this.generation = generation;
            this.snapshotLimit = snapshotLimit;
            this.nodeCount = nodeCount;
        }

        public boolean publish(
                int snapshotIndex,
                long offsetNanos,
                float[] posX,
                float[] posY,
                float[] posZ
        ) {
            return owner.publish(
                    generation,
                    snapshotIndex,
                    offsetNanos,
                    posX,
                    posY,
                    posZ,
                    nodeCount,
                    snapshotLimit
            );
        }
    }
}
