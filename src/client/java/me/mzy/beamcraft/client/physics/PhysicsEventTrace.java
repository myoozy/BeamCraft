package me.mzy.beamcraft.client.physics;

import java.util.Locale;

/**
 * Optional allocation-free substep ring buffer for manually bounded impact-spike diagnosis.
 * A completed capture is formatted only after recording has stopped.
 */
final class PhysicsEventTrace {
    /** Ten seconds at the current 2000 Hz physics rate. */
    private static final int DEFAULT_CAPACITY = 20_000;

    private final int capacity;
    private final long[] sequence;
    private final int[] substep;
    private final long[] totalNs;
    private final long[] internalNs;
    private final long[] breakCommitNs;
    private final long[] sapNs;
    private final long[] candidateNs;
    private final long[] colorNs;
    private final long[] softNs;
    private final long[] environmentNs;
    private final int[] contacts;
    private final int[] certificateSkipped;
    private final int[] aabbPassed;
    private final int[] resolved;
    private final int[] sweptResolved;
    private final int[] brokenBeams;
    private final int[] breakGroups;
    private final int[] brokenTriangles;

    private boolean available;
    private boolean recording;
    private long nextSequence;
    private int writeIndex;
    private int size;

    PhysicsEventTrace() {
        this(DEFAULT_CAPACITY);
    }

    PhysicsEventTrace(int capacity) {
        this.capacity = capacity;
        sequence = new long[capacity];
        substep = new int[capacity];
        totalNs = new long[capacity];
        internalNs = new long[capacity];
        breakCommitNs = new long[capacity];
        sapNs = new long[capacity];
        candidateNs = new long[capacity];
        colorNs = new long[capacity];
        softNs = new long[capacity];
        environmentNs = new long[capacity];
        contacts = new int[capacity];
        certificateSkipped = new int[capacity];
        aabbPassed = new int[capacity];
        resolved = new int[capacity];
        sweptResolved = new int[capacity];
        brokenBeams = new int[capacity];
        breakGroups = new int[capacity];
        brokenTriangles = new int[capacity];
    }

    void configure(boolean available) {
        this.available = available;
        recording = false;
        reset();
    }

    boolean start() {
        if (!available || recording) return false;
        reset();
        recording = true;
        return true;
    }

    CompletedCapture stop() {
        if (!recording) return null;
        recording = false;
        return new CompletedCapture(this, "manual-stop", writeIndex, size, nextSequence);
    }

    private void reset() {
        nextSequence = 0L;
        writeIndex = 0;
        size = 0;
    }

    boolean enabled() {
        return recording;
    }

    boolean available() {
        return available;
    }

    void record(int tickSubstep,
                  long total, long internal, long breakCommit, long sap, long candidate, long color,
                  long soft, long environment, int contactCount, int certSkipped,
                  int passedAabb, int resolvedCount, int sweptResolvedCount, int beamBreaks,
                  int newBreakGroups, int triangleBreaks) {
        if (!recording) return;

        int index = writeIndex;
        sequence[index] = nextSequence++;
        substep[index] = tickSubstep;
        totalNs[index] = total;
        internalNs[index] = internal;
        breakCommitNs[index] = breakCommit;
        sapNs[index] = sap;
        candidateNs[index] = candidate;
        colorNs[index] = color;
        softNs[index] = soft;
        environmentNs[index] = environment;
        contacts[index] = contactCount;
        certificateSkipped[index] = certSkipped;
        aabbPassed[index] = passedAabb;
        resolved[index] = resolvedCount;
        sweptResolved[index] = sweptResolvedCount;
        brokenBeams[index] = beamBreaks;
        breakGroups[index] = newBreakGroups;
        brokenTriangles[index] = triangleBreaks;
        writeIndex = (writeIndex + 1) % capacity;
        if (size < capacity) size++;
    }

    private String formatCsv(String completedTriggerReason, int completedWriteIndex,
                             int completedSize, long completedTotalSamples) {
        StringBuilder output = new StringBuilder(completedSize * 100);
        output.append("[BeamCraft physics trace] trigger=").append(completedTriggerReason)
                .append(" retained_samples=").append(completedSize)
                .append(" total_samples=").append(completedTotalSamples).append('\n');
        output.append("sequence,tick_substep,total_us,internal_us,break_commit_us,sap_us,candidate_us,color_us,")
                .append("soft_us,environment_us,contacts,cert_skip,aabb_passed,resolved,ccd_resolved,")
                .append("broken_beams,new_break_groups,broken_triangles\n");
        int first = (completedWriteIndex - completedSize + capacity) % capacity;
        for (int entry = 0; entry < completedSize; entry++) {
            int index = (first + entry) % capacity;
            output.append(sequence[index]).append(',').append(substep[index]).append(',')
                    .append(micros(totalNs[index])).append(',')
                    .append(micros(internalNs[index])).append(',')
                    .append(micros(breakCommitNs[index])).append(',')
                    .append(micros(sapNs[index])).append(',')
                    .append(micros(candidateNs[index])).append(',')
                    .append(micros(colorNs[index])).append(',')
                    .append(micros(softNs[index])).append(',')
                    .append(micros(environmentNs[index])).append(',')
                    .append(contacts[index]).append(',').append(certificateSkipped[index]).append(',')
                    .append(aabbPassed[index]).append(',').append(resolved[index]).append(',')
                    .append(sweptResolved[index]).append(',')
                    .append(brokenBeams[index]).append(',').append(breakGroups[index]).append(',')
                    .append(brokenTriangles[index]).append('\n');
        }
        return output.toString();
    }

    /**
     * A disarmed capture whose comparatively expensive CSV formatting can run away
     * from both the physics worker and render thread. The owning trace remains
     * immutable after its one-shot capture completes.
     */
    static final class CompletedCapture {
        private final PhysicsEventTrace trace;
        private final String triggerReason;
        private final int writeIndex;
        private final int size;
        private final long totalSamples;

        private CompletedCapture(PhysicsEventTrace trace, String triggerReason, int writeIndex,
                                 int size, long totalSamples) {
            this.trace = trace;
            this.triggerReason = triggerReason;
            this.writeIndex = writeIndex;
            this.size = size;
            this.totalSamples = totalSamples;
        }

        String formatCsv() {
            return trace.formatCsv(triggerReason, writeIndex, size, totalSamples);
        }

        int retainedSamples() {
            return size;
        }
    }

    private static String micros(long nanos) {
        return String.format(Locale.ROOT, "%.3f", nanos / 1_000.0);
    }
}
