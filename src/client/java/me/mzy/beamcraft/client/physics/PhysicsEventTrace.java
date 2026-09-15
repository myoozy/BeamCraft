package me.mzy.beamcraft.client.physics;

import java.util.Locale;

/**
 * Optional allocation-free substep ring buffer for impact-spike diagnosis.
 * A completed capture is formatted once, after the post-trigger window.
 */
final class PhysicsEventTrace {
    private static final int DEFAULT_CAPACITY = 384;
    private static final int DEFAULT_POST_TRIGGER = 128;

    private final int capacity;
    private final int postTriggerSamples;
    private final long[] sequence;
    private final int[] substep;
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

    private boolean enabled;
    private long internalTriggerNs;
    private long nextSequence;
    private int writeIndex;
    private int size;
    private int postRemaining = -1;
    private String triggerReason;

    PhysicsEventTrace() {
        this(DEFAULT_CAPACITY, DEFAULT_POST_TRIGGER);
    }

    PhysicsEventTrace(int capacity, int postTriggerSamples) {
        this.capacity = capacity;
        this.postTriggerSamples = postTriggerSamples;
        sequence = new long[capacity];
        substep = new int[capacity];
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

    void configure(boolean enabled, double internalTriggerMs) {
        this.enabled = enabled;
        this.internalTriggerNs = Math.max(0L, Math.round(internalTriggerMs * 1_000_000.0));
        nextSequence = 0L;
        writeIndex = 0;
        size = 0;
        postRemaining = -1;
        triggerReason = null;
    }

    boolean enabled() {
        return enabled;
    }

    String record(int tickSubstep,
                  long internal, long breakCommit, long sap, long candidate, long color,
                  long soft, long environment, int contactCount, int certSkipped,
                  int passedAabb, int resolvedCount, int sweptResolvedCount, int beamBreaks,
                  int newBreakGroups, int triangleBreaks) {
        if (!enabled) return null;

        int index = writeIndex;
        sequence[index] = nextSequence++;
        substep[index] = tickSubstep;
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

        if (postRemaining < 0) {
            if (resolvedCount > 0) armPostWindow("soft-contact");
            else if (beamBreaks > 0 || newBreakGroups > 0 || triangleBreaks > 0) armPostWindow("fracture");
            else if (internalTriggerNs > 0L && internal >= internalTriggerNs) armPostWindow("internal-force-spike");
        }

        if (postRemaining >= 0 && --postRemaining <= 0) {
            enabled = false;
            return formatCsv();
        }
        return null;
    }

    private void armPostWindow(String reason) {
        triggerReason = reason;
        postRemaining = postTriggerSamples + 1;
    }

    private String formatCsv() {
        StringBuilder output = new StringBuilder(size * 100);
        output.append("[BeamCraft physics trace] trigger=").append(triggerReason).append('\n');
        output.append("sequence,tick_substep,internal_us,break_commit_us,sap_us,candidate_us,color_us,")
                .append("soft_us,environment_us,contacts,cert_skip,aabb_passed,resolved,ccd_resolved,")
                .append("broken_beams,new_break_groups,broken_triangles\n");
        int first = (writeIndex - size + capacity) % capacity;
        for (int entry = 0; entry < size; entry++) {
            int index = (first + entry) % capacity;
            output.append(sequence[index]).append(',').append(substep[index]).append(',')
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

    private static String micros(long nanos) {
        return String.format(Locale.ROOT, "%.3f", nanos / 1_000.0);
    }
}
