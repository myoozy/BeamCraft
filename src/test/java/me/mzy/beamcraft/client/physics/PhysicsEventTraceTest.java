package me.mzy.beamcraft.client.physics;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

class PhysicsEventTraceTest {
    @Test
    void captureIncludesPreTriggerAndPostTriggerSubsteps() {
        PhysicsEventTrace trace = new PhysicsEventTrace(8, 2);
        trace.configure(true, 10.0);

        assertNull(record(trace, 0, 0));
        assertNull(record(trace, 1, 0));
        assertNull(record(trace, 2, 3));
        assertNull(record(trace, 3, 0));
        PhysicsEventTrace.CompletedCapture capture = record(trace, 4, 0);

        assertNotNull(capture);
        String dump = capture.formatCsv();
        assertTrue(dump.contains("trigger=soft-contact"));
        assertTrue(dump.contains("sequence,tick_substep"));
        assertTrue(dump.contains("0,0,"));
        assertTrue(dump.contains("4,4,"));
        assertTrue(!trace.enabled());
    }

    private static PhysicsEventTrace.CompletedCapture record(PhysicsEventTrace trace, int substep, int resolved) {
        return trace.record(substep,
                100L, 0L, 0L, 0L, 0L, 50L, 25L,
                10, 0, 2, resolved, 0, 0, 0, 0);
    }
}
