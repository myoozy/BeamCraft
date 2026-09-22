package me.mzy.beamcraft.client.physics;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

class PhysicsEventTraceTest {
    @Test
    void manualCaptureRunsUntilStoppedAndRetainsNewestSamples() {
        PhysicsEventTrace trace = new PhysicsEventTrace(4);
        trace.configure(true);
        assertTrue(trace.start());

        record(trace, 0, 0);
        record(trace, 1, 0);
        record(trace, 2, 3);
        record(trace, 3, 0);
        record(trace, 4, 0);
        assertTrue(trace.enabled());
        PhysicsEventTrace.CompletedCapture capture = trace.stop();

        assertNotNull(capture);
        String dump = capture.formatCsv();
        assertTrue(dump.contains("trigger=manual-stop"));
        assertTrue(dump.contains("retained_samples=4 total_samples=5"));
        assertTrue(dump.contains("sequence,tick_substep"));
        assertTrue(dump.lines().noneMatch(line -> line.startsWith("0,0,")));
        assertTrue(dump.contains("1,1,"));
        assertTrue(dump.contains("4,4,"));
        assertTrue(!trace.enabled());
    }

    @Test
    void disabledTraceCannotStart() {
        PhysicsEventTrace trace = new PhysicsEventTrace(4);
        trace.configure(false);

        assertTrue(!trace.start());
        assertNull(trace.stop());
    }

    private static void record(PhysicsEventTrace trace, int substep, int resolved) {
        trace.record(substep,
                200L, 100L, 0L, 0L, 0L, 0L, 50L, 25L,
                10, 0, 2, resolved, 0, 0, 0, 0);
    }
}
