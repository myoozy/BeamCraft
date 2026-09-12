package me.mzy.beamcraft.client.debug;

import java.util.Locale;

/**
 * Opt-in timing for the client-side asset load phases: the JBeam scan/parse, the
 * Assimp DAE import, material indexing, and flexbody binding.
 *
 * <p>All of that work happens on the client thread while a vehicle spawns, so
 * these lines exist to answer "which phase is actually costing the seconds"
 * during investigation rather than as ongoing telemetry. Nothing runs when
 * {@link #ENABLED} is false: {@link #start} returns 0, and every logger returns
 * without formatting.
 *
 * <p>Per-file lines go through {@link #logSlowFile} so a single pathological file
 * stands out without printing one line per file — the JBeam scan alone touches
 * several hundred files.
 */
public final class LoadTiming {

    /** Set to {@code true} to log load-phase timings. */
    public static final boolean ENABLED = true;

    /** Files slower than this are logged individually by {@link #logSlowFile}. */
    public static final double SLOW_FILE_MS = 20.0;

    private LoadTiming() {
    }

    /** Begins a timed section. Returns 0 when disabled, which every logger treats as "skip". */
    public static long start() {
        return ENABLED ? System.nanoTime() : 0L;
    }

    /** Milliseconds elapsed since {@code startedNanos}; 0 when disabled. */
    public static double elapsedMs(long startedNanos) {
        return ENABLED && startedNanos != 0L ? (System.nanoTime() - startedNanos) / 1_000_000.0 : 0.0;
    }

    /** Logs {@code <label>: <ms> ms} for a completed section. */
    public static void log(String label, long startedNanos) {
        if (!ENABLED || startedNanos == 0L) {
            return;
        }
        System.out.println(label + ": " + format(elapsedMs(startedNanos)) + " ms");
    }

    /** Logs {@code <label>: <ms> ms} only when the section exceeded {@link #SLOW_FILE_MS}. */
    public static void logSlowFile(String label, long startedNanos) {
        if (!ENABLED || startedNanos == 0L) {
            return;
        }
        double ms = elapsedMs(startedNanos);
        if (ms >= SLOW_FILE_MS) {
            System.out.println(label + ": " + format(ms) + " ms");
        }
    }

    private static String format(double milliseconds) {
        return String.format(Locale.ROOT, "%.1f", milliseconds);
    }
}
