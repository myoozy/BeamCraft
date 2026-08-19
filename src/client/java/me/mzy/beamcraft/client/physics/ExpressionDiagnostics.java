package me.mzy.beamcraft.client.physics;

import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * Throttled diagnostic sink for JBeam expression evaluation failures.
 *
 * <p>Vehicle load must never die because a single expression is unsupported or
 * malformed, but it must not stay completely silent either: the first distinct
 * failures are logged with the offending expression and a reason, and anything
 * beyond the cap is collapsed into one summary line so a fleet of vehicles can't
 * spam the console.
 */
public final class ExpressionDiagnostics {

    /** Maximum number of distinct (expression, reason) pairs logged before suppressing. */
    private static final int MAX_LOGGED = 40;

    private static final Set<String> LOGGED = ConcurrentHashMap.newKeySet();
    private static final AtomicInteger PRINTED = new AtomicInteger();
    private static final AtomicInteger SUPPRESSED = new AtomicInteger();
    private static final AtomicBoolean SUMMARY_PRINTED = new AtomicBoolean();

    private ExpressionDiagnostics() {}

    /** Log (once per distinct pair) that evaluating {@code expression} failed for {@code reason}. */
    public static void warn(String expression, String reason) {
        if (expression != null && expression.length() > 200) {
            expression = expression.substring(0, 200) + "…";
        }
        String key = (expression == null ? "" : expression) + " || " + reason;
        if (LOGGED.add(key)) {
            if (PRINTED.get() < MAX_LOGGED) {
                PRINTED.incrementAndGet();
                System.err.println("[BeamExpression] " + expression + "  →  " + reason);
            } else {
                SUPPRESSED.incrementAndGet();
                if (SUMMARY_PRINTED.compareAndSet(false, true)) {
                    System.err.println("[BeamExpression] … further distinct expression failures suppressed (throttled)");
                }
            }
        }
    }

    /** Clear all throttling state (used by tests). */
    public static void reset() {
        LOGGED.clear();
        PRINTED.set(0);
        SUPPRESSED.set(0);
        SUMMARY_PRINTED.set(false);
    }

    /** Number of distinct failures actually logged (capped at {@link #MAX_LOGGED}). */
    public static int loggedCount() {
        return PRINTED.get();
    }
}
