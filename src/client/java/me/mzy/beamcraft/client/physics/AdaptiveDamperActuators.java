package me.mzy.beamcraft.client.physics;

import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * Standalone BeamNG-compatible adaptive damper actuator backend.
 *
 * <p>Mirrors {@code lua/vehicle/controller/drivingDynamics/actuators/adaptiveDampers.lua}.
 * Each registered controller instance owns a set of <em>bounded</em> beams,
 * addressed by the beams' authored JBeam {@code name}, and a mode table. Setting
 * a mode re-derives every runtime damping channel from the beam's immutable
 * authored value times the mode coefficient and clamps it back to the
 * cutoff-aware stability ceiling the assembled configuration admits:
 *
 * <pre>
 *   runtime = min(authored * coefficient, stabilityCeiling)
 *   split   = authored.beamDampVelocitySplit * velocitySplitCoef   (compression and rebound)
 * </pre>
 *
 * <p>Because every application starts from the authored base, switching
 * {@code hard -> soft -> hard} or resetting never compounds and never loses the
 * authored value. This deliberately has no dependency on drive modes or the
 * electrics bus; it is a plain actuator surface a future controller can drive.
 *
 * <h2>Threading</h2>
 * {@link #setDamperMode(String, String)} only reads the immutable mode tables and
 * enqueues a command, so it is safe to call from an MC/controller thread while
 * physics runs. {@link #applyPending()} drains that queue at the start of a
 * physics sub-step on the worker itself, which is the only place the SoA damping
 * arrays are mutated. {@link #applyDamperMode(String, String)} is the
 * deterministic, immediate entry point used by tests and by assembly; it must not
 * be called while a sub-step is mutating the same vehicle.
 */
public final class AdaptiveDamperActuators {
    /**
     * Mode applied during assembly when the author defined it, matching the
     * controller's neutral setting (the same coefficients the base beam already
     * has, so this is a no-op on the authored values).
     */
    public static final String DEFAULT_MODE = "regular";

    private final SoftBodyVehicle vehicle;
    private final Map<String, AdaptiveDamperController> controllers = new ConcurrentHashMap<>();

    /**
     * Cross-thread commands waiting for the next sub-step. One slot per queued
     * request; a later command for the same controller simply wins because the
     * queue is drained in order.
     */
    private final ConcurrentLinkedQueue<Command> pending = new ConcurrentLinkedQueue<>();

    private record Command(String controllerName, String modeName) {}

    AdaptiveDamperActuators(SoftBodyVehicle vehicle) {
        this.vehicle = vehicle;
    }

    // ------------------------------------------------------------------ registration

    /**
     * Registers every parsed controller, resolving beam names against the
     * bounded-beam name index. Replaces any previously registered set, so a
     * re-assembly is idempotent. Malformed entries are skipped rather than
     * failing the whole vehicle.
     */
    void registerAll(List<AdaptiveDamperSpec> specs) {
        controllers.clear();
        pending.clear();
        if (specs == null) return;

        BoundedBeamContainer beams = vehicle.boundedBeams;
        for (AdaptiveDamperSpec spec : specs) {
            if (spec == null || spec.instanceName() == null || spec.instanceName().isEmpty()) continue;
            Map<String, AdaptiveDamperMode> modes = spec.modes();
            if (modes == null || modes.isEmpty()) continue;

            int[] indices = resolveBeamIndices(beams, spec.dampBeamNames());
            controllers.put(spec.instanceName(),
                    new AdaptiveDamperController(spec.instanceName(), indices, Map.copyOf(modes)));
        }
    }

    private static int[] resolveBeamIndices(BoundedBeamContainer beams, List<String> beamNames) {
        if (beamNames == null || beamNames.isEmpty()) return new int[0];
        int[] collected = new int[beamNames.size()];
        int size = 0;
        for (String beamName : beamNames) {
            for (int index : beams.indicesForName(beamName)) {
                // A duplicate name in a single controller's list must not drive
                // the same beam twice.
                boolean duplicate = false;
                for (int i = 0; i < size; i++) {
                    if (collected[i] == index) {
                        duplicate = true;
                        break;
                    }
                }
                if (duplicate) continue;
                if (size == collected.length) collected = java.util.Arrays.copyOf(collected, size * 2);
                collected[size++] = index;
            }
        }
        return java.util.Arrays.copyOf(collected, size);
    }

    // ------------------------------------------------------------------ API

    /**
     * Queues {@code modeName} for {@code controllerName}. Returns {@code false}
     * and leaves every state untouched when either name is unknown; the mode
     * table is immutable after assembly, so the check is valid on any thread.
     *
     * @see #applyPending()
     */
    public boolean setDamperMode(String controllerName, String modeName) {
        if (!canApply(controllerName, modeName)) return false;
        pending.add(new Command(controllerName, modeName));
        return true;
    }

    /**
     * Queues {@code modeName} for every registered controller that defines it.
     *
     * <p>Exists because a vehicle can carry several adaptive damper controllers
     * (the ETK800 has one per axle) that share the same mode names, and a caller
     * issuing a ride-mode command should not have to enumerate them.
     *
     * @return the number of controllers that queued the mode; {@code 0} when no
     *         controller knows it.
     */
    public int setAllDamperModes(String modeName) {
        if (modeName == null) return 0;
        int queued = 0;
        for (AdaptiveDamperController controller : controllers.values()) {
            if (!controller.hasMode(modeName)) continue;
            pending.add(new Command(controller.instanceName(), modeName));
            queued++;
        }
        return queued;
    }

    /**
     * Applies {@code modeName} to {@code controllerName} immediately on the
     * calling thread. Returns {@code false} and mutates nothing when either name
     * is unknown.
     */
    public boolean applyDamperMode(String controllerName, String modeName) {
        AdaptiveDamperController controller = controller(controllerName);
        AdaptiveDamperMode mode = controller == null ? null : controller.mode(modeName);
        if (mode == null) return false;
        apply(controller, mode);
        return true;
    }

    /** True when {@code controllerName} names a registered controller. */
    public boolean hasController(String controllerName) {
        return controllerName != null && controllers.containsKey(controllerName);
    }

    /** Currently applied mode of a controller, or {@code null} when unknown/unset. */
    public String currentMode(String controllerName) {
        AdaptiveDamperController controller = controller(controllerName);
        return controller == null ? null : controller.currentModeName();
    }

    /** Registered controller names; mainly for diagnostics and tests. */
    public Collection<String> controllerNames() {
        return List.copyOf(controllers.keySet());
    }

    /** Null-tolerant lookup; {@link ConcurrentHashMap} rejects a null key outright. */
    public AdaptiveDamperController controller(String controllerName) {
        return controllerName == null ? null : controllers.get(controllerName);
    }

    // ------------------------------------------------------------------ physics hook

    /**
     * Drains queued commands. Called by the internal-force solver at the start of
     * every sub-step so the damping arrays are only ever touched by the thread
     * that owns the sub-step.
     */
    void applyPending() {
        Command command;
        while ((command = pending.poll()) != null) {
            applyDamperMode(command.controllerName(), command.modeName());
        }
    }

    /** Applies {@link #DEFAULT_MODE} to every controller that defines it. */
    void applyDefaultModes() {
        for (AdaptiveDamperController controller : controllers.values()) {
            AdaptiveDamperMode mode = controller.mode(DEFAULT_MODE);
            if (mode != null) apply(controller, mode);
        }
    }

    /**
     * Re-derives every active controller from its authored values. Mirrors the
     * BeamNG controller's empty {@code reset()}: the selected mode stays selected
     * and is simply reapplied, so no scaling accumulates across resets.
     */
    void reset() {
        pending.clear();
        for (AdaptiveDamperController controller : controllers.values()) {
            AdaptiveDamperMode mode = controller.mode(controller.currentModeName());
            if (mode != null) apply(controller, mode);
        }
    }

    void clear() {
        controllers.clear();
        pending.clear();
    }

    // ------------------------------------------------------------------ internals

    /** Null-tolerant lookup; {@link ConcurrentHashMap} rejects a null key outright. */
    private boolean canApply(String controllerName, String modeName) {
        AdaptiveDamperController controller = controller(controllerName);
        return controller != null && controller.hasMode(modeName);
    }

    private void apply(AdaptiveDamperController controller, AdaptiveDamperMode mode) {
        BoundedBeamContainer beams = vehicle.boundedBeams;
        int count = beams.count;
        for (int index : controller.beamIndices()) {
            if (index < 0 || index >= count) continue;

            float ceiling = beams.dampStabilityCeiling[index];
            beams.damp[index] = scaled(beams.authoredDamp[index], mode.beamDampCoef(), ceiling);
            beams.dampFast[index] = scaled(
                    beams.authoredDampFast[index], mode.beamDampFastCoef(), ceiling);
            beams.dampRebound[index] = scaled(
                    beams.authoredDampRebound[index], mode.beamDampReboundCoef(), ceiling);
            beams.dampReboundFast[index] = scaled(
                    beams.authoredDampReboundFast[index], mode.beamDampReboundFastCoef(), ceiling);

            // adaptiveDampers.lua passes the same scaled split for both the
            // compression and the rebound channel, overriding any authored
            // rebound-specific split.
            float split = scaledSplit(beams.authoredDampVelocitySplit[index],
                    mode.beamDampVelocitySplitCoef());
            beams.dampVelocitySplit[index] = split;
            beams.dampVelocitySplitRebound[index] = split;
        }
        controller.setCurrentModeName(mode.name());
    }

    private static float scaled(float authored, float coefficient, float ceiling) {
        float value = authored * coefficient;
        if (Float.isNaN(value)) return ceiling;
        return Math.min(value, ceiling);
    }

    /** The velocity split is a threshold, not a stiffness/damping coefficient, so it is never stability-clamped. */
    private static float scaledSplit(float authored, float coefficient) {
        float value = authored * coefficient;
        if (Float.isNaN(value)) return authored;
        return Math.min(value, Float.MAX_VALUE);
    }
}
