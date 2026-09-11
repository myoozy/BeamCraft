package me.mzy.beamcraft.client.physics;

import java.util.Map;

/**
 * One registered {@code adaptiveDampers} controller instance.
 *
 * <p>Fields other than {@link #currentModeName()} are immutable once assembly
 * finished, which is what makes the actuator queries safe to call while the
 * physics worker is running.
 */
public final class AdaptiveDamperController {
    private final String instanceName;
    private final int[] beamIndices;
    private final Map<String, AdaptiveDamperMode> modes;
    private volatile String currentModeName;

    AdaptiveDamperController(String instanceName, int[] beamIndices,
                             Map<String, AdaptiveDamperMode> modes) {
        this.instanceName = instanceName;
        this.beamIndices = beamIndices;
        this.modes = modes;
    }

    public String instanceName() {
        return instanceName;
    }

    /** Bounded-beam indices this controller drives; never {@code null}. */
    public int[] beamIndices() {
        return beamIndices;
    }

    /** Immutable mode table keyed by mode name. */
    public Map<String, AdaptiveDamperMode> modes() {
        return modes;
    }

    /** Name of the mode currently applied, or {@code null} before the first command. */
    public String currentModeName() {
        return currentModeName;
    }

    public boolean hasMode(String modeName) {
        return modeName != null && modes.containsKey(modeName);
    }

    AdaptiveDamperMode mode(String modeName) {
        return modeName == null ? null : modes.get(modeName);
    }

    void setCurrentModeName(String modeName) {
        this.currentModeName = modeName;
    }
}
