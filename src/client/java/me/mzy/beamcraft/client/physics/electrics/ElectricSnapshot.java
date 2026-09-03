package me.mzy.beamcraft.client.physics.electrics;

import java.util.Map;
import java.util.Set;

/** Immutable, thread-safe view of one vehicle's electric signals. */
public final class ElectricSnapshot {
    public static final ElectricSnapshot EMPTY = new ElectricSnapshot(0L, Map.of(), new double[0]);

    private final long revision;
    private final Map<String, Integer> signalIds;
    private final double[] values;

    ElectricSnapshot(long revision, Map<String, Integer> signalIds, double[] values) {
        this.revision = revision;
        this.signalIds = Map.copyOf(signalIds);
        this.values = values.clone();
    }

    public long revision() {
        return revision;
    }

    public int signalId(String signal) {
        Integer id = signal == null ? null : signalIds.get(signal.trim());
        return id == null ? -1 : id;
    }

    public boolean contains(String signal) {
        return signal != null && signalIds.containsKey(signal.trim());
    }

    public double get(String signal) {
        return get(signalId(signal));
    }

    public double get(int signalId) {
        return signalId >= 0 && signalId < values.length ? values[signalId] : 0.0;
    }

    public Set<String> signals() {
        return signalIds.keySet();
    }
}
