package me.mzy.beamcraft.client.physics.electrics;

import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Mutable per-vehicle signal bus. Writers update this object between prepared
 * physics steps; the worker consumes only {@link ElectricSnapshot snapshots}.
 */
public final class ElectricBus {
    private static final int INITIAL_CAPACITY = 16;

    private final Map<String, Integer> signalIds = new LinkedHashMap<>();
    private double[] values = new double[INITIAL_CAPACITY];
    private long revision;
    private long snapshotRevision = -1L;
    private ElectricSnapshot cachedSnapshot = ElectricSnapshot.EMPTY;

    public synchronized int register(String signal) {
        String name = requireName(signal);
        Integer existing = signalIds.get(name);
        if (existing != null) {
            return existing;
        }

        int id = signalIds.size();
        if (id >= values.length) {
            values = Arrays.copyOf(values, values.length * 2);
        }
        signalIds.put(name, id);
        revision++;
        return id;
    }

    public synchronized int signalId(String signal) {
        if (signal == null) {
            return -1;
        }
        Integer id = signalIds.get(signal.trim());
        return id == null ? -1 : id;
    }

    public synchronized void set(String signal, double value) {
        requireFinite(value);
        int id = register(signal);
        setRegistered(id, value);
    }

    public synchronized void set(int signalId, double value) {
        if (signalId < 0 || signalId >= signalIds.size()) {
            throw new IndexOutOfBoundsException("Unknown electric signal id: " + signalId);
        }
        setRegistered(signalId, value);
    }

    public synchronized double get(String signal) {
        int id = signalId(signal);
        return id < 0 ? 0.0 : values[id];
    }

    public synchronized double get(int signalId) {
        return signalId >= 0 && signalId < signalIds.size() ? values[signalId] : 0.0;
    }

    public synchronized ElectricSnapshot snapshot() {
        if (snapshotRevision != revision) {
            cachedSnapshot = new ElectricSnapshot(
                    revision, signalIds, Arrays.copyOf(values, signalIds.size()));
            snapshotRevision = revision;
        }
        return cachedSnapshot;
    }

    /** Keeps stable signal ids while returning every value to its inactive state. */
    public synchronized void resetValues() {
        Arrays.fill(values, 0, signalIds.size(), 0.0);
        revision++;
    }

    public synchronized void clear() {
        signalIds.clear();
        values = new double[INITIAL_CAPACITY];
        revision++;
    }

    private void setRegistered(int id, double value) {
        requireFinite(value);
        if (Double.doubleToLongBits(values[id]) != Double.doubleToLongBits(value)) {
            values[id] = value;
            revision++;
        }
    }

    private static String requireName(String signal) {
        if (signal == null || signal.isBlank()) {
            throw new IllegalArgumentException("Electric signal name must not be blank");
        }
        return signal.trim();
    }

    private static void requireFinite(double value) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException("Electric signal values must be finite");
        }
    }
}
