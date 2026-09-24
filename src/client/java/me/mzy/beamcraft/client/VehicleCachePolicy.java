package me.mzy.beamcraft.client;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/** Pure LRU selection logic for the sleeping-vehicle cache. */
final class VehicleCachePolicy {
    record Candidate<T>(T value, long estimatedBytes, long lastUseOrder) {
        Candidate {
            estimatedBytes = Math.max(1L, estimatedBytes);
        }
    }

    private VehicleCachePolicy() {
    }

    static <T> List<T> selectEvictions(
            List<Candidate<T>> candidates,
            long budgetBytes,
            long heapUsedBytes,
            long maxHeapBytes,
            double heapHighWatermark
    ) {
        long retainedBytes = 0L;
        for (Candidate<T> candidate : candidates) {
            retainedBytes = saturatingAdd(retainedBytes, candidate.estimatedBytes());
        }

        long budgetOverage = Math.max(0L, retainedBytes - Math.max(0L, budgetBytes));
        long heapLimit = maxHeapBytes <= 0L
                ? Long.MAX_VALUE
                : (long) Math.floor(maxHeapBytes * heapHighWatermark);
        long heapOverage = Math.max(0L, heapUsedBytes - heapLimit);
        long bytesToRelease = Math.max(budgetOverage, heapOverage);
        if (bytesToRelease == 0L) {
            return List.of();
        }

        List<Candidate<T>> oldestFirst = new ArrayList<>(candidates);
        oldestFirst.sort(Comparator.comparingLong(Candidate::lastUseOrder));
        List<T> evictions = new ArrayList<>();
        long selectedBytes = 0L;
        for (Candidate<T> candidate : oldestFirst) {
            evictions.add(candidate.value());
            selectedBytes = saturatingAdd(selectedBytes, candidate.estimatedBytes());
            if (selectedBytes >= bytesToRelease) {
                break;
            }
        }
        return evictions;
    }

    private static long saturatingAdd(long left, long right) {
        if (left >= Long.MAX_VALUE - right) return Long.MAX_VALUE;
        return left + right;
    }
}
