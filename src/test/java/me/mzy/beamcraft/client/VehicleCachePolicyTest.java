package me.mzy.beamcraft.client;

import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class VehicleCachePolicyTest {

    @Test
    void evictsOldestEntriesUntilBudgetIsMet() {
        List<VehicleCachePolicy.Candidate<String>> candidates = List.of(
                new VehicleCachePolicy.Candidate<>("newest", 40, 30),
                new VehicleCachePolicy.Candidate<>("oldest", 40, 10),
                new VehicleCachePolicy.Candidate<>("middle", 40, 20)
        );

        assertEquals(List.of("oldest", "middle"), VehicleCachePolicy.selectEvictions(
                candidates, 50, 0, 1_000, 0.85));
    }

    @Test
    void heapHighWatermarkCanEvictEvenBelowCacheBudget() {
        List<VehicleCachePolicy.Candidate<String>> candidates = List.of(
                new VehicleCachePolicy.Candidate<>("oldest", 30, 1),
                new VehicleCachePolicy.Candidate<>("newest", 30, 2)
        );

        assertEquals(List.of("oldest"), VehicleCachePolicy.selectEvictions(
                candidates, 1_000, 880, 1_000, 0.85));
    }

    @Test
    void keepsEverythingWhenBothLimitsHaveHeadroom() {
        assertTrue(VehicleCachePolicy.selectEvictions(
                List.of(new VehicleCachePolicy.Candidate<>("vehicle", 40, 1)),
                100, 500, 1_000, 0.85).isEmpty());
    }
}
