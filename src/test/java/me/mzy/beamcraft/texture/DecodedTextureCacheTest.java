package me.mzy.beamcraft.texture;

import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class DecodedTextureCacheTest {

    private static final class TestKey {
        final String id;

        TestKey(String id) {
            this.id = id;
        }

        @Override
        public boolean equals(Object o) {
            return o instanceof TestKey that && id.equals(that.id);
        }

        @Override
        public int hashCode() {
            return id.hashCode();
        }

        @Override
        public String toString() {
            return id;
        }
    }

    private static DecodedTextureCache.ImageLoader<TestKey> loader(AtomicInteger loads, int value) {
        return key -> {
            loads.incrementAndGet();
            return DecodedImage.of(1, 1, new byte[] {(byte) value, (byte) value, (byte) value, (byte) 0xFF}, true);
        };
    }

    @Test
    void acquiresOnceAndRefCounts() throws Exception {
        AtomicInteger loads = new AtomicInteger();
        DecodedTextureCache<TestKey> cache = new DecodedTextureCache<>(16);
        cache.retainNamespace("ns"); // keep the idle entry reusable across releases
        TestKey key = new TestKey("a");

        DecodedImage first = cache.acquire(key, "ns", loader(loads, 10));
        DecodedImage second = cache.acquire(key, "ns", loader(loads, 99));
        assertSame(first, second, "second acquire must not re-decode");
        assertEquals(1, loads.get());
        assertEquals(1, cache.size());
        assertEquals(1, cache.pinnedCount(), "one distinct entry pinned (refs 2)");

        cache.release(key); // one of the two pins
        assertEquals(1, cache.pinnedCount(), "still pinned (refs 1)");
        cache.release(key); // last pin
        assertEquals(0, cache.pinnedCount(), "released but still cached");
        assertEquals(1, cache.size());

        DecodedImage third = cache.acquire(key, "ns", loader(loads, 99));
        assertSame(first, third, "idle entry is reused");
        assertEquals(1, loads.get());
        cache.release(key);
    }

    @Test
    void failedDecodeIsNotCached() throws Exception {
        AtomicInteger loads = new AtomicInteger();
        DecodedTextureCache<TestKey> cache = new DecodedTextureCache<>(16);
        TestKey key = new TestKey("bad");

        assertThrows(IOException.class, () -> cache.acquire(key, "ns", k -> {
            throw new IOException("boom");
        }));
        assertEquals(0, cache.size());
        assertEquals(0, loads.get());

        // A later successful acquire still decodes fresh (nothing was cached).
        cache.acquire(key, "ns", loader(loads, 5));
        assertEquals(1, loads.get());
        cache.release(key);
    }

    @Test
    void namespaceReleaseEvictsIdleVehicleEntriesButKeepsSharedAndPinned() throws Exception {
        DecodedTextureCache<TestKey> cache = new DecodedTextureCache<>(16);
        TestKey vehicleTex = new TestKey("vehicle");
        TestKey commonTex = new TestKey("common");
        TestKey pinnedTex = new TestKey("pinned");

        cache.retainNamespace("car");
        cache.acquire(vehicleTex, "car", loader(new AtomicInteger(), 1));
        cache.acquire(commonTex, null, loader(new AtomicInteger(), 2)); // shared, never namespace-evicted
        cache.acquire(pinnedTex, "car", loader(new AtomicInteger(), 3));
        cache.acquire(pinnedTex, "car", loader(new AtomicInteger(), 3)); // pin twice
        cache.release(vehicleTex); // vehicle texture now idle (namespace still retained)

        cache.releaseNamespace("car"); // last owner released -> idle "car" entries evicted

        // Idle "car" entries are evicted; the pinned one and the shared one survive.
        assertEquals(2, cache.size(), "idle vehicle entries evicted, pinned+shared kept");
        assertEquals(0, cache.retainedNamespaceCount());

        // The pinned acquire survives the namespace release, but releasing it
        // while "car" has no owner left must evict it immediately (no leak).
        cache.release(pinnedTex);
        assertEquals(2, cache.size(), "still pinned once");
        cache.release(pinnedTex);
        assertEquals(1, cache.size(), "now-idle orphaned vehicle entry evicted; shared kept");
    }

    /**
     * Regression for the lifecycle leak: {@code releaseNamespace} runs while a
     * vehicle entry is still pinned; the namespace is then gone, so the final
     * {@link #release} must evict the entry instead of leaving it orphaned.
     */
    @Test
    void finalReleaseEvictsEntryWhoseNamespaceWasReleasedWhilePinned() throws Exception {
        DecodedTextureCache<TestKey> cache = new DecodedTextureCache<>(16);
        TestKey tex = new TestKey("tex");

        cache.retainNamespace("car");
        cache.acquire(tex, "car", loader(new AtomicInteger(), 1));
        cache.releaseNamespace("car"); // namespace goes away while tex is pinned
        assertEquals(1, cache.size());
        assertEquals(1, cache.pinnedCount());

        cache.release(tex); // last pin; namespace already gone -> must be reclaimed
        assertEquals(0, cache.pinnedCount());
        assertEquals(0, cache.size(), "orphaned vehicle entry must not survive its namespace");
        assertEquals(0, cache.retainedNamespaceCount());
    }

    /**
     * Regression for the lifecycle semantics: {@link #release} evicts an entry
     * whenever its namespace is no longer retained, not only when it was
     * explicitly released while pinned. A namespace that was never retained
     * (pure memoization use without {@link #retainNamespace}) therefore cannot
     * keep a cached entry alive past its final release.
     */
    @Test
    void finalReleaseEvictsEntryWhoseNamespaceIsNotRetained() throws Exception {
        DecodedTextureCache<TestKey> cache = new DecodedTextureCache<>(16);
        TestKey key = new TestKey("tex");

        cache.acquire(key, "car", loader(new AtomicInteger(), 1)); // "car" never retained
        assertEquals(1, cache.size());
        cache.release(key);
        assertEquals(0, cache.size(), "entry under an unretained namespace is reclaimed on final release");
        assertEquals(0, cache.retainedNamespaceCount());
    }

    /**
     * Regression for the namespace-metadata leak: releasing many unique
     * namespaces must retain neither their entries nor their names once every
     * acquire is released. {@code namespaceRefs} is the only namespace state,
     * and it must be empty afterwards (the released-namespace set that used to
     * keep every name forever is gone).
     */
    @Test
    void releasingManyUniqueNamespacesRetainsNoMetadata() throws Exception {
        DecodedTextureCache<TestKey> cache = new DecodedTextureCache<>(16);
        List<TestKey> keys = new ArrayList<>();
        int namespaces = 500;
        for (int i = 0; i < namespaces; i++) {
            String ns = "ns" + i;
            TestKey key = new TestKey("k" + i);
            keys.add(key);
            cache.retainNamespace(ns);
            cache.acquire(key, ns, loader(new AtomicInteger(), i));
            cache.releaseNamespace(ns); // namespace goes away while the entry is pinned
        }
        assertEquals(namespaces, cache.size(), "each entry orphaned but still pinned");
        assertEquals(0, cache.retainedNamespaceCount(), "no namespace retains remain");

        for (TestKey key : keys) {
            cache.release(key);
        }
        assertEquals(0, cache.size(), "every orphaned entry reclaimed on final release");
        assertEquals(0, cache.pinnedCount());
        assertEquals(0, cache.retainedNamespaceCount(), "no namespace-name metadata left behind");
    }

    /**
     * Regression for the cache bound: a one-time batch of acquires may exceed
     * {@code maxEntries} while pinned, but releasing them must shrink the cache
     * back to the cap (LRU), not leave it over the bound indefinitely.
     */
    @Test
    void boundIsEnforcedWhenPinnedEntriesAreReleased() throws Exception {
        DecodedTextureCache<TestKey> cache = new DecodedTextureCache<>(2);
        cache.retainNamespace("ns"); // keep idle entries namespace-eligible, so only LRU shrink applies
        TestKey a = new TestKey("a");
        TestKey b = new TestKey("b");
        TestKey c = new TestKey("c");

        cache.acquire(a, "ns", loader(new AtomicInteger(), 1));
        cache.acquire(b, "ns", loader(new AtomicInteger(), 2));
        cache.acquire(c, "ns", loader(new AtomicInteger(), 3));
        assertEquals(3, cache.size(), "pinned entries may exceed the cap temporarily");

        cache.release(a); // a becomes idle -> LRU shrink evicts it (oldest idle)
        cache.release(b);
        cache.release(c);
        assertEquals(2, cache.size(), "LRU shrink on release returns the cache to the cap");
        assertEquals(0, cache.pinnedCount());
    }

    @Test
    void sizeBoundEvictsIdleLru() throws Exception {
        DecodedTextureCache<TestKey> cache = new DecodedTextureCache<>(2);
        cache.retainNamespace("ns"); // namespace retained, so only the LRU bound shrinks the cache
        AtomicInteger loads = new AtomicInteger();
        TestKey a = new TestKey("a");
        TestKey b = new TestKey("b");
        TestKey c = new TestKey("c");

        cache.acquire(a, "ns", loader(loads, 1)); // loads 1
        cache.acquire(b, "ns", loader(loads, 2)); // loads 2
        cache.release(a);
        cache.release(b);
        assertEquals(2, cache.size());

        cache.acquire(c, "ns", loader(loads, 3)); // loads 3; bound 2 evicts idle LRU a
        assertEquals(2, cache.size());
        assertEquals(3, loads.get());

        // a was evicted; re-acquiring it re-decodes (loads 4) and evicts idle b (LRU) to stay in bound.
        cache.acquire(a, "ns", loader(loads, 9));
        assertEquals(4, loads.get());
        assertEquals(2, cache.size());
        cache.release(a);
        cache.release(c);
    }

    @Test
    void pinnedEntriesSurviveSizeBound() throws Exception {
        DecodedTextureCache<TestKey> cache = new DecodedTextureCache<>(1);
        TestKey a = new TestKey("a");
        TestKey b = new TestKey("b");
        cache.acquire(a, "ns", loader(new AtomicInteger(), 1)); // pinned, never released
        cache.acquire(b, "ns", loader(new AtomicInteger(), 2)); // bound exceeded but a is pinned
        assertEquals(2, cache.size(), "pinned entries must never be evicted");
        cache.release(a);
        cache.release(b);
    }

    @Test
    void unboundedWhenMaxNonPositive() throws Exception {
        DecodedTextureCache<TestKey> cache = new DecodedTextureCache<>(0);
        for (int i = 0; i < 100; i++) {
            cache.acquire(new TestKey("k" + i), "ns", loader(new AtomicInteger(), i));
        }
        assertEquals(100, cache.size());
    }

    @Test
    void releaseIsIdempotentAndSafeForUnknownKeys() {
        DecodedTextureCache<TestKey> cache = new DecodedTextureCache<>(16);
        cache.release(new TestKey("never-acquired"));
        cache.release(null);
        cache.retainNamespace(null);
        cache.releaseNamespace(null);
        cache.releaseNamespace("unknown");
        assertEquals(0, cache.retainedNamespaceCount());
    }

    @Test
    void concurrentAcquireDecodesOnce() throws Exception {
        DecodedTextureCache<TestKey> cache = new DecodedTextureCache<>(16);
        AtomicInteger loads = new AtomicInteger();
        TestKey key = new TestKey("shared");
        int threads = 8;
        Thread[] pool = new Thread[threads];
        DecodedImage[] results = new DecodedImage[threads];
        for (int t = 0; t < threads; t++) {
            int id = t;
            pool[t] = new Thread(() -> {
                try {
                    results[id] = cache.acquire(key, "ns", loader(loads, 1));
                } catch (IOException e) {
                    throw new RuntimeException(e);
                }
            });
            pool[t].start();
        }
        for (Thread t : pool) {
            t.join();
        }
        assertEquals(1, loads.get(), "concurrent acquires of one key share one decode");
        for (DecodedImage r : results) {
            assertSame(results[0], r);
        }
        for (int t = 0; t < threads; t++) {
            cache.release(key);
        }
        assertEquals(0, cache.pinnedCount());
    }

    @Test
    void clearRemovesEverything() throws Exception {
        DecodedTextureCache<TestKey> cache = new DecodedTextureCache<>(16);
        TestKey key = new TestKey("k");
        cache.acquire(key, "ns", loader(new AtomicInteger(), 1));
        cache.retainNamespace("ns");
        cache.clear();
        assertEquals(0, cache.size());
        assertEquals(0, cache.retainedNamespaceCount());
    }

    @Test
    void diagnosticCounts() throws Exception {
        DecodedTextureCache<TestKey> cache = new DecodedTextureCache<>(16);
        cache.acquire(new TestKey("a"), "x", loader(new AtomicInteger(), 1));
        cache.acquire(new TestKey("b"), "x", loader(new AtomicInteger(), 2));
        assertTrue(cache.size() >= 2);
        assertTrue(cache.pinnedCount() >= 2);
        cache.retainNamespace("x");
        cache.retainNamespace("y");
        assertEquals(2, cache.retainedNamespaceCount());
    }
}
