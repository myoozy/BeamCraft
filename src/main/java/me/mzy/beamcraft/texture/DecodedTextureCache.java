package me.mzy.beamcraft.texture;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Ref-counted, namespace-aware cache of decoded textures.
 *
 * <p>Keys are opaque identities (e.g. {@code TextureResource}); each entry holds
 * exactly one immutable {@link DecodedImage} and the decoded pixels are the
 * only data retained -- the source bytes (archive reads) are consumed by the
 * loader and never cached. Failed decodes are not cached at all: when the
 * loader throws, {@link #acquire} propagates the exception and stores nothing,
 * so a permanently corrupt texture is re-decoded (and re-fails) on every
 * acquire rather than pinned in memory.
 *
 * <p><b>Reference counting</b>: every {@link #acquire} must be paired with a
 * {@link #release}. A counted (pinned) entry is never evicted. When its count
 * reaches zero it becomes idle but stays cached for reuse (cheap re-acquire)
 * until evicted by the namespace release or the size bound.
 *
 * <p><b>Lifecycle</b>: callers that own a batch of textures (a loaded vehicle)
 * pair {@link #retainNamespace} with {@link #releaseNamespace}. When the last
 * retain of a namespace is released, every idle entry that was acquired under
 * that namespace is evicted, so vehicle-only textures are reclaimed while
 * shared/common entries survive. Shared/common ownership is represented by a
 * {@code null} namespace; such entries are never evicted by a namespace release.
 * A pinned entry survives a namespace release, but once its last acquire is
 * released while the namespace is no longer retained it is evicted, so a
 * vehicle-only texture can never outlive its namespace (no leak when
 * {@code releaseNamespace} ran while the entry was still pinned).
 * {@code namespaceRefs} is the single source of truth for whether a namespace
 * has an owner, so an entry whose final acquire is released while its namespace
 * is not retained (never retained, or already released) is reclaimed instead of
 * left cached; there is no separate set of released namespace names to retain.
 *
 * <p><b>Bounding</b>: a hard entry cap is enforced by evicting the least
 * recently used <em>idle</em> entries (pinned entries are never evicted). The
 * bound is re-checked on every {@link #release} as well as on
 * {@link #acquire}, so a one-time batch of acquires cannot keep the cache above
 * the cap once its entries are released; the cap may be exceeded only by
 * pinned entries, and only until they are released. Pass a non-positive cap
 * for an unbounded cache.
 *
 * <p><b>Thread safety</b>: all public methods synchronize on the cache
 * instance, so the cache is safe to call from any thread. The loader runs
 * inside the lock: on the first acquire of a key, a slow decode holds the lock
 * and blocks other cache operations. This is acceptable for the current
 * client-side usage (decodes happen at vehicle-spawn time on the client
 * thread); document the trade-off before moving acquires onto a hot path.
 *
 * @param <K> opaque texture identity type (must have value semantics, e.g.
 *            {@code TextureResource})
 */
public final class DecodedTextureCache<K> {

    /** Loads (decodes) the pixels for a key. May read from disk/archive. */
    @FunctionalInterface
    public interface ImageLoader<K> {
        /**
         * @param key the texture identity
         * @return a decoded image (must not be null)
         * @throws IOException if the bytes cannot be read or decoded; the
         *                     cache never retains the failure
         */
        DecodedImage load(K key) throws IOException;
    }

    private static final class Entry<K> {
        final K key;
        final DecodedImage image;
        final String namespace;
        int refs;
        long lastAccess;

        Entry(K key, DecodedImage image, String namespace, long lastAccess) {
            this.key = key;
            this.image = image;
            this.namespace = namespace;
            this.refs = 1;
            this.lastAccess = lastAccess;
        }
    }

    private final int maxEntries;
    private final Map<K, Entry<K>> entries = new HashMap<>();
    /** Live owner counts per namespace; a namespace is "retained" iff present. */
    private final Map<String, Integer> namespaceRefs = new HashMap<>();
    private long clock;

    /** Creates a cache with a default cap of 1024 entries. */
    public DecodedTextureCache() {
        this(1024);
    }

    /**
     * @param maxEntries maximum number of retained entries before idle LRU
     *                   eviction kicks in; non-positive means unbounded
     */
    public DecodedTextureCache(int maxEntries) {
        this.maxEntries = maxEntries;
    }

    /**
     * Obtains the decoded image for {@code key}, decoding it on first use via
     * {@code loader} and retaining it until {@link #release}. Concurrent
     * acquires of the same key share one decode.
     *
     * @param key       texture identity (must be a valid map key, non-null)
     * @param namespace owning namespace used for lifecycle eviction; may be
     *                  null for shared/common entries that are never evicted by
     *                  a namespace release
     * @param loader    decode function; must not return null
     * @return the decoded image (pinned)
     * @throws IOException when the loader fails; nothing is cached in that case
     */
    public synchronized DecodedImage acquire(K key, String namespace, ImageLoader<K> loader) throws IOException {
        Objects.requireNonNull(key, "key");
        Objects.requireNonNull(loader, "loader");
        Entry<K> entry = entries.get(key);
        if (entry != null) {
            entry.refs++;
            entry.lastAccess = ++clock;
            return entry.image;
        }
        DecodedImage image = Objects.requireNonNull(loader.load(key), "loader returned null image");
        Entry<K> created = new Entry<>(key, image, namespace, ++clock);
        entries.put(key, created);
        evictIdleIfOverBound();
        return image;
    }

    /**
     * Releases one prior acquire of {@code key}. Idempotent and safe to call
     * with an unknown or already-released key.
     *
     * <p>Two things happen here that {@link #acquire} alone cannot guarantee:
     * <ul>
     *   <li>If the release drops an entry to idle and its namespace is no
     *       longer retained (absent from {@code namespaceRefs}), the entry is
     *       evicted immediately - there is no namespace owner left to reclaim
     *       it later, so keeping it would leak a vehicle-only texture
     *       indefinitely.</li>
     *   <li>The idle LRU bound is enforced again, so a one-time batch of
     *       acquires cannot keep the cache above {@code maxEntries} forever
     *       once the entries are released. Pinned entries are never evicted;
     *       the cap may be temporarily exceeded only by pinned entries.</li>
     * </ul>
     */
    public synchronized void release(K key) {
        if (key == null) {
            return;
        }
        Entry<K> entry = entries.get(key);
        if (entry == null || entry.refs <= 0) {
            return;
        }
        entry.refs--;
        if (entry.refs == 0) {
            // Namespace no longer retained (released while pinned, or never
            // retained): there is no owner left to reclaim this entry, so evict
            // now that it is idle instead of leaving it cached.
            if (entry.namespace != null && !namespaceRefs.containsKey(entry.namespace)) {
                entries.remove(key);
            }
            evictIdleIfOverBound();
        }
    }

    /** Records one live owner of a namespace. Idempotent. */
    public synchronized void retainNamespace(String namespace) {
        if (namespace == null) {
            return;
        }
        namespaceRefs.merge(namespace, 1, Integer::sum);
    }

    /**
     * Releases one owner of a namespace. When the last owner releases, all
     * <em>idle</em> entries acquired under that namespace are evicted; entries
     * still pinned at that point are evicted the moment their last acquire is
     * released, because the namespace is no longer retained. No-op for unknown
     * namespaces.
     */
    public synchronized void releaseNamespace(String namespace) {
        if (namespace == null) {
            return;
        }
        Integer count = namespaceRefs.get(namespace);
        if (count == null) {
            return;
        }
        if (count > 1) {
            namespaceRefs.put(namespace, count - 1);
            return;
        }
        // Last owner: drop the namespace from the ref map (that map is the
        // single source of truth for "retained") and evict every idle entry
        // acquired under it now. Entries still pinned here are evicted later by
        // release(), once idle, because the namespace is no longer present.
        namespaceRefs.remove(namespace);
        entries.entrySet().removeIf(e -> e.getValue().refs == 0 && namespace.equals(e.getValue().namespace));
    }

    /** Number of retained decoded textures (diagnostic). */
    public synchronized int size() {
        return entries.size();
    }

    /** Number of textures still pinned by an acquire (diagnostic). */
    public synchronized int pinnedCount() {
        int pinned = 0;
        for (Entry<K> entry : entries.values()) {
            if (entry.refs > 0) {
                pinned++;
            }
        }
        return pinned;
    }

    /** Number of namespaces currently retained (diagnostic). */
    public synchronized int retainedNamespaceCount() {
        return namespaceRefs.size();
    }

    /** Removes every entry, including pinned ones. Only for shutdown/tests. */
    public synchronized void clear() {
        entries.clear();
        namespaceRefs.clear();
    }

    private void evictIdleIfOverBound() {
        if (maxEntries <= 0 || entries.size() <= maxEntries) {
            return;
        }
        List<Entry<K>> idle = new ArrayList<>();
        for (Entry<K> entry : entries.values()) {
            if (entry.refs == 0) {
                idle.add(entry);
            }
        }
        idle.sort(Comparator.comparingLong(e -> e.lastAccess));
        int excess = entries.size() - maxEntries;
        for (int i = 0; i < excess && i < idle.size(); i++) {
            entries.remove(idle.get(i).key);
        }
    }
}
