package me.mzy.beamcraft.texture;

import java.util.Set;

/**
 * Decides which lifecycle namespace a resolved texture belongs to, so the
 * {@link DecodedTextureCache} knows whether an entry should be reclaimed when a
 * vehicle is released.
 *
 * <p>Ownership is determined by the physical source the texture was resolved
 * from (its {@code sourceId}): a texture inside a source owned by the
 * requesting vehicle's namespace is vehicle-only and is reclaimed when that
 * namespace's last reference is released. Everything else -- textures from a
 * common source, a source owned by a <em>different</em> namespace, or an
 * unknown source -- is <em>shared</em> and is represented by {@code null}, the
 * cache's durable-ownership marker: never reclaimed by a namespace release
 * (only by the cache's size bound). This keeps common assets valid for as long
 * as any vehicle uses them while still reclaiming vehicle-only decodes on
 * release.
 *
 * <p>Shared ownership deliberately reuses {@code null} (the same value used
 * when no namespace applies) rather than a magic {@code "common"} namespace:
 * {@code null} has unambiguous durable semantics in {@link DecodedTextureCache}
 * (no one can ever {@code releaseNamespace(null)}), whereas a magic string
 * could collide with a real namespace or be released by accident.
 */
public final class TextureOwnership {

    private TextureOwnership() {
    }

    /**
     * @param sourceId          the texture's physical source identity (see
     *                          {@code TextureResource#sourceId})
     * @param commonSourceIds   canonical ids of all registered common sources,
     *                          or null if none
     * @param fallbackNamespace the vehicle namespace the caller is acquiring
     *                          for (already lowercased), or null
     * @param namespaceSourceIds canonical ids of the sources owned by
     *                          {@code fallbackNamespace}, or null
     * @return {@code fallbackNamespace} for that namespace's own sources,
     *         otherwise {@code null} (shared/common/foreign -- durable, never
     *         reclaimed by a namespace release)
     */
    public static String resolve(String sourceId, Set<String> commonSourceIds,
                                 String fallbackNamespace, Set<String> namespaceSourceIds) {
        if (sourceId == null) {
            return null;
        }
        // A common source is shared regardless of what the caller owns; the
        // explicit check also makes "common wins over the namespace fallback"
        // deliberate rather than accidental.
        if (commonSourceIds != null && commonSourceIds.contains(sourceId)) {
            return null;
        }
        if (fallbackNamespace != null && namespaceSourceIds != null && namespaceSourceIds.contains(sourceId)) {
            return fallbackNamespace;
        }
        return null;
    }
}
