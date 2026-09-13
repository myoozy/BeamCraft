package me.mzy.beamcraft.client.assets;

import java.util.List;

/**
 * The result of one {@link AssetScanner} scan: the conflict-resolved entries
 * for a namespace and the distinct containers that actually contributed them.
 * {@code sources()} drives {@code MaterialLibrary}'s texture-source
 * registration (and its release tracking), so it only contains containers with
 * a winning entry — a container whose resources were all shadowed elsewhere is
 * never registered.
 */
public final class NamespaceScan {

    private final String namespace;
    private final List<ResolvedEntry> entries;
    private final List<AssetSource> sources;

    NamespaceScan(String namespace, List<ResolvedEntry> entries, List<AssetSource> sources) {
        this.namespace = namespace;
        this.entries = entries;
        this.sources = sources;
    }

    /** Lowercased namespace (vehicle name or {@code "common"}). */
    public String namespace() {
        return namespace;
    }

    /** Conflict-resolved entries, sorted by logical path for determinism. */
    public List<ResolvedEntry> entries() {
        return entries;
    }

    /** Distinct containers of the winning entries, in root order. */
    public List<AssetSource> sources() {
        return sources;
    }
}
